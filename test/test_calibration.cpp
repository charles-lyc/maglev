/**
 * Maglev Calibration Test Harness
 *
 * Simulates the STM32 maglev system on a Linux host by:
 *   1. Mocking HAL types/functions
 *   2. Simulating ADC hall-sensor readings
 *   3. Running calibration via a background "ISR" thread with pacing
 *   4. Verifying the system transitions and offsets are correct
 *
 * A small delay is inserted in the ISR simulation loop to prevent the
 * ISR thread from racing far ahead of the calibration thread, which
 * would break the busy-wait timing that calibrate() relies on.
 */

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cassert>
#include <thread>
#include <atomic>
#include <chrono>

/* ---------- Pull in HAL mock before any src headers ---------- */
#include "hal_mock.h"

/* ---------- Provide extern HAL handles referenced by user.h ---------- */
DMA_HandleTypeDef hdma_adc;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;
ADC_HandleTypeDef hadc;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

/* Timer instance storage */
static TIM_Instance_Mock tim1_inst = {0, {0,0,0,0}};
static TIM_Instance_Mock tim3_inst = {0, {0,0,0,0}};

/* ---------- Include application sources ---------- */
#include "../src/maglev.h"
#include "../src/maglev.cpp"
#include "../src/pid.cpp"
#include "../src/filter.cpp"
#include "../src/differential.cpp"
#include "../src/LowPassFilter2p.cpp"
#include "../src/motor.cpp"

/* ================================================================
 * Test infrastructure
 * ================================================================ */

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_ASSERT(cond, msg) do { \
    if (!(cond)) { \
        printf("  [FAIL] %s  (line %d)\n", msg, __LINE__); \
        tests_failed++; \
    } else { \
        printf("  [PASS] %s\n", msg); \
        tests_passed++; \
    } \
} while(0)

#define TEST_ASSERT_NEAR(a, b, tol, msg) do { \
    if (fabs((double)(a) - (double)(b)) > (double)(tol)) { \
        printf("  [FAIL] %s  (got %.4f, expected ~%.4f, tol %.4f, line %d)\n", \
               msg, (double)(a), (double)(b), (double)(tol), __LINE__); \
        tests_failed++; \
    } else { \
        printf("  [PASS] %s  (value: %.4f)\n", msg, (double)(a)); \
        tests_passed++; \
    } \
} while(0)

/* ================================================================
 * Simulated ADC data (hall sensors)
 * ================================================================ */
volatile uint16_t sim_adc_raw[3];

/* Motor output tracking */
static float last_motor_a = 0.0f;
static float last_motor_b = 0.0f;
static bool last_drv_en_state = false;

static void sim_drv_en(bool en) {
    last_drv_en_state = en;
    printf("    drv_en(%s)\n", en ? "true" : "false");
}

static void sim_motor_write(float a, float b) {
    last_motor_a = a;
    last_motor_b = b;
}

/* ISR simulation delay: ~100us per tick to prevent the ISR thread from
   racing ahead of the calibration thread busy-wait logic. */
static const auto ISR_DELAY = std::chrono::microseconds(100);

/**
 * Helper: run calibration in a background thread while simulating ISR ticks.
 * Returns the number of ISR ticks that ran during calibration.
 */
static int run_calibration_with_isr(Maglev &mag) {
    std::atomic<bool> cal_done(false);
    std::thread cal_thread([&]() {
        mag.calibrate();
        cal_done.store(true);
    });

    int isr_ticks = 0;
    while (!cal_done.load()) {
        mag.adc_sample_callback();
        isr_ticks++;
        std::this_thread::sleep_for(ISR_DELAY);
        if (isr_ticks > 10000) break;
    }
    cal_thread.join();
    return isr_ticks;
}

/**
 * Helper: initialize timer instances to known state.
 */
static void reset_timer_instances() {
    memset(&tim1_inst, 0, sizeof(tim1_inst));
    memset(&tim3_inst, 0, sizeof(tim3_inst));
    htim1.Instance = &tim1_inst;
    htim3.Instance = &tim3_inst;
    tim1_inst.AutoReload = 1000;
    tim3_inst.AutoReload = 1000;
}

/* ================================================================
 * Test 1: Basic calibration with known offsets
 * ================================================================ */
static void test_calibration_basic() {
    printf("\n=== Test 1: Basic Calibration with Known Offsets ===\n");

    Maglev mag;
    reset_timer_instances();

    sim_adc_raw[0] = 2048;
    sim_adc_raw[1] = 1500;
    sim_adc_raw[2] = 2000;

    mag.init((uint16_t *)sim_adc_raw,
             false, false,
             sim_drv_en, sim_motor_write);

    printf("  Maglev initialized.\n");

    last_motor_a = 0; last_motor_b = 0;
    mag.adc_sample_callback();
    TEST_ASSERT(last_motor_a == 0.0f && last_motor_b == 0.0f,
                "Motor output is zero before calibration (MODE_READY)");

    printf("  Running 200 ADC callbacks to settle filters...\n");
    for (int i = 0; i < 200; i++)
        mag.adc_sample_callback();

    printf("  Starting calibration...\n");
    int isr_ticks = run_calibration_with_isr(mag);
    printf("  Calibration completed after %d ISR ticks.\n", isr_ticks);

    TEST_ASSERT(isr_ticks >= 100, "At least 100 ISR ticks occurred during calibration");

    printf("  Running 500 post-calibration callbacks...\n");
    for (int i = 0; i < 500; i++)
        mag.adc_sample_callback();

    printf("  Motor A: %.4f, Motor B: %.4f\n", last_motor_a, last_motor_b);
    TEST_ASSERT_NEAR(last_motor_a, 0.0f, 5.0f,
                     "Motor A near zero after calibration at rest");
    TEST_ASSERT_NEAR(last_motor_b, 0.0f, 5.0f,
                     "Motor B near zero after calibration at rest");
}

/* ================================================================
 * Test 2: Calibration with inverted channels
 * ================================================================ */
static void test_calibration_inverted() {
    printf("\n=== Test 2: Calibration with Inverted Channel A ===\n");

    Maglev mag;
    reset_timer_instances();

    sim_adc_raw[0] = 2048;
    sim_adc_raw[1] = 1500;
    sim_adc_raw[2] = 2000;

    mag.init((uint16_t *)sim_adc_raw,
             true, false,
             sim_drv_en, sim_motor_write);

    for (int i = 0; i < 200; i++)
        mag.adc_sample_callback();

    int isr_ticks = run_calibration_with_isr(mag);
    printf("  Calibration completed after %d ISR ticks.\n", isr_ticks);
    TEST_ASSERT(isr_ticks >= 100, "Calibration with inverted channel (100+ ticks)");

    for (int i = 0; i < 500; i++)
        mag.adc_sample_callback();

    printf("  Motor A: %.4f, Motor B: %.4f\n", last_motor_a, last_motor_b);
    TEST_ASSERT_NEAR(last_motor_a, 0.0f, 5.0f,
                     "Motor A near zero (inverted channel)");
    TEST_ASSERT_NEAR(last_motor_b, 0.0f, 5.0f,
                     "Motor B near zero (inverted channel)");
}

/* ================================================================
 * Test 3: Calibration followed by disturbance
 * ================================================================ */
static void test_calibration_then_disturbance() {
    printf("\n=== Test 3: Calibration then Position Disturbance ===\n");

    Maglev mag;
    reset_timer_instances();

    sim_adc_raw[0] = 2048;
    sim_adc_raw[1] = 2048;
    sim_adc_raw[2] = 2000;

    mag.init((uint16_t *)sim_adc_raw,
             false, false,
             sim_drv_en, sim_motor_write);

    for (int i = 0; i < 200; i++)
        mag.adc_sample_callback();

    int isr_ticks = run_calibration_with_isr(mag);
    TEST_ASSERT(isr_ticks >= 100, "Initial calibration completed (100+ ticks)");

    for (int i = 0; i < 500; i++)
        mag.adc_sample_callback();

    printf("  Rest motor A: %.4f, B: %.4f\n", last_motor_a, last_motor_b);

    printf("  Applying disturbance: channel A +200\n");
    sim_adc_raw[0] = 2248;

    for (int i = 0; i < 500; i++)
        mag.adc_sample_callback();

    printf("  After disturbance: Motor A: %.4f, Motor B: %.4f\n",
           last_motor_a, last_motor_b);

    TEST_ASSERT(fabs(last_motor_a) > 1.0f,
                "Motor A produces corrective output after disturbance");
    TEST_ASSERT_NEAR(last_motor_b, 0.0f, 5.0f,
                     "Motor B remains near zero (no disturbance on B)");
}

/* ================================================================
 * Test 4: Re-calibration resets offsets
 * ================================================================ */
static void test_recalibration() {
    printf("\n=== Test 4: Re-calibration After Position Change ===\n");

    Maglev mag;
    reset_timer_instances();

    sim_adc_raw[0] = 2048;
    sim_adc_raw[1] = 2048;
    sim_adc_raw[2] = 2000;

    mag.init((uint16_t *)sim_adc_raw,
             false, false,
             sim_drv_en, sim_motor_write);

    for (int i = 0; i < 200; i++)
        mag.adc_sample_callback();
    run_calibration_with_isr(mag);
    for (int i = 0; i < 500; i++)
        mag.adc_sample_callback();

    sim_adc_raw[0] = 2300;
    sim_adc_raw[1] = 2300;
    for (int i = 0; i < 500; i++)
        mag.adc_sample_callback();

    printf("  Before re-cal: Motor A: %.4f, B: %.4f\n", last_motor_a, last_motor_b);
    TEST_ASSERT(fabs(last_motor_a) > 1.0f || fabs(last_motor_b) > 1.0f,
                "Non-zero motor output before re-calibration");

    printf("  Re-calibrating...\n");
    int isr_ticks = run_calibration_with_isr(mag);
    TEST_ASSERT(isr_ticks >= 100, "Re-calibration completed (100+ ticks)");

    for (int i = 0; i < 500; i++)
        mag.adc_sample_callback();

    printf("  After re-cal: Motor A: %.4f, B: %.4f\n", last_motor_a, last_motor_b);
    TEST_ASSERT_NEAR(last_motor_a, 0.0f, 5.0f, "Motor A near zero after re-cal");
    TEST_ASSERT_NEAR(last_motor_b, 0.0f, 5.0f, "Motor B near zero after re-cal");
}

/* ================================================================
 * Test 5: Calibration guard in STARTUP mode
 * ================================================================ */
static void test_calibrate_startup_guard() {
    printf("\n=== Test 5: Calibration Guard -- STARTUP Mode ===\n");
    Maglev mag;
    mag.calibrate();
    TEST_ASSERT(true, "calibrate() returned without hanging in MODE_STARTUP");
}

/* ================================================================
 * Test 6: Detect function and mode transitions
 * ================================================================ */
static void test_detect_mode_transitions() {
    printf("\n=== Test 6: Detect Function and Mode Transitions ===\n");

    Maglev mag;
    reset_timer_instances();

    sim_adc_raw[0] = 2048;
    sim_adc_raw[1] = 2048;
    sim_adc_raw[2] = 2048;

    mag.init((uint16_t *)sim_adc_raw,
             false, false,
             sim_drv_en, sim_motor_write);

    for (int i = 0; i < 200; i++)
        mag.adc_sample_callback();
    run_calibration_with_isr(mag);

    printf("  main_loop with no magnet...\n");
    for (int i = 0; i < 200; i++)
        mag.adc_sample_callback();
    last_drv_en_state = true;
    mag.main_loop();

    TEST_ASSERT(last_drv_en_state == false,
                "Driver disabled when no magnet (MODE_READY)");

    for (int i = 0; i < 200; i++)
        mag.adc_sample_callback();
    TEST_ASSERT_NEAR(last_motor_a, 0.0f, 1.0f,
                     "Motor A zero in MODE_READY");

    printf("  Simulating magnet (channel 2 drops by 800)...\n");
    sim_adc_raw[2] = 2048 - 800;
    for (int i = 0; i < 500; i++)
        mag.adc_sample_callback();

    last_drv_en_state = false;
    mag.main_loop();

    TEST_ASSERT(last_drv_en_state == true,
                "Driver enabled when magnet detected (MODE_RUN)");

    for (int i = 0; i < 500; i++)
        mag.adc_sample_callback();

    printf("  Motor A: %.4f, B: %.4f\n", last_motor_a, last_motor_b);
    TEST_ASSERT_NEAR(last_motor_a, 0.0f, 5.0f,
                     "Motor A near zero with stable magnet");
    TEST_ASSERT_NEAR(last_motor_b, 0.0f, 5.0f,
                     "Motor B near zero with stable magnet");
}

/* ================================================================
 * Test 7: Full system flow -- init, calibrate, drift, recalibrate
 * ================================================================ */
static void test_button_recalibration() {
    printf("\n=== Test 7: Full Flow -- Calibrate, Drift, Recalibrate ===\n");

    Maglev mag;
    reset_timer_instances();

    sim_adc_raw[0] = 2048;
    sim_adc_raw[1] = 2048;
    sim_adc_raw[2] = 2048;

    mag.init((uint16_t *)sim_adc_raw,
             true, false,
             sim_drv_en, sim_motor_write);

    printf("  Step 1: Initial calibration...\n");
    for (int i = 0; i < 200; i++)
        mag.adc_sample_callback();
    run_calibration_with_isr(mag);
    for (int i = 0; i < 500; i++)
        mag.adc_sample_callback();

    TEST_ASSERT_NEAR(last_motor_a, 0.0f, 5.0f, "Motor A zero after init cal");
    TEST_ASSERT_NEAR(last_motor_b, 0.0f, 5.0f, "Motor B zero after init cal");

    printf("  Step 2: Drift +100...\n");
    sim_adc_raw[0] = 2148;
    sim_adc_raw[1] = 2148;
    for (int i = 0; i < 500; i++)
        mag.adc_sample_callback();

    printf("  Motor A: %.4f, B: %.4f\n", last_motor_a, last_motor_b);
    TEST_ASSERT(fabs(last_motor_a) > 1.0f || fabs(last_motor_b) > 1.0f,
                "Non-zero output after drift");

    printf("  Step 3: Button re-calibration...\n");
    run_calibration_with_isr(mag);
    for (int i = 0; i < 500; i++)
        mag.adc_sample_callback();

    printf("  Motor A: %.4f, B: %.4f\n", last_motor_a, last_motor_b);
    TEST_ASSERT_NEAR(last_motor_a, 0.0f, 5.0f, "Motor A zero after re-cal");
    TEST_ASSERT_NEAR(last_motor_b, 0.0f, 5.0f, "Motor B zero after re-cal");
}

/* ================================================================
 * Main
 * ================================================================ */
int main() {
    printf("========================================\n");
    printf("  Maglev Calibration Test Suite\n");
    printf("========================================\n");

    test_calibrate_startup_guard();
    test_calibration_basic();
    test_calibration_inverted();
    test_calibration_then_disturbance();
    test_recalibration();
    test_detect_mode_transitions();
    test_button_recalibration();

    printf("\n========================================\n");
    printf("  Results: %d passed, %d failed\n", tests_passed, tests_failed);
    printf("========================================\n");

    return tests_failed > 0 ? 1 : 0;
}
