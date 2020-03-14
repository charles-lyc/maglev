#ifndef SW_CONF_H
#define SW_CONF_H

// larger than configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY can be 
// masked INT by freertos (taskENTER_CRITICAL()), otherwise you can
// only use __disable_irq().
#define INT_PRIO_CAN					3
#define INT_PRIO_FOC					1
#define INT_PRIO_ENC_SPI				2
#define INT_PRIO_ENC_PWM				2
#define INT_PRIO_TIM1					3		// test

#define FOC_TIM_SINGLE_FREQ				(40000)
#define FOC_CTRL_FREQ					(FOC_TIM_SINGLE_FREQ/2)
#define PID_CTRL_DT						(1.0f/(FOC_CTRL_FREQ))

#define ADC_REGULAR_PERIOD				(2000000)	// us

#define ENC_SAMPLE_PERIOD				(1.0f/10000)

#define FLASH_FW_CONF_START_ADDR		(0x8004000)
#define FLASH_MC_CONF_START_ADDR		(0x8008000)

#define CAN_LOCAL_ID					(CAN_FOC_ROLL)
#define FIRMWARE_VEWRSION				(1)

#endif
