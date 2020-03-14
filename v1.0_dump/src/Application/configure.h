#ifndef CONFIGURE_H
#define CONFIGURE_H

/* -------------------------- Current project ------------------------------- */
#define DEFAULT_Kp					5
#define DEFAULT_Ki					0.1
#define DEFAULT_Kd					0.1
#define DEFAULT_SAMPLE_RATE_HZ		500
#define PID_OUTPUT_FACTOR			1
#define MOTOR_DEAD_TIME				0
#define INTERGRAL_CONSTAIN			1000

#define SerialPort_Debug			(1)
#define UART1_TX_BUFFER_SIZE 		(128)
#define UART1_RX_BUFFER_SIZE 		(128)
#define UART2_TX_BUFFER_SIZE 		(128)
#define UART2_RX_BUFFER_SIZE 		(128)
#define UART3_TX_BUFFER_SIZE 		(128)
#define UART3_RX_BUFFER_SIZE 		(128)

/* -------------------------- Compatible ------------------------------------ */
/* Clock */
#define HSECLK						8000000
#define SYSCLK						48000000	// f(HCLK)
#define AHB1CLK						48000000
#define AHB2CLK						48000000
#define AHB3CLK						48000000
#define APB1CLK						48000000
#define APB2CLK						48000000

/* Bootloader */
// vector table offset for FW
#define NO_BOOTLOADER
#ifdef NO_BOOTLOADER
#ifndef VECT_TAB_OFFSET
#define VECT_TAB_OFFSET  			(0x0000)
#else
#define VECT_TAB_OFFSET  			(0x4000)
#endif
#define BOOT_SIGNATURE_ADDR			(SRAM_BASE+0x3F00)
#endif

/* Flash */
#define FLASH_CONFIGURE_ADDR		(0x8100000)
#define FLASH_CONFIGURE_SIZE		(0x4000)

/* Debug */
// #define USE_FULL_ASSERT 			// define in IDE
#ifdef DEBUG
#define USE_PRINT
#define DEBUG_PRINT(...) 			printf(__VA_ARGS__)
#else
//#define printf(...)
#define DEBUG_PRINT(str...) 		do { } while(0)
#endif

/* Serial */
#define SerialPort_FC				(1)     // uart #1
#define SerialPort_RF				(2)
#define SERIAL_RF_BAUDRATE			(115200)
#define SERIAL_FC_BAUDRATE			(57600)
#define RF_TX_BUFFER_SIZE			(512)
#define RF_RX_BUFFER_SIZE			(1024)
#define RCCH_UPDATE_PERIOD_APP		(200)
#define RCCH_UPDATE_PERIOD_FC		(40)	// cant be too little, incaseof client cant initial
#define RCCH_UPDATE_PERIOD_FC_IDLE	(200)

/* RM024 */
//#define RM024_USE_PIN_TO_EN_ATMODE
#define RM024_DYNAMIC_POWER
//#define RM024_DYNAMIC_BANDWIDTH
#define PAIR_PORT					GPIOD
#define PAIR_PIN					GPIO_Pin_12
#define PPM_PORT					GPIOD
#define PPM_PIN						GPIO_Pin_13
#ifdef DEBUG
#define RM024_DEFAULT_POWER			EEPROM_LOW_POWER
#else
#define RM024_DEFAULT_POWER			EEPROM_FULL_POWER
#endif
#define RM024_DEFAULT_CHANNEL		0x10
#define RM024_DEFAULT_SYSTEMID		0x10
#define RM024_DEFAULT_PROFILE		EEPROM_RF_PROFILE_280_43_FCC_FEC
#define RM024_PAIR_CHANNEL			0	
#define RM024_PAIR_SYSTEMID			0
// large retry times result in "stuck", making AT mode unacessable!
#ifdef GROUND_CONTROL
#define RM024_DEFAULT_TX_RETRIES	2
#else
#define RM024_DEFAULT_TX_RETRIES	3
#endif
#define RM024_DEFAULT_BC_ATTEMPT	1

/* Radio */
#define ADC_CHANNEL_NUM				(3)
#define ADC_FILTER_SIZE				(4)
#define STICK_NEUTRAL_ZONE			(1000)		// in +-10K 10%
#define WHEEL_NEUTRAL_ZONE			(2000)		// in +-10K	20%
#define NB_CHANNEL_CURVE_POINTS 	(10)
#define NB_RC_CHANNELS				(5)
#define NB_ALL_CHANNELS				(6)

/* Mavlink */
// #define MAVLINK_CRC_NO_CHECK		NEVER USE IT! Everytime you parse pkt, crc will be ALWAYS correct.
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define MAVLINK_SEND_UART_BYTES 	MavlinkSendBytes
#define MAVLINK_START_UART_SEND		MavlinkStartCallback
#define MAVLINK_END_UART_SEND		MavlinkEndCallback
#define MAVLINK_CH_RF				MAVLINK_COMM_0
#ifdef GROUND_CONTROL
#define MAVLINK_CH_APP				MAVLINK_COMM_1
#define MAVLINK_CH_FC				MAVLINK_CH_RF
#define MAVLINK_CH_DEBUG			MAVLINK_COMM_2
#else
#define MAVLINK_CH_APP				MAVLINK_CH_RF
#define MAVLINK_CH_FC				MAVLINK_COMM_1
#define MAVLINK_CH_DEBUG			MAVLINK_COMM_2
#endif

/* Button */
#define BTN_LONG_PRESS_TIME			(1000)
#define MSG_BTN_QUEUE_SIZE			(5)

/* LED */
#define LED_BRIGHTNESS				(20)	// 0~255

/* USB */
#define USB_CDC_UART

/* Version */
#define RM024_PROFILE_VERSION		0x34 	// should +1 when Change anything below
#define LOCAL_RC_VERSION			{3,2,6}
#define CONFIGURE_VERSION			(0x00000001)

#endif

