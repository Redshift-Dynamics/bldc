/*
	Copyright 2026

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.
*/

#ifndef HW_REDSHIFT_TACHION_CORE_H_
#define HW_REDSHIFT_TACHION_CORE_H_

#if !defined(HW_REDSHIFT_TACHION_T18) && !defined(HW_REDSHIFT_TACHION_T36)
#error "Must define a Redshift Tachion hardware variant"
#endif

// HW properties
#define HW_HAS_3_SHUNTS
#define HW_HAS_PHASE_SHUNTS
#define SERVO_BUZZER

// LEDs
#define LED_GREEN_GPIO			GPIOB
#define LED_GREEN_PIN			5
#define LED_RED_GPIO			GPIOB
#define LED_RED_PIN				6

#define LED_GREEN_ON()			palSetPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_GREEN_OFF()			palClearPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_RED_ON()			palSetPad(LED_RED_GPIO, LED_RED_PIN)
#define LED_RED_OFF()			palClearPad(LED_RED_GPIO, LED_RED_PIN)

// Power stage disable, active high.
#define PS_DISABLE_GPIO			GPIOC
#define PS_DISABLE_PIN			9
#define ENABLE_GATE()			palClearPad(PS_DISABLE_GPIO, PS_DISABLE_PIN)
#define DISABLE_GATE()			palSetPad(PS_DISABLE_GPIO, PS_DISABLE_PIN)

// Power and carry buttons are active-low inputs.
#define MOTORBUT_GPIO			GPIOD
#define MOTORBUT_PIN			2
#define POWERBUT_GPIO			GPIOB
#define POWERBUT_PIN			4
#define POWERON_GPIO			GPIOB
#define POWERON_PIN				3

#define MOTORBUT_PRESSED()		(!palReadPad(MOTORBUT_GPIO, MOTORBUT_PIN))
#define POWERBUT_PRESSED()		(!palReadPad(POWERBUT_GPIO, POWERBUT_PIN))
#define POWERON_ON()			palSetPad(POWERON_GPIO, POWERON_PIN)
#define POWERON_OFF()			palClearPad(POWERON_GPIO, POWERON_PIN)

#define HW_SHUTDOWN_GPIO		POWERON_GPIO
#define HW_SHUTDOWN_PIN			POWERON_PIN
#define HW_SHUTDOWN_HOLD_ON()	POWERON_ON()
#define HW_SHUTDOWN_HOLD_OFF()	POWERON_OFF()
#define HW_SAMPLE_SHUTDOWN()	hw_sample_shutdown_button()

// Expose carry and power buttons to Lisp as (gpio-read 'pin-hw-1/2).
#define PIN_HW_1_GPIO			MOTORBUT_GPIO
#define PIN_HW_1				MOTORBUT_PIN
#define PIN_HW_2_GPIO			POWERBUT_GPIO
#define PIN_HW_2				POWERBUT_PIN

// Fan and buzzer
#define FAN_GPIO				GPIOB
#define FAN_PIN					7
#define FAN_ON()				palSetPad(FAN_GPIO, FAN_PIN)
#define FAN_OFF()				palClearPad(FAN_GPIO, FAN_PIN)

#define BUZZER_GPIO				GPIOC
#define BUZZER_PIN				6
#define BUZZER_ON()				hw_redshift_tachion_buzzer_on()
#define BUZZER_OFF()			hw_redshift_tachion_buzzer_off()

// NAND chip select on SPI3, active low. Keep disabled unless explicitly used.
#define NAND_CS_GPIO			GPIOC
#define NAND_CS_PIN				15
#define NAND_CS_ENABLE()		palClearPad(NAND_CS_GPIO, NAND_CS_PIN)
#define NAND_CS_DISABLE()		palSetPad(NAND_CS_GPIO, NAND_CS_PIN)

/*
 * ADC Vector
 *
 * 0  (1):	PA0	IN0		CURRENT_C
 * 1  (2):	PA1	IN1		CURRENT_B
 * 2  (3):	PA2	IN2		CURRENT_A
 * 3  (1):	PC1	IN11	VOLTAGE_C
 * 4  (2):	PC2	IN12	VOLTAGE_B
 * 5  (3):	PC3	IN13	VOLTAGE_A
 * 6  (1):	PB0	IN8		ADC_8
 * 7  (2):	PB1	IN9		ADC_9
 * 8  (3):	PA3	IN3		MOS_TEMP
 * 9  (1):	PC4	IN14	MOTOR_TEMP
 * 10 (2):	PC5	IN15	ACTUAL_12V
 * 11 (3):	PC0	IN10	BUS_VOLTAGE
 * 12 (1):	Vrefint
 * 13 (2):	PC0	IN10	BUS_VOLTAGE duplicate
 * 14 (3):	PA0	IN0		CURRENT_C duplicate
 */
#define HW_ADC_CHANNELS			15
#define HW_ADC_INJ_CHANNELS		3
#define HW_ADC_NBR_CONV			5

// ADC indexes. Curr1/Sens1 are phase A.
#define ADC_IND_CURR1			2
#define ADC_IND_CURR2			1
#define ADC_IND_CURR3			0
#define ADC_IND_SENS1			5
#define ADC_IND_SENS2			4
#define ADC_IND_SENS3			3
#define ADC_IND_EXT				6
#define ADC_IND_EXT2			7
#define ADC_IND_TEMP_MOS		8
#define ADC_IND_TEMP_MOTOR		9
#define ADC_IND_ACTUAL_12V		10
#define ADC_IND_VIN_SENS		11
#define ADC_IND_VREFINT			12

#ifndef V_REG
#define V_REG					3.3
#endif
#define VIN_R1					120000.0
#define VIN_R2					2200.0
#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN		1.0
#endif
#ifndef REDSHIFT_TACHION_CURRENT_MEASUREMENT_LIMIT
#define REDSHIFT_TACHION_CURRENT_MEASUREMENT_LIMIT	550.0
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES		(V_REG / \
		(2.0 * REDSHIFT_TACHION_CURRENT_MEASUREMENT_LIMIT))
#endif

#define ACTUAL_12V_R1			10000.0
#define ACTUAL_12V_R2			2200.0

#define GET_INPUT_VOLTAGE()		((V_REG / 4095.0) * \
		(float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2))
#define GET_ACTUAL_12V_VOLTAGE()	((V_REG / 4095.0) * \
		(float)ADC_Value[ADC_IND_ACTUAL_12V] * \
		((ACTUAL_12V_R1 + ACTUAL_12V_R2) / ACTUAL_12V_R2))

// NTC thermistors: upper NTC, lower 4k7 to GND.
#define NTC_PULLDOWN_RES		4700.0
#define NTC_RES(adc_val)		(((4095.0 / (float)(adc_val)) - 1.0) * \
		NTC_PULLDOWN_RES)
#define NTC_TEMP(adc_ind)		hw_redshift_tachion_get_temp()
#define NTC_RES_MOTOR(adc_val)	NTC_RES(adc_val)
#define NTC_TEMP_MOTOR(beta)	(1.0 / \
		((logf(NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) / 10000.0) / \
		beta) + (1.0 / 298.15)) - 273.15)

#define NTC_TEMP_MOS1()			(1.0 / \
		((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS]) / 10000.0) / 3380.0) + \
		(1.0 / 298.15)) - 273.15)

#define ADC_VOLTS(ch)			((float)ADC_Value[ch] / 4095.0 * V_REG)

// COMM-port ADC GPIOs
#define HW_ADC_EXT_GPIO			GPIOB
#define HW_ADC_EXT_PIN			0
#define HW_ADC_EXT2_GPIO		GPIOB
#define HW_ADC_EXT2_PIN			1

// UART peripheral
#define HW_UART_DEV				SD3
#define HW_UART_GPIO_AF			GPIO_AF_USART3
#define HW_UART_TX_PORT			GPIOB
#define HW_UART_TX_PIN			10
#define HW_UART_RX_PORT			GPIOB
#define HW_UART_RX_PIN			11

// ICU Peripheral for servo buzzer/servo output on the redshift buzzer pin.
#define HW_ICU_TIMER			TIM3
#define HW_ICU_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)
#define HW_ICU_DEV				ICUD3
#define HW_ICU_CHANNEL			ICU_CHANNEL_1
#define HW_ICU_GPIO_AF			GPIO_AF_TIM3
#define HW_ICU_GPIO				BUZZER_GPIO
#define HW_ICU_PIN				BUZZER_PIN

// CAN peripheral
#define HW_CAN_DEV				CAND1
#define HW_CAN_GPIO_AF			GPIO_AF_CAN1
#define HW_CANRX_PORT			GPIOB
#define HW_CANRX_PIN			8
#define HW_CANTX_PORT			GPIOB
#define HW_CANTX_PIN			9

// External SPI pins on PA4..PA7.
#define HW_SPI_DEV				SPID1
#define HW_SPI_GPIO_AF			GPIO_AF_SPI1
#define HW_SPI_PORT_NSS			GPIOA
#define HW_SPI_PIN_NSS			4
#define HW_SPI_PORT_SCK			GPIOA
#define HW_SPI_PIN_SCK			5
#define HW_SPI_PORT_MISO		GPIOA
#define HW_SPI_PIN_MISO			6
#define HW_SPI_PORT_MOSI		GPIOA
#define HW_SPI_PIN_MOSI			7

// Software-I2C scaffold on the external connector. Hardware I2C is not used.
#define HW_I2C_DEV				I2CD2
#define HW_I2C_GPIO_AF			GPIO_AF_I2C2
#define HW_I2C_SDA_PORT			GPIOA
#define HW_I2C_SDA_PIN			4
#define HW_I2C_SCL_PORT			GPIOA
#define HW_I2C_SCL_PIN			7

// LSM6DS3 on SPI3 pins. The regular LSM6DS3 SPI path in imu.c uses spi_bb;
// redirect those calls to the Redshift Tachion hardware-SPI implementation.
#define LSM6DS3_USE_SPI
#define LSM6DS3_SPI_DEV			SPID3
#define LSM6DS3_SPI_GPIO_AF		GPIO_AF_SPI3
#define LSM6DS3_SPI_CR1			(SPI_CR1_CPOL | SPI_CR1_CPHA | ((uint16_t)0x0010))
#define LSM6DS3_NSS_GPIO		GPIOC
#define LSM6DS3_NSS_PIN			13
#define LSM6DS3_SCK_GPIO		GPIOC
#define LSM6DS3_SCK_PIN			10
#define LSM6DS3_MISO_GPIO		GPIOC
#define LSM6DS3_MISO_PIN		11
#define LSM6DS3_MOSI_GPIO		GPIOC
#define LSM6DS3_MOSI_PIN		12

// LSM6DS3 has no magnetometer.
#ifndef APPCONF_IMU_USE_MAGNETOMETER
#define APPCONF_IMU_USE_MAGNETOMETER	false
#endif

#ifdef IMU_IMU_H_
#define spi_bb_init				hw_redshift_tachion_lsm6ds3_spi_bb_init
#define lsm6ds3_init			hw_redshift_tachion_lsm6ds3_spi_init
#define lsm6ds3_set_rate_hz		hw_redshift_tachion_lsm6ds3_spi_set_rate_hz
#define lsm6ds3_set_filter		hw_redshift_tachion_lsm6ds3_spi_set_filter
#define lsm6ds3_set_read_callback \
	hw_redshift_tachion_lsm6ds3_spi_set_read_callback
#define lsm6ds3_stop			hw_redshift_tachion_lsm6ds3_spi_stop
#endif

// Hall pins. These are hall inputs only; ABI encoder mode is stubbed for now.
#define HW_HALL_ENC_GPIO1		GPIOC
#define HW_HALL_ENC_PIN1		8
#define HW_HALL_ENC_GPIO2		GPIOC
#define HW_HALL_ENC_PIN2		7
#define HW_HALL_ENC_GPIO3		GPIOB
#define HW_HALL_ENC_PIN3		12
// This timer is intentionally not TIM5: TIM5 is the firmware timebase used by
// timer_time_now(), and AHRS/RPY depends on it for IMU dt. ABI is not valid on
// this pinout, so TIM4 is only a harmless sink for encoder_init/deinit.
#define HW_ENC_TIM				TIM4
#define HW_ENC_TIM_AF			GPIO_AF_TIM4
#define HW_ENC_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
#define HW_ENC_EXTI_PORTSRC		EXTI_PortSourceGPIOB
#define HW_ENC_EXTI_PINSRC		EXTI_PinSource12
#define HW_ENC_EXTI_CH			EXTI15_10_IRQn
#define HW_ENC_EXTI_LINE		EXTI_Line12
#define HW_ENC_EXTI_ISR_VEC		EXTI15_10_IRQHandler
#define HW_ENC_TIM_ISR_CH		TIM4_IRQn
#define HW_ENC_TIM_ISR_VEC		TIM4_IRQHandler

// Measurement macros
#define ADC_V_L1				ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2				ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3				ADC_Value[ADC_IND_SENS3]
#define ADC_V_ZERO				(ADC_Value[ADC_IND_VIN_SENS] / 2)

#define READ_HALL1()			palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()			palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()			palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

// Align current sample order with phase A/B/C.
#define HW_GET_INJ_CURR1()		ADC_GetInjectedConversionValue(ADC3, \
		ADC_InjectedChannel_1)
#define HW_GET_INJ_CURR2()		ADC_GetInjectedConversionValue(ADC2, \
		ADC_InjectedChannel_1)
#define HW_GET_INJ_CURR3()		ADC_GetInjectedConversionValue(ADC1, \
		ADC_InjectedChannel_1)

#define HW_DEAD_TIME_NSEC		450.0

// Default setting overrides
#ifndef MCCONF_L_MIN_VOLTAGE
#define MCCONF_L_MIN_VOLTAGE			35.0
#endif
#ifndef MCCONF_DEFAULT_MOTOR_TYPE
#define MCCONF_DEFAULT_MOTOR_TYPE		MOTOR_TYPE_FOC
#endif
#ifndef MCCONF_FOC_F_ZV
#define MCCONF_FOC_F_ZV					30000.0
#endif
#ifndef MCCONF_FOC_CONTROL_SAMPLE_MODE
#define MCCONF_FOC_CONTROL_SAMPLE_MODE	FOC_CONTROL_SAMPLE_MODE_V0_V7
#endif
#ifndef MCCONF_L_MAX_VOLTAGE
#define MCCONF_L_MAX_VOLTAGE			180.0
#endif
#ifndef MCCONF_L_MAX_ABS_CURRENT
#define MCCONF_L_MAX_ABS_CURRENT		550.0
#endif
// Setting limits
#ifndef HW_LIM_CURRENT
#define HW_LIM_CURRENT			-550.0, 550.0
#endif
#ifndef HW_LIM_CURRENT_IN
#define HW_LIM_CURRENT_IN		-250.0, 250.0
#endif
#ifndef HW_LIM_CURRENT_ABS
#define HW_LIM_CURRENT_ABS		0.0, 550.0
#endif
#define HW_LIM_VIN				35.0, 200.0
#define HW_LIM_ERPM				-200e3, 200e3
#define HW_LIM_DUTY_MIN			0.0, 0.01
#define HW_LIM_DUTY_MAX			0.0, 0.999
#define HW_LIM_TEMP_FET			-40.0, 110.0

// Functions
float hw_redshift_tachion_get_temp(void);
bool hw_sample_shutdown_button(void);
void hw_redshift_tachion_buzzer_on(void);
void hw_redshift_tachion_buzzer_off(void);
void hw_redshift_tachion_disable_flash(void);
void hw_redshift_tachion_early_init(void);
void hw_redshift_tachion_motorbut_update(void);
void buzzer_init(void);

#define HW_EARLY_INIT()			hw_redshift_tachion_early_init()

#endif /* HW_REDSHIFT_TACHION_CORE_H_ */
