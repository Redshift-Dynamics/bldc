/*
	Copyright 2026

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.
*/

#include "hw.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "terminal.h"
#include "commands.h"
#include "mc_interface.h"
#include "pwm_servo.h"

#include <math.h>

// Settings
#define POWERBUT_HOLD_MS				2000
#define POWERBUT_SAMPLE_MS				10
#define POWERBUT_SHUTDOWN_ERPM			100.0
#define POWERBUT_SHUTDOWN_CURRENT		2.0
#define MOTORBUT_CURRENT_LIMIT			2.0
#define MOTORBUT_THREAD_SLEEP_MS		20

// Variables
static volatile bool i2c_running = false;
static volatile bool power_latched = false;
static volatile bool powerbut_shutdown_ready = false;
static volatile bool poweroff_armed = false;
static int powerbut_hold_ms = 0;
static bool motorbut_limit_active = false;
static mc_configuration motorbut_saved_conf;
static mc_configuration motorbut_limited_conf;
static THD_WORKING_AREA(motorbut_thread_wa, 512);

// Private functions
static void terminal_button_test(int argc, const char **argv);
static bool powerbut_shutdown_allowed(void);
static bool wait_powerbut_long_press(void);
static void motorbut_apply_limit(bool enable);
static THD_FUNCTION(motorbut_thread, arg);

#ifdef USE_HARDWARE_SPI
#include "hw_redshift_tachion_lsm6ds3_spi.c"
#endif

void hw_redshift_tachion_buzzer_on(void) {
	pwm_servo_set_duty(0.5);
}

void hw_redshift_tachion_buzzer_off(void) {
	pwm_servo_set_duty(0.0);
}

void buzzer_init(void) {
	pwm_servo_init(4000, 0.0);
	BUZZER_ON();
	chThdSleepMilliseconds(30);
	BUZZER_OFF();
}

void hw_redshift_tachion_disable_flash(void) {
	NAND_CS_DISABLE();
	palSetPadMode(NAND_CS_GPIO, NAND_CS_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
}

void hw_redshift_tachion_early_init(void) {
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	palSetPadMode(POWERBUT_GPIO, POWERBUT_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(POWERON_GPIO, POWERON_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	HW_SHUTDOWN_HOLD_OFF();

	hw_redshift_tachion_disable_flash();

	if (wait_powerbut_long_press()) {
		HW_SHUTDOWN_HOLD_ON();
		power_latched = true;
		powerbut_shutdown_ready = false;
		buzzer_init();
	}
}

void hw_init_gpio(void) {
	// GPIO clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// Keep NAND disabled before any other SPI3 device can be initialized.
	hw_redshift_tachion_disable_flash();

	// LEDs
	LED_GREEN_OFF();
	LED_RED_OFF();
	palSetPadMode(LED_GREEN_GPIO, LED_GREEN_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(LED_RED_GPIO, LED_RED_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	// Power stage disable, fan and power latch
	DISABLE_GATE();
	palSetPadMode(PS_DISABLE_GPIO, PS_DISABLE_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	ENABLE_GATE();

	FAN_OFF();
	palSetPadMode(FAN_GPIO, FAN_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	palSetPadMode(POWERON_GPIO, POWERON_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	if (power_latched) {
		HW_SHUTDOWN_HOLD_ON();
	} else {
		HW_SHUTDOWN_HOLD_OFF();
	}

	// TIM1 high-side outputs
	palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	// TIM1 low-side outputs
	palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	// Hall sensors and active-low buttons
	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(MOTORBUT_GPIO, MOTORBUT_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(POWERBUT_GPIO, POWERBUT_PIN, PAL_MODE_INPUT_PULLUP);

	// Keep the external SPI CS high by default.
	palSetPad(HW_SPI_PORT_NSS, HW_SPI_PIN_NSS);
	palSetPadMode(HW_SPI_PORT_NSS, HW_SPI_PIN_NSS,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	// Keep the LSM6DS3 SPI3 CS high until the IMU driver takes over.
	palSetPad(LSM6DS3_NSS_GPIO, LSM6DS3_NSS_PIN);
	palSetPadMode(LSM6DS3_NSS_GPIO, LSM6DS3_NSS_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	// ADC Pins
	palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);

	palSetPadMode(GPIOB, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOB, 1, PAL_MODE_INPUT_ANALOG);

	palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 4, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 5, PAL_MODE_INPUT_ANALOG);

	terminal_register_command_callback(
			"test_button",
			"Try sampling the Redshift Tachion buttons",
			0,
			terminal_button_test);

	chThdCreateStatic(motorbut_thread_wa, sizeof(motorbut_thread_wa),
			NORMALPRIO, motorbut_thread, NULL);
}

void hw_setup_adc_channels(void) {
	// ADC1 regular channels
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 4, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 5, ADC_SampleTime_15Cycles);

	// ADC2 regular channels
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_12, 2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_15, 4, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_10, 5, ADC_SampleTime_15Cycles);

	// ADC3 regular channels
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_3, 3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_10, 4, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_0, 5, ADC_SampleTime_15Cycles);

	// Injected channels
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_2, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_0, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_1, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_2, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_0, 3, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_1, 3, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_2, 3, ADC_SampleTime_15Cycles);
}

void hw_start_i2c(void) {
	if (!i2c_running) {
		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_OUTPUT_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_OUTPUT_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		i2c_running = true;
	}
}

void hw_stop_i2c(void) {
	if (i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN, PAL_MODE_INPUT);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN, PAL_MODE_INPUT);
		i2c_running = false;
	}
}

void hw_try_restore_i2c(void) {
	if (i2c_running) {
		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);
	}
}

bool hw_sample_shutdown_button(void) {
	if (!powerbut_shutdown_ready) {
		if (!POWERBUT_PRESSED()) {
			powerbut_shutdown_ready = true;
		}

		return true;
	}

	if (poweroff_armed) {
		if (!powerbut_shutdown_allowed()) {
			poweroff_armed = false;
			powerbut_hold_ms = 0;
			return true;
		}

		return POWERBUT_PRESSED();
	}

	if (POWERBUT_PRESSED()) {
		if (powerbut_hold_ms < POWERBUT_HOLD_MS) {
			powerbut_hold_ms += POWERBUT_SAMPLE_MS;
		}

		if (powerbut_hold_ms >= POWERBUT_HOLD_MS &&
				powerbut_shutdown_allowed()) {
			poweroff_armed = true;
			BUZZER_ON();
			chThdSleepMilliseconds(30);
			BUZZER_OFF();
		}
	} else {
		powerbut_hold_ms = 0;
	}

	return true;
}

float hw_redshift_tachion_get_temp(void) {
	return NTC_TEMP_MOS1();
}

static void terminal_button_test(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	commands_printf("sample    : %d", HW_SAMPLE_SHUTDOWN() ? 1 : 0);
	commands_printf("motorbut  : %d", MOTORBUT_PRESSED() ? 1 : 0);
	commands_printf("powerbut  : %d", POWERBUT_PRESSED() ? 1 : 0);
	commands_printf("poweron   : %d", palReadPad(POWERON_GPIO, POWERON_PIN));
	commands_printf("latched   : %d", power_latched ? 1 : 0);
	commands_printf("power hold: %d ms", powerbut_hold_ms);
	commands_printf("mot limit : %d", motorbut_limit_active ? 1 : 0);
	commands_printf("rpm       : %.1f", (double)mc_interface_get_rpm());
	commands_printf("current   : %.2f A",
			(double)mc_interface_get_tot_current_filtered());
	commands_printf("actual12v : %.2f V", (double)GET_ACTUAL_12V_VOLTAGE());
}

static bool powerbut_shutdown_allowed(void) {
	return fabsf(mc_interface_get_rpm()) < POWERBUT_SHUTDOWN_ERPM &&
			fabsf(mc_interface_get_tot_current_filtered()) <
			POWERBUT_SHUTDOWN_CURRENT;
}

static bool wait_powerbut_long_press(void) {
	for (int i = 0;i < (POWERBUT_HOLD_MS / POWERBUT_SAMPLE_MS);i++) {
		if (!POWERBUT_PRESSED()) {
			return false;
		}

		chThdSleepMilliseconds(POWERBUT_SAMPLE_MS);
	}

	return POWERBUT_PRESSED();
}

static void motorbut_apply_limit(bool enable) {
	if (enable == motorbut_limit_active) {
		return;
	}

	if (enable) {
		motorbut_saved_conf = *mc_interface_get_configuration();

		motorbut_limited_conf = motorbut_saved_conf;
		motorbut_limited_conf.l_current_min = -MOTORBUT_CURRENT_LIMIT;
		motorbut_limited_conf.l_current_max = MOTORBUT_CURRENT_LIMIT;
		motorbut_limited_conf.l_current_min_scale = 1.0;
		motorbut_limited_conf.l_current_max_scale = 1.0;
		motorbut_limited_conf.lo_current_min = -MOTORBUT_CURRENT_LIMIT;
		motorbut_limited_conf.lo_current_max = MOTORBUT_CURRENT_LIMIT;
		mc_interface_set_configuration(&motorbut_limited_conf);
		motorbut_limit_active = true;
	} else {
		mc_interface_set_configuration(&motorbut_saved_conf);
		motorbut_limit_active = false;
	}
}

void hw_redshift_tachion_motorbut_update(void) {
	motorbut_apply_limit(MOTORBUT_PRESSED());
}

static THD_FUNCTION(motorbut_thread, arg) {
	(void)arg;
	chRegSetThreadName("tachion motorbut");
	chThdSleepMilliseconds(1000);

	for (;;) {
		hw_redshift_tachion_motorbut_update();
		chThdSleepMilliseconds(MOTORBUT_THREAD_SLEEP_MS);
	}
}
