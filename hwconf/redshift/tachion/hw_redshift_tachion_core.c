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
#include "spi_bb.h"
#include "lsm6ds3.h"

#include <math.h>
#include <stdio.h>

// Settings
#define POWERBUT_HOLD_MS				2000
#define POWERBUT_SAMPLE_MS				10
#define POWERBUT_SHUTDOWN_ERPM			100.0
#define POWERBUT_SHUTDOWN_CURRENT		2.0
#define MOTORBUT_CURRENT_LIMIT			2.0
#define MOTORBUT_THREAD_SLEEP_MS		20
#define TACHION_STR_IMPL(...)			#__VA_ARGS__
#define TACHION_STR(...)				TACHION_STR_IMPL(__VA_ARGS__)

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
static void terminal_imu_probe(int argc, const char **argv);
static void terminal_selfcheck(int argc, const char **argv);
static void tachion_imu_probe_init(spi_bb_state *spi,
		int *miso_pullup, int *miso_pulldown);
static uint8_t tachion_imu_probe_read_reg(spi_bb_state *spi, uint8_t reg);
static bool tachion_imu_whoami_valid(uint8_t val);
static int tachion_selfcheck_print_bool(const char *name, bool ok);
static int tachion_selfcheck_print_level(const char *name, int level,
		int expected);
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
	terminal_register_command_callback(
			"tachion_imu_probe",
			"Probe the Redshift Tachion LSM6DS3 SPI pins",
			"[reg]",
			terminal_imu_probe);
	terminal_register_command_callback(
			"tachion_selfcheck",
			"Run Redshift Tachion hardware self-check",
			0,
			terminal_selfcheck);

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

static void terminal_imu_probe(int argc, const char **argv) {
	int reg = LSM6DS3_ACC_GYRO_WHO_AM_I_REG;

	if (argc == 2) {
		sscanf(argv[1], "%i", &reg);
	}

	if (argc > 2 || reg < 0 || reg > 0xFF) {
		commands_printf("Usage: tachion_imu_probe [reg]");
		return;
	}

	spi_bb_state spi;
	int miso_pullup = 0;
	int miso_pulldown = 0;
	tachion_imu_probe_init(&spi, &miso_pullup, &miso_pulldown);

	commands_printf("Tachion IMU probe reg 0x%02X", reg);
	commands_printf("CS imu:%d nand:%d miso pu/pd:%d/%d",
			palReadPad(LSM6DS3_NSS_GPIO, LSM6DS3_NSS_PIN),
			palReadPad(NAND_CS_GPIO, NAND_CS_PIN),
			miso_pullup, miso_pulldown);

	for (int i = 0;i < 8;i++) {
		uint8_t val = tachion_imu_probe_read_reg(&spi, (uint8_t)reg);
		commands_printf("read_pu[%d] reg 0x%02X = 0x%02X (%d)",
				i, reg, val, val);
		chThdSleepMilliseconds(2);
	}

	palSetPadMode(LSM6DS3_MISO_GPIO, LSM6DS3_MISO_PIN, PAL_MODE_INPUT_PULLDOWN);

	for (int i = 0;i < 8;i++) {
		uint8_t val = tachion_imu_probe_read_reg(&spi, (uint8_t)reg);
		commands_printf("read_pd[%d] reg 0x%02X = 0x%02X (%d)",
				i, reg, val, val);
		chThdSleepMilliseconds(2);
	}
}

static void terminal_selfcheck(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	int fail = 0;

	commands_printf("Tachion selfcheck: %s", HW_NAME);
#ifdef USE_HARDWARE_SPI
	commands_printf("imu build path    : custom hardware SPI");
	commands_printf("imu driver thread : %s", m_thread_ref ? "running" : "not running");
#else
	commands_printf("imu build path    : standard system SPI driver");
#endif
	commands_printf("current meas limit: %.1f A",
			(double)REDSHIFT_TACHION_CURRENT_MEASUREMENT_LIMIT);
	commands_printf("max abs current   : %.1f A",
			(double)MCCONF_L_MAX_ABS_CURRENT);
	commands_printf("HW_LIM_CURRENT    : " TACHION_STR(HW_LIM_CURRENT));
	commands_printf("HW_LIM_CURRENT_IN : " TACHION_STR(HW_LIM_CURRENT_IN));
	commands_printf("HW_LIM_CURRENT_ABS: " TACHION_STR(HW_LIM_CURRENT_ABS));

	int power_latch_level = palReadPad(POWERON_GPIO, POWERON_PIN);
	fail += tachion_selfcheck_print_bool("power latch",
			power_latch_level == (power_latched ? 1 : 0));
	commands_printf("power latch detail: level=%d latched=%d ready=%d armed=%d hold=%dms",
			power_latch_level,
			power_latched ? 1 : 0,
			powerbut_shutdown_ready ? 1 : 0,
			poweroff_armed ? 1 : 0,
			powerbut_hold_ms);

	bool motorbut_pressed = MOTORBUT_PRESSED();
	fail += tachion_selfcheck_print_bool("motor limit sync",
			motorbut_limit_active == motorbut_pressed);
	commands_printf("buttons           : motor=%d power=%d mot_limit=%d",
			motorbut_pressed ? 1 : 0,
			POWERBUT_PRESSED() ? 1 : 0,
			motorbut_limit_active ? 1 : 0);

	fail += tachion_selfcheck_print_level("gate disable",
			palReadPad(PS_DISABLE_GPIO, PS_DISABLE_PIN), 0);
	commands_printf("fan               : level=%d",
			palReadPad(FAN_GPIO, FAN_PIN));
	commands_printf("leds              : green=%d red=%d",
			palReadPad(LED_GREEN_GPIO, LED_GREEN_PIN),
			palReadPad(LED_RED_GPIO, LED_RED_PIN));
	commands_printf("i2c scaffold      : %s",
			i2c_running ? "running" : "stopped");

	hw_redshift_tachion_disable_flash();
	fail += tachion_selfcheck_print_level("external spi cs",
			palReadPad(HW_SPI_PORT_NSS, HW_SPI_PIN_NSS), 1);
	fail += tachion_selfcheck_print_level("nand cs",
			palReadPad(NAND_CS_GPIO, NAND_CS_PIN), 1);

	spi_bb_state spi;
	int miso_pullup = 0;
	int miso_pulldown = 0;
	tachion_imu_probe_init(&spi, &miso_pullup, &miso_pulldown);
	fail += tachion_selfcheck_print_level("imu cs idle",
			palReadPad(LSM6DS3_NSS_GPIO, LSM6DS3_NSS_PIN), 1);
	fail += tachion_selfcheck_print_bool("imu miso idle",
			miso_pullup == 1 && miso_pulldown == 0);
	commands_printf("imu miso detail   : pullup=%d pulldown=%d",
			miso_pullup, miso_pulldown);

	uint8_t who_pu[3] = {0};
	uint8_t who_pd[3] = {0};
	for (int i = 0;i < 3;i++) {
		who_pu[i] = tachion_imu_probe_read_reg(&spi,
				LSM6DS3_ACC_GYRO_WHO_AM_I_REG);
		chThdSleepMilliseconds(2);
	}

	palSetPadMode(LSM6DS3_MISO_GPIO, LSM6DS3_MISO_PIN,
			PAL_MODE_INPUT_PULLDOWN);
	for (int i = 0;i < 3;i++) {
		who_pd[i] = tachion_imu_probe_read_reg(&spi,
				LSM6DS3_ACC_GYRO_WHO_AM_I_REG);
		chThdSleepMilliseconds(2);
	}

	bool imu_ok = tachion_imu_whoami_valid(who_pu[0]) ||
			tachion_imu_whoami_valid(who_pu[1]) ||
			tachion_imu_whoami_valid(who_pu[2]) ||
			tachion_imu_whoami_valid(who_pd[0]) ||
			tachion_imu_whoami_valid(who_pd[1]) ||
			tachion_imu_whoami_valid(who_pd[2]);
	fail += tachion_selfcheck_print_bool("imu whoami", imu_ok);
	commands_printf("imu whoami pu     : 0x%02X 0x%02X 0x%02X",
			who_pu[0], who_pu[1], who_pu[2]);
	commands_printf("imu whoami pd     : 0x%02X 0x%02X 0x%02X",
			who_pd[0], who_pd[1], who_pd[2]);
	if (!imu_ok) {
		if (who_pu[0] == 0xFF && who_pd[0] == 0x00 &&
				miso_pullup == 1 && miso_pulldown == 0) {
			commands_printf("imu diag          : MISO floating/no response");
		} else if (who_pu[0] == 0xFF && who_pd[0] == 0xFF) {
			commands_printf("imu diag          : MISO held high");
		} else if (who_pu[0] == 0x00 && who_pd[0] == 0x00) {
			commands_printf("imu diag          : MISO held low");
		} else {
			commands_printf("imu diag          : unexpected WHO_AM_I pattern");
		}
	}
	fail += tachion_selfcheck_print_level("imu cs after",
			palReadPad(LSM6DS3_NSS_GPIO, LSM6DS3_NSS_PIN), 1);

	commands_printf("hall              : %d %d %d",
			READ_HALL1(), READ_HALL2(), READ_HALL3());
	commands_printf("adc raw           : vin=%d 12v=%d mos=%d motor=%d",
			ADC_Value[ADC_IND_VIN_SENS],
			ADC_Value[ADC_IND_ACTUAL_12V],
			ADC_Value[ADC_IND_TEMP_MOS],
			ADC_Value[ADC_IND_TEMP_MOTOR]);

	float vin = GET_INPUT_VOLTAGE();
	float actual12v = GET_ACTUAL_12V_VOLTAGE();
	float mos_temp = NTC_TEMP_MOS1();
	fail += tachion_selfcheck_print_bool("vin adc",
			vin > 10.0 && vin < 250.0);
	fail += tachion_selfcheck_print_bool("12v rail",
			actual12v > 6.0 && actual12v < 16.0);
	fail += tachion_selfcheck_print_bool("mos temp",
			mos_temp > -40.0 && mos_temp < 150.0);
	commands_printf("analog values     : vin=%.2fV 12v=%.2fV mos=%.1fC",
			(double)vin, (double)actual12v, (double)mos_temp);
	commands_printf("inj current raw   : a=%d b=%d c=%d",
			HW_GET_INJ_CURR1(),
			HW_GET_INJ_CURR2(),
			HW_GET_INJ_CURR3());
	commands_printf("motor runtime     : rpm=%.1f current=%.2fA",
			(double)mc_interface_get_rpm(),
			(double)mc_interface_get_tot_current_filtered());

	commands_printf("selfcheck result  : %s (%d fail)",
			fail == 0 ? "OK" : "FAIL", fail);
}

static void tachion_imu_probe_init(spi_bb_state *spi,
		int *miso_pullup, int *miso_pulldown) {
	hw_redshift_tachion_disable_flash();
	palSetPad(LSM6DS3_NSS_GPIO, LSM6DS3_NSS_PIN);
	palSetPadMode(LSM6DS3_NSS_GPIO, LSM6DS3_NSS_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	palSetPadMode(LSM6DS3_MISO_GPIO, LSM6DS3_MISO_PIN, PAL_MODE_INPUT_PULLUP);
	chThdSleepMilliseconds(1);
	*miso_pullup = palReadPad(LSM6DS3_MISO_GPIO, LSM6DS3_MISO_PIN);
	palSetPadMode(LSM6DS3_MISO_GPIO, LSM6DS3_MISO_PIN, PAL_MODE_INPUT_PULLDOWN);
	chThdSleepMilliseconds(1);
	*miso_pulldown = palReadPad(LSM6DS3_MISO_GPIO, LSM6DS3_MISO_PIN);

	spi->nss_gpio = LSM6DS3_NSS_GPIO;
	spi->nss_pin = LSM6DS3_NSS_PIN;
	spi->sck_gpio = LSM6DS3_SCK_GPIO;
	spi->sck_pin = LSM6DS3_SCK_PIN;
	spi->mosi_gpio = LSM6DS3_MOSI_GPIO;
	spi->mosi_pin = LSM6DS3_MOSI_PIN;
	spi->miso_gpio = LSM6DS3_MISO_GPIO;
	spi->miso_pin = LSM6DS3_MISO_PIN;
	spi->mutex_init_done = false;

	spi_bb_init(spi);
}

static uint8_t tachion_imu_probe_read_reg(spi_bb_state *spi, uint8_t reg) {
	uint8_t res = 0;

	chMtxLock(&spi->mutex);
	spi_bb_begin(spi);
	spi_bb_exchange_8_mode_3(spi, reg | 0x80);
	spi_bb_delay();
	res = spi_bb_exchange_8_mode_3(spi, 0);
	spi_bb_end(spi);
	chMtxUnlock(&spi->mutex);

	return res;
}

static bool tachion_imu_whoami_valid(uint8_t val) {
	return val == 0x69 || val == 0x6A || val == 0x6C;
}

static int tachion_selfcheck_print_bool(const char *name, bool ok) {
	commands_printf("%-18s: %s", name, ok ? "OK" : "FAIL");
	return ok ? 0 : 1;
}

static int tachion_selfcheck_print_level(const char *name, int level,
		int expected) {
	bool ok = level == expected;
	commands_printf("%-18s: %s level=%d expected=%d",
			name, ok ? "OK" : "FAIL", level, expected);
	return ok ? 0 : 1;
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
