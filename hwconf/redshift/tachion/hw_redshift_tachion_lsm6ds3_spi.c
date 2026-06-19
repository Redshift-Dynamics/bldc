/*
	Copyright 2026

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.
*/

#include "hw.h"
#include "lsm6ds3.h"
#include "terminal.h"
#include "commands.h"
#include "utils_math.h"

#include <stdio.h>
#include <string.h>

static thread_t *m_thread_ref = NULL;
static SPIConfig m_spicfg;
static int m_rate_hz = 1000;
static IMU_FILTER m_filter = IMU_FILTER_LOW;
static void (*m_read_callback)(float *accel, float *gyro, float *mag) = 0;

static bool read_reg(uint8_t reg, uint8_t *res);
static bool write_reg(uint8_t reg, uint8_t value);
static bool read_gyro_accel(uint8_t *res);
static void terminal_read_reg(int argc, const char **argv);
static THD_FUNCTION(lsm6ds3_thread, arg);

void hw_redshift_tachion_lsm6ds3_spi_stop(void);

void hw_redshift_tachion_lsm6ds3_spi_set_rate_hz(int hz) {
	m_rate_hz = hz;
}

void hw_redshift_tachion_lsm6ds3_spi_set_filter(IMU_FILTER filter) {
	m_filter = filter;
}

void hw_redshift_tachion_lsm6ds3_spi_set_read_callback(
		void(*func)(float *accel, float *gyro, float *mag)) {
	m_read_callback = func;
}

void hw_redshift_tachion_lsm6ds3_spi_bb_init(spi_bb_state *spi_state) {
	palSetPad(spi_state->nss_gpio, spi_state->nss_pin);
	palSetPadMode(spi_state->nss_gpio, spi_state->nss_pin,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(spi_state->sck_gpio, spi_state->sck_pin,
			PAL_MODE_ALTERNATE(LSM6DS3_SPI_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(spi_state->mosi_gpio, spi_state->mosi_pin,
			PAL_MODE_ALTERNATE(LSM6DS3_SPI_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(spi_state->miso_gpio, spi_state->miso_pin,
			PAL_MODE_ALTERNATE(LSM6DS3_SPI_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	m_spicfg.end_cb = NULL;
	m_spicfg.ssport = spi_state->nss_gpio;
	m_spicfg.sspad = spi_state->nss_pin;
	m_spicfg.cr1 = LSM6DS3_SPI_CR1;

	spiStart(&LSM6DS3_SPI_DEV, &m_spicfg);
}

void hw_redshift_tachion_lsm6ds3_spi_init(i2c_bb_state *i2c_state,
		spi_bb_state *spi_state, stkalign_t *work_area,
		size_t work_area_size) {
	(void)i2c_state;

	if (!spi_state) {
		return;
	}

	hw_redshift_tachion_lsm6ds3_spi_stop();
	hw_redshift_tachion_lsm6ds3_spi_bb_init(spi_state);

	uint8_t rxb[1] = {0};
	bool res = read_reg(LSM6DS3_ACC_GYRO_WHO_AM_I_REG, rxb);
	if (!res || (rxb[0] != 0x69 && rxb[0] != 0x6A && rxb[0] != 0x6C)) {
		commands_printf("LSM6DS3 SPI WHO_AM_I failed (rx: %d)", rxb[0]);
		return;
	}

	bool is_trc = false;
	if (rxb[0] == 0x6A) {
		is_trc = true;
	}

	uint8_t regv = LSM6DS3_ACC_GYRO_FS_XL_16g;
	regv |= LSM6DS3_ACC_GYRO_ODR_XL_6660Hz;

#define LSM6DS3TRC_BW0_XL 0x1
#define LSM6DS3TRC_LPF1_BW_SEL 0x2
	if (is_trc) {
		regv |= LSM6DS3TRC_BW0_XL;
	} else if (m_rate_hz >= 208 && m_filter >= IMU_FILTER_MEDIUM) {
		int scaled_rate = m_filter == IMU_FILTER_HIGH ?
				m_rate_hz / 2 : m_rate_hz;
		if (scaled_rate <= 208) {
			regv |= LSM6DS3_ACC_GYRO_BW_XL_50Hz;
		} else if (scaled_rate <= 416) {
			regv |= LSM6DS3_ACC_GYRO_BW_XL_100Hz;
		} else if (scaled_rate <= 833) {
			regv |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
		}
	}

	res = write_reg(LSM6DS3_ACC_GYRO_CTRL1_XL, regv);
	if (!res) {
		commands_printf("LSM6DS3 Accel Config FAILED");
		return;
	}

	if (is_trc) {
#define LSM6DS3TRC_LPF2_XL_EN 0x80
#define LSM6DS3TRC_HPCF_XL_ODR50 0x00
#define LSM6DS3TRC_HPCF_XL_ODR100 0x20
		regv = 0;
		if (m_filter == IMU_FILTER_MEDIUM) {
			regv |= LSM6DS3TRC_LPF2_XL_EN | LSM6DS3TRC_HPCF_XL_ODR50;
		} else if (m_filter == IMU_FILTER_HIGH) {
			regv |= LSM6DS3TRC_LPF2_XL_EN | LSM6DS3TRC_HPCF_XL_ODR100;
		}

		res = write_reg(LSM6DS3_ACC_GYRO_CTRL8_XL, regv);
		if (!res) {
			commands_printf("LSM6DS3 Accel Filter Config FAILED");
			return;
		}
	}

	regv = LSM6DS3_ACC_GYRO_FS_G_2000dps;
	if (is_trc) {
		regv |= LSM6DS3TRC_ACC_GYRO_ODR_G_6660Hz;
	} else {
		if (m_rate_hz <= 13) {
			regv |= LSM6DS3_ACC_GYRO_ODR_G_13Hz;
		} else if (m_rate_hz <= 26) {
			regv |= LSM6DS3_ACC_GYRO_ODR_G_26Hz;
		} else if (m_rate_hz <= 52) {
			regv |= LSM6DS3_ACC_GYRO_ODR_G_52Hz;
		} else if (m_rate_hz <= 104) {
			regv |= LSM6DS3_ACC_GYRO_ODR_G_104Hz;
		} else if (m_rate_hz <= 208) {
			regv |= LSM6DS3_ACC_GYRO_ODR_G_208Hz;
		} else if (m_rate_hz <= 416) {
			regv |= LSM6DS3_ACC_GYRO_ODR_G_416Hz;
		} else if (m_rate_hz <= 833) {
			regv |= LSM6DS3_ACC_GYRO_ODR_G_833Hz;
		} else {
			regv |= LSM6DS3_ACC_GYRO_ODR_G_1660Hz;
		}
	}

	res = write_reg(LSM6DS3_ACC_GYRO_CTRL2_G, regv);
	if (!res) {
		commands_printf("LSM6DS3 Gyro Config FAILED");
		return;
	}

	if (is_trc) {
#define LSM6DS3TRC_FTYPE_L 0x00
#define LSM6DS3TRC_FTYPE_M 0x01
#define LSM6DS3TRC_FTYPE_H 0x10
		regv = 0;
		if (m_filter == IMU_FILTER_LOW) {
			regv |= LSM6DS3TRC_FTYPE_L;
		} else if (m_filter == IMU_FILTER_MEDIUM) {
			regv |= LSM6DS3TRC_FTYPE_M;
		} else if (m_filter == IMU_FILTER_HIGH) {
			regv |= LSM6DS3TRC_FTYPE_H;
		}

		res = write_reg(LSM6DS3_ACC_GYRO_CTRL6_G, regv);
		if (!res) {
			commands_printf("LSM6DS3 Gyro Filter FAILED");
			return;
		}
	}

	regv = 0;
	if (is_trc) {
		regv = LSM6DS3_ACC_GYRO_LPF1_SEL_G_ENABLED;
	} else if (m_rate_hz >= 208 && m_filter >= IMU_FILTER_MEDIUM) {
		regv = LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED;
	}

	res = write_reg(LSM6DS3_ACC_GYRO_CTRL4_C, regv);
	if (!res) {
		commands_printf("LSM6DS3 Misc Filter Config FAILED");
		return;
	}

	regv = LSM6DS3_ACC_GYRO_BDU_BLOCK_UPDATE |
			LSM6DS3_ACC_GYRO_IF_INC_ENABLED;
	res = write_reg(LSM6DS3_ACC_GYRO_CTRL3_C, regv);
	if (!res) {
		commands_printf("LSM6DS3 BDU Config FAILED");
		return;
	}

	terminal_register_command_callback(
			"lsm_read_reg",
			"Read register of the Redshift Tachion LSM6DS3",
			"[reg]",
			terminal_read_reg);

	m_thread_ref = chThdCreateStatic(work_area, work_area_size,
			NORMALPRIO, lsm6ds3_thread, NULL);
}

void hw_redshift_tachion_lsm6ds3_spi_stop(void) {
	if (m_thread_ref != NULL) {
		chThdTerminate(m_thread_ref);
		chThdWait(m_thread_ref);
	}

	m_thread_ref = NULL;
	terminal_unregister_callback(terminal_read_reg);
}

static bool read_reg(uint8_t reg, uint8_t *res) {
	uint8_t txb[2] = {reg | 0x80, 0};
	uint8_t rxb[2] = {0, 0};

	spiAcquireBus(&LSM6DS3_SPI_DEV);
	spiStart(&LSM6DS3_SPI_DEV, &m_spicfg);
	spiSelect(&LSM6DS3_SPI_DEV);
	spiExchange(&LSM6DS3_SPI_DEV, sizeof(txb), txb, rxb);
	spiUnselect(&LSM6DS3_SPI_DEV);
	spiReleaseBus(&LSM6DS3_SPI_DEV);

	*res = rxb[1];
	return true;
}

static bool write_reg(uint8_t reg, uint8_t value) {
	uint8_t txb[2] = {reg & 0x7F, value};

	spiAcquireBus(&LSM6DS3_SPI_DEV);
	spiStart(&LSM6DS3_SPI_DEV, &m_spicfg);
	spiSelect(&LSM6DS3_SPI_DEV);
	spiSend(&LSM6DS3_SPI_DEV, sizeof(txb), txb);
	spiUnselect(&LSM6DS3_SPI_DEV);
	spiReleaseBus(&LSM6DS3_SPI_DEV);

	return true;
}

static bool read_gyro_accel(uint8_t *res) {
	uint8_t txb[13] = {LSM6DS3_ACC_GYRO_OUTX_L_G | 0x80};
	uint8_t rxb[13] = {0};

	spiAcquireBus(&LSM6DS3_SPI_DEV);
	spiStart(&LSM6DS3_SPI_DEV, &m_spicfg);
	spiSelect(&LSM6DS3_SPI_DEV);
	spiExchange(&LSM6DS3_SPI_DEV, sizeof(txb), txb, rxb);
	spiUnselect(&LSM6DS3_SPI_DEV);
	spiReleaseBus(&LSM6DS3_SPI_DEV);

	memcpy(res, rxb + 1, 12);
	return true;
}

static void terminal_read_reg(int argc, const char **argv) {
	if (argc == 2) {
		int reg = -1;
		sscanf(argv[1], "%d", &reg);

		if (reg >= 0) {
			uint8_t res = 0;
			bool ok = read_reg(reg, &res);

			if (ok) {
				char bl[9];
				utils_byte_to_binary(res & 0xFF, bl);
				commands_printf("Reg: %s", bl);
			} else {
				commands_printf("Read failed\n");
			}
		} else {
			commands_printf("Invalid argument(s)\n");
		}
	} else {
		commands_printf("This command requires one argument\n");
	}
}

static THD_FUNCTION(lsm6ds3_thread, arg) {
	(void)arg;
	chRegSetThreadName("LSM6DS3");

	const systime_t interval = US2ST(1000000 / m_rate_hz) - 1;

	while (!chThdShouldTerminateX()) {
		systime_t start_time = chVTGetSystemTimeX();

		uint8_t rxb[12];
		bool res = read_gyro_accel(rxb);

		if (res) {
			float gx = (float)((int16_t)((uint16_t)rxb[1] << 8) + rxb[0]) *
					4.375 * (2000 / 125) / 1000;
			float gy = (float)((int16_t)((uint16_t)rxb[3] << 8) + rxb[2]) *
					4.375 * (2000 / 125) / 1000;
			float gz = (float)((int16_t)((uint16_t)rxb[5] << 8) + rxb[4]) *
					4.375 * (2000 / 125) / 1000;
			float ax = (float)((int16_t)((uint16_t)rxb[7] << 8) + rxb[6]) *
					0.061 * (16 >> 1) / 1000;
			float ay = (float)((int16_t)((uint16_t)rxb[9] << 8) + rxb[8]) *
					0.061 * (16 >> 1) / 1000;
			float az = (float)((int16_t)((uint16_t)rxb[11] << 8) +
					rxb[10]) * 0.061 * (16 >> 1) / 1000;

			if (m_read_callback) {
				float tmp_accel[3] = {ax, ay, az};
				float tmp_gyro[3] = {gx, gy, gz};
				float tmp_mag[3] = {1, 2, 3};
				m_read_callback(tmp_accel, tmp_gyro, tmp_mag);
			}
		} else {
			chThdSleep(1);
			continue;
		}

		systime_t sleep_ticks = 1;
		systime_t remaining_sleep_time = start_time + interval -
				chVTGetSystemTimeX();
		if (remaining_sleep_time > 0 && remaining_sleep_time <= interval) {
			sleep_ticks = remaining_sleep_time;
		}

		chThdSleep(sleep_ticks);
	}
}
