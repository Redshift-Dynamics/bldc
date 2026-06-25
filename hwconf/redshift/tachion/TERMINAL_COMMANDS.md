# Redshift Tachion Terminal Commands

This file lists terminal commands registered by the Redshift Tachion target
code in `hwconf/redshift/tachion`. The firmware also has standard global
terminal commands registered outside this target.

## test_button

Usage:

```text
test_button
```

Availability:

Always registered by `hw_init_gpio()`.

Purpose:

Prints Redshift Tachion button and power-latch diagnostics:

- `sample`: current `HW_SAMPLE_SHUTDOWN()` result
- `motorbut`: carry/motor button state
- `powerbut`: power button state
- `poweron`: power latch GPIO state
- `latched`: internal power latch state
- `power hold`: accumulated power-button hold time
- `mot limit`: carry/motor button current-limit state
- `rpm`: current motor RPM estimate
- `current`: total filtered current
- `actual12v`: measured 12 V rail

## tachion_imu_probe

Usage:

```text
tachion_imu_probe
tachion_imu_probe 0x0F
tachion_imu_probe 15
```

Availability:

Always registered by `hw_init_gpio()`.

Purpose:

Directly probes the LSM6DS3 SPI pins using software SPI mode 3, independent of
the app IMU type setting. By default it reads `WHO_AM_I` register `0x0F`.

Pins:

- IMU CS: `PC13`
- IMU SCK: `PC10`
- IMU MISO: `PC11`
- IMU MOSI: `PC12`
- NAND CS forced high: `PC15`

Output:

- `CS imu`: IMU chip-select level after setup, expected `1` before reads
- `CS nand`: NAND chip-select level after setup, expected `1`
- `miso pu/pd`: MISO level with internal pull-up and pull-down
- `read_pu[n]`: repeated reads with MISO configured as input pull-up
- `read_pd[n]`: repeated reads with MISO configured as input pull-down

Expected `WHO_AM_I` values:

- `0x69`
- `0x6A`
- `0x6C`

Useful interpretation:

- Repeated `read_pu = 0xFF` and `read_pd = 0x00`: MISO is floating during
  the transaction; the sensor is not driving it.
- Repeated `read_pu = 0xFF` and `read_pd = 0xFF`: MISO is held high during
  the transaction.
- Repeated `read_pu = 0x00` and `read_pd = 0x00`: MISO is held low during
  the transaction.
- `miso pu/pd:1/0`: MISO follows internal pulls when idle.
- `miso pu/pd:0/0`: MISO is held low.
- `miso pu/pd:1/1`: MISO is held high.

## tachion_selfcheck

Usage:

```text
tachion_selfcheck
```

Availability:

Always registered by `hw_init_gpio()`.

Purpose:

Prints one status report for the Redshift Tachion target-specific hardware:

- selected target name and IMU build path
- current measurement and firmware current-limit settings
- power latch, shutdown state, power button, and carry/motor button current limit
- gate-disable, fan, LED, external SPI CS, NAND CS, and IMU CS levels
- IMU MISO idle pull-up/pull-down behavior
- IMU `WHO_AM_I` reads with MISO pull-up and pull-down
- hall input levels, ADC raw values, decoded VIN/12 V/MOS temperature, injected
  current samples, RPM, and filtered motor current

Checks marked `FAIL` are included in the final `selfcheck result` count. Valid
IMU `WHO_AM_I` values are `0x69`, `0x6A`, and `0x6C`.

IMU diagnostics use the same direct software-SPI probe as `tachion_imu_probe`.
This intentionally bypasses app configuration and can report whether MISO is
floating, held high, held low, or returning a valid ID.

## lsm_read_reg

Usage:

```text
lsm_read_reg 15
```

Availability:

Registered by the Redshift Tachion custom LSM6DS3 hardware-SPI driver after
successful IMU initialization.

This command is only part of the Redshift Tachion custom hardware-SPI path:

- `USE_HARDWARE_SPI` must be enabled for the variant.
- The LSM6DS3 custom driver must pass initialization.

Current variant note:

- `redshift_tachion_t18`: `USE_HARDWARE_SPI` is enabled.
- `redshift_tachion_t36`: `USE_HARDWARE_SPI` is enabled.
- On both variants this command appears only if the custom hardware-SPI IMU
  driver initializes successfully.

Output:

Prints the selected register value in binary as `Reg: XXXXXXXX`.
