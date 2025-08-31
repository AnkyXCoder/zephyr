/*
 * Copyright 2025 Ankitmukar Modi
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BQ25601 Datasheet: https://www.ti.com/lit/ds/symlink/bq25601.pdf
 */

#define DT_DRV_COMPAT ti_bq25601

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/charger.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ti_bq25601, CONFIG_CHARGER_LOG_LEVEL);

/*  */
#define BQ25601_REG04                 0x04
#define BQ25601_REG04_VREG_MASK       GENMASK(7, 3)
/* Charge Voltage Configuration
 * Offset: 3.856 V
 * Range: 3.856 V (0b00000) to 4.624 V (0b11000)
 * VREG = 3.856V + (x * 32mV), where x = value from 0b00000 to 0b11000
 * Special Value: (0b01111): 4.352 V
 * Value above 0b11000 (4.624 V) is clamped to register value 0b11000 (4.624 V)
 */
#define BQ25601_REG04_VREG_BASE_MV    3856
#define BQ25601_REG04_VREG_STEP_MV    32
#define BQ25601_REG04_VREG_MAX        0b11000
#define BQ25601_REG04_VREG_SPECIAL    0b01111
#define BQ25601_REG04_VREG_SPECIAL_MV 4352
#define BQ25601_REG04_VREG_MV(x)                                                                   \
	(((x) == BQ25601_REG04_VREG_SPECIAL)                                                       \
		 ? BQ25601_REG04_VREG_SPECIAL_MV                                                   \
		 : BQ25601_REG04_VREG_BASE_MV +                                                    \
			   ((MIN((x), BQ25601_REG04_VREG_MAX)) * BQ25601_REG04_VREG_STEP_MV))
#define BQ25601_REG04_TOPOFF_TIMER_MASK    GENMASK(2, 1)
#define BQ25601_REG04_TOPOFF_TIMER_DISABLE 0b00
#define BQ25601_REG04_TOPOFF_TIMER_15MIN   0b01
#define BQ25601_REG04_TOPOFF_TIMER_30MIN   0b10
#define BQ25601_REG04_TOPOFF_TIMER_45MIN   0b11
#define BQ25601_REG04_VRECHG_MASK          BIT(0)

/* Timer Configurations Register */
#define BQ25601_REG05                       0x05
#define BQ25601_REG05_EN_TERMINATION_MASK   BIT(7)
#define BQ25601_REG05_WATCHDOG_MASK         GENMASK(5, 4)
#define BQ25601_REG05_WATCHDOG_DISABLE      0b00
#define BQ25601_REG05_WATCHDOG_40S          0b01
#define BQ25601_REG05_WATCHDOG_80S          0b10
#define BQ25601_REG05_WATCHDOG_160S         0b11
#define BQ25601_REG05_EN_TIMER_MASK         BIT(3)
#define BQ25601_REG05_CHARGE_TIMER_MASK     BIT(2)
#define BQ25601_REG05_CHARGE_TIMER_5H       0b0
#define BQ25601_REG05_CHARGE_TIMER_10H      0b1
#define BQ25601_REG05_THERM_REGULATION_MASK BIT(1)
#define BQ25601_REG05_JEITA_ISET_MASK       BIT(0)

/* Output Voltage Configuration Register */
#define BQ25601_REG06                0x06
#define BQ25601_REG06_OVP_MASK       GENMASK(7, 6)
#define BQ25601_REG06_OVP_5P5V       0b00
#define BQ25601_REG06_OVP_6P5V       0b01
#define BQ25601_REG06_OVP_10P5V      0b10
#define BQ25601_REG06_OVP_14V        0b11
#define BQ25601_REG06_BOOSTV_MASK    GENMASK(5, 4)
#define BQ25601_REG06_BOOSTV_4P85V   0b00
#define BQ25601_REG06_BOOSTV_5P0V    0b01
#define BQ25601_REG06_BOOSTV_5P15V   0b10
#define BQ25601_REG06_BOOSTV_5P3V    0b11
#define BQ25601_REG06_VINDPM_MASK    GENMASK(3, 0)
/* VINDPM = 3900mV + (x * 100mV), where x = value from 0b0000 to 0b1111 */
#define BQ25601_REG06_VINDPM_BASE_MV 3900
#define BQ25601_REG06_VINDPM_STEP_MV 100
#define BQ25601_REG06_VINDPM_MV(x)                                                                 \
	(BQ25601_REG06_VINDPM_BASE_MV + ((x) * BQ25601_REG06_VINDPM_STEP_MV))

/* Configuration Register */
#define BQ25601_REG07                        0x07
#define BQ25601_REG07_EN_IINDET_MASK         BIT(7)
#define BQ25601_REG07_TMR2X_EN_MASK          BIT(6)
#define BQ25601_REG07_BATFET_DIS_MASK        BIT(5)
#define BQ25601_REG07_JEITA_VSET_MASK        BIT(4)
#define BQ25601_REG07_BATFAT_DLY_EN_MASK     BIT(3)
#define BQ25601_REG07_BATFAT_RST_EN_MASK     BIT(2)
#define BQ25601_REG07_VDPM_BAT_TRACK_MASK    GENMASK(1, 0)
#define BQ25601_REG07_VDPM_BAT_TRACK_DISABLE 0b00
#define BQ25601_REG07_VDPM_BAT_TRACK_200MV   0b01
#define BQ25601_REG07_VDPM_BAT_TRACK_250MV   0b10
#define BQ25601_REG07_VDPM_BAT_TRACK_300MV   0b11

/* Status Register */
#define BQ25601_REG08                    0x08
#define BQ25601_REG08_VBUS_STAT_MASK     GENMASK(7, 5)
#define BQ25601_REG08_VBUS_STAT_NO_INPUT 0b000
#define BQ25601_REG08_VBUS_STAT_USB_HOST 0b001
#define BQ25601_REG08_VBUS_STAT_ADAPTER  0b010
#define BQ25601_REG08_VBUS_STAT_OTG      0b111
#define BQ25601_REG08_CHRG_STAT_MASK     GENMASK(4, 3)
#define BQ25601_REG08_CHRG_STAT_IDLE     0b00
#define BQ25601_REG08_CHRG_STAT_PRECHG   0b01
#define BQ25601_REG08_CHRG_STAT_FASTCHG  0b10
#define BQ25601_REG08_CHRG_STAT_CHGDONE  0b11
#define BQ25601_REG08_PG_STAT_MASK       BIT(2)
#define BQ25601_REG08_THERM_STAT_MASK    BIT(1)
#define BQ25601_REG08_VSYS_STAT_MASK     BIT(0)

/* Fault Detection */
#define BQ25601_REG09                     0x09
#define BQ25601_REG09_WATCHDOG_FAULT_MASK BIT(7)
#define BQ25601_REG09_BOOST_FAULT_MASK    BIT(6)
#define BQ25601_REG09_CHRG_FAULT_MASK     GENMASK(5, 4)
#define BQ25601_REG09_
#define BQ25601_REG09_BAT_FAULT_MASK BIT(3)
#define BQ25601_REG09_NTC_FAULT_MASK GENMASK(2, 0)

#define BQ25601_REG0A                    0x0A
#define BQ25601_REG0A_VBUS_GD_MASK       BIT(7)
#define BQ25601_REG0A_VINDPM_STAT_MASK   BIT(6)
#define BQ25601_REG0A_IINDPM_STAT_MASK   BIT(5)
#define BQ25601_REG0A_TOPOFF_ACTIVE_MASK BIT(3)
#define BQ25601_REG0A_ACOV_STAT_MASK     BIT(2)
#define BQ25601_REG0A_VINDPM_INT_MASK    BIT(1)
#define BQ25601_REG0A_IINDPM_INT_MASK    BIT(0)

/* Manufacture ID */
#define BQ25601_REG0B                     0x0B
#define BQ25601_REG0B_RST_MASK            BIT(0)
#define BQ25601_REG0B_RST_KEEP_REG_VAL    0
#define BQ25601_REG0B_RST_DEFAULT_REG_VAL 1
#define BQ25601_REG0B_PN                  0b0010
