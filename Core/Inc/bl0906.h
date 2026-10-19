/**
  ******************************************************************************
  * @file           : bl0906.h
  * @brief          : Driver for Shanghai Belling BL0906 6-Channel Energy Meter
  *                   Based on official ESPHome component & Belling Datasheet.
  ******************************************************************************
  */

#ifndef __BL0906_H
#define __BL0906_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define BL0906_NUM_CHANNELS           6

/* BL0906 UART Commands */
#define BL0906_CMD_READ               0x35
#define BL0906_CMD_WRITE              0xCA

/* Register Addresses (from official ESPHome constants.h) */
#define BL0906_REG_I_1_RMS            0x0D
#define BL0906_REG_I_2_RMS            0x0E
#define BL0906_REG_I_3_RMS            0x0F
#define BL0906_REG_I_4_RMS            0x10
#define BL0906_REG_I_5_RMS            0x13
#define BL0906_REG_I_6_RMS            0x14

#define BL0906_REG_V_RMS              0x16

#define BL0906_REG_WATT_1             0x23
#define BL0906_REG_WATT_2             0x24
#define BL0906_REG_WATT_3             0x25
#define BL0906_REG_WATT_4             0x26
#define BL0906_REG_WATT_5             0x29
#define BL0906_REG_WATT_6             0x2A
#define BL0906_REG_WATT_SUM           0x2C

#define BL0906_REG_CF_1_CNT           0x30
#define BL0906_REG_CF_2_CNT           0x31
#define BL0906_REG_CF_3_CNT           0x32
#define BL0906_REG_CF_4_CNT           0x33
#define BL0906_REG_CF_5_CNT           0x36
#define BL0906_REG_CF_6_CNT           0x37
#define BL0906_REG_CF_SUM_CNT         0x39

#define BL0906_REG_FREQUENCY          0x4E
#define BL0906_REG_TEMPERATURE        0x5E

#define BL0906_REG_USR_WRPROT         0x9E
#define BL0906_REG_SOFT_RESET         0x9F

/* Conversion Multipliers (ESPHome official references for Athom EM6 / Belling) */
/* V_RMS: 1.097 * (5 * 20k) / (13162 * 100k) = 1.097 / 13162 */
#define BL0906_UREF                   (0.000083345996f)

/* I_RMS: 1.097 / (12875 * 5.1) */
#define BL0906_IREF                   (0.0000167066f)

/* WATT (Active Power) */
#define BL0906_PREF                   (0.0058388f)
#define BL0906_WATT_REF               (16.0f * BL0906_PREF)

/* Energy (kWh) */
#define BL0906_EREF                   (0.0003125f)
#define BL0906_CF_REF                 (16.0f * BL0906_EREF)

/* Line Frequency (10 MHz / cycle_count) */
#define BL0906_FREF                   (10000000.0f)

/**
  * @brief Data structure holding parsed measurements from BL0906
  */
typedef struct {
    float voltage;                          /* Tegangan Grid AC (V) */
    float frequency;                        /* Frekuensi Listrik (Hz) */
    float temperature;                      /* Suhu Internal IC (°C) */
    float current[BL0906_NUM_CHANNELS];     /* Arus per Channel 1..6 (A) */
    float power[BL0906_NUM_CHANNELS];       /* Daya Aktif per Channel 1..6 (W) */
    float energy[BL0906_NUM_CHANNELS];      /* Energi per Channel 1..6 (kWh) */
    float total_power;                      /* Total Daya 6 Channel (W) */
    float total_energy;                     /* Total Energi (kWh) */

    /* Raw Register Values */
    uint32_t raw_voltage;
    uint32_t raw_current[BL0906_NUM_CHANNELS];
    int32_t  raw_power[BL0906_NUM_CHANNELS];
    int32_t  raw_total_power;
    uint32_t raw_energy[BL0906_NUM_CHANNELS];
    uint32_t raw_total_energy;
    uint32_t raw_frequency;
    int32_t  raw_temperature;

    /* Status & Debug */
    bool is_valid;                          /* true jika pembacaan terakhir sukses */
    uint32_t read_success_count;            /* jumlah siklus pembacaan sukses */
    uint32_t read_fail_count;               /* jumlah pembacaan gagal/timeout/checksum error */
} BL0906_Data_t;

/* Driver Functions (LL Driver) */
void BL0906_Init(USART_TypeDef *USARTx);
bool BL0906_ReadRegister(uint8_t reg_addr, uint32_t *raw_out);
bool BL0906_ReadAll(BL0906_Data_t *data);
void BL0906_ResetEnergy(void);

#ifdef __cplusplus
}
#endif

#endif /* __BL0906_H */
