/**
  ******************************************************************************
  * @file           : bl0906.c
  * @brief          : Driver implementation for Shanghai Belling BL0906
  *                   Energy Metering IC using STM32 Low-Layer (LL) driver.
  *                   Based on official ESPHome component logic & Belling App Note.
  ******************************************************************************
  */

#include "bl0906.h"
#include <string.h>

/* Pointer ke instance USART/UART hardware (misal UART5) */
static USART_TypeDef *s_bl0906_uart = NULL;

/* Tabel alamat register Current 1 ~ 6 */
static const uint8_t s_reg_current[BL0906_NUM_CHANNELS] = {
    BL0906_REG_I_1_RMS, /* CH1: 0x0D */
    BL0906_REG_I_2_RMS, /* CH2: 0x0E */
    BL0906_REG_I_3_RMS, /* CH3: 0x0F */
    BL0906_REG_I_4_RMS, /* CH4: 0x10 */
    BL0906_REG_I_5_RMS, /* CH5: 0x13 */
    BL0906_REG_I_6_RMS  /* CH6: 0x14 */
};

/* Tabel alamat register Active Power 1 ~ 6 */
static const uint8_t s_reg_power[BL0906_NUM_CHANNELS] = {
    BL0906_REG_WATT_1,  /* CH1: 0x23 */
    BL0906_REG_WATT_2,  /* CH2: 0x24 */
    BL0906_REG_WATT_3,  /* CH3: 0x25 */
    BL0906_REG_WATT_4,  /* CH4: 0x26 */
    BL0906_REG_WATT_5,  /* CH5: 0x29 */
    BL0906_REG_WATT_6   /* CH6: 0x2A */
};

/* Tabel alamat register Energy Pulse Count 1 ~ 6 */
static const uint8_t s_reg_energy[BL0906_NUM_CHANNELS] = {
    BL0906_REG_CF_1_CNT, /* CH1: 0x30 */
    BL0906_REG_CF_2_CNT, /* CH2: 0x31 */
    BL0906_REG_CF_3_CNT, /* CH3: 0x32 */
    BL0906_REG_CF_4_CNT, /* CH4: 0x33 */
    BL0906_REG_CF_5_CNT, /* CH5: 0x36 */
    BL0906_REG_CF_6_CNT  /* CH6: 0x37 */
};

/**
  * @brief  Helper sign-extension 24-bit ke 32-bit bertanda (signed)
  */
static inline int32_t to_signed24(uint32_t val)
{
    if (val & 0x00800000UL)
    {
        return (int32_t)(val | 0xFF000000UL);
    }
    return (int32_t)val;
}

/**
  * @brief  Fungsi checksum resmi BL0906 (ESPHome standard):
  *         Checksum = ((Address + Data_L + Data_M + Data_H) & 0xFF) ^ 0xFF
  */
static inline uint8_t BL0906_CalcChecksum(uint8_t addr, uint8_t l, uint8_t m, uint8_t h)
{
    return (uint8_t)((addr + l + m + h) ^ 0xFF);
}

/**
  * @brief  Bersihkan data sampah dan flag error di hardware UART LL
  */
static void BL0906_FlushRx(void)
{
    if (s_bl0906_uart == NULL) return;

    LL_USART_ClearFlag_ORE(s_bl0906_uart);
    LL_USART_ClearFlag_FE(s_bl0906_uart);
    LL_USART_ClearFlag_NE(s_bl0906_uart);
    LL_USART_ClearFlag_PE(s_bl0906_uart);

    while (LL_USART_IsActiveFlag_RXNE_RXFNE(s_bl0906_uart))
    {
        (void)LL_USART_ReceiveData8(s_bl0906_uart);
    }
}

/**
  * @brief  Kirim 1 byte via LL
  */
static bool BL0906_SendByte(uint8_t byte)
{
    uint32_t timeout = 200000;
    while (!LL_USART_IsActiveFlag_TXE_TXFNF(s_bl0906_uart))
    {
        if (--timeout == 0) return false;
    }
    LL_USART_TransmitData8(s_bl0906_uart, byte);
    return true;
}

/**
  * @brief  Kirim array byte via LL dan tunggu bit terakhir selesai keluar pin (TC = 1)
  */
static bool BL0906_SendArray(const uint8_t *data, uint16_t len)
{
    LL_USART_ClearFlag_TC(s_bl0906_uart);

    for (uint16_t i = 0; i < len; i++)
    {
        if (!BL0906_SendByte(data[i])) return false;
    }

    uint32_t timeout = 200000;
    while (!LL_USART_IsActiveFlag_TC(s_bl0906_uart))
    {
        if (--timeout == 0) return false;
    }
    return true;
}

/**
  * @brief  Menerima array byte via LL dengan timeout berbasis g_ms_tick
  */
static bool BL0906_ReceiveArray(uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    uint16_t count = 0;
    uint32_t start_tick = g_ms_tick;

    while (count < len)
    {
        if (LL_USART_IsActiveFlag_ORE(s_bl0906_uart))
        {
            LL_USART_ClearFlag_ORE(s_bl0906_uart);
        }
        if (LL_USART_IsActiveFlag_FE(s_bl0906_uart))
        {
            LL_USART_ClearFlag_FE(s_bl0906_uart);
        }
        if (LL_USART_IsActiveFlag_RXNE_RXFNE(s_bl0906_uart))
        {
            buf[count++] = LL_USART_ReceiveData8(s_bl0906_uart);
        }

        if ((g_ms_tick - start_tick) >= timeout_ms)
        {
            return false;
        }
    }
    return true;
}

/**
  * @brief  Inisialisasi driver BL0906 pada UART LL
  */
void BL0906_Init(USART_TypeDef *USARTx)
{
    s_bl0906_uart = USARTx;

    /* Buka proteksi penulisan register BL0906 (User Write Operation Enable) */
    static const uint8_t wr_enable_cmd[6] = { BL0906_CMD_WRITE, BL0906_REG_USR_WRPROT, 0x55, 0x55, 0x00, 0xB7 };
    BL0906_FlushRx();
    BL0906_SendArray(wr_enable_cmd, sizeof(wr_enable_cmd));
    LL_mDelay(20);
}

/**
  * @brief  Membaca 1 buah register 24-bit dari BL0906 dengan validasi checksum
  * @param  reg_addr Alamat register (misal BL0906_REG_V_RMS)
  * @param  raw_out  Pointer untuk menyimpan hasil raw 24-bit
  * @retval true jika berhasil dan checksum valid, false jika gagal
  */
bool BL0906_ReadRegister(uint8_t reg_addr, uint32_t *raw_out)
{
    if (s_bl0906_uart == NULL || raw_out == NULL) return false;

    /* 1. Bersihkan sisa data lama di buffer UART */
    BL0906_FlushRx();

    /* 2. Kirim perintah Read: 0x35 <reg_addr> */
    uint8_t tx_cmd[2] = { BL0906_CMD_READ, reg_addr };
    if (!BL0906_SendArray(tx_cmd, 2))
    {
        return false;
    }

    /* 3. BL0906 membalas 4 byte: [Data_L, Data_M, Data_H, Checksum] */
    uint8_t rx_buf[4] = {0};
    if (!BL0906_ReceiveArray(rx_buf, 4, 100))
    {
        return false;
    }

    /* 4. Validasi Checksum */
    uint8_t expected_chk = BL0906_CalcChecksum(reg_addr, rx_buf[0], rx_buf[1], rx_buf[2]);
    if (expected_chk != rx_buf[3])
    {
        return false; /* Data korup / checksum tidak cocok */
    }

    /* 5. Susun nilai 24-bit (Data_L + Data_M + Data_H) */
    *raw_out = ((uint32_t)rx_buf[2] << 16) | ((uint32_t)rx_buf[1] << 8) | (uint32_t)rx_buf[0];

    /* Jeda singkat antar register agar hardware BL0906 stabil (sesuai ESPHome note) */
    LL_mDelay(10);
    return true;
}

/**
  * @brief  Membaca seluruh parameter (Tegangan, 6 Arus, 6 Daya, Total Daya, Energi, Frekuensi, Suhu)
  * @param  data Pointer ke struct BL0906_Data_t
  * @retval true jika pembacaan parameter utama berhasil
  */
bool BL0906_ReadAll(BL0906_Data_t *data)
{
    if (data == NULL) return false;

    bool any_success = false;
    uint32_t raw_val = 0;

    /* 1. Baca Tegangan Grid RMS (0x16) */
    if (BL0906_ReadRegister(BL0906_REG_V_RMS, &raw_val))
    {
        data->raw_voltage = raw_val;
        data->voltage = (float)raw_val * BL0906_UREF;
        any_success = true;
    }

    /* 2. Baca Frekuensi Listrik (0x4E) */
    if (BL0906_ReadRegister(BL0906_REG_FREQUENCY, &raw_val))
    {
        data->raw_frequency = raw_val;
        if (raw_val > 0)
        {
            data->frequency = BL0906_FREF / (float)raw_val;
        }
        else
        {
            data->frequency = 0.0f;
        }
        any_success = true;
    }

    /* 3. Baca Suhu Internal Chip (0x5E) */
    if (BL0906_ReadRegister(BL0906_REG_TEMPERATURE, &raw_val))
    {
        int32_t s24 = to_signed24(raw_val);
        data->raw_temperature = s24;
        /* Formula resmi ESPHome: value = ((s24 - 64) * 12.5 / 59) - 40 */
        data->temperature = (((float)s24 - 64.0f) * 12.5f / 59.0f) - 40.0f;
        any_success = true;
    }

    /* 4. Baca Arus Channel 1 ~ 6 */
    for (int i = 0; i < BL0906_NUM_CHANNELS; i++)
    {
        if (BL0906_ReadRegister(s_reg_current[i], &raw_val))
        {
            data->raw_current[i] = raw_val;
            data->current[i] = (float)raw_val * BL0906_IREF;
            any_success = true;
        }
    }

    /* 5. Baca Daya Aktif Channel 1 ~ 6 (Signed 24-bit) */
    for (int i = 0; i < BL0906_NUM_CHANNELS; i++)
    {
        if (BL0906_ReadRegister(s_reg_power[i], &raw_val))
        {
            int32_t s24 = to_signed24(raw_val);
            data->raw_power[i] = s24;
            data->power[i] = (float)s24 * BL0906_PREF;
            any_success = true;
        }
    }

    /* 6. Baca Total Daya 6 Channel (0x2C) */
    if (BL0906_ReadRegister(BL0906_REG_WATT_SUM, &raw_val))
    {
        int32_t s24 = to_signed24(raw_val);
        data->raw_total_power = s24;
        data->total_power = (float)s24 * BL0906_WATT_REF;
        any_success = true;
    }

    /* 7. Baca Energi Channel 1 ~ 6 */
    for (int i = 0; i < BL0906_NUM_CHANNELS; i++)
    {
        if (BL0906_ReadRegister(s_reg_energy[i], &raw_val))
        {
            data->raw_energy[i] = raw_val;
            data->energy[i] = (float)raw_val * BL0906_EREF;
            any_success = true;
        }
    }

    /* 8. Baca Total Energi (0x39) */
    if (BL0906_ReadRegister(BL0906_REG_CF_SUM_CNT, &raw_val))
    {
        data->raw_total_energy = raw_val;
        data->total_energy = (float)raw_val * BL0906_CF_REF;
        any_success = true;
    }

    /* Perbarui status pembacaan */
    data->is_valid = any_success;
    if (any_success)
    {
        data->read_success_count++;
    }
    else
    {
        data->read_fail_count++;
    }

    return any_success;
}

/**
  * @brief  Reset akumulasi energi BL0906 ke 0 via LL
  */
void BL0906_ResetEnergy(void)
{
    if (s_bl0906_uart == NULL) return;

    /* Perintah Soft Reset dari ESPHome constants: {0xCA, 0x9F, 0x5A, 0x5A, 0x5A, 0x52} */
    static const uint8_t reset_cmd[6] = { BL0906_CMD_WRITE, BL0906_REG_SOFT_RESET, 0x5A, 0x5A, 0x5A, 0x52 };
    BL0906_SendArray(reset_cmd, sizeof(reset_cmd));
    LL_mDelay(50);

    /* Buka kembali proteksi penulisan */
    static const uint8_t wr_enable_cmd[6] = { BL0906_CMD_WRITE, BL0906_REG_USR_WRPROT, 0x55, 0x55, 0x00, 0xB7 };
    BL0906_SendArray(wr_enable_cmd, sizeof(wr_enable_cmd));
    LL_mDelay(20);
}
