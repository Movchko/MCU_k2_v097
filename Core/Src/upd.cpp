/*
 * upd.cpp
 *
 *  Created on: Apr 1, 2026
 *      Author: 79099
 */
#include "main.h"
#include "backend.h"

#define FLASH_CFG_SECTOR     (31u)
#define BOOT_MAIN_APP_START_ADDR 0x800C000u
#define BOOT_DEF_APP_ADDR  0x8054000
#define BOOT_APP_SIZE_BYTES      (18u * 8u * 1024u)
#define UPDATE_APP_ADDR          (BOOT_MAIN_APP_START_ADDR + BOOT_APP_SIZE_BYTES)
#define UPDATE_APP_SECTORS       18u
#define FLASH_BASE_ADDR          0x08000000u
#define FLASH_BANK_SIZE_BYTES    0x00040000u
#define FLASH_SECTOR_SIZE_BYTES  0x2000u

#define APP_VERSION_U32 2u

static uint8_t  g_upd_started = 0u;
static uint8_t  g_upd_flash_error = 0u;
static uint32_t g_upd_quad_index = 0xFFFFFFFFu;
static uint8_t  g_upd_quad_mask = 0u;
static uint32_t g_upd_quad_buf[4] = {0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu};

static bool UpdateFlushQuad(void)
{
    if (g_upd_quad_index == 0xFFFFFFFFu) {
        return true;
    }
    uint32_t addr = UPDATE_APP_ADDR + (g_upd_quad_index * 16u);
    HAL_StatusTypeDef st = HAL_FLASH_Unlock();
    if (st != HAL_OK) {
        g_upd_flash_error = 1u;
        return false;
    }
#if defined(FLASH_TYPEPROGRAM_QUADWORD_NS)
    uint32_t prog_type = FLASH_TYPEPROGRAM_QUADWORD_NS;
#else
    uint32_t prog_type = FLASH_TYPEPROGRAM_QUADWORD;
#endif
    st = HAL_FLASH_Program(prog_type, addr, reinterpret_cast<uint32_t>(&g_upd_quad_buf[0]));
    (void)HAL_FLASH_Lock();
    if (st != HAL_OK) {
        g_upd_flash_error = 1u;
        return false;
    }
    g_upd_quad_index = 0xFFFFFFFFu;
    g_upd_quad_mask = 0u;
    for (uint8_t i = 0; i < 4u; i++) {
        g_upd_quad_buf[i] = 0xFFFFFFFFu;
    }
    return true;
}

static bool UpdateEraseArea(void)
{
    uint32_t abs_sector = (UPDATE_APP_ADDR - FLASH_BASE_ADDR) / FLASH_SECTOR_SIZE_BYTES;
    uint32_t sectors_left = UPDATE_APP_SECTORS;
    HAL_StatusTypeDef st = HAL_FLASH_Unlock();
    if (st != HAL_OK) {
        return false;
    }
    while ((sectors_left > 0u) && (st == HAL_OK)) {
        FLASH_EraseInitTypeDef erase = {};
        uint32_t sector_err = 0u;
        uint32_t bank = (abs_sector < 32u) ? FLASH_BANK_1 : FLASH_BANK_2;
        uint32_t sector_in_bank = (bank == FLASH_BANK_1) ? abs_sector : (abs_sector - 32u);
        uint32_t sectors_to_end_bank = 32u - sector_in_bank;
        uint32_t chunk = (sectors_left < sectors_to_end_bank) ? sectors_left : sectors_to_end_bank;
#if defined(FLASH_TYPEERASE_SECTORS_NS)
        erase.TypeErase = FLASH_TYPEERASE_SECTORS_NS;
#else
        erase.TypeErase = FLASH_TYPEERASE_SECTORS;
#endif
        erase.Banks = bank;
        erase.Sector = sector_in_bank;
        erase.NbSectors = chunk;
        st = HAL_FLASHEx_Erase(&erase, &sector_err);
        abs_sector += chunk;
        sectors_left -= chunk;
    }
    (void)HAL_FLASH_Lock();
    return (st == HAL_OK);
}



uint8_t SetUpdateWord(uint32_t num, uint32_t word)
{
	const uint32_t max_words = BOOT_APP_SIZE_BYTES / 4u;
    if (num >= max_words) {
        return 0u;
    }
    if (g_upd_flash_error) {
        return 0u;
    }
    if (!g_upd_started) {
        if (!UpdateEraseArea()) {
            g_upd_flash_error = 1u;
            return 0u;
        }
        g_upd_started = 1u;
    }

    uint32_t quad_index = num / 4u;
    uint8_t word_index = static_cast<uint8_t>(num % 4u);
    if ((g_upd_quad_index != 0xFFFFFFFFu) && (g_upd_quad_index != quad_index)) {
        if (!UpdateFlushQuad()) {
            return 0u;
        }
    }
    const uint32_t *quad_ptr = reinterpret_cast<const uint32_t *>(UPDATE_APP_ADDR + quad_index * 16u);
    bool quad_programmed = (quad_ptr[0] != 0xFFFFFFFFu) ||
                           (quad_ptr[1] != 0xFFFFFFFFu) ||
                           (quad_ptr[2] != 0xFFFFFFFFu) ||
                           (quad_ptr[3] != 0xFFFFFFFFu);
    if (quad_programmed) {
        uint32_t cur = quad_ptr[word_index];
        return (cur == word) ? 1u : 0u;
    }
    if (g_upd_quad_index == 0xFFFFFFFFu) {
        g_upd_quad_index = quad_index;
        g_upd_quad_mask = 0u;
        for (uint8_t i = 0; i < 4u; i++) {
            g_upd_quad_buf[i] = 0xFFFFFFFFu;
        }
    }
    if ((g_upd_quad_mask & static_cast<uint8_t>(1u << word_index)) != 0u) {
        return (g_upd_quad_buf[word_index] == word) ? 1u : 0u;
    }
    g_upd_quad_buf[word_index] = word;
    g_upd_quad_mask |= static_cast<uint8_t>(1u << word_index);

    if (g_upd_quad_mask == 0x0Fu) {
        if (!UpdateFlushQuad()) {
            return 0u;
        }
    }
    return 1u;
}

uint8_t GetUpdateWord(uint32_t num, uint32_t *word)
{
    if (word == nullptr) {
        return 0u;
    }
    const uint32_t max_words = BOOT_APP_SIZE_BYTES / 4u;
    if (num >= max_words) {
        return 0u;
    }
    uint32_t quad_index = num / 4u;
    uint8_t word_index = static_cast<uint8_t>(num % 4u);
    if ((g_upd_quad_index == quad_index) &&
        ((g_upd_quad_mask & static_cast<uint8_t>(1u << word_index)) != 0u)) {
        *word = g_upd_quad_buf[word_index];
        return 1u;
    }
    const uint32_t *ptr = reinterpret_cast<const uint32_t *>(UPDATE_APP_ADDR + num * 4u);
    *word = *ptr;
    return 1u;
}

uint8_t FinishUpdateTransmit(void)
{
    if (g_upd_flash_error) {
        return 0u;
    }
    if (g_upd_quad_index != 0xFFFFFFFFu) {
        if (!UpdateFlushQuad()) {
            return 0u;
        }
    }
    NVIC_SystemReset();
    return 1u;
}

uint32_t GetAppVersion(void)
{
    return APP_VERSION_U32;
}
