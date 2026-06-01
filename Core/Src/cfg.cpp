#include "app.h"
extern "C" {
#include "backend.h"
}
#include "device_config.h"
#include "mku_cfg_flash.h"
#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_flash.h"
#include "stm32h5xx_hal_flash_ex.h"

#define MKU_CFG_HEADER_SIZE  8u
#define QUADWORD_SIZE        16u
#define FLASH_CFG_SECTOR     (31u)
#define MKU_CFG_HEADER_MAGIC 0x4D4B5543u

extern MKUCfg g_cfg;
extern MKUCfg g_saved_cfg;

bool FlashReadConfig(MKUCfg *out)
{
    if (out == nullptr) {
        return false;
    }
    const uint32_t *p = reinterpret_cast<const uint32_t *>(FLASH_CFG_ADDR);
    if (p[0] != MKU_CFG_HEADER_MAGIC) {
        return false;
    }
    uint32_t sz = p[1];
    if (sz != sizeof(MKUCfg) || sz > FLASH_CFG_SIZE - MKU_CFG_HEADER_SIZE) {
        return false;
    }
    memcpy(out, p + 2, sz);
    return true;
}

void FlashWriteData(uint8_t *ConfigPtr, uint32_t ConfigSize)
{
    if (ConfigPtr == nullptr || ConfigSize != sizeof(MKUCfg) ||
        ConfigSize > FLASH_CFG_SIZE - MKU_CFG_HEADER_SIZE) {
        return;
    }

    __attribute__((aligned(16))) uint8_t buf[FLASH_CFG_SIZE_BYTES];
    uint32_t *hdr = reinterpret_cast<uint32_t *>(buf);
    hdr[0] = MKU_CFG_HEADER_MAGIC;
    hdr[1] = ConfigSize;
    memcpy(buf + MKU_CFG_HEADER_SIZE, ConfigPtr, ConfigSize);

    uint32_t total = MKU_CFG_HEADER_SIZE + ConfigSize;
    uint32_t n_quad = (total + QUADWORD_SIZE - 1u) / QUADWORD_SIZE;

    FLASH_EraseInitTypeDef erase;
    uint32_t sector_err = 0u;
#if defined(FLASH_TYPEERASE_SECTORS_NS)
    erase.TypeErase = FLASH_TYPEERASE_SECTORS_NS;
#else
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
#endif
    erase.Banks = FLASH_BANK_2;
    erase.Sector = FLASH_CFG_SECTOR;
    erase.NbSectors = 1;

    HAL_StatusTypeDef st = HAL_FLASH_Unlock();
    if (st != HAL_OK) {
        return;
    }
    st = HAL_FLASHEx_Erase(&erase, &sector_err);
    if (st != HAL_OK) {
        HAL_FLASH_Lock();
        return;
    }
#if defined(FLASH_TYPEPROGRAM_QUADWORD_NS)
    uint32_t prog_type = FLASH_TYPEPROGRAM_QUADWORD_NS;
#else
    uint32_t prog_type = FLASH_TYPEPROGRAM_QUADWORD;
#endif
    for (uint32_t i = 0u; i < n_quad && st == HAL_OK; i++) {
        uint32_t addr = FLASH_CFG_ADDR + i * QUADWORD_SIZE;
        st = HAL_FLASH_Program(prog_type, addr,
                               reinterpret_cast<uint32_t>(buf + i * QUADWORD_SIZE));
    }
    HAL_FLASH_Lock();
}

void SaveConfig(void)
{
    uint32_t size = GetConfigSize();
    (void)size;
    FlashWriteData(reinterpret_cast<uint8_t *>(&g_cfg), size);
    g_saved_cfg = g_cfg;
}

uint32_t GetConfigSize(void)
{
    return static_cast<uint32_t>(sizeof(g_cfg));
}

uint32_t GetConfigWord(uint16_t num)
{
    uint32_t byte_index = static_cast<uint32_t>(num) * 4u;
    uint32_t cfg_size   = GetConfigSize();
    if (byte_index + 4u > cfg_size) {
        return 0u;
    }
    uint8_t *p = reinterpret_cast<uint8_t *>(&g_cfg);
    uint32_t word = 0u;
    word |= (static_cast<uint32_t>(p[byte_index + 0]) << 24);
    word |= (static_cast<uint32_t>(p[byte_index + 1]) << 16);
    word |= (static_cast<uint32_t>(p[byte_index + 2]) << 8);
    word |= (static_cast<uint32_t>(p[byte_index + 3]) << 0);
    return word;
}

void SetConfigWord(uint16_t num, uint32_t word)
{
    uint32_t byte_index = static_cast<uint32_t>(num) * 4u;
    uint32_t cfg_size   = GetConfigSize();
    if (byte_index + 4u > cfg_size) {
        return;
    }
    uint8_t *p = reinterpret_cast<uint8_t *>(&g_cfg);
    p[byte_index + 0] = static_cast<uint8_t>((word >> 24) & 0xFFu);
    p[byte_index + 1] = static_cast<uint8_t>((word >> 16) & 0xFFu);
    p[byte_index + 2] = static_cast<uint8_t>((word >> 8)  & 0xFFu);
    p[byte_index + 3] = static_cast<uint8_t>((word >> 0)  & 0xFFu);
}
