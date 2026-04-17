/**
 * @file mku_cfg_flash.h
 * @brief Область конфигов MKUCfg во Flash — объявления секции .mku_cfg
 */
#ifndef MCU_MKU_CFG_FLASH_H
#define MCU_MKU_CFG_FLASH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t _mku_cfg_start[];
extern uint8_t _mku_cfg_end[];

#define FLASH_CFG_ADDR       ((uint32_t)_mku_cfg_start)
#define FLASH_CFG_SIZE       ((uint32_t)(_mku_cfg_end - _mku_cfg_start))
#define FLASH_CFG_SIZE_BYTES 0x2000u

#ifdef __cplusplus
}
#endif

#endif /* MCU_MKU_CFG_FLASH_H */

