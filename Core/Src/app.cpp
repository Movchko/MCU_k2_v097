#include "app.h"

extern "C" {
#include "backend.h"
}

#include "device_config.h"
#include "device_igniter.hpp"
#include "mku_cfg_flash.h"
#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_flash.h"
#include "stm32h5xx_hal_flash_ex.h"

#include <string.h>

#include "main.h"

/* K2 config stored in linker section .mku_cfg (FLASH_CFG) */
#define FLASH_CFG_SECTOR     (31u)
#define MKU_CFG_HEADER_MAGIC 0x4D4B5543u
#include <string.h>

/* Конфиг MKUCfg */
static MKUCfg g_cfg;
static MKUCfg g_saved_cfg;

/* Виртуальные устройства:
 * DevIndex mapping for backend:
 *   BoardDevicesList[0] = physical MCU_K2
 *   BoardDevicesList[1] = Igniter spichka1 (l_adr=1)
 *   BoardDevicesList[2] = Igniter spichka2 (l_adr=2)
 *   BoardDevicesList[3] = Igniter spichka3 (l_adr=3)
 *
 * Конструктор VDeviceIgniter принимает Num, который напрямую используется в SendMessage(Num,...)
 * => Num должен совпадать с индексом BoardDevicesList. */
static VDeviceIgniter g_igniter1(1);
static VDeviceIgniter g_igniter2(2);
static VDeviceIgniter g_igniter3(3);

/* Планировщик запуска спичек по старту тушения */
static uint32_t g_extinguish_deadline_ms[NUM_DEV_IN_MCU];
static uint8_t  g_extinguish_armed[NUM_DEV_IN_MCU];

/* Кольцевой буфер принятых CAN-пакетов */
#define APP_CAN_RX_RING_SIZE  256
typedef struct {
    uint32_t id;
    uint8_t  data[8];
    uint8_t bus;
} AppCanRxEntry;

static AppCanRxEntry     can_rx_ring[APP_CAN_RX_RING_SIZE];
static volatile uint8_t  can_rx_head = 0;
static volatile uint8_t  can_rx_tail = 0;

/* Флаги активности шин CAN */
volatile uint8_t CAN1_Active = 0;
volatile uint8_t CAN2_Active = 0;
static uint32_t can1_last_rx_tick = 0;
static uint32_t can2_last_rx_tick = 0;

/* -------- Service callbacks for backend fire -------- */
void RcvSetSystemTime(uint8_t *data) { (void)data; }
void RcvStatusFire() {}
void RcvReplyStatusFire() {}

static int8_t App_FindIgniterSlotByMsgId(uint32_t MsgID)
{
    can_ext_id_t id;
    id.ID = MsgID & 0x0FFFFFFFu;

    if ((id.field.d_type & 0x7Fu) != DEVICE_IGNITER_TYPE) {
        return -1;
    }
    if ((id.field.h_adr != g_cfg.UId.devId.h_adr) ||
        ((id.field.zone & 0x7Fu) != (g_cfg.UId.devId.zone & 0x7Fu))) {
        return -1;
    }

    if ((id.field.l_adr & 0x3Fu) == 1u && g_cfg.VDtype[0] == DEVICE_IGNITER_TYPE) {
        return 0;
    }
    if ((id.field.l_adr & 0x3Fu) == 2u && g_cfg.VDtype[1] == DEVICE_IGNITER_TYPE) {
        return 1;
    }
    if ((id.field.l_adr & 0x3Fu) == 3u && g_cfg.VDtype[2] == DEVICE_IGNITER_TYPE) {
        return 2;
    }
    return -1;
}

extern "C" void RcvStartExtinguishment(uint32_t MsgID, uint8_t *MsgData, uint8_t is_mine)
{
    if (is_mine == 0u) {
        return;
    }

    int8_t ign_slot = App_FindIgniterSlotByMsgId(MsgID);
    if (ign_slot < 0) {
        return;
    }

    /* payload backend fire: [0]=cmd, [1]=zone, [2]=zone_delay_s, [3]=module_delay_s */
    uint8_t zd = MsgData[2];
    uint8_t md = MsgData[3];
    uint32_t delay_ms = ((uint32_t)zd + (uint32_t)md) * 1000u;

    g_extinguish_deadline_ms[(uint8_t)ign_slot] = HAL_GetTick() + delay_ms;
    g_extinguish_armed[(uint8_t)ign_slot] = 1u;
    SetReplyStartExtinguishment((uint8_t)(ign_slot + 1)); /* slot0->dev1, slot1->dev2, slot2->dev3 */
}

extern "C" void RcvStopExtinguishment(uint32_t MsgID, uint8_t *MsgData, uint8_t is_mine)
{
    (void)MsgData;
    if (is_mine == 0u) {
        return;
    }

    int8_t ign_slot = App_FindIgniterSlotByMsgId(MsgID);
    if (ign_slot < 0) {
        return;
    }

    g_extinguish_armed[(uint8_t)ign_slot] = 0u;
    SetReplyStopExtinguishment((uint8_t)(ign_slot + 1)); /* slot0->dev1, slot1->dev2, slot2->dev3 */
}

/* callback статуса: отправляем его через CAN по протоколу backend */
static void VDeviceSetStatus(uint8_t DNum, uint8_t Code, const uint8_t *Parameters) {
    uint8_t data[7] = {0};
    for (uint8_t i = 0; i < 7; i++) {
        data[i] = Parameters[i];
    }
    SendMessage(DNum, Code, data, 0, BUS_CAN12);
}

void SetHAdr(uint8_t h_adr) {
    g_cfg.UId.devId.h_adr = h_adr;
    extern uint8_t nDevs;
    extern Device BoardDevicesList[];
    for (uint8_t i = 0; i < nDevs; i++) {
        BoardDevicesList[i].h_adr = g_cfg.UId.devId.h_adr;
    }
    SaveConfig();
}

extern "C" {

/* -------- Config / flash backend glue -------- */
void DefaultConfig(void) {
    uint32_t uid0 = HAL_GetUIDw0();
    uint32_t uid1 = HAL_GetUIDw1();
    uint32_t uid2 = HAL_GetUIDw2();

    memset(&g_cfg, 0, sizeof(g_cfg));

    g_cfg.UId.UId0 = uid0;
    g_cfg.UId.UId1 = uid1;
    g_cfg.UId.UId2 = uid2;
    g_cfg.UId.UId3 = HAL_GetDEVID();
    g_cfg.UId.UId4 = 1;

    g_cfg.UId.devId.zone  = 0;
    g_cfg.UId.devId.l_adr = 0;

    uint8_t hadr = static_cast<uint8_t>(uid0 & 0xFFu);
    if (hadr == 0u) {
        hadr = static_cast<uint8_t>(uid1 & 0xFFu);
        if (hadr == 0u) {
            hadr = 1u;
        }
    }

    g_cfg.UId.devId.h_adr  = hadr;
    g_cfg.UId.devId.d_type = DEVICE_MCU_K2;

    /* VDtype slots:
     * Devices[0] -> igniter1
     * Devices[1] -> igniter2
     * Devices[2] -> igniter3
     */
    g_cfg.VDtype[0] = DEVICE_IGNITER_TYPE;
    g_cfg.VDtype[1] = DEVICE_IGNITER_TYPE;
    g_cfg.VDtype[2] = DEVICE_IGNITER_TYPE;

    DeviceIgniterConfig *ign_cfg1 = reinterpret_cast<DeviceIgniterConfig*>(g_cfg.Devices[0].reserv);
    DeviceIgniterConfig *ign_cfg2 = reinterpret_cast<DeviceIgniterConfig*>(g_cfg.Devices[1].reserv);
    DeviceIgniterConfig *ign_cfg3 = reinterpret_cast<DeviceIgniterConfig*>(g_cfg.Devices[2].reserv);

    memset(ign_cfg1, 0, sizeof(DeviceIgniterConfig));
    memset(ign_cfg2, 0, sizeof(DeviceIgniterConfig));
    memset(ign_cfg3, 0, sizeof(DeviceIgniterConfig));

    ign_cfg1->disable_sc_check     = 1u;
    ign_cfg1->threshold_break_low  = 1000u;
    ign_cfg1->threshold_break_high = 3000u;
    ign_cfg1->burn_retry_count     = 0u;

    ign_cfg2->disable_sc_check     = 1u;
    ign_cfg2->threshold_break_low  = 1000u;
    ign_cfg2->threshold_break_high = 3000u;
    ign_cfg2->burn_retry_count     = 0u;

    ign_cfg3->disable_sc_check     = 1u;
    ign_cfg3->threshold_break_low  = 1000u;
    ign_cfg3->threshold_break_high = 3000u;
    ign_cfg3->burn_retry_count     = 0u;
}

uint32_t GetConfigSize(void) {
    return static_cast<uint32_t>(sizeof(g_cfg));
}

uint32_t GetConfigWord(uint16_t num) {
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

void SetConfigWord(uint16_t num, uint32_t word) {
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

#define MKU_CFG_HEADER_SIZE  8u
#define QUADWORD_SIZE        16u

static bool FlashReadConfig(MKUCfg *out) {
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

static void FlashWriteData(uint8_t *ConfigPtr, uint32_t ConfigSize) {
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

void SaveConfig(void) {
    uint32_t size = GetConfigSize();
    FlashWriteData(reinterpret_cast<uint8_t *>(&g_cfg), size);
    g_saved_cfg = g_cfg;
}

void ResetMCU(void) {
    NVIC_SystemReset();
}

uint32_t GetID(void) {
    uint32_t id0 = HAL_GetUIDw0();
    uint32_t id1 = HAL_GetUIDw1();
    uint32_t id2 = HAL_GetUIDw2();
    return (id0 ^ id1 ^ id2);
}

void MCU_K2CommandCB(uint8_t Command, uint8_t *Parameters) {
    if (Command == 20) {
        g_cfg.UId.devId.zone = Parameters[0];
        SaveConfig();
    }
}

void CommandCB(uint8_t Dev, uint8_t Command, uint8_t *Parameters) {
    switch (Dev) {
    case 0:
        MCU_K2CommandCB(Command, Parameters);
        break;
    case 1:
        g_igniter1.CommandCB(Command, Parameters);
        break;
    case 2:
        g_igniter2.CommandCB(Command, Parameters);
        break;
    case 3:
        g_igniter3.CommandCB(Command, Parameters);
        break;
    default:
        break;
    }
}

void ListenerCommandCB(uint32_t MsgID, uint8_t *MsgData) {
    (void)MsgID;
    (void)MsgData;
}

void App_CanOnRx(uint8_t bus) {
    uint32_t now = HAL_GetTick();
    if (bus == 1u) {
        CAN1_Active = 1u;
        can1_last_rx_tick = now;
    } else if (bus == 2u) {
        CAN2_Active = 1u;
        can2_last_rx_tick = now;
    }
}

void App_CanRxPush(uint32_t id, const uint8_t *data, uint8_t bus) {
    uint8_t next = static_cast<uint8_t>(can_rx_head + 1u);
    if (next >= APP_CAN_RX_RING_SIZE) {
        next = 0u;
    }
    if (next == can_rx_tail) {
        can_rx_tail++;
        if (can_rx_tail >= APP_CAN_RX_RING_SIZE) {
            can_rx_tail = 0u;
        }
    }
    can_rx_ring[can_rx_head].id = id;
    can_rx_ring[can_rx_head].bus = bus;
    memcpy(can_rx_ring[can_rx_head].data, data, 8u);
    can_rx_head = next;
}

void App_CanProcess(void) {
    while (can_rx_head != can_rx_tail) {
        AppCanRxEntry *e = &can_rx_ring[can_rx_tail];
        can_rx_tail++;
        if (can_rx_tail >= APP_CAN_RX_RING_SIZE) {
            can_rx_tail = 0u;
        }
        ProtocolParse(e->id, e->data, e->bus);
    }
}

/* Timer tick (1ms) */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

void App_Init(void) {
    extern Device BoardDevicesList[];
    extern uint8_t nDevs;

    if (!FlashReadConfig(&g_cfg)) {
        DefaultConfig();
        SaveConfig();
    }

    g_saved_cfg = g_cfg;
    SetConfigPtr(reinterpret_cast<uint8_t *>(&g_saved_cfg),
                 reinterpret_cast<uint8_t *>(&g_cfg));

    /* Init virtual igniters from config slots */
    g_igniter1.DeviceInit(&g_cfg.Devices[0]);
    g_igniter1.VDeviceSetStatus = VDeviceSetStatus;
    g_igniter1.VDeviceSaveCfg   = SaveConfig;
    g_igniter1.Init();

    g_igniter2.DeviceInit(&g_cfg.Devices[1]);
    g_igniter2.VDeviceSetStatus = VDeviceSetStatus;
    g_igniter2.VDeviceSaveCfg   = SaveConfig;
    g_igniter2.Init();

    g_igniter3.DeviceInit(&g_cfg.Devices[2]);
    g_igniter3.VDeviceSetStatus = VDeviceSetStatus;
    g_igniter3.VDeviceSaveCfg   = SaveConfig;
    g_igniter3.Init();

    /* BoardDevicesList: physical + 3 virtual devices */
    nDevs = 1;
    BoardDevicesList[0].zone  = g_cfg.UId.devId.zone;
    BoardDevicesList[0].h_adr = g_cfg.UId.devId.h_adr;
    BoardDevicesList[0].l_adr = g_cfg.UId.devId.l_adr;
    BoardDevicesList[0].d_type = DEVICE_MCU_K2;

    if (nDevs < MAX_DEVS) {
        BoardDevicesList[nDevs].zone  = g_cfg.UId.devId.zone;
        BoardDevicesList[nDevs].h_adr = g_cfg.UId.devId.h_adr;
        BoardDevicesList[nDevs].l_adr = 1; /* spichka1 */
        BoardDevicesList[nDevs].d_type = DEVICE_IGNITER_TYPE;
        nDevs++;
    }
    if (nDevs < MAX_DEVS) {
        BoardDevicesList[nDevs].zone  = g_cfg.UId.devId.zone;
        BoardDevicesList[nDevs].h_adr = g_cfg.UId.devId.h_adr;
        BoardDevicesList[nDevs].l_adr = 2; /* spichka2 */
        BoardDevicesList[nDevs].d_type = DEVICE_IGNITER_TYPE;
        nDevs++;
    }
    if (nDevs < MAX_DEVS) {
        BoardDevicesList[nDevs].zone  = g_cfg.UId.devId.zone;
        BoardDevicesList[nDevs].h_adr = g_cfg.UId.devId.h_adr;
        BoardDevicesList[nDevs].l_adr = 3; /* spichka3 */
        BoardDevicesList[nDevs].d_type = DEVICE_IGNITER_TYPE;
        nDevs++;
    }

    extern bool isListener;
    isListener = true;
}

void App_Timer1ms(void) {
    static uint16_t led_cnt = 0u;
    static uint16_t status_cnt = 0u;
    static uint16_t u24_calc_cnt = 0u;

    uint32_t now = HAL_GetTick();

    /* Heartbeat and status to PPKY */
    if (status_cnt < 1000u) {
        status_cnt++;
    } else {
        status_cnt = 0u;
        uint8_t status_data[7] = {0};

        status_data[0] = (uint8_t)(now & 0xFFu);
        status_data[1] = (uint8_t)((now >> 8) & 0xFFu);
        status_data[2] = (uint8_t)((now >> 16) & 0xFFu);
        status_data[3] = (uint8_t)((now >> 24) & 0xFFu);
        status_data[4] = (uint8_t)(CAN1_Active | (CAN2_Active << 1));


        /* U24: 0.1V code как у ППКУ (power=(code/10)). */
        {
            const uint32_t VREF_MV = 3300u;
            const uint32_t ADC_MAX = 4095u;
            const uint32_t DIV_K   = 11u;

            uint32_t raw_u24 = ADC_GetU24Filtered();
            uint32_t v_adc_mv = (raw_u24 * VREF_MV) / ADC_MAX;
            uint32_t u24_mv   = v_adc_mv * DIV_K;
            uint32_t code_01v = u24_mv / 100u;
            if (code_01v > 255u) {
                code_01v = 255u;
            }
            status_data[5] = (uint8_t)code_01v;
        }
        status_data[6] = 0u;
        SendMessage(0, 0, status_data, SEND_NOW, BUS_CAN12);
    }

    /* LED toggle (debug) */
    if (led_cnt < 1000u) {
        led_cnt++;
    } else {
        led_cnt = 0u;
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }

    /* Планировщик запуска спичек по команде StartExtinguishment */
    for (uint8_t i = 0; i < NUM_DEV_IN_MCU; i++) {
        if (!g_extinguish_armed[i]) {
            continue;
        }
        if ((int32_t)(now - g_extinguish_deadline_ms[i]) >= 0) {
            g_extinguish_armed[i] = 0u;

            uint8_t params[7] = {0,0,0,0,0,0,0};
            /* dispatch по виртуальному слоту VDtype[i] */
            if (i == 0u) {
                g_igniter1.CommandCB(10, params);
            } else if (i == 1u) {
                g_igniter2.CommandCB(10, params);
            } else if (i == 2u) {
                g_igniter3.CommandCB(10, params);
            }
        }
    }

    /* Апдейты активности CAN */
    App_UpdateCanActivity();

    /* Обновление ADC-состояния линии, только когда ШИМ выключен */
    if (!g_igniter1.IsPwmActive()) {
        uint16_t raw = ADC_GetIgniter1Filtered();
        uint16_t mv = (uint16_t)((uint32_t)raw * 3300u / 4095u);
        g_igniter1.UpdateLineFromAdcMv(mv);
    }
    if (!g_igniter2.IsPwmActive()) {
        uint16_t raw = ADC_GetIgniter2Filtered();
        uint16_t mv = (uint16_t)((uint32_t)raw * 3300u / 4095u);
        g_igniter2.UpdateLineFromAdcMv(mv);
    }
    if (!g_igniter3.IsPwmActive()) {
        uint16_t raw = ADC_GetIgniter3Filtered();
        uint16_t mv = (uint16_t)((uint32_t)raw * 3300u / 4095u);
        g_igniter3.UpdateLineFromAdcMv(mv);
    }

    /* Timer и процессинг устройств */
    g_igniter1.Timer1ms();
    g_igniter2.Timer1ms();
    g_igniter3.Timer1ms();

    /* CAN processing */
    App_CanProcess();
    BackendProcess();

    /* Применяем ШИМ на соответствующие каналы:
     *  TIM3 CH4 -> igniter1 (спичка1)
     *  TIM2 CH2 -> igniter2 (спичка2)
     *  TIM4 CH4 -> igniter3 (спичка3)
     */
    uint16_t pwm1 = g_igniter1.GetPwm();
    if (pwm1 > 0u) {
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm1);
    } else {
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
    }

    uint16_t pwm2 = g_igniter2.GetPwm();
    if (pwm2 > 0u) {
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm2);
    } else {
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
    }

    uint16_t pwm3 = g_igniter3.GetPwm();
    if (pwm3 > 0u) {
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm3);
    } else {
        HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
    }
}

void App_UpdateCanActivity(void) {
    uint32_t now = HAL_GetTick();

    if (can1_last_rx_tick != 0u) {
        if ((now - can1_last_rx_tick) >= 3000u) {
            CAN1_Active = 0u;
            can1_last_rx_tick = 0u;
        }
    }

    if (can2_last_rx_tick != 0u) {
        if ((now - can2_last_rx_tick) >= 3000u) {
            CAN2_Active = 0u;
            can2_last_rx_tick = 0u;
        }
    }
}

} /* extern "C" */

