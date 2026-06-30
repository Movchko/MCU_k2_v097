#include "app.h"

extern "C" {
#include "backend.h"
}

#include "device_config.h"
#include "device_igniter.hpp"
#include "app_igniter_launch.hpp"
#include "mku_cfg_flash.h"
#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_flash.h"
#include "stm32h5xx_hal_flash_ex.h"

#include <string.h>

#include "main.h"
#include "mku_cfg_flash.h"

extern DTS_HandleTypeDef hdts;

/* Конфиг MKUCfg */
MKUCfg g_cfg;
MKUCfg g_saved_cfg;

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

static void App_ArmIgniterSlot(uint8_t ign_slot, uint8_t zd, uint8_t md)
{
    uint32_t delay_ms = ((uint32_t)zd + (uint32_t)md) * 1000u;

    g_extinguish_deadline_ms[ign_slot] = HAL_GetTick() + delay_ms;
    g_extinguish_armed[ign_slot] = 1u;
    SetReplyStartExtinguishment((uint8_t)(ign_slot + 1u));
}

static void App_ArmIgniterSlotWithLaunch(uint8_t ign_slot, uint8_t launch_type, uint8_t cmd_zd, uint8_t cmd_md)
{
    uint8_t zd = 0u;
    uint8_t md = 0u;

    Backend_ResolveIgniterStartDelays(launch_type, cmd_zd, cmd_md, ign_slot,
                                      g_cfg.zone_delay, g_cfg.module_delay, NUM_DEV_IN_MCU,
                                      &zd, &md);
    App_ArmIgniterSlot(ign_slot, zd, md);
}

static void App_ArmAllIgnitersWithLaunch(uint8_t launch_type, uint8_t cmd_zd, uint8_t cmd_md)
{
    for (uint8_t i = 0u; i < NUM_DEV_IN_MCU; i++) {
        if (g_cfg.VDtype[i] == DEVICE_IGNITER_TYPE) {
            App_ArmIgniterSlotWithLaunch(i, launch_type, cmd_zd, cmd_md);
        }
    }
}

extern "C" void RcvStartExtinguishment(uint32_t MsgID, uint8_t *MsgData, uint8_t is_mine)
{
    if (is_mine == 0u) {
        return;
    }

    uint8_t zd = MsgData[2];
    uint8_t md = MsgData[3];
    uint8_t launch_type = MsgData[4];

    if (Backend_IsIgniterBroadcastId(MsgID)) {
        if (!Backend_StartExtinguishZoneMatches(MsgID, MsgData[1], g_cfg.UId.devId.zone)) {
            return;
        }
        App_ArmAllIgnitersWithLaunch(launch_type, zd, md);
        return;
    }

    int8_t ign_slot = App_FindIgniterSlotByMsgId(MsgID);
    if (ign_slot < 0) {
        return;
    }

    App_ArmIgniterSlotWithLaunch((uint8_t)ign_slot, launch_type, zd, md);
}

static uint8_t App_IsIgniterSlot(uint8_t slot, void *ctx)
{
    (void)ctx;
    if (slot >= NUM_DEV_IN_MCU) {
        return 0u;
    }
    return (g_cfg.VDtype[slot] == DEVICE_IGNITER_TYPE) ? 1u : 0u;
}

static uint8_t App_IsIgniterBurnRunning(uint8_t slot, void *ctx)
{
    (void)ctx;
    if (slot == 0u) {
        return g_igniter1.IsBurnRunning() ? 1u : 0u;
    }
    if (slot == 1u) {
        return g_igniter2.IsBurnRunning() ? 1u : 0u;
    }
    if (slot == 2u) {
        return g_igniter3.IsBurnRunning() ? 1u : 0u;
    }
    return 0u;
}

static void App_FireIgniterSlot(uint8_t slot, void *ctx)
{
    (void)ctx;
    g_extinguish_armed[slot] = 0u;
    uint8_t params[7] = {0, 0, 0, 0, 0, 0, 0};
    if (slot == 0u) {
        g_igniter1.CommandCB(10, params);
    } else if (slot == 1u) {
        g_igniter2.CommandCB(10, params);
    } else if (slot == 2u) {
        g_igniter3.CommandCB(10, params);
    }
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

    g_cfg.zone_delay = 5u;
    g_cfg.module_delay[0] = 0u;
    g_cfg.module_delay[1] = 2u;
    g_cfg.module_delay[2] = 4u;

    DeviceIgniterConfig *ign_cfg1 = reinterpret_cast<DeviceIgniterConfig*>(g_cfg.Devices[0].reserv);
    DeviceIgniterConfig *ign_cfg2 = reinterpret_cast<DeviceIgniterConfig*>(g_cfg.Devices[1].reserv);
    DeviceIgniterConfig *ign_cfg3 = reinterpret_cast<DeviceIgniterConfig*>(g_cfg.Devices[2].reserv);

    memset(ign_cfg1, 0, sizeof(DeviceIgniterConfig));
    memset(ign_cfg2, 0, sizeof(DeviceIgniterConfig));
    memset(ign_cfg3, 0, sizeof(DeviceIgniterConfig));

    ign_cfg1->disable_sc_check     = 0u;
    ign_cfg1->threshold_break_low  = 100;
    ign_cfg1->threshold_break_high = 1000;
    ign_cfg1->burn_retry_count     = 0u;

    ign_cfg2->disable_sc_check     = 0u;
    ign_cfg2->threshold_break_low  = 100;
    ign_cfg2->threshold_break_high = 1000;
    ign_cfg2->burn_retry_count     = 0u;

    ign_cfg3->disable_sc_check     = 0u;
    ign_cfg3->threshold_break_low  = 100;
    ign_cfg3->threshold_break_high = 1000;
    ign_cfg3->burn_retry_count     = 0u;
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

        int32_t temperature;
        if(HAL_DTS_GetTemperature(&hdts, &temperature)!= HAL_OK)
        {
            /* DTS GetTemperature Error */
        }

        if(temperature > 128) temperature = 128;
        if(temperature < -128) temperature = -128;

        uint8_t temp = (uint8_t)temperature;

        uint8_t status_data[7] = {0};

        status_data[0] = (uint8_t)(now / 1000u);
        status_data[1] = temp;
        status_data[2] = 0u;
        status_data[3] = 0u;
        status_data[4] = (uint8_t)(CAN1_Active | (CAN2_Active << 1));

        const uint32_t VREF_MV = 3300u;
        const uint32_t ADC_MAX = 4095u;
        uint32_t raw_u24 = ADC_GetU24Filtered();
        uint32_t v_adc_mv = (raw_u24 * VREF_MV) / ADC_MAX;
        uint32_t u24_mv = v_adc_mv;
        uint32_t code_1v = (u24_mv + 500) / 1000u;
        if (code_1v > 255u) code_1v = 255u;
        status_data[5] = (uint8_t)code_1v;
        status_data[6] = App_GetCanStateMask();
        SendMessage(0, 0, status_data, SEND_NOW, BUS_CAN12);

        uint8_t pos_data[7] = {0u, 0u, 0u, 0u, 0u, 0u, 0u};
        SendMessage(0, ServiceCmd_PositionDevice, pos_data, SEND_NOW, BUS_CAN12);
    }

    /* LED toggle (debug) */
    if (led_cnt < 1000u) {
        led_cnt++;
    } else {
        led_cnt = 0u;
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
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


    /* CAN processing (приём 142 — постановка в очередь) */
    BackendProcess();

    /* Планировщик: одна спичка за раз, следующая — после завершения цикла предыдущей */
    AppIgniter_RunSequentialScheduler(NUM_DEV_IN_MCU, now, g_extinguish_deadline_ms, g_extinguish_armed,
                                      nullptr, App_IsIgniterSlot, App_IsIgniterBurnRunning,
                                      App_FireIgniterSlot, nullptr);

    g_igniter1.Timer1ms();
    g_igniter2.Timer1ms();
    g_igniter3.Timer1ms();

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

} /* extern "C" */

