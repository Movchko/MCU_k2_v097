#include "app.h"

extern "C" {
#include "backend.h"
}

#include "main.h"

#define APP_CAN_RX_RING_SIZE  256
#define CAN_WATCHDOG_PERIOD_MS 3000u
#define CAN_TX_STALL_RECOVERY_MS 3000u
typedef struct {
    uint32_t id;
    uint8_t  data[8];
    uint8_t  bus;
} AppCanRxEntry;
static AppCanRxEntry can_rx_ring[APP_CAN_RX_RING_SIZE];
static volatile uint8_t can_rx_head = 0;
static volatile uint8_t can_rx_tail = 0;

volatile uint8_t CAN1_Active = 0;
volatile uint8_t CAN2_Active = 0;
static uint32_t can1_last_rx_tick = 0;
static uint32_t can2_last_rx_tick = 0;
static uint32_t can_state_last_update_ms = 0;

typedef enum CANState {
    CAN_STATE_ACTIVE   = 0,
	CAN_STATE_SHORT  = 1,
	CAN_STATE_BREAK = 2
} CANStateKind;

volatile uint8_t CAN1_State = CAN_STATE_ACTIVE;
volatile uint8_t CAN2_State = CAN_STATE_ACTIVE;

typedef struct {
  uint32_t id;
  uint8_t  data[8];
} CanTxEntry;

#define CAN_TX_RING_SIZE  256
static CanTxEntry can1_tx_ring[CAN_TX_RING_SIZE];
static volatile uint8_t can1_tx_head = 0;
static volatile uint8_t can1_tx_tail = 0;
static CanTxEntry can2_tx_ring[CAN_TX_RING_SIZE];
static volatile uint8_t can2_tx_head = 0;
static volatile uint8_t can2_tx_tail = 0;
static uint32_t can1_tx_stall_since_ms = 0;
static uint32_t can2_tx_stall_since_ms = 0;
static uint32_t can_watchdog_last_ms = 0;

/* Диагностические счётчики для отладки проблем шины/очередей. */
static uint32_t g_can_tx_drop_count[2] = {0u, 0u};
static uint32_t g_can_busoff_recover_count[2] = {0u, 0u};
static uint32_t g_can_tx_stall_recover_count[2] = {0u, 0u};

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

void FDCAN_StartAll(void)
{
  FDCAN_FilterTypeDef sFilter;

  sFilter.IdType = FDCAN_EXTENDED_ID;
  sFilter.FilterIndex = 0;
  sFilter.FilterType = FDCAN_FILTER_MASK;
  sFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilter.FilterID1 = 0x00000000u;
  sFilter.FilterID2 = 0x00000000u;

  HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilter);
  HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilter);

  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_Start(&hfdcan2);

  HAL_FDCAN_ActivateNotification(&hfdcan1,
                                 FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                 FDCAN_IT_BUS_OFF |
                                 FDCAN_IT_ERROR_WARNING |
                                 FDCAN_IT_ERROR_PASSIVE,
                                 0);

  HAL_FDCAN_ActivateNotification(&hfdcan2,
                                 FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                 FDCAN_IT_BUS_OFF |
                                 FDCAN_IT_ERROR_WARNING |
                                 FDCAN_IT_ERROR_PASSIVE,
                                 0);
}

static void CanTxEnqueueOne(CanTxEntry *ring, volatile uint8_t *head, volatile uint8_t *tail,
                            uint32_t id, const uint8_t *data, uint8_t bus_idx)
{
  uint8_t next = (uint8_t)(*head + 1u);
  if (next >= CAN_TX_RING_SIZE) {
    next = 0u;
  }
  if (next == *tail) {
    if (bus_idx < 2u) {
      g_can_tx_drop_count[bus_idx]++;
    }
    (*tail)++;
    if (*tail >= CAN_TX_RING_SIZE) {
      *tail = 0u;
    }
  }

  ring[*head].id = id;
  for (uint8_t i = 0; i < 8u; i++) {
    ring[*head].data[i] = data[i];
  }
  *head = next;
}

static void CanTxEnqueue(uint32_t id, const uint8_t *data, uint8_t busMask)
{
  if ((busMask & BUS_CAN0) != 0u) {
    CanTxEnqueueOne(can1_tx_ring, &can1_tx_head, &can1_tx_tail, id, data, 0u);
  }
  if ((busMask & BUS_CAN1) != 0u) {
    CanTxEnqueueOne(can2_tx_ring, &can2_tx_head, &can2_tx_tail, id, data, 1u);
  }
}

static void App_CanTxProcessBus(FDCAN_HandleTypeDef *hfdcan,
                                CanTxEntry *ring,
                                volatile uint8_t *head,
                                volatile uint8_t *tail,
                                uint8_t bus_idx,
                                uint32_t *stall_since_ms)
{
  uint32_t now = HAL_GetTick();
  while (*head != *tail) {
    CanTxEntry *e = &ring[*tail];
    FDCAN_TxHeaderTypeDef txHeader;

    txHeader.Identifier = e->id;
    txHeader.IdType = FDCAN_EXTENDED_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = FDCAN_DLC_BYTES_8;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0U) {
      if (*stall_since_ms == 0u) {
        *stall_since_ms = now;
      }
      break;
    }
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, e->data) != HAL_OK) {
      if (*stall_since_ms == 0u) {
        *stall_since_ms = now;
      }
      break;
    }

    (void)bus_idx;
    *stall_since_ms = 0u;

    (*tail)++;
    if (*tail >= CAN_TX_RING_SIZE) {
      *tail = 0u;
    }
  }
}

void App_CanTxProcess(void)
{
  App_CanTxProcessBus(&hfdcan1, can1_tx_ring, &can1_tx_head, &can1_tx_tail, 0u, &can1_tx_stall_since_ms);
  App_CanTxProcessBus(&hfdcan2, can2_tx_ring, &can2_tx_head, &can2_tx_tail, 1u, &can2_tx_stall_since_ms);
}

void CANSendData(uint8_t *Buf)
{
  uint32_t id = (*(uint32_t *)Buf);
  uint8_t bus = Buf[4 + 8];
  CanTxEnqueue(id, &Buf[4], bus);
}

FDCAN_ProtocolStatusTypeDef curprotocolStatus1 = {};
FDCAN_ProtocolStatusTypeDef curprotocolStatus2 = {};

static uint8_t DecodeCanStateFromLec(uint32_t lec)
{
  if ((lec & 0x1u) == 0u) {
    return (uint8_t)CAN_STATE_ACTIVE;
  }
  return ((lec & 0x2u) != 0u) ? (uint8_t)CAN_STATE_BREAK : (uint8_t)CAN_STATE_SHORT;
}

static void App_UpdateCanLineState(void)
{
  uint32_t now = HAL_GetTick();
  if ((now - can_state_last_update_ms) < 1000u) {
    return;
  }
  can_state_last_update_ms = now;

  if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &curprotocolStatus1) == HAL_OK) {
    CAN1_State = DecodeCanStateFromLec((uint32_t)curprotocolStatus1.LastErrorCode);
  }
  if (HAL_FDCAN_GetProtocolStatus(&hfdcan2, &curprotocolStatus2) == HAL_OK) {
    CAN2_State = DecodeCanStateFromLec((uint32_t)curprotocolStatus2.LastErrorCode);
  }
}

static void App_CanRecoverBus(FDCAN_HandleTypeDef *hfdcan, uint8_t bus_idx)
{
  uint16_t try_cnt = 0xFFFFu;
  SET_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
  while (((hfdcan->Instance->CCCR & FDCAN_CCCR_INIT) == 0U) && (try_cnt--)) {}
  try_cnt = 0xFFFFu;
  CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
  while (((hfdcan->Instance->CCCR & FDCAN_CCCR_INIT) != 0U) && (try_cnt--)) {}
  if (bus_idx < 2u) {
    g_can_busoff_recover_count[bus_idx]++;
  }
}

static void App_CanWatchdog(void)
{
  uint32_t now = HAL_GetTick();
  if ((now - can_watchdog_last_ms) < CAN_WATCHDOG_PERIOD_MS) {
    return;
  }
  can_watchdog_last_ms = now;

  FDCAN_ProtocolStatusTypeDef st = {};
  if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &st) == HAL_OK && st.BusOff) {
    App_CanRecoverBus(&hfdcan1, 0u);
    can1_tx_stall_since_ms = 0u;
  }
  if (HAL_FDCAN_GetProtocolStatus(&hfdcan2, &st) == HAL_OK && st.BusOff) {
    App_CanRecoverBus(&hfdcan2, 1u);
    can2_tx_stall_since_ms = 0u;
  }

  if (can1_tx_head != can1_tx_tail && can1_tx_stall_since_ms != 0u &&
      (now - can1_tx_stall_since_ms) >= CAN_TX_STALL_RECOVERY_MS) {
    App_CanRecoverBus(&hfdcan1, 0u);
    g_can_tx_stall_recover_count[0]++;
    can1_tx_stall_since_ms = 0u;
  }
  if (can2_tx_head != can2_tx_tail && can2_tx_stall_since_ms != 0u &&
      (now - can2_tx_stall_since_ms) >= CAN_TX_STALL_RECOVERY_MS) {
    App_CanRecoverBus(&hfdcan2, 1u);
    g_can_tx_stall_recover_count[1]++;
    can2_tx_stall_since_ms = 0u;
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) {
    return;
  }

  FDCAN_RxHeaderTypeDef rxHeader;
  uint8_t data[8];

  uint8_t src_bus;
  uint8_t dst_bus_mask;
  if (hfdcan == &hfdcan1) {
    src_bus = BUS_CAN0;
    dst_bus_mask = BUS_CAN1;
  } else {
    src_bus = BUS_CAN1;
    dst_bus_mask = BUS_CAN0;
  }

  App_CanOnRx(src_bus);

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, data) != HAL_OK) {
      break;
    }
    if (rxHeader.IdType != FDCAN_EXTENDED_ID) {
      continue;
    }

    if (GetRetranslate() == 0) {
      uint8_t tx_data[8];
      memcpy(tx_data, data, sizeof(tx_data));
      if (tx_data[0] == ServiceCmd_PositionDevice && tx_data[1] < 0xFFu) {
        tx_data[1]++;
      }
      CanTxEnqueue(rxHeader.Identifier, tx_data, dst_bus_mask);
    }
    App_CanRxPush(rxHeader.Identifier, data, src_bus);
  }

}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
  if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != RESET) {
    FDCAN_ProtocolStatusTypeDef protocolStatus = {};
    HAL_FDCAN_GetProtocolStatus(hfdcan, &protocolStatus);
    if (protocolStatus.BusOff) {
      if (hfdcan == &hfdcan1) {
        App_CanRecoverBus(hfdcan, 0u);
        can1_tx_stall_since_ms = 0u;
      } else if (hfdcan == &hfdcan2) {
        App_CanRecoverBus(hfdcan, 1u);
        can2_tx_stall_since_ms = 0u;
      }
    }
  }
}

void App_UpdateCanActivity(void)
{
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

    App_UpdateCanLineState();
    App_CanWatchdog();
}

void App_CanOnRx(uint8_t bus)
{
    uint32_t now = HAL_GetTick();
    if (bus == 1u) {
        CAN1_Active = 1u;
        can1_last_rx_tick = now;
    } else if (bus == 2u) {
        CAN2_Active = 1u;
        can2_last_rx_tick = now;
    }
}

void App_CanRxPush(uint32_t id, const uint8_t *data, uint8_t bus)
{
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

void App_CanProcess(void)
{
    while (can_rx_head != can_rx_tail) {
        AppCanRxEntry *e = &can_rx_ring[can_rx_tail];
        can_rx_tail++;
        if (can_rx_tail >= APP_CAN_RX_RING_SIZE) {
            can_rx_tail = 0u;
        }
        ProtocolParse(e->id, e->data, e->bus);
    }
}

uint8_t App_GetCanStateMask(void)
{
    return (uint8_t)((CAN1_State & 0x3u) | ((CAN2_State & 0x3u) << 2));
}
