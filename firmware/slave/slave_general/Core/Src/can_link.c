/* can_link — HAL transport for the RobStride CAN chain (see can_link.h). */
#include "can_link.h"
#include "main.h"

static CAN_RxHeaderTypeDef can_link_rx_header;   /* ISR-owned RX header */

volatile uint32_t can_tx_ok_count = 0;
volatile uint32_t can_tx_error_count = 0;
volatile uint32_t can_last_tx_error = 0;

volatile uint32_t can_raw_rx_count = 0;
volatile uint32_t can_last_ext_id = 0;
volatile uint32_t can_last_rx_word0 = 0;
volatile uint32_t can_last_rx_word1 = 0;
volatile uint8_t  can_last_dlc = 0;
volatile uint8_t  can_last_ide = 0;

static uint32_t tx_mailbox;
static uint8_t rx_data[8];   /* ISR-owned RX payload buffer */

HAL_StatusTypeDef can_link_init(void)
{
    /* Accept every incoming frame into FIFO0. */
    CAN_FilterTypeDef f = {0};
    f.FilterBank = 0;
    f.FilterMode = CAN_FILTERMODE_IDMASK;
    f.FilterScale = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh = 0x0000;
    f.FilterIdLow = 0x0000;
    f.FilterMaskIdHigh = 0x0000;
    f.FilterMaskIdLow = 0x0000;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterActivation = ENABLE;
    f.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK) {
        return HAL_ERROR;
    }
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        return HAL_ERROR;
    }
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

void can_link_error_counters(uint8_t *tec, uint8_t *rec)
{
    uint32_t esr = hcan1.Instance->ESR;
    if (tec) *tec = (uint8_t)((esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos);
    if (rec) *rec = (uint8_t)((esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos);
}

/* Single transmit path for every CAN frame. The F446 bxCAN has only 3 TX
 * mailboxes; motor_runtime_update() bursts one frame per motor each tick,
 * so with >3 motors the later frames (e.g. CAN ids 4,5) were silently dropped
 * for lack of a free mailbox. Wait (bounded) for a mailbox to free before
 * queueing. A full 8-byte extended frame at 1 Mbps drains in ~110 µs, so the
 * wait is effectively a few hundred microseconds even when all 3 are busy. */
HAL_StatusTypeDef can_link_tx(uint32_t ext_id, const uint8_t msg[8])
{
    CAN_TxHeaderTypeDef hdr = {
        .StdId = 0x0,
        .ExtId = ext_id,
        .IDE   = CAN_ID_EXT,
        .RTR   = CAN_RTR_DATA,
        .DLC   = 8,
    };

    uint32_t t0 = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0u) {
        if ((HAL_GetTick() - t0) > 1u) {
            break;
        }
    }

    HAL_StatusTypeDef status =
        HAL_CAN_AddTxMessage(&hcan1, &hdr, (uint8_t *)msg, &tx_mailbox);
    if (status == HAL_OK) {
        can_tx_ok_count++;
    } else {
        can_tx_error_count++;
        can_last_tx_error = HAL_CAN_GetError(&hcan1);
    }
    return status;
}

/* RX side: read each pending FIFO0 frame out of the peripheral, record the
 * wire-level diagnostics, and hand the frame to the app hook. */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can_link_rx_header, rx_data);
    can_raw_rx_count++;
    can_last_ext_id = can_link_rx_header.ExtId;
    can_last_dlc = can_link_rx_header.DLC;
    can_last_ide = can_link_rx_header.IDE;
    can_last_rx_word0 = ((uint32_t)rx_data[0]) |
                        ((uint32_t)rx_data[1] << 8) |
                        ((uint32_t)rx_data[2] << 16) |
                        ((uint32_t)rx_data[3] << 24);
    can_last_rx_word1 = ((uint32_t)rx_data[4]) |
                        ((uint32_t)rx_data[5] << 8) |
                        ((uint32_t)rx_data[6] << 16) |
                        ((uint32_t)rx_data[7] << 24);

    can_link_on_frame(can_link_rx_header.ExtId, rx_data);
}
