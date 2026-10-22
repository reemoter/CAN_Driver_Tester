#ifndef FAKE_HAL_H
#define FAKE_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY } HAL_StatusTypeDef;
typedef struct { uint32_t ESR, TSR; } CAN_TypeDef;
typedef struct { CAN_TypeDef *Instance; } CAN_HandleTypeDef;
typedef struct { uint32_t StdId, IDE, RTR, DLC; } CAN_TxHeaderTypeDef;
typedef CAN_TxHeaderTypeDef CAN_RxHeaderTypeDef;
typedef struct {
    uint32_t FilterBank, FilterMode, FilterScale, FilterMaskIdLow;
    uint32_t FilterFIFOAssignment, FilterActivation, SlaveStartFilterBank;
} CAN_FilterTypeDef;
extern CAN_TypeDef fake_can;
#define CAN1 (&fake_can)
#define CAN_ID_STD 0U
#define CAN_RTR_DATA 0U
#define CAN_TX_MAILBOX0 1U
#define CAN_TX_MAILBOX1 2U
#define CAN_TX_MAILBOX2 4U
#define CAN_TSR_RQCP0 1U
#define CAN_TSR_TXOK0 2U
#define CAN_FILTERMODE_IDMASK 0U
#define CAN_FILTERSCALE_32BIT 0U
#define CAN_RX_FIFO0 0U
#define ENABLE 1U
#define CAN_IT_RX_FIFO0_MSG_PENDING 1U
#define CAN_FLAG_BOF 1U
#define CAN_FLAG_EPV 2U
#define HAL_CAN_ERROR_RX_FOV0 4U
#define GPIO_PIN_RESET 0U
#define GPIO_PIN_SET 1U
#define OUTPUT_PWR_EPOS4_GPIO_Port 0U
#define OUTPUT_PWR_EPOS4_Pin 0U
#define __HAL_CAN_GET_FLAG(h, f) 0U
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __set_PRIMASK(x) ((void)(x))
#define __DMB() ((void)0)
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *h);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h, CAN_TxHeaderTypeDef *header, uint8_t *data, uint32_t *mailbox);
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *h, uint32_t mailbox);
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *h, CAN_FilterTypeDef *filter);
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *h);
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *h, uint32_t flags);
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *h, uint32_t fifo);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *h, uint32_t fifo, CAN_RxHeaderTypeDef *header, uint8_t *data);
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *h);
HAL_StatusTypeDef HAL_CAN_DeInit(CAN_HandleTypeDef *h);
HAL_StatusTypeDef HAL_CAN_Init(CAN_HandleTypeDef *h);
void HAL_GPIO_WritePin(uint32_t port, uint32_t pin, uint32_t value);
#endif
