#ifndef BOARD_IDENTITY_H
#define BOARD_IDENTITY_H
#include "main.h"

/* Original board selector order: DEVICE0=MSB, DEVICE1=LSB. */
#define BOARD_ID_FROM_LEVELS(device0, device1) \
    ((((device0) ? 2U : 0U) | ((device1) ? 1U : 0U)) + 1U)
#define BOARD_IP_BASE_LAST 22U
#define BOARD_HOST_FOR_ID(id) (BOARD_IP_BASE_LAST + (id))

typedef struct {
    uint8_t id, raw_selector, stable, initialized;
    uint32_t samples, error;
    uint8_t ip[4], netmask[4], gateway[4], mac[6];
} BoardIdentity;

/* Read-only diagnostics. ID is latched at boot; changing switches needs reset. */
extern volatile BoardIdentity board_identity;
HAL_StatusTypeDef BoardIdentity_Init(void);
const uint8_t *BoardIdentity_Mac(void); /* Persistent storage, not stack memory. */
void BoardIdentity_CopyIPv4(uint8_t ip[4], uint8_t mask[4], uint8_t gateway[4]);
#endif
