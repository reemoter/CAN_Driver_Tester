#ifndef BOARD_IDENTITY_H
#define BOARD_IDENTITY_H
#include "main.h"

/* Compile-time identity. No GPIO selector: PD0/PD1 belong to sensor wiring.
 * Change BOARD_CONFIG_ID (1..4), rebuild and download to select a board.
 * ID 1/2/3/4 -> 192.168.0.23/24/25/26, MAC last byte = ID.
 * Can also be overridden by compiler -DBOARD_CONFIG_ID=... . */
#ifndef BOARD_CONFIG_ID
#define BOARD_CONFIG_ID 2U
#endif
#define BOARD_IP_BASE_LAST 22U
#define BOARD_HOST_FOR_ID(id) (BOARD_IP_BASE_LAST + (id))
#define BOARD_SELECTOR_UNUSED 0xFFU

_Static_assert(BOARD_CONFIG_ID >= 1U && BOARD_CONFIG_ID <= 4U,
               "BOARD_CONFIG_ID must be 1..4");

typedef struct {
    uint8_t id, raw_selector, stable, initialized;
    uint32_t samples, error;
    uint8_t ip[4], netmask[4], gateway[4], mac[6];
} BoardIdentity;

/* Read-only diagnostics. raw_selector=0xFF, samples=0: no physical selector.
 * stable means a valid fixed configuration; retained for UDP compatibility. */
extern volatile BoardIdentity board_identity;
HAL_StatusTypeDef BoardIdentity_Init(void);
const uint8_t *BoardIdentity_Mac(void); /* Persistent storage, not stack memory. */
void BoardIdentity_CopyIPv4(uint8_t ip[4], uint8_t mask[4], uint8_t gateway[4]);
#endif
