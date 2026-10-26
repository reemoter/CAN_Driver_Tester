#include "board_identity.h"
#include <string.h>

_Static_assert(BOARD_HOST_FOR_ID(1U) > 0U && BOARD_HOST_FOR_ID(4U) < 255U,
               "Board IPv4 range must not wrap or select network/broadcast");

volatile BoardIdentity board_identity;
static uint8_t mac_address[6];

HAL_StatusTypeDef BoardIdentity_Init(void)
{
    if (board_identity.initialized) return HAL_OK;
    const uint8_t id = (uint8_t)BOARD_CONFIG_ID;
    const uint8_t ip[4] = {192, 168, 0, (uint8_t)BOARD_HOST_FOR_ID(BOARD_CONFIG_ID)};
    const uint8_t mask[4] = {255, 255, 255, 0};
    const uint8_t gateway[4] = {192, 168, 0, 1};
    const uint8_t mac[6] = {0x00, 0x80, 0xE1, 0x00, 0x00, id};
    memcpy(mac_address, mac, sizeof(mac));
    memcpy((void *)board_identity.ip, ip, sizeof(ip));
    memcpy((void *)board_identity.netmask, mask, sizeof(mask));
    memcpy((void *)board_identity.gateway, gateway, sizeof(gateway));
    memcpy((void *)board_identity.mac, mac, sizeof(mac));
    board_identity.id = id;
    board_identity.raw_selector = BOARD_SELECTOR_UNUSED;
    board_identity.samples = 0;
    board_identity.stable = 1;
    board_identity.error = 0;
    board_identity.initialized = 1;
    return HAL_OK;
}

const uint8_t *BoardIdentity_Mac(void) { return mac_address; }

void BoardIdentity_CopyIPv4(uint8_t ip[4], uint8_t mask[4], uint8_t gateway[4])
{
    for (uint32_t i = 0; i < 4U; i++) {
        ip[i] = board_identity.ip[i];
        mask[i] = board_identity.netmask[i];
        gateway[i] = board_identity.gateway[i];
    }
}
