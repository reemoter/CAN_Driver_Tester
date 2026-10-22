#include "board_identity.h"
#include <string.h>

_Static_assert(BOARD_HOST_FOR_ID(1U) > 0U && BOARD_HOST_FOR_ID(4U) < 255U,
               "Board IPv4 range must not wrap or select network/broadcast");

volatile BoardIdentity board_identity;
static uint8_t mac_address[6];

HAL_StatusTypeDef BoardIdentity_Init(void)
{
    if (board_identity.initialized) return HAL_OK;
    /* Initialize only the selector pins before the generated network startup.
     * MX_GPIO_Init later applies the same input/pulldown configuration. */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = INPUT_DEVICE0_Pin | INPUT_DEVICE1_Pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(INPUT_DEVICE0_GPIO_Port, &gpio);
    /* Read both selector bits in one port sample. */
    uint8_t previous = 0xFFU, consecutive = 0U;
    for (uint32_t sample = 0; sample < 50U; sample++) {
        uint32_t pins = INPUT_DEVICE0_GPIO_Port->IDR;
        uint8_t id = (uint8_t)BOARD_ID_FROM_LEVELS(pins & INPUT_DEVICE0_Pin,
                                                  pins & INPUT_DEVICE1_Pin);
        board_identity.samples++;
        board_identity.raw_selector = (uint8_t)(id - 1U);
        consecutive = id == previous ? (uint8_t)(consecutive + 1U) : 1U;
        previous = id;
        if (consecutive >= 8U) {
            /* Fixed /24 subnet, widened addition avoids silent uint8 overflow.
             * Base .22 is the original default; board hosts are .23 through .26. */
            uint32_t host = BOARD_HOST_FOR_ID(id);
            if (id < 1U || id > 4U || host == 0U || host >= 255U || host == 1U) {
                board_identity.error = 2U;
                return HAL_ERROR;
            }
            const uint8_t ip[4] = {192, 168, 0, (uint8_t)host};
            const uint8_t mask[4] = {255, 255, 255, 0};
            const uint8_t gateway[4] = {192, 168, 0, 1};
            const uint8_t mac[6] = {0x00, 0x80, 0xE1, 0x00, 0x00, id};
            memcpy(mac_address, mac, sizeof(mac));
            memcpy((void *)board_identity.ip, ip, sizeof(ip));
            memcpy((void *)board_identity.netmask, mask, sizeof(mask));
            memcpy((void *)board_identity.gateway, gateway, sizeof(gateway));
            memcpy((void *)board_identity.mac, mac, sizeof(mac));
            board_identity.id = id;
            board_identity.stable = 1;
            board_identity.error = 0;
            board_identity.initialized = 1;
            return HAL_OK;
        }
        HAL_Delay(1U);
    }
    board_identity.error = 1U; /* Selector did not settle; do not invent an ID. */
    return HAL_ERROR;
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
