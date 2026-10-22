#include "main.h"

/* Board MII pinout from ABS_CONTROLLER_FW.ioc. The generic CubeMX Ethernet
 * interface has no BSP-generated MSP implementation in this project.
 * If CubeMX later generates a strong HAL_ETH_MspInit, consolidate these there. */
void HAL_ETH_MspInit(ETH_HandleTypeDef *eth)
{
    if (eth->Instance != ETH) return;
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_ETH_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF11_ETH;
    /* PC1 MDC, PC2 TXD2, PC3 TX_CLK, PC4 RXD0, PC5 RXD1 */
    gpio.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOC, &gpio);
    /* PA0 CRS, PA1 RX_CLK, PA2 MDIO, PA3 COL, PA7 RX_DV */
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &gpio);
    /* PB0 RXD2, PB1 RXD3, PB8 TXD3, PB11 TX_EN, PB12 TXD0, PB13 TXD1 */
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
    HAL_GPIO_Init(GPIOB, &gpio);
}

void HAL_ETH_MspDeInit(ETH_HandleTypeDef *eth)
{
    if (eth->Instance != ETH) return;
    __HAL_RCC_ETH_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_7);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13);
}
