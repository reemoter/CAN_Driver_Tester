#include "ether_phy.h"
#include "app_time.h"

volatile EthernetDiagnostics ethernet_diagnostics = {.phy_address = 0xFFFFFFFFU};
static uint32_t scan_address = 1U;
static uint8_t found;
static uint8_t mdio_initialized;

/* CubeMX owns these instances; this application file owns PHY maintenance. */
extern struct netif gnetif;
extern ETH_HandleTypeDef heth;

void EthernetPhy_Process(void)
{
    EthernetPhy_Check(&gnetif, &heth);
}

static uint8_t read_phy(ETH_HandleTypeDef *eth, uint32_t reg, uint32_t *value)
{
    if (HAL_ETH_ReadPHYRegister(eth, ethernet_diagnostics.phy_address, reg, value)==HAL_OK) return 1;
    ethernet_diagnostics.mdio_errors++;
    return 0;
}

void EthernetPhy_Check(struct netif *netif, ETH_HandleTypeDef *eth)
{
    uint32_t status, control, advertised, partner;
    ethernet_diagnostics.poll_count++;
    ethernet_diagnostics.last_poll_ms=AppTime_NowMs();
    if (!mdio_initialized) {
        /* This HAL version does not configure MDC during HAL_ETH_Init.
         * Set its divider from HCLK before the first management transaction. */
        HAL_ETH_SetMDIOClockRange(eth);
        mdio_initialized=1;
    }
    if (!found) {
        uint32_t id1=0, id2=0;
        /* One address per poll; no startup busy wait or PHY reset. */
        HAL_StatusTypeDef first=HAL_ETH_ReadPHYRegister(eth,scan_address,2,&id1);
        HAL_StatusTypeDef second=HAL_ETH_ReadPHYRegister(eth,scan_address,3,&id2);
        ethernet_diagnostics.scan_count++;
        ethernet_diagnostics.scan_address=scan_address;
        ethernet_diagnostics.scan_id1=id1;
        ethernet_diagnostics.scan_id2=id2;
        ethernet_diagnostics.scan_hal_status=(uint32_t)(first!=HAL_OK ? first : second);
        ethernet_diagnostics.mii_address_register=eth->Instance->MACMIIAR;
        if (first!=HAL_OK) ethernet_diagnostics.mdio_errors++;
        if (second!=HAL_OK) ethernet_diagnostics.mdio_errors++;
        if (first==HAL_OK && second==HAL_OK &&
            (id1 & 0xFFFFU)!=0U && (id1 & 0xFFFFU)!=0xFFFFU) {
            ethernet_diagnostics.phy_address=scan_address;
            ethernet_diagnostics.phy_id=(id1<<16)|(id2 & 0xFFFFU);
            found=1;
        } else { scan_address=(scan_address+1U)%32U; return; }
    }
    /* Clause 22 BMSR link status is latched low: read twice. */
    if (!read_phy(eth,1,&status) || !read_phy(eth,1,&status) || !read_phy(eth,0,&control)) return;
    ethernet_diagnostics.basic_status=status;
    ethernet_diagnostics.basic_control=control;
    ethernet_diagnostics.link_up=(status & 4U)!=0U;
    if (!(status & 4U)) {
        if (eth->gState==HAL_ETH_STATE_STARTED) (void)HAL_ETH_Stop(eth);
        ethernet_diagnostics.mac_started=0;
        netif_set_link_down(netif);
        return;
    }
    uint32_t speed=ETH_SPEED_10M, duplex=ETH_HALFDUPLEX_MODE;
    if (control & 0x1000U) {
        if (!(status & 0x20U)) return; /* wait for auto-negotiation */
        if (!read_phy(eth,4,&advertised) || !read_phy(eth,5,&partner)) return;
        ethernet_diagnostics.advertised=advertised;
        ethernet_diagnostics.partner=partner;
        uint32_t common=advertised & partner;
        if (common & 0x100U) { speed=ETH_SPEED_100M; duplex=ETH_FULLDUPLEX_MODE; }
        else if (common & 0x80U) speed=ETH_SPEED_100M;
        else if (common & 0x40U) duplex=ETH_FULLDUPLEX_MODE;
        else if (!(common & 0x20U)) return;
    } else {
        if (control & 0x2000U) speed=ETH_SPEED_100M;
        if (control & 0x100U) duplex=ETH_FULLDUPLEX_MODE;
    }
    if (eth->gState!=HAL_ETH_STATE_STARTED) {
        ETH_MACConfigTypeDef config;
        if (HAL_ETH_GetMACConfig(eth,&config)!=HAL_OK) { ethernet_diagnostics.start_errors++; return; }
        config.Speed=speed; config.DuplexMode=duplex;
        if (HAL_ETH_SetMACConfig(eth,&config)!=HAL_OK || HAL_ETH_Start(eth)!=HAL_OK) {
            ethernet_diagnostics.start_errors++; return;
        }
    }
    ethernet_diagnostics.mac_started=1;
    ethernet_diagnostics.speed_mbps=speed==ETH_SPEED_100M ? 100U : 10U;
    ethernet_diagnostics.full_duplex=duplex==ETH_FULLDUPLEX_MODE;
    netif_set_link_up(netif);
}
