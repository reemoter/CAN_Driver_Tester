#ifndef ETHER_PHY_H
#define ETHER_PHY_H
#include "lwip/netif.h"
#include "main.h"
typedef struct {
    uint32_t phy_address, phy_id, basic_control, basic_status;
    uint32_t advertised, partner, link_up, mac_started, speed_mbps, full_duplex;
    uint32_t mdio_errors, start_errors;
    uint32_t poll_count, scan_count, scan_address, scan_id1, scan_id2;
    uint32_t scan_hal_status, mii_address_register, last_poll_ms;
} EthernetDiagnostics;
extern volatile EthernetDiagnostics ethernet_diagnostics;
/* Scheduler entry point; independent of regenerated BSP link-check body. */
void EthernetPhy_Process(void);
void EthernetPhy_Check(struct netif *netif, ETH_HandleTypeDef *eth);
#endif
