#ifndef __GD_LWIP_H_
#define __GD_LWIP_H_

#include "lwip/opt.h"
#include "lwip/mem.h"
#include "lwip/memp.h"
#include "netif/etharp.h"
#include "lwip/dhcp.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "ethernetif.h"

#if WITH_RTOS
#include "lwip/tcpip.h"
#endif /* WITH_RTOS */

extern uint8_t IP_ADDRESS[4];
extern uint8_t NETMASK_ADDRESS[4];
extern uint8_t GATEWAY_ADDRESS[4];

void refresh_ip4(void);
void gd32_lwip_init(void);
#endif

