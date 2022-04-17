#ifndef __mx_lwip_H
#define __mx_lwip_H
#ifdef __cplusplus
 extern "C" {
#endif

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
void lwip_stack_init(void);

#ifdef __cplusplus
}
#endif
#endif
