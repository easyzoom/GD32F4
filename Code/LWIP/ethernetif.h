#ifndef __ETHERNETIF_H__
#define __ETHERNETIF_H__

#include "lwip/err.h"
#include "lwip/netif.h"
#include "cmsis_os.h"


/* Structure that include link thread parameters */
struct link_str {
  struct netif *netif;
  osSemaphoreId semaphore;
};


extern uint8_t MACAddr[6] ;

err_t ethernetif_init(struct netif *netif);
void ethernetif_input(void const * argument);
void ethernetif_set_link(void const *argument);
void ethernetif_update_config(struct netif *netif);
void ethernetif_notify_conn_changed(struct netif *netif);

u32_t sys_jiffies(void);
u32_t sys_now(void);

void enet_system_setup(void);
uint16_t ethernetif_read_regs(uint16_t phyAddr, uint16_t regAddr, uint16_t* regvalue);
uint16_t ethernetif_write_regs(uint16_t phyAddr, uint16_t regAddr, uint16_t *regvalue);

#endif


