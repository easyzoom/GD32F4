#ifndef __ETHERNETIF_H__
#define __ETHERNETIF_H__

#include "lwip/err.h"
#include "lwip/netif.h"
#include "cmsis_os.h"

struct link_str {
  struct netif *netif;
  osSemaphoreId semaphore;
};


extern uint8_t MACAddr[6] ;

typedef struct enet_init_config
{
    enet_mediamode_enum mediamode;
    enet_chksumconf_enum checksum;
    enet_frmrecept_enum recept;
    uint32_t duplexmode;
    uint32_t speed;
}enet_init_config_t;


err_t ethernetif_init(struct netif *netif);

void gd32_eth_gpio_init(void);
void enet_mac_dma_config(void);
void ethernetif_input(void * pvParameters);
void ethernetif_set_link(void const *argument);
void ethernetif_update_config(struct netif *netif);
void ethernetif_notify_conn_changed(struct netif *netif);

u32_t sys_jiffies(void);
u32_t sys_now(void);


#endif

