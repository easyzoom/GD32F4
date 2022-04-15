#include "main.h"
#include "lwip/opt.h"
#include "lwip/timeouts.h"
#include "netif/ethernet.h"
#include "netif/etharp.h"
#include "lwip/ethip6.h"
#include "ethernetif.h"
#include <string.h>
#include "cmsis_os.h"
#include "lwip/tcpip.h"
#include "switch_app.h"

/* The time to block waiting for input. */
#define TIME_WAITING_FOR_INPUT ( portMAX_DELAY )
osSemaphoreId s_xSemaphore = NULL;
#define ETHERNETIF_INPUT_TASK_STACK_SIZE          (350)
#define ETHERNETIF_INPUT_TASK_PRIO                (configMAX_PRIORITIES - 1)
#define LOWLEVEL_OUTPUT_WAITING_TIME              (250)
/* The time to block waiting for input */
#define LOWLEVEL_INPUT_WAITING_TIME               ((portTickType )100)

/* define those to better describe your network interface */
#define IFNAME0 'G'
#define IFNAME1 'D'

uint8_t MACAddr[6];
enet_init_config_t enet_init_object;
/* ENET RxDMA/TxDMA descriptor */
extern enet_descriptors_struct  rxdesc_tab[ENET_RXBUF_NUM], txdesc_tab[ENET_TXBUF_NUM];

/* ENET receive buffer  */
extern uint8_t rx_buff[ENET_RXBUF_NUM][ENET_RXBUF_SIZE]; 

/* ENET transmit buffer */
extern uint8_t tx_buff[ENET_TXBUF_NUM][ENET_TXBUF_SIZE]; 

/*global transmit and receive descriptors pointers */
extern enet_descriptors_struct  *dma_current_txdesc;
extern enet_descriptors_struct  *dma_current_rxdesc;

/* preserve another ENET RxDMA/TxDMA ptp descriptor for normal mode */
enet_descriptors_struct  ptp_txstructure[ENET_TXBUF_NUM];
enet_descriptors_struct  ptp_rxstructure[ENET_RXBUF_NUM];

static struct netif *low_netif = NULL;
xSemaphoreHandle g_rx_semaphore = NULL;

void gd32_eth_gpio_init(void)
{
    /* configures the nested vectored interrupt controller */
    nvic_vector_table_set(NVIC_VECTTAB_FLASH, 0x0);
    nvic_irq_enable(ENET_IRQn, 2, 0);

    /* configures the different GPIO ports */
    gpio_af_set(GPIOC, GPIO_AF_0, GPIO_PIN_13);
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_13);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_MAX,GPIO_PIN_13);
  
    /* enable SYSCFG clock */
    rcu_periph_clock_enable(RCU_SYSCFG);
  
#ifdef MII_MODE 
  
#ifdef PHY_CLOCK_MCO
    /* output HXTAL clock (25MHz) on CKOUT0 pin(PA8) to clock the PHY */
    rcu_ckout0_config(RCU_CKOUT0SRC_HXTAL, RCU_CKOUT0_DIV1);
#endif /* PHY_CLOCK_MCO */

    syscfg_enet_phy_interface_config(SYSCFG_ENET_PHY_MII);

#elif defined RMII_MODE
    /* choose DIV2 to get 50MHz from 200MHz on CKOUT0 pin (PA8) to clock the PHY */
    rcu_ckout0_config(RCU_CKOUT0SRC_PLLP, RCU_CKOUT0_DIV4);
    syscfg_enet_phy_interface_config(SYSCFG_ENET_PHY_RMII);

#endif /* MII_MODE */

#ifdef MII_MODE

    /* PA1: ETH_MII_RX_CLK */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_1);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_MAX,GPIO_PIN_1);
    
    /* PA2: ETH_MDIO */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_MAX,GPIO_PIN_2);
    
    /* PA7: ETH_MII_RX_DV */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_7);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_MAX,GPIO_PIN_7);   
    
    gpio_af_set(GPIOA, GPIO_AF_11, GPIO_PIN_1);
    gpio_af_set(GPIOA, GPIO_AF_11, GPIO_PIN_2);
    gpio_af_set(GPIOA, GPIO_AF_11, GPIO_PIN_7);
    
    /* PG11: ETH_MII_TX_EN */
    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_11);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_MAX,GPIO_PIN_11);
    
    /* PG13: ETH_MII_TXD0 */
    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_13);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_MAX,GPIO_PIN_13);
    
    /* PG14: ETH_MII_TXD1 */
    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_14);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_MAX,GPIO_PIN_14);   
    
    gpio_af_set(GPIOG, GPIO_AF_11, GPIO_PIN_11);
    gpio_af_set(GPIOG, GPIO_AF_11, GPIO_PIN_13);
    gpio_af_set(GPIOG, GPIO_AF_11, GPIO_PIN_14);

    /* PC1: ETH_MDC */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_1);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_MAX,GPIO_PIN_1);
    
    /* PC2: ETH_MII_TXD2 */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_MAX,GPIO_PIN_2);
    
    /* PC3: ETH_MII_TX_CLK */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_MAX,GPIO_PIN_3);  

    /* PC4: ETH_MII_RXD0 */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_4);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_MAX,GPIO_PIN_4);
    
    /* PC5: ETH_MII_RXD1 */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_MAX,GPIO_PIN_5); 
    
    gpio_af_set(GPIOC, GPIO_AF_11, GPIO_PIN_1);
    gpio_af_set(GPIOC, GPIO_AF_11, GPIO_PIN_2);
    gpio_af_set(GPIOC, GPIO_AF_11, GPIO_PIN_3);
    gpio_af_set(GPIOC, GPIO_AF_11, GPIO_PIN_4);
    gpio_af_set(GPIOC, GPIO_AF_11, GPIO_PIN_5);

    /* PH2: ETH_MII_CRS */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_MAX,GPIO_PIN_2);
    
    /* PH3: ETH_MII_COL */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_MAX,GPIO_PIN_3);
    
    /* PH6: ETH_MII_RXD2 */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_MAX,GPIO_PIN_6);  

    /* PH7: ETH_MII_RXD3 */
    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_7);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_MAX,GPIO_PIN_7);
    
    gpio_af_set(GPIOH, GPIO_AF_11, GPIO_PIN_2);
    gpio_af_set(GPIOH, GPIO_AF_11, GPIO_PIN_3);
    gpio_af_set(GPIOH, GPIO_AF_11, GPIO_PIN_6);
    gpio_af_set(GPIOH, GPIO_AF_11, GPIO_PIN_7);

    /* PI8: ETH_INT */
    gpio_mode_set(GPIOI, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_8);
    gpio_output_options_set(GPIOI, GPIO_OTYPE_PP, GPIO_OSPEED_MAX,GPIO_PIN_8);  

    /* PI10: ETH_MII_RX_ER */
    gpio_mode_set(GPIOI, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_10);
    gpio_output_options_set(GPIOI, GPIO_OTYPE_PP, GPIO_OSPEED_MAX,GPIO_PIN_10);

    gpio_af_set(GPIOI, GPIO_AF_11, GPIO_PIN_8);
    gpio_af_set(GPIOI, GPIO_AF_11, GPIO_PIN_10);

    /* PB8: ETH_MII_TXD3 */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_8);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_MAX,GPIO_PIN_8);
    
    gpio_af_set(GPIOB, GPIO_AF_11, GPIO_PIN_8);
      
#elif defined RMII_MODE

    /* PA1: ETH_RMII_REF_CLK */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_1);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_MAX,GPIO_PIN_1);
    
    /* PA2: ETH_MDIO */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_MAX,GPIO_PIN_2);
    
    /* PA7: ETH_RMII_CRS_DV */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_7);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_MAX,GPIO_PIN_7);   
    
    gpio_af_set(GPIOA, GPIO_AF_11, GPIO_PIN_1);
    gpio_af_set(GPIOA, GPIO_AF_11, GPIO_PIN_2);
    gpio_af_set(GPIOA, GPIO_AF_11, GPIO_PIN_7);

    /* PG11: ETH_RMII_TX_EN */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_11);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_MAX,GPIO_PIN_11);
    
    /* PG13: ETH_RMII_TXD0 */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_MAX,GPIO_PIN_12);
    
    /* PG14: ETH_RMII_TXD1 */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_13);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_MAX,GPIO_PIN_13);   
    
    gpio_af_set(GPIOB, GPIO_AF_11, GPIO_PIN_11);
    gpio_af_set(GPIOB, GPIO_AF_11, GPIO_PIN_12);
    gpio_af_set(GPIOB, GPIO_AF_11, GPIO_PIN_13);    

    /* PC1: ETH_MDC */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_1);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_MAX,GPIO_PIN_1);

    /* PC4: ETH_RMII_RXD0 */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_4);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_MAX,GPIO_PIN_4);
    
    /* PC5: ETH_RMII_RXD1 */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_MAX,GPIO_PIN_5); 
    
    gpio_af_set(GPIOC, GPIO_AF_11, GPIO_PIN_1);
    gpio_af_set(GPIOC, GPIO_AF_11, GPIO_PIN_4);
    gpio_af_set(GPIOC, GPIO_AF_11, GPIO_PIN_5);
    switch_reset();
#endif /* MII_MODE */
}

/*!
    \brief      configures the ethernet interface
    \param[in]  none
    \param[out] none
    \retval     none
*/
void enet_mac_dma_config(void)
{
    ErrStatus reval_state = ERROR;
    
    /* enable ethernet clock  */
    rcu_periph_clock_enable(RCU_ENET);
    rcu_periph_clock_enable(RCU_ENETTX);
    rcu_periph_clock_enable(RCU_ENETRX);
    
    /* reset ethernet on AHB bus */
    enet_deinit();

    reval_state = enet_software_reset();
    if(ERROR == reval_state)
    {
        while(1);
    }
    
/* configure the parameters which are usually less cared for enet initialization */  
//  enet_initpara_config(HALFDUPLEX_OPTION, ENET_CARRIERSENSE_ENABLE|ENET_RECEIVEOWN_ENABLE|ENET_RETRYTRANSMISSION_DISABLE|ENET_BACKOFFLIMIT_10|ENET_DEFERRALCHECK_DISABLE);
//  enet_initpara_config(DMA_OPTION, ENET_FLUSH_RXFRAME_ENABLE|ENET_SECONDFRAME_OPT_ENABLE|ENET_NORMAL_DESCRIPTOR);
#ifdef CHECKSUM_BY_HARDWARE
    enet_init_object.mediamode = ENET_100M_FULLDUPLEX;
    enet_init_object.checksum = ENET_AUTOCHECKSUM_DROP_FAILFRAMES;
    enet_init_object.recept = ENET_BROADCAST_FRAMES_PASS;
    enet_init_object.duplexmode = ENET_MODE_FULLDUPLEX;
    enet_init_object.speed = ENET_SPEEDMODE_100M;
#else
    enet_init_object.mediamode = ENET_AUTO_NEGOTIATION;
    enet_init_object.checksum = ENET_NO_AUTOCHECKSUM;
    enet_init_object.recept = ENET_BROADCAST_FRAMES_PASS;
#endif /* CHECKSUM_BY_HARDWARE */

    if(0 == enet_init(enet_init_object.mediamode, enet_init_object.checksum, enet_init_object.recept))
    {
        while(1);
    }
    enet_interrupt_enable(ENET_DMA_INT_NIE);
    enet_interrupt_enable(ENET_DMA_INT_RIE);
}

/**
* In this function, the hardware should be initialized.
* Called from ethernetif_init().
*
* @param netif the already initialized lwip network interface structure
*        for this ethernetif
*/
static void low_level_init(struct netif *netif)
{
    uint32_t i;

    /* set netif MAC hardware address length */
    netif->hwaddr_len = ETHARP_HWADDR_LEN;

    /* set netif MAC hardware address */
    netif->hwaddr[0] =  MACAddr[0];
    netif->hwaddr[1] =  MACAddr[1];
    netif->hwaddr[2] =  MACAddr[2];
    netif->hwaddr[3] =  MACAddr[3];
    netif->hwaddr[4] =  MACAddr[4];
    netif->hwaddr[5] =  MACAddr[5];

    /* set netif maximum transfer unit */
    netif->mtu = 1500;

    /* accept broadcast address and ARP traffic */
    /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
#if LWIP_ARP
    netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;
#else
    netif->flags |= NETIF_FLAG_BROADCAST;
#endif /* LWIP_ARP */

    /* initialize MAC address in ethernet MAC */ 
    enet_mac_address_set(ENET_MAC_ADDRESS0, netif->hwaddr);
 
    /* Set netif link flag */
    netif->flags |= NETIF_FLAG_LINK_UP;
  

    /* initialize descriptors list: chain/ring mode */
#ifdef SELECT_DESCRIPTORS_ENHANCED_MODE
    enet_ptp_enhanced_descriptors_chain_init(ENET_DMA_TX);
    enet_ptp_enhanced_descriptors_chain_init(ENET_DMA_RX);
#else

    /* Initialize Tx Descriptors list: Chain Mode */
    enet_descriptors_chain_init(ENET_DMA_TX);
    /* Initialize Rx Descriptors list: Chain Mode */
    enet_descriptors_chain_init(ENET_DMA_RX);
    
//    enet_descriptors_ring_init(ENET_DMA_TX);
//    enet_descriptors_ring_init(ENET_DMA_RX);  
    
#endif /* SELECT_DESCRIPTORS_ENHANCED_MODE */  

    /* enable ethernet Rx interrrupt */
    {   int i;
        for(i=0; i<ENET_RXBUF_NUM; i++){ 
            enet_rx_desc_immediate_receive_complete_interrupt(&rxdesc_tab[i]);
        }
    }


#ifdef CHECKSUM_BY_HARDWARE
    /* enable the TCP, UDP and ICMP checksum insertion for the Tx frames */
    for(i=0; i < ENET_TXBUF_NUM; i++){
        enet_transmit_checksum_config(&txdesc_tab[i], ENET_CHECKSUM_TCPUDPICMP_FULL);
    }
#endif /* CHECKSUM_BY_HARDWARE */

    /* create a binary semaphore used for informing ethernetif of frame reception */
    if (g_rx_semaphore == NULL){
        vSemaphoreCreateBinary(g_rx_semaphore);
        xSemaphoreTake( g_rx_semaphore, 0);
    }
    /* create the task that handles the ETH_MAC */
    xTaskCreate(ethernetif_input, "ETHERNETIF_INPUT", ETHERNETIF_INPUT_TASK_STACK_SIZE, NULL,
                ETHERNETIF_INPUT_TASK_PRIO,NULL);

    /* enable MAC and DMA transmission and reception */
    enet_enable();
}
/**
* This function should do the actual transmission of the packet. The packet is
* contained in the pbuf that is passed to the function. This pbuf
* might be chained.
*
* @param netif the lwip network interface structure for this ethernetif
* @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
* @return ERR_OK if the packet could be sent
*         an err_t value if the packet couldn't be sent
*
* @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
*       strange results. You might consider waiting for space in the DMA queue
*       to become availale since the stack doesn't retry to send a packet
*       dropped because of memory failure (except for the TCP timers).
*/

static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
    static xSemaphoreHandle s_tx_semaphore = NULL;
    struct pbuf *q;
    uint8_t *buffer ;
    uint16_t framelength = 0;
    ErrStatus reval = ERROR;
  
    SYS_ARCH_DECL_PROTECT(sr);
    
    if (s_tx_semaphore == NULL){
        vSemaphoreCreateBinary (s_tx_semaphore);
    }

    if (xSemaphoreTake(s_tx_semaphore, LOWLEVEL_OUTPUT_WAITING_TIME)){    
        SYS_ARCH_PROTECT(sr);
      
        while((uint32_t)RESET != (dma_current_txdesc->status & ENET_TDES0_DAV)){
        }      
        buffer = (uint8_t *)(enet_desc_information_get(dma_current_txdesc, TXDESC_BUFFER_1_ADDR));

        for(q = p; q != NULL; q = q->next){ 
            memcpy((uint8_t *)&buffer[framelength], q->payload, q->len);
            framelength = framelength + q->len;
        }

       /* transmit descriptors to give to DMA */ 
#ifdef SELECT_DESCRIPTORS_ENHANCED_MODE
        reval = ENET_NOCOPY_PTPFRAME_TRANSMIT_ENHANCED_MODE(framelength, NULL);
#else
        reval = ENET_NOCOPY_FRAME_TRANSMIT(framelength);
#endif /* SELECT_DESCRIPTORS_ENHANCED_MODE */

        SYS_ARCH_UNPROTECT(sr);
        
        /* give semaphore and exit */
        xSemaphoreGive(s_tx_semaphore);
    }
    
    if(SUCCESS == reval){
        return ERR_OK;
    }else{
        while(1){
        }
    }
    
}

/**
* Should allocate a pbuf and transfer the bytes of the incoming
* packet from the interface into the pbuf.
*
* @param netif the lwip network interface structure for this ethernetif
* @return a pbuf filled with the received packet (including MAC header)
*         NULL on memory error
*/
static struct pbuf * low_level_input(struct netif *netif)
{
    struct pbuf *p= NULL, *q;
    uint32_t l =0;
    u16_t len;
    uint8_t *buffer;

    /* obtain the size of the packet and put it into the "len" variable. */
    len = enet_desc_information_get(dma_current_rxdesc, RXDESC_FRAME_LENGTH);
    buffer = (uint8_t *)(enet_desc_information_get(dma_current_rxdesc, RXDESC_BUFFER_1_ADDR));
  
    if (len > 0){
        /* We allocate a pbuf chain of pbufs from the Lwip buffer pool */
        p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
    }
  
    if (p != NULL){
        for(q = p; q != NULL; q = q->next){
            memcpy((uint8_t *)q->payload, (u8_t*)&buffer[l], q->len);
            l = l + q->len;
        }
    }
#ifdef SELECT_DESCRIPTORS_ENHANCED_MODE
    ENET_NOCOPY_PTPFRAME_RECEIVE_ENHANCED_MODE(NULL);
  
#else
    
    ENET_NOCOPY_FRAME_RECEIVE();
#endif /* SELECT_DESCRIPTORS_ENHANCED_MODE */

    return p;
}

/**
* This function is the ethernetif_input task, it is processed when a packet 
* is ready to be read from the interface. It uses the function low_level_input() 
* that should handle the actual reception of bytes from the network
* interface. Then the type of the received packet is determined and
* the appropriate input function is called.
*
* @param netif the lwip network interface structure for this ethernetif
*/
void ethernetif_input(void * pvParameters)
{
    struct pbuf *p;
    SYS_ARCH_DECL_PROTECT(sr);
  
    for( ;; ){   
        if(pdTRUE == xSemaphoreTake(g_rx_semaphore, LOWLEVEL_INPUT_WAITING_TIME)){ 
TRY_GET_NEXT_FRAME:
            SYS_ARCH_PROTECT(sr);
            p = low_level_input(low_netif);
            SYS_ARCH_UNPROTECT(sr);
          
            if   (p != NULL){
                if (ERR_OK != low_netif->input( p, low_netif)){
                    pbuf_free(p);
                }else{
                    goto TRY_GET_NEXT_FRAME;
                }
            }
        }
    }
}


/**
* Should be called at the beginning of the program to set up the
* network interface. It calls the function low_level_init() to do the
* actual setup of the hardware.
*
* This function should be passed as a parameter to netif_add().
*
* @param netif the lwip network interface structure for this ethernetif
* @return ERR_OK if the loopif is initialized
*         ERR_MEM if private data couldn't be allocated
*         any other err_t on error
*/
err_t ethernetif_init(struct netif *netif)
{
    LWIP_ASSERT("netif != NULL", (netif != NULL));

#if LWIP_NETIF_HOSTNAME
    /* initialize interface hostname */
    netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

    netif->name[0] = IFNAME0;
    netif->name[1] = IFNAME1;
    /* We directly use etharp_output() here to save a function call.
    * You can instead declare your own function an call etharp_output()
    * from it if you have to do some checks before sending (e.g. if link
    * is available...) */

#if LWIP_IPV4
#if LWIP_ARP || LWIP_ETHERNET
#if LWIP_ARP
    netif->output = etharp_output;
#else
    /* The user should write its own code in low_level_output_arp_off function */
    netif->output = low_level_output_arp_off;
#endif /* LWIP_ARP */
#endif /* LWIP_ARP || LWIP_ETHERNET */
#endif /* LWIP_IPV4 */

#if LWIP_IPV6
    netif->output_ip6 = ethip6_output;
#endif /* LWIP_IPV6 */
    netif->linkoutput = low_level_output;

    /* initialize the hardware */
    low_level_init(netif);

    return ERR_OK;
}

/**
* @brief  Returns the current time in milliseconds
*         when LWIP_TIMERS == 1 and NO_SYS == 1
* @param  None
* @retval Time
*/
u32_t sys_jiffies(void)
{
  return xTaskGetTickCount();
}


/**
* @brief  Returns the current time in milliseconds
*         when LWIP_TIMERS == 1 and NO_SYS == 1
* @param  None
* @retval Time
*/
u32_t sys_now(void)
{
  return xTaskGetTickCount();
}

/**
  * @brief  This function sets the netif link status.
  * @param  netif: the network interface
  * @retval None
  */
void ethernetif_set_link(void const *argument)
{
  uint32_t regvalue = 0;
  struct link_str *link_arg = (struct link_str *)argument;

  for(;;)
  {
    /* Read PHY_BSR*/
    enet_phy_write_read(ENET_PHY_READ, PHY_ADDRESS, PHY_REG_BSR, (uint16_t *)&regvalue);

    regvalue &= PHY_LINKED_STATUS;

    /* Check whether the netif link down and the PHY link is up */
    if(!netif_is_link_up(link_arg->netif) && (regvalue))
    {
      /* network cable is connected */
      netif_set_link_up(link_arg->netif);
    }
    else if(netif_is_link_up(link_arg->netif) && (!regvalue))
    {
      /* network cable is dis-connected */
      netif_set_link_down(link_arg->netif);
    }

    /* Suspend thread for 200 ms */
    osDelay(200);
  }
}

#if LWIP_NETIF_LINK_CALLBACK
/**
  * @brief  Link callback function, this function is called on change of link status
  *         to update low level driver configuration.
* @param  netif: The network interface
  * @retval None
  */
void ethernetif_update_config(struct netif *netif)
{
    if(netif_is_link_up(netif))
    {
        /* Restart the auto-negotiation */
        enet_mac_dma_config();

        /* Restart MAC interface */
        enet_enable();
    }
    else
    {
        /* Stop MAC interface */
        enet_disable();
    }

    ethernetif_notify_conn_changed(netif);
}

/* USER CODE BEGIN 8 */
/**
  * @brief  This function notify user about link status changement.
  * @param  netif: the network interface
  * @retval None
  */
__weak void ethernetif_notify_conn_changed(struct netif *netif)
{
  /* NOTE : This is function could be implemented in user file
            when the callback is needed,
  */

}
/* USER CODE END 8 */
#endif /* LWIP_NETIF_LINK_CALLBACK */
