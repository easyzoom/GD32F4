#include "mdio.h"
#include "ethernetif.h"

static uint16_t ethernetif_read_regs(uint16_t phyAddr, uint16_t regAddr, uint16_t* regvalue)
{
    if(enet_phy_write_read(ENET_PHY_READ, phyAddr, regAddr, regvalue) == SUCCESS)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

static uint16_t ethernetif_write_regs(uint16_t phyAddr, uint16_t regAddr, uint16_t *regvalue)
{
    if(enet_phy_write_read(ENET_PHY_WRITE, phyAddr, regAddr, regvalue) == SUCCESS)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

/**
 * @brief 读取PHY寄存器数据
 * 
 * @param [in] phyAddr PHY物理地址
 * @param [in] regAddr PHY物理地址中的寄存器地址
 * @param [in] *data    读取到的数据
 * @return 0 成功 >0 失败
 */
uint16_t mdio_read_regs(uint8_t phyAddr, uint8_t regAddr, uint16_t* data)
{
    uint16_t state;
    uint16_t v_data = 0;

    state = ethernetif_read_regs(phyAddr, regAddr, &v_data);
    *data = (uint16_t)v_data;

    return state;
}


/**
 * @brief 写入数据到PHY寄存器
 * 
 * @param [in] phyAddr PHY物理地址
 * @param [in] regAddr PHY物理地址中的寄存器地址
 * @param [in] data    写入的数据
 * @return 0 成功 >0 失败
 */
uint16_t mdio_write_regs(uint8_t phyAddr, uint8_t regAddr, uint16_t data)
{
    uint16_t state;
    
    state = ethernetif_write_regs(phyAddr, regAddr, &data);
    
    return state;
}


