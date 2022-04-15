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
 * @brief ��ȡPHY�Ĵ�������
 * 
 * @param [in] phyAddr PHY�����ַ
 * @param [in] regAddr PHY�����ַ�еļĴ�����ַ
 * @param [in] *data    ��ȡ��������
 * @return 0 �ɹ� >0 ʧ��
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
 * @brief д�����ݵ�PHY�Ĵ���
 * 
 * @param [in] phyAddr PHY�����ַ
 * @param [in] regAddr PHY�����ַ�еļĴ�����ַ
 * @param [in] data    д�������
 * @return 0 �ɹ� >0 ʧ��
 */
uint16_t mdio_write_regs(uint8_t phyAddr, uint8_t regAddr, uint16_t data)
{
    uint16_t state;
    
    state = ethernetif_write_regs(phyAddr, regAddr, &data);
    
    return state;
}


