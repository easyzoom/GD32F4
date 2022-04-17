#include "mdio.h"
#include "ethernetif.h"

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
//    uint16_t v_data = 0;

    state = ethernetif_read_regs(phyAddr, regAddr, data);
//    *data = v_data;

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
