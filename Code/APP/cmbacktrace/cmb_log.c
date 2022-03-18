/**
 * @file cmb_log.c
 * @author Letter (zhengkeqiang@ut.cn)
 * @brief 
 * @version 0.1
 * @date 2019-02-21
 * 
 * @Copyright (c) 2019 Unicook
 * 
 */

#include "cmb_log.h"
#include "stdarg.h"
#include "flash.h"
#include "stdio.h"
#include "cmd.h"

#if CMB_LOG_USING_FLASH == 1

void cmbLogFlash(char *buffer, short size);


/**
 * @brief cmbacktrace 输出函数
 * 
 * @param format 格式字符串
 * @param ... 参数
 */
void cmbLog(const char *format, ...)
{
    char buffer[CMB_LOG_BUFFER_SIZE];
    va_list args;
    short length;

    va_start(args, format);
    length = vsnprintf(buffer, CMB_LOG_BUFFER_SIZE - 8, format, args);
    va_end(args); 

    buffer[length++] = '\r';
    buffer[length++] = '\n';
    buffer[length] = 0;
    buffer[length + 1] = 0;
    buffer[length + 2] = 0;

    printf("%s", buffer);
    length = ((length + 2) / 4) * 4;
    cmbLogFlash(buffer, length);
}


/**
 * @brief cmbacktrace 输出到Flash
 * 
 * @param buffer 数据
 * @param size 数据大小
 */
void cmbLogFlash(char *buffer, short size)
{
    static int address = 0xFFFFFFFF;

    if (address == 0xFFFFFFFF)
    {
        Flash_Init();
        address = CMB_FLASH_ADDRESS;
        while (*(int *)address != 0xFFFFFFFF
            && address < (CMB_FLASH_ADDRESS + CMB_FLASH_SECTOR_SIZE - CMB_LOG_STOARGE_SIZE))
        {
            address += CMB_LOG_STOARGE_SIZE;
        }
        if (address >= (CMB_FLASH_ADDRESS + CMB_FLASH_SECTOR_SIZE))
        {
            Flash_Erase(CMB_FLASH_SECTOR);
            address = CMB_FLASH_ADDRESS;
        }
    }
    Flash_Init();
    address = ((address + 3) / 4) * 4;
    Flash_Write(address, buffer, size);
    address += size;
}


/**
 * @brief 打印最近几次cmbacktrace记录在Flash中的错误信息
 * 
 * @param count 打印的数量
 */
void cmbPrintLog(short count)
{
    int address = CMB_FLASH_ADDRESS;
    char *p;

    Flash_Init();

    while (*(int *)address != 0xFFFFFFFF
        && address < (CMB_FLASH_ADDRESS + CMB_FLASH_SECTOR_SIZE - CMB_LOG_STOARGE_SIZE))
    {
        address += CMB_LOG_STOARGE_SIZE;
    }
    address -= CMB_LOG_STOARGE_SIZE;
    
    if (*(int *)address == 0xFFFFFFFF)
    {
        printf("cmbacktrace log not found\r\n");
        return;
    }

    while (count --)
    {
        printf("log address: 0x%08x\r\n", address);
        p = (char *)address;
        for (short i = 0; i < CMB_LOG_STOARGE_SIZE; i++)
        {
            if (*(p + i) == 0xFF)
            {
                break;
            }
            if (*(p + i) != 0)
            {
                printf("%c", *(p + i));
            }
        }
        printf("\r\n");
        address -= CMB_LOG_STOARGE_SIZE;
        if (address < CMB_FLASH_ADDRESS)
        {
            return;
        }
    }
}
UINSH_FUNCTION_EXPORT_CMD(cmbPrintLog, cmblog, print last cmb log);


/**
 * @brief 清空cmbacktrace数据
 * 
 */
void cmbRestore(void)
{
    Flash_Init();
    Flash_Erase(CMB_FLASH_SECTOR);
}
UINSH_FUNCTION_EXPORT_CMD(cmbRestore, cmbrestore, restore cmb log);

#endif /** CMB_LOG_USING_FLASH == 1 */
