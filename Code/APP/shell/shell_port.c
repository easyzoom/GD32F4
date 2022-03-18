#include "shell.h"
#include "gd32f4xx.h"
#include "usart.h"
#include "log.h"

SHELL_TypeDef shell;

#define SHELL_CFG_TASK_SHELL_STK_SIZE 512
static TaskHandle_t ShellTaskCreate_Handle = NULL;/* 创建任务句柄 */

extern void switch_uart_log(SHELL_TypeDef *shell);
extern void switch_log_level(SHELL_TypeDef *shell);

SHELL_KeyFunctionDef keyFuncList[] =
{
    {SHELL_KEY_CTRL_T,      switch_uart_log},
    {SHELL_KEY_CTRL_D,      switch_log_level}
};


/**
 * @brief 用户shell写
 * 
 * @param data 数据
 */
void user_shell_write(char data)
{
    serial_transmit(USART2, (uint8_t *)&data, 1);
}


/**
 * @brief 用户shell读
 * 
 * @param data 数据
 * @return char 状态
 */
signed char user_shell_read(char *data)
{
    serial_receive(USART2, (uint8_t *)data, 1);
    return 1;
}


/**
 * @brief 用户shell初始化
 * 
 */
void user_shell_init(void)
{
    shell.write = user_shell_write;
    shell.read = user_shell_read;
    shellInit(&shell);
    shellSetKeyFuncList(&shell, keyFuncList, sizeof(keyFuncList) / sizeof(SHELL_KeyFunctionDef));

     xTaskCreate((TaskFunction_t )shellTask,
                (const char*    )"App_Shell",
                (uint16_t       )SHELL_CFG_TASK_SHELL_STK_SIZE,
                (void*          )&shell,
                (UBaseType_t    )2,
                (TaskHandle_t*  )&ShellTaskCreate_Handle);
}

