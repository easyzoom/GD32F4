#include "log.h"
#include "usart.h"
#include "shell.h"
/**
 * @brief 串口日志写
 * 
 * @param buffer 
 * @param len 
 */
void uart_log_write(char *buffer, short len)
{
    serial_transmit(USART2, (uint8_t *)buffer, len);
}

/**
 * @brief 调试串口日志
 * 
 */
static Log uartLog = {
    .write = uart_log_write,
    .active = true,
    .level = LOG_DEBUG
};


/**
 * @brief 初始化Log
 * 
 */
void log_init(void)
{
    logRegister(&uartLog);
}

/**
 * @brief 切换串口Log状态
 * 
 * @param shell shell
 */
void switch_uart_log(SHELL_TypeDef *shell)
{
    uartLog.active = uartLog.active ? 0 : 1;
    printf(CSI(31)"set log active: %s\r\n"CSI(39), uartLog.active ? "ENABLED": "DISABLED");
}


/**
 * @brief 切换日志级别
 * 
 * @param shell shell
 */
void switch_log_level(SHELL_TypeDef *shell)
{
    uartLog.level = uartLog.level < LOG_ALL ? (LogLevel)(uartLog.level + 1) : LOG_NONE;
    printf(CSI(31)"set log level: %s\r\n"CSI(39), logLevelText[uartLog.level]);
}
