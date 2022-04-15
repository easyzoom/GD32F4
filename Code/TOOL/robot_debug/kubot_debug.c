#include "kubot_debug.h"

int fputc(int ch, FILE *f)
{
    usart_data_transmit(UART3, (uint32_t)ch);
    while(!usart_flag_get(UART3, USART_FLAG_TBE));
    return ch;
}
