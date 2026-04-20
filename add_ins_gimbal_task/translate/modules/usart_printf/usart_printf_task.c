//
// Created by LEGION on 25-8-20.
//

#include "usart_printf_task.h"
#include "../../translate/all_tou_h.h"

void usart_printf(const char *fmt,...);

// 串口打印
 void usart_printf(const char *fmt,...)
 {
     static uint8_t tx_buf[256] = {0};
     static va_list ap;
     static uint16_t len;
     va_start(ap, fmt);
     //return length of string
     //返回字符串长度
     len = vsprintf((char *)tx_buf, fmt, ap);
     va_end(ap);
     HAL_UART_Transmit_DMA(&huart6, tx_buf, len);
 }
