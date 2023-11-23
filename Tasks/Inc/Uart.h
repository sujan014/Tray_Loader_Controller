#ifndef __UART_H__
#define __UART_H__

#include <stdint.h>

#include "main.h"

#define MAX_SIZE    200
#define RX_LEN      41

void Uart1_Printf(const char *fmt,...);
void Uart2_Printf(const char *fmt,...);
void DMA_Process_Data(void);

#endif
