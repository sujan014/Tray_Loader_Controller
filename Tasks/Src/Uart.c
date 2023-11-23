#include "stm32f4xx_hal.h"
#include "Uart.h"
#include "main.h"
#include "Task_Main.h"

#include <stdarg.h>
#include <string.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

uint8_t rxBuffer[MAX_SIZE];
uint16_t rx_index = 0;
extern char emg_exit;

void Uart1_Printf(const char *fmt,...)
{
  char szBuf[1024] = {0,};
  va_list ap;
  va_start(ap,fmt);    
  vsprintf(szBuf,fmt,ap);
  va_end(ap);
  HAL_UART_Transmit(&huart1, (uint8_t *)&szBuf, strlen(szBuf), 10);
}

void Uart2_Printf(const char *fmt,...)
{
  char szBuf[1024] = {0,};
  va_list ap;
  va_start(ap,fmt);
  vsprintf(szBuf,fmt,ap);
  va_end(ap);
  HAL_UART_Transmit(&huart2, (uint8_t *)&szBuf, strlen(szBuf), 1000);
}

extern uint8_t rx_cnt;

char frame_match = 0;
void DMA_Process_Data(void)
{
    char* ptr = strtok((char*)rxBuffer, "@");    
    ptr = strtok(NULL, "@");
    
    frame_match = 0;
    
    if ( ptr[8] == 'I' && ptr[9] == 'N' && ptr[10] == 'S' && tray == FEED_TRAY)
    {
        frame_match = 1;
    }
    else if (  ptr[8] == 'P' && ptr[9] == 'A' && ptr[10] == 'S' && tray == OK_TRAY)
    {
        frame_match = 1;
    }
    else if (  ptr[8] == 'F' && ptr[9] == 'A' && ptr[10] == 'L' && tray == NG_TRAY)
    {
        frame_match = 1;
    }
    else if (  ptr[8] == 'E' && ptr[9] == 'M' && ptr[10] == 'P' && tray == EMPTY_TRAY)
    {
        frame_match = 1;
    }
    //else    {        HAL_UART_Transmit(&huart2, (uint8_t *)ptr, strlen(ptr), 100);    }
    if (frame_match)
    {
        ptr = strtok(NULL, "@");
        if ( ptr[0] == 'N' && ptr[1] == 'O' && ptr[2] == 'R' && ptr[3] == 'M' )
        {
            emg_exit = 0;
            manual_mode = manual_direction = MANUAL_RESET;
            BUZZER(0);
        }
        else if ( ptr[0] == 'S' && ptr[1] == 'T' && ptr[2] == 'O' && ptr[3] == 'P' )
        {
            emg_exit = 1;
        }
        else if ( ptr[0] == 'U' && ptr[1] == 'P' )
        {
            emg_exit = 0;
            manual_mode = MANUAL_SET;
            manual_direction = MANUAL_UP;
            BUZZER(0);
        }
        else if ( ptr[0] == 'D' && ptr[1] == 'O' && ptr[2] == 'W' && ptr[3] == 'N' )
        {
            emg_exit = 0;
            manual_mode = MANUAL_SET;
            manual_direction = MANUAL_DOWN;
            BUZZER(0);
        }
    }
    
    memset(rxBuffer, '\0', sizeof(rxBuffer));
    rx_index = 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    HAL_UART_Transmit(&huart2, rxBuffer, (strlen((const char*)rxBuffer)), 100);
    DMA_Process_Data();        
    HAL_UART_Receive_DMA (&huart1, rxBuffer, RX_LEN);
    
    /*
    UART1_rxBuffer[rc_cnt++] = rx_data;        
    HAL_UART_Receive_DMA (&huart1, &rx_data, 1);
    rc_cnt = (rc_cnt >= 10)?0:rc_cnt;
    if (rx_data == '\n')
    {
        tx_cnt = 0;
        HAL_UART_Transmit(&huart1, (uint8_t*)("Timeout reset\n"), 14, 100);
        memset(UART1_rxBuffer, '\0', sizeof(UART1_rxBuffer));
        rc_cnt = 0;
    }
    */
}
