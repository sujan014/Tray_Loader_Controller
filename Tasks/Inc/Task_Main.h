#ifndef __TASK_MAIN_H__
#define __TASK_MAIN_H__

#include "main.h"

typedef enum
{
    FEED_TRAY = 0,
    OK_TRAY = 1,
    NG_TRAY = 2,
    EMPTY_TRAY = 3,
    NONE = 10
} Tray_TypeDef;

extern RTC_HandleTypeDef hrtc;

extern __IO uint64_t SystemTickCount_ms;
extern __IO uint64_t SystemBlink_ms;
extern __IO uint64_t msgTick_ms;
extern __IO uint64_t timer_tick;
extern uint32_t ocTimer;
extern long int enc_A;
extern uint32_t opTimecheck;
extern uint16_t dutycycle;
extern char reset;
extern char duty_change;
extern char ocCheck_flag;
extern char ocFlag;
extern char emg_exit;
extern char bootloader_Flag;
extern char opCheck_Flag;
extern Tray_TypeDef tray;
extern char manual_mode;
extern char prev_manualmode;
extern char manual_direction;
extern char newmanual_direction;

#define SOL_OFF     0
#define SOL_ON      1

#define BOOT_FLAG   0x424F4F54
#define APP_FLAG      0x4150504C

#define WR_TRAY_QTY(x)		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, x)
#define RD_TRAY_QTY		HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1)

#define WR_BOOT_FLAG(x)		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, x)
#define RD_BOOT_FLAG		HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2)

/******************************* System Task ********************************/
void Delay_us(uint16_t us);
void TimeReset(uint16_t time);
uint8_t Timeout(void);
void TimeStop(void);
void Buzzer_On(uint16_t time);

/******************************* Task ********************************/
void Sensor_Test(void);
void EXTI_Test(void);
void Task_Main(void);
void SendStatus(void);

/******************************* Timer ********************************/
void Reset_Duty(void);

/******************************* Sol control ********************************/
uint8_t Sol_On_Off(uint8_t no, uint8_t state, uint16_t time);
    
#endif
