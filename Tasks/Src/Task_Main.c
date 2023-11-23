#include <string.h>
#include <stdlib.h>
#include "usb_host.h"
#include "Task_Main.h"
#include "main.h"
#include "Uart.h"
#include "adc.h"
#include "Tray.h"

__IO uint64_t SystemTickCount_ms = 0;
__IO uint64_t SystemBlink_ms = 0;
__IO uint64_t msgTick_ms = 0;
__IO uint64_t timer_tick = 0;

Tray_TypeDef ReadID(void);
void USB_Ready(void);

extern ADC_HandleTypeDef hadc1;


extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

long int encoder_cnt = 0, encoder_cntprev = 0;
long int enc_A = 0, enc_B = 0, enc_Z = 0;
long int prev_encA = 0, prev_encB = 0, prev_encZ = 0;

/*
#define FEED_TRAY       0
#define OK_TRAY         1
#define NG_TRAY         2
#define EMPTY_TRAY         3
*/

uint8_t* local_IP;
uint8_t local_IP_INS[4] =       {192,168,11,11};
uint8_t local_IP_PASS[4] =   {192,168,11,12};
uint8_t local_IP_FAIL[4] =      {192,168,11,13};
uint8_t local_IP_EMPTY[4] = {192,168,11,14};
uint8_t server_IP[4] =           {192,168,11,2};

uint8_t rx_cnt = 0, rx_limit_stop = 10;;
uint8_t rxdata = 0;
//uint8_t tray = FEED_TRAY;
Tray_TypeDef tray;
Tray_TypeDef prev_tray = NONE;

char reset = 0;
char bootloader_Flag = 0;

ApplicationTypeDef prev_Appli_state = APPLICATION_DISCONNECT;
extern ApplicationTypeDef Appli_state;

int readqty = 0;
uint32_t pingcounter = 0;

void BootCheck(void)
{
    if (bootloader_Flag == 1)
    {
        bootloader_Flag = 0;            
        WR_BOOT_FLAG(BOOT_FLAG);
        MSG2("Bootloader entered: %08x\n", RD_BOOT_FLAG);
        Buzzer_On(100);Buzzer_On(100);Buzzer_On(100);
        NVIC_SystemReset();
    }
}

void Task_Main(void)
{
    uint32_t t = 0;    
    MSG2("DEBUG PGM START\n");
    Buzzer_On(50); Buzzer_On(50); Buzzer_On(50); 
    tray_level = RD_TRAY_QTY;    
    //MSG2("tray_level: %d\n", readqty);
    tray = ReadID();
    HAL_Delay(1000);
    Tray_Init();
    
    while(1)
    {
        MX_USB_HOST_Process();
        USB_Ready();                
        
        tray = ReadID();
        if (prev_tray != tray)
            MSG2("Tray: %d\n", tray);
        prev_tray = tray;
        
        if (emg_exit  == 1)
        {            
            while(t++ >= 50)
            {                
                BUZZER_TG;
                LED2_TG;
                t = 0;
            }
        }
        else
        {
            if (t++ >= 500)     // NORM: 0.5 sec blink 
            {
                LED2_TG;
                t = 0;
            }
        }
        Delay_us(1000);
        //BootCheck();
                
        if (DOOR_IN == 0)       // Door closed
        {
            if (manual_mode == MANUAL_SET)      // Manual Mode ON
            {
                if (manual_direction == MANUAL_UP)
                {
                    Manual_TrayUP();
                }
                else if (manual_direction == MANUAL_DOWN)
                {
                    Manual_TrayDown(0);
                }
                manual_direction = MANUAL_RESET;
                if (newmanual_direction)
                {
                    manual_direction = newmanual_direction;
                    newmanual_direction = MANUAL_RESET;
                }
                prev_manualmode = MANUAL_SET;
            }
            if (manual_mode == MANUAL_RESET)             // Manual Mode OFF
            {
                if (prev_manualmode == MANUAL_SET)
                {
                    Manual_ModeEnd();
                }
                //if (rx_cnt > rx_limit_stop)     continue;
                switch (tray)
                {
                    case FEED_TRAY:
                        Feed_tray();
                        break;
                    
                    case OK_TRAY:                    
                    case NG_TRAY:                    
                    case EMPTY_TRAY:
                        Empty_tray();
                        break;
                    case NONE:
                        break;
                }
            }
        }
        else
        {
            MSG2("Door opened main loop\n");
            while(DOOR_IN == 1)
            {
                Buzzer_On(500);
                HAL_Delay(500);
            }
            MSG2("Door closed\n");
            manual_direction = MANUAL_RESET;
        }
        
        if (prev_LIM1 != LIM1_IN)
        {
            if (LIM1_IN == 1)
                MSG2("Limit switch 1 open\n");
            else
                MSG2("Limit switch 1 closed\n");
        }
        if (prev_LIM2 != LIM2_IN)
        {
            if (LIM2_IN == 1)
                MSG2("Limit switch 2 open\n");
            else
                MSG2("Limit switch 2 closed\n");
        }
        
        prev_manualmode = manual_mode;
        prev_door = DOOR_IN;
        prev_LIM1 = LIM1_IN;
        prev_LIM2 = LIM2_IN;
        /*  if (encoder_cnt != encoder_cntprev)        {    MSG1("encoder: %d\n", encoder_cnt);     }    */
        prev_encA = enc_A;

        if (ocFlag == 1)
        {
            char i = 0;
            
            while(i++ < 100)
            {
                HAL_Delay(100);
                LED2_TG;
                BUZZER_TG;
                //Buzzer_On(50);
            }
            ocFlag = 0;
        }        
    }    
}

void Sensor_Test(void)
{    
    if (DOOR_IN == 0)   MSG2("DOOR CLOSE\n");    
    if (LIM1_IN == 0)   MSG2("LIM1 IN\n");
    if (LIM2_IN == 0)   MSG2("LIM2 IN\n");
    if (IR1_IN == 0)   MSG2("IR1 IN\n");
    if (IR2_IN == 0)   MSG2("IR2 IN\n");
    if (ECT1_IN == 0)   MSG2("ETC1_IN\n");
    if (ECT2_IN == 0)   MSG2("ETC2_IN\n");
    if (PHOTO1_IN == 0)   MSG2("PHOTO1 IN\n");
    if (PHOTO2_IN == 0)   MSG2("PHOTO2 IN\n");
    if (CYLIN1_IN == 0)   MSG2("CYLIN1 IN\n");
    if (CYLIN2_IN == 0)   MSG2("CYLIN2 IN\n");
    if (encoder_cnt != encoder_cntprev)
    {
        MSG2("encoder: %d\n", encoder_cnt);            
    }
    
    if (DOOR_IN == 0)   MSG1("DOOR CLOSE\n");    
    if (LIM1_IN == 0)   MSG1("LIM1 IN\n");
    if (LIM2_IN == 0)   MSG1("LIM2 IN\n");
    if (IR1_IN == 0)   MSG1("IR1 IN\n");
    if (IR2_IN == 0)   MSG1("IR2 IN\n");
    if (ECT1_IN == 0)   MSG1("ETC1_IN\n");
    if (ECT2_IN == 0)   MSG1("ETC2_IN\n");
    if (PHOTO1_IN == 0)   MSG1("PHOTO1 IN\n");
    if (PHOTO2_IN == 0)   MSG1("PHOTO2 IN\n");
    if (CYLIN1_IN == 0)   MSG1("CYLIN1 IN\n");
    if (CYLIN2_IN == 0)   MSG1("CYLIN2 IN\n");
    if (encoder_cnt != encoder_cntprev)
    {
        MSG1("encoder: %d\n", encoder_cnt);            
    }
    encoder_cntprev = encoder_cnt;    
}
void EXTI_Test(void)
{
    if (prev_encA != enc_A)
    {
        MSG1("enc_A: %d\n", enc_A);
    }
    if (prev_encB != enc_B)
    {
        MSG1("enc_B: %d\n", enc_B);
    }
    if (prev_encZ != enc_Z)
    {
        MSG1("enc_Z: %d\n", enc_Z);
    }
    
    prev_encA = enc_A;
    prev_encB = enc_B;
    prev_encZ = enc_Z;
}

uint8_t *strptr;
char pingstr[100] = {'\0',};
char ipstr[20] = {'\0',};
char traystr[20] = {'\0',};
char state[20] = {'\0',};
char door[5] = {'\0',};
char lim1[5] = {'\0',};
char lim2[5] = {'\0',};
char traydetect[5] = {'\0',};
char tray_ht1[5] = {'\0',};
char tray_ht2[5] = {'\0',};
char percent[5] = {'\0',};

void SendStatus(void)
{           
    switch (tray)
    {
        case FEED_TRAY:
            sprintf(traystr, "KI-TRAY-INS");
            break;        
        case OK_TRAY:
            sprintf(traystr, "KI-TRAY-PAS");
            break;
        case NG_TRAY:
            sprintf(traystr, "KI-TRAY-FAL");
            break;
        case EMPTY_TRAY:
            sprintf(traystr, "KI-TRAY-EMP");
            break;
        case NONE:
            sprintf(traystr, "KI-TRAY-NON");
            break;
    }
    
    if (ocFlag == 0)    //if (curr_limit[0] <= motor_current && motor_current <= curr_limit[1])
    {
        if (get_OperationStatus() == 0)
            sprintf(state,"NORM");
        else
            sprintf(state,"OERR");
    }
    else
        sprintf(state,"OVCR");
            
    if (DOOR_IN == 1)
    {
        // door opened
        door[0] = '0';
        door[1] = '\0';
    }
    else
    {
        // door closed
        door[0] = '1';
        door[1] = '\0';
    }
        
    if (LIM1_IN == 0)
    {
        // limit sw1 detected
        lim1[0] = '1';
        lim1[1] = '\0';
    }
    else
    {
        // limit sw1 not detected
        lim1[0] = '0';
        lim1[1] = '\0';
    }
    if (LIM2_IN == 0)
    {
        // limit sw2 detected
        lim2[0] = '1';
        lim2[1] = '\0';
    }
    else
    {
        // limit sw2 not detected
        lim2[0] = '0';
        lim2[1] = '\0';
    }        
    
    if (ECT1_IN == 0)
    {
        // Tray present
        traydetect[0] = '1';
        traydetect[1] = '\0';
    }
    else
    {
        // Tray empty
        traydetect[0] = '0';
        traydetect[1] = '\0';
    }
    if (IR1_IN == 0)
    {
        // IR1 tray detected
        tray_ht1[0] = '1';
        tray_ht1[1] = '\0';
    }
    else
    {
        // IR1 tray not detected
        tray_ht1[0] = '0';
        tray_ht1[1] = '\0';
    }
    if (IR2_IN == 0)
    {
        // IR2 tray detected
        tray_ht2[0] = '1';
        tray_ht2[1] = '\0';
    }
    else
    {
        // IR2 tray not detected
        tray_ht2[0] = '0';
        tray_ht2[1] = '\0';
    }
    sprintf(percent,"%03d", tray_level);
    
    prev_ECT1 = ECT1_IN;
    
    rx_cnt++;
    pingcounter++;
    if (pingcounter >= 9999999)     pingcounter = 0;    
    sprintf(pingstr, "<STX@%s@%s@XXXX@%07d@%s@%s@%s@%s@%s@%s@%03d@EOF>", traystr, state, pingcounter, door, lim1, lim2, traydetect, tray_ht1, tray_ht2, tray_level);
    //MSG1("%s", pingstr);
    strptr = (uint8_t*)&pingstr;
    HAL_UART_Transmit(&huart1, strptr, strlen(pingstr), 5);        //HAL_UART_Transmit(&huart1, (uint8_t *)&pingstr, strlen(pingstr), 5);
    //MSG2(" enc_A: %d\n", enc_A);    
}

void Reset_OCTimer(void)
{
    ocTimer = 0;
    ocCheck_flag = ocFlag = 0;
}

void Reset_Duty(void)
{
    dutycycle = 10;                               
    int dutypulse = (htim2.Init.Period*dutycycle)/100 - 1;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, dutypulse);
    Reset_OCTimer();
}

void USB_Ready(void)
{
    if (prev_Appli_state != APPLICATION_START && Appli_state == APPLICATION_START)
    {
        MSG2("USB Ready\n");
        Buzzer_On(10);Buzzer_On(10);
    }
    else if (prev_Appli_state != APPLICATION_DISCONNECT && Appli_state == APPLICATION_DISCONNECT)
    {
        MSG2("USB removed\n");
        Buzzer_On(10);
    }    
    prev_Appli_state = Appli_state;    
}

uint8_t Sol_On_Off(uint8_t pin_no, uint8_t state, uint16_t time)
{
    if (1 <= pin_no && pin_no <= 4)
    {
        if (state == SOL_ON)
        {
            HAL_GPIO_WritePin(GPIOE, pin_no, GPIO_PIN_SET);
        } 
        else 
        {
            HAL_GPIO_WritePin(GPIOE, pin_no, GPIO_PIN_RESET);
        }
        if (time > 0){
            HAL_Delay(time);
        }
    }    
}

Tray_TypeDef ReadID(void)
{
    uint8_t b0 = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4);
    uint8_t b1 = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_6);
    uint8_t num = b1<<1|b0;
    
    switch (num)
    {
        case FEED_TRAY:
            local_IP = local_IP_INS;
            return FEED_TRAY;
            break;
        case OK_TRAY:
            local_IP = local_IP_PASS;
            return OK_TRAY;
            break;
        case NG_TRAY:
            local_IP = local_IP_FAIL;
            return NG_TRAY;
            break;
        case EMPTY_TRAY:
            local_IP = local_IP_EMPTY;
            return EMPTY_TRAY;
            break;
    }
    return NONE;
}

// Timeout start
void TimeReset(uint16_t time)
{
    timer_tick = time;
    reset = 1;
}

// check timeout in ms
uint8_t Timeout(void)       
{
    if (timer_tick == 0)
    {
        reset = 0;
        return 1;
    }
    return 0;
}

void TimeStop(void)
{
    reset = 0;
    timer_tick = 0;
}

void Buzzer_On(uint16_t time)
{
	if(time == 0)
	{
		BUZZER(1); 
	}
	else
	{
		BUZZER(1);      HAL_Delay(time); 
		BUZZER(0);      HAL_Delay(time); 
	}
}

void Delay_us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}
