#include <math.h>
#include "main.h"
#include "Tray.h"
#include "Task_Main.h"
#include "Uart.h"
#include "adc.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

uint32_t full_Encoder = 0;
uint32_t empty_Encoder = 0;
uint32_t ocTimer = 0;
uint32_t ocTimeout = 500-1;
long int start_encValue = 0;
uint32_t opTimecheck = 0;
uint32_t opTimevalue = 300;

uint16_t duty_period = 250;
uint16_t dutycycle = 10;

int tray_level = 100;
int tray_encoder = 3561;
int full_encoder = 3300;

float motor_current = 0;
float curr_limit[2] = {0, 6.0};

#define true        1
#define false       0    

uint8_t prev_door = 0;
uint8_t prev_LIM1 = 0;
uint8_t prev_LIM2 = 0;
uint8_t prev_ECT1 = 0;

char duty_change = 0;
char emg_exit = 0;              // emergency motor stop
char ocCheck_flag = 0;
char ocFlag = 0;
char OpErrFlag = 0;
char tray_prevEmpty = 0;
char tray_newlyfilled = 0;
char opCheck_Flag = 0;
char manual_mode = 0;
char prev_manualmode = 0;
char manual_direction = 0;
char newmanual_direction = 0;
char Z_RobotCheck_IR1 = 0, Z_RobotCheck_IR2 = 0;

#define TRAY_STEP_DECREASE    {\
                                                            if (prev_manualmode == MANUAL_RESET)\
                                                            {\
                                                                if (tray_level > 0)     {tray_level -= 1;}\
                                                            }\
                                                        }
#define TRAY_STEP_INCREASE    {\
                                                        if (prev_manualmode == MANUAL_RESET)\
                                                        {\
                                                            if (tray_level < 100)     {tray_level += 1;}\
                                                        }\
                                                    }\
    
void Op_Init(void)
{
    start_encValue = enc_A;
    OpErrFlag = 0;
    opCheck_Flag = 1;
    opTimecheck = opTimevalue;
}
char Check_OperationError(void)
{    
    if (start_encValue == enc_A)
    {
        if (opTimecheck == 0)
        {
            OpErrFlag = 1;
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
            MSG2("Operation timeout\n");
        }
    }
    else
        OpErrFlag = 0;
    return OpErrFlag;
}
char get_OperationStatus(void)
{
    return OpErrFlag;
}
void Op_Reset(void)
{
    opCheck_Flag = 0;
    OpErrFlag = 0;
    opTimecheck = opTimevalue;
}

char Check_DoorOpen(void)
{
    if (DOOR_IN == 1)       // Door Opened
    {
        //emg_exit = 0;
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
        MSG2("Door opened during operation\n");
        while(DOOR_IN == 1)
        {
            Buzzer_On(500);
            HAL_Delay(500);
        }
        MSG2("Door closed\n");
        HAL_Delay(2000);
        Reset_Duty();
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
        return 1;
    }
    return 0;
}

char Check_EmergencyExit(void)
{
    if (emg_exit)
    {
        //emg_exit = 0;
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
        MSG2("Emergency Exit\n");
        return 1;
    }
    return 0;
}

char Check_OverCurrent(void)
{
    if (motor_current  > curr_limit[1])
    {
        if (!ocCheck_flag)
        {
            ocCheck_flag = 1;
            MSG2("Overcurrent: %0.2f, start ocTimer: %d\n", motor_current, ocTimer);
        }
        if (ocTimer > ocTimeout)
        {
            ocCheck_flag = 0;
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
            ocFlag = 1;
            MSG2("Overcurrent: %0.2f, ocTimer: %d\n", motor_current, ocTimer);
            return 1;
        }
    }
    else
    {
        ocCheck_flag = 0;
        ocTimer = 0;
    }
    return 0;
}

char zRobotIR1_Check(void)
{
    uint16_t ra = 0;

    // check IR1 is blocked less than 2 seconds, its robot arm so no action,     if blocked more than 2 seconds, tray inserted, take action    
    for(ra = 0; ra < 200; ra++)
    {
        HAL_Delay(10);
        if (IR1_IN == 1)
            return 0;
    }
    return 1;
}

char zRobotIR2_Check(void)
{
    uint16_t ra = 0;

    // check IR2 is blocked less than 2 seconds, its robot arm so no action,     if blocked more than 2 seconds, tray inserted, take action    
    for(ra = 0; ra < 100; ra++)
    {
        HAL_Delay(10);
        if (IR2_IN == 0)
            return 0;
    }
    return 1;
}

void Manual_TrayUP(void)
{
    uint8_t curcnt = 0;
    
    Sol_On_Off( 1,  SOL_OFF, 400);
    
    MSG2("\nMANUAL MODE TRAY UP\n");
    Reset_Duty();
    MOTOR_UP;
    Op_Init();
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    TimeReset(100);
    while ( LIM1_IN == 1 )      // Bottom limit switch contact
    {
        motor_current = measure_current();
        
        if (LIM1_IN == 0){  MSG2("Manual mode Tray reached Top\n");  break; }
        
        if ( Check_OperationError() )   break;
        if ( Timeout() )
        {
            duty_change  = 1;
            TimeReset(100);
            if (curcnt++ >= 15){
                curcnt = 0;
                MSG2("mot cur: %0.2f A\n", motor_current);
            }
        }
        if (manual_direction == MANUAL_DOWN)   {    newmanual_direction = MANUAL_DOWN; break;   }
        if (manual_mode == MANUAL_RESET)        break;
        if (Check_DoorOpen())
        {
            manual_direction = MANUAL_UP;
            manual_mode = MANUAL_SET;
        }
        if (Check_OverCurrent() == 1)   // if Over current stop and end
        {
            goto OverCurrent;
        }
        if ( Check_EmergencyExit() )
        {
            goto Emg_Exit;
        }
    }
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

    enc_A = 0;      // reset encoder value    
    OverCurrent:
    Emg_Exit:
    HAL_Delay(400);
    
    Sol_On_Off( 1,  SOL_ON, 200);
}

// ir2_check = 0, move down upto LIM2
// ir2_check = 1, move down upto IR2
void Manual_TrayDown(char ir2_check)
{
    uint8_t curcnt = 0;
    
    Sol_On_Off( 1,  SOL_OFF, 400);
    
    MSG2("\nMANUAL MODE TRAY DOWN\n");
    Reset_Duty();
    MOTOR_DOWN;
    Op_Init();
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    TimeReset(100);
    
    while(LIM2_IN == 1)      // Bottom limit switch contact
    {
        motor_current = measure_current();
        
        if ( ir2_check == 1 && IR2_IN == 1)
        {                    
            MSG2("Tray reached below IR2\n\n");
            break;
        }
        if (LIM2_IN == 0){  MSG2("Manual mode Tray reached Bottom\n"); break;   }
        
        if ( Check_OperationError() )   break;
        if ( Timeout() )
        {
            duty_change  = 1;
            TimeReset(100);
            if (curcnt++ >= 15){
                curcnt = 0;
                MSG2("mot cur: %0.2f A\n", motor_current);
            }
        }
        if (manual_direction == MANUAL_UP)       {    newmanual_direction = MANUAL_UP; break;   }
        if (manual_mode == MANUAL_RESET)   {MSG2("manual off \n");     break;  }
        if (Check_DoorOpen())
        {
            manual_direction = MANUAL_DOWN;
            manual_mode = MANUAL_SET;
        }
        if (Check_OverCurrent() == 1)   // if Over current stop and end
        {
            goto OverCurrent;
        }
        if ( Check_EmergencyExit() )
        {
            goto Emg_Exit;
        }
    }
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);    

    enc_A = 0;      // reset encoder value    
    OverCurrent:
    Emg_Exit:
    HAL_Delay(400);
    
    Sol_On_Off( 1,  SOL_ON, 200);
}

void Manual_ModeEnd(void)
{
    uint8_t curcnt = 0;
    MSG2("Manual mode end\n");
    if (ECT1_IN == 1)       // Tray sensor is empty, go to LIM1 or LIM2 according to Tray name
    {
        if (tray == FEED_TRAY)
        {
            Manual_TrayDown(0);
        }
        else if ( tray == FEED_TRAY || tray == OK_TRAY || tray == NG_TRAY || tray == EMPTY_TRAY )
        {
            Manual_TrayUP();
        }
    }
    else        // tray sensor is not empty, go to IR position
    {
        Sol_On_Off( 1,  SOL_OFF, 400);
        
        if (IR2_IN == 0 && LIM2_IN == 1)        // IR2 is broken, go down as long as IR2 s blocked
        {
            Reset_Duty();
            MOTOR_DOWN;
            Op_Init();
            HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
            TimeReset(100);
            while(IR2_IN == 0)      // while IR is unbroken
            {
                motor_current = measure_current();
                if (LIM2_IN == 0)
                {
                    MSG2("Manual Mode end reached bottom: %d\n\n", tray_level);
                    break;
                }
                if ( Check_OperationError() )   goto OpErr_Exit;
                if (Timeout())
                {                
                    duty_change  = 1;
                    TimeReset(100);
                    if (curcnt++ >= 15){
                        curcnt = 0;
                        MSG2("mot cur: %0.2f A\n", motor_current);
                    }
                }
                if (Check_DoorOpen())
                {
                    manual_direction = MANUAL_DOWN;                    
                }
                if (Check_OverCurrent() == 1)   // if Over current stop and end
                {
                    goto OverCurrent;
                }
                if ( Check_EmergencyExit() )
                {
                    goto Emg_Exit;
                }
            }
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
                                        
            TimeStop();
        }        
        if (IR2_IN == 1 && LIM1_IN == 1 && ECT1_IN == 0)        // IR2 is connected and LIM1 switch (Upper) = open
        {
            HAL_Delay(300);
            Reset_Duty();
            MOTOR_UP;
            Op_Init();
            HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
            TimeReset(100);
            while(IR2_IN == 1)      // while IR is unbroken
            {
                motor_current = measure_current();
                if (LIM1_IN == 0)
                {
                    MSG2("Manual End Tray reached Top: %d\n\n", tray_level);
                    break;
                }
                if ( Check_OperationError() )   goto OpErr_Exit;
                if (Timeout())
                {                
                    duty_change  = 1;
                    TimeReset(100);
                    if (curcnt++ >= 15){
                        curcnt = 0;
                        MSG2("mot cur: %0.2f A\n", motor_current);
                    }
                }
                if (Check_DoorOpen())
                {
                    manual_direction = MANUAL_UP;                    
                }
                if (Check_OverCurrent() == 1)   // if Over current stop and end
                {
                    goto OverCurrent;
                }
                if ( Check_EmergencyExit() )
                {
                    goto Emg_Exit;
                }
            }
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
                                        
            TimeStop();
        }
        OverCurrent:    
        Emg_Exit:
        OpErr_Exit:
        
        HAL_Delay(300);    Sol_On_Off( 1,  SOL_ON, 200);        
    }
}

void Tray_Init(void)
{
    uint8_t curcnt = 0;
    uint8_t tray_is_empty = false;
    
    switch (tray)
    {
        case FEED_TRAY:
            // if empty, go bottom
            if (ECT1_IN == 1)           // Tray Empty, DKT Sensor
            {
                tray_is_empty = true;
            }
            if (ECT1_IN == 1 && LIM2_IN == 1)           // Tray Empty, DKT Sensor
            {
                tray_is_empty = true;
                
                MSG2("tray empty\t Motor DOWN\n");
                HAL_Delay(500);
                Reset_Duty();
                MOTOR_DOWN;
                Op_Init();
                HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
                TimeReset(100);
                while(LIM2_IN == 1)      // Bottom limit switch contact
                {
                    motor_current = measure_current();
                    
                    if (LIM2_IN == 0) break;
                    
                    if ( Check_OperationError() )   break;
                    if ( Timeout() )
                    {
                        duty_change  = 1;
                        TimeReset(100);
                        if (curcnt++ >= 15){
                            curcnt = 0;
                            MSG2("mot cur: %0.2f A\n", motor_current);
                        }
                    }                    
                    Check_DoorOpen();
                    if (Check_OverCurrent() == 1)   // if Over current stop and end
                    {
                        goto OverCurrent;
                    }
                    if ( Check_EmergencyExit() )
                    {
                        goto Emg_Exit;
                    }
                }
                HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
                MSG2("Tray reached Bottom\n");
            
                //tray_level = 100;        // assume tray is filled here irrespective
                enc_A = 0;      // reset encoder value
                tray_prevEmpty = 1;
                
                //HAL_Delay(300);    Sol_On_Off( 1,  SOL_ON, 200);
            }
            break;        
        case OK_TRAY:            
        case NG_TRAY:            
        case EMPTY_TRAY:
            // if empty, go top
            if (ECT1_IN == 1)           // Tray Empty, DKT Sensor
            {
                if (tray == EMPTY_TRAY)
                {
                    tray_is_empty = true;
                }
            }
            if (ECT1_IN == 1 && LIM1_IN == 1)           // Tray Empty, (DKT Sensor), => MOVE UP
            {                
                if (tray == EMPTY_TRAY)
                {
                    tray_is_empty = true;
                }
                MSG2("tray empty\t Motor UP\n");
                HAL_Delay(500);
                MOTOR_UP;
                Reset_Duty();
                Op_Init();
                HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
                TimeReset(100);
                while( LIM1_IN == 1 && IR2_IN == 1 )        //while(LIM1_IN == 1)      // Bottom limit switch contact
                {
                    motor_current = measure_current();
                                        
                    if (IR2_IN == 0)
                    {                
                        break;
                    }
                    if ( Check_OperationError() )   break;
                    if (Timeout())
                    {                        
                        duty_change  = 1;
                        TimeReset(100);
                        if (curcnt++ >= 15){
                            curcnt = 0;
                            MSG2("mot cur: %0.2f A\n", motor_current);
                        }
                    }
                    Check_DoorOpen();
                    if (Check_OverCurrent() == 1)   // if Over current stop and end
                    {
                        goto OverCurrent;
                    }
                    if ( Check_EmergencyExit() )
                    {
                        goto Emg_Exit;
                    }
                }
                HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
                MSG2("Tray reached Top\n");

                tray_level = 0;        // assume tray is filled empty irrespective
                WR_TRAY_QTY(tray_level);
                
                //if (tray == EMPTY_TRAY){    HAL_Delay(300);    Sol_On_Off( 1,  SOL_ON, 200);    }
            }
            break;
        case NONE:            
            break;
    }
    if ( tray == FEED_TRAY || tray == EMPTY_TRAY )
    {
        if (tray_is_empty == false)
        {            
            HAL_Delay(300);    Sol_On_Off( 1,  SOL_ON, 200);
        }
    }
    return;
    
    OverCurrent:
    Emg_Exit:
    if ( tray == FEED_TRAY || tray == EMPTY_TRAY )
    {
        if (tray_is_empty == false)
        {            
            HAL_Delay(300);    Sol_On_Off( 1,  SOL_ON, 200);
        }
    }
    //Op_Reset();
}

void Feed_tray(void)
{
    uint8_t tray_is_empty = false;
    uint16_t t_adc5 = 0;
    int curcnt = 0;    
    
    if (emg_exit)   return;
    if (ECT1_IN == 1 && LIM2_IN == 0)
    {
        tray_level = 0;
        return;
    }
    
    if (prev_door != DOOR_IN)
        MSG2("Door Close\n");
    if (ECT1_IN == 1)           // Tray Empty, DKT Sensor
    {               
        Tray_Empty:
        if (LIM2_IN == 1)
        {
            tray_is_empty = true;
            Sol_On_Off( 1,  SOL_OFF, 100);
            
            MSG2("tray empty\t Motor DOWN\n");
            HAL_Delay(500);
            Reset_Duty();
            MOTOR_DOWN;
            Op_Init();
            HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
            TimeReset(100);
            while(LIM2_IN == 1)      // Bottom limit switch contact
            {
                motor_current = measure_current();
                if ( Check_OperationError() )   goto OpErr_Exit;
                if (Timeout())
                {                    
                    duty_change  = 1;
                    TimeReset(100);
                    if (curcnt++ >= 15){
                        curcnt = 0;
                        MSG2("mot cur: %0.2f A\n", motor_current);
                    }
                }
                Check_DoorOpen();
                if (Check_OverCurrent() == 1)   // if Over current stop and end
                {
                    goto OverCurrent;
                }
                if ( Check_EmergencyExit() )
                {
                    goto Emg_Exit;
                }
            }
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
            MSG2("Tray reached Bottom\n");

            tray_level = 0;
            //tray_level = 100;        // assume tray is filled here irrespective
            enc_A = 0;      // reset encoder value
            WR_TRAY_QTY(tray_level);
            TimeStop();
            
            //HAL_Delay(300);    Sol_On_Off( 1,  SOL_ON, 200);              // No need to SOL ON if tray is not there
        }
        tray_prevEmpty = 1;                
    }
    else if ( ECT1_IN == 0 )        // empty tray is filled
    {
        if (tray_prevEmpty == 1)
        {
            int percent = 0;
            
            tray_newlyfilled = 0;
            //percent = (tray_encoder - enc_A)/34;
            //tray_level = percent-1;
            //WR_TRAY_QTY(tray_level);
            MSG2("Tray refilled\n");
            //tray_level = 100;       // tray always filled 100% assumption
            tray_prevEmpty = 0;
            tray_newlyfilled = 1;            // this variable added to avoid tray = 100% repetition
        }
    }
    if (IR2_IN == 1)
    {
        long int t_encA = 0;
        Z_RobotCheck_IR2 = zRobotIR2_Check();
    
        if ( Z_RobotCheck_IR2 == 1 && IR2_IN == 1 && LIM1_IN == 1 && ECT1_IN == 0 )        // IR2 is connected and LIM1 switch (Upper) = open
        {
            Sol_On_Off( 1,  SOL_OFF, 300);
            
            MSG2("tray out\t Motor UP\n");            
            t_encA = enc_A;
            Reset_Duty();
            MOTOR_UP;
            Op_Init();
            HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
            TimeReset(100);
            while(IR2_IN == 1)      // while IR is unbroken
            {
                motor_current = measure_current();
                if (IR2_IN == 0)
                {
                    break;
                }
                if (ECT1_IN == 1)       // Tray Empty when moving up
                {
                    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
                    HAL_Delay(200);
                    MSG2("Tray empty direction changed\t");
                    return;
                    goto Abrupt_Stop;
                }
                if (LIM1_IN == 0)
                {
                    MSG2("Tray reached Top: %d\n\n", tray_level);
                    break;
                }
                if ( Check_OperationError() )   goto OpErr_Exit;
                if (Timeout())
                {                
                    duty_change  = 1;
                    TimeReset(100);
                    if (curcnt++ >= 15){
                        curcnt = 0;
                        MSG2("mot cur: %0.2f A\n", motor_current);
                    }
                }
                Check_DoorOpen();
                if (Check_OverCurrent() == 1)   // if Over current stop and end
                {
                    goto OverCurrent;
                }
                if ( Check_EmergencyExit() )
                {
                    goto Emg_Exit;
                }
            }
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
            if (tray_newlyfilled == 1)        // Refilled tray is moved to IR2, donot decrease.
            {
                int percent = 0;
                
                tray_newlyfilled = 0;
                percent = (tray_encoder - enc_A)/34;
                tray_level = percent-1;
            }
            else
            {
                float fl = fabs((float)t_encA - (float)enc_A);
                if  ( fl > 30 )  TRAY_STEP_DECREASE;
                MSG2("t_encA: %d \t enc_A: %d\t diff: %0.1f\n ", t_encA, enc_A, fl);
            }
            
            MSG2("Tray reached IR2: %d\n\n", tray_level);        
            WR_TRAY_QTY(tray_level);
            TimeStop();
            
            HAL_Delay(300);    Sol_On_Off( 1,  SOL_ON, 200);
            
        }
        else if (LIM1_IN == 0)          // Upper limit reached, but not necessarily empty
        {
            // CHECK WHETHER TO COMMENT OR UNCOMNMENT THIS SECTION IN DETAIL
            HAL_Delay(200);     // move motor down

            Sol_On_Off( 1,  SOL_OFF, 50);
            
            MSG2("Tray reached TOP\t Motor DOWN\n");
            HAL_Delay(500);
            Reset_Duty();
            MOTOR_DOWN;
            TimeReset(100);
            Op_Init();
            HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
            while (LIM2_IN == 1)      // Bottom limit switch contact
            {
                motor_current = measure_current();
                if ( Check_OperationError() )   goto OpErr_Exit;
                if (Timeout())
                {                
                    duty_change  = 1;
                    TimeReset(100);
                    if (curcnt++ >= 15){
                        curcnt = 0;
                        MSG2("mot cur: %0.2f A\n", motor_current);
                    }
                }
                Check_DoorOpen();
                if (Check_OverCurrent() == 1)   // if Over current stop and end
                {
                    goto OverCurrent;
                }
                if ( Check_EmergencyExit() )
                {
                    goto Emg_Exit;
                }
            }
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
            MSG2("Tray reached Bottom\n");

            //tray_level = 100;        // when tray moves from upper to lower end, assume tray is filled here irrespective
            WR_TRAY_QTY(tray_level);
            TimeStop();
            
            HAL_Delay(300);    Sol_On_Off( 1,  SOL_ON, 200);
        }
    }
    return;
    OverCurrent:    
    Emg_Exit:
    OpErr_Exit:
    Abrupt_Stop:
    if (tray_is_empty == false)
    {
        HAL_Delay(300);    Sol_On_Off( 1,  SOL_ON, 200);
    }
    //Op_Reset();
}

void Empty_tray(void)
{
    uint8_t tray_is_empty = false;
    uint16_t t_adc5 = 0;
    uint16_t curcnt = 0;    
    
    if (emg_exit)   return;
    if (ECT1_IN == 1 && LIM1_IN == 0)
    {
        tray_level = 0;
        return;
    }
    
    if (prev_door != DOOR_IN)
        MSG2("Door Close\n");
    if (IR2_IN == 1)
    {
        long int t_encA = 0;
        Z_RobotCheck_IR2 = zRobotIR2_Check();
    
        if ( Z_RobotCheck_IR2 == 1 && IR2_IN == 1 && IR1_IN == 1 && LIM1_IN == 1 && ECT1_IN == 0)        // IR2 is connected and LIM1 switch (Upper) = open, and DKT sensor  = 1 (tray present) => MOVE UP
        {
            if ( tray == EMPTY_TRAY )
            {
                Sol_On_Off( 1, SOL_OFF, 300);
            }
    
            MSG2("TRAY OUT\t Motor UP\n");
            t_encA = enc_A;
            Reset_Duty();
            MOTOR_UP;
            Op_Init();
            HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
            TimeReset(100);
            while (IR2_IN == 1)      // while IR2 is unbroken (tray removed)
            {
                motor_current = measure_current();
                if (IR2_IN == 0)
                {
                    //TRAY_STEP_DECREASE;
                    break;
                }
                if (LIM1_IN == 0)
                {
                    tray_level = 0;     // not necessarily, may be modified later
                    MSG2("Tray reached Top: %d\n\n", tray_level);
                    break;
                }
                if ( Check_OperationError() )   goto OpErr_Exit;
                if (Timeout())
                {                    
                    duty_change  = 1;
                    TimeReset(100);
                    if (curcnt++ >= 15){
                        curcnt = 0;
                        MSG2("mot cur: %0.2f A\n", motor_current);
                    }
                }
                Check_DoorOpen();
                if (Check_OverCurrent() == 1)   // if Over current stop and end
                {
                    goto OverCurrent;
                }
                if ( Check_EmergencyExit() )
                {
                    goto Emg_Exit;
                }
            }
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
            MSG2("Tray reached IR2: %d\n\n", tray_level);
            float fl = fabs((float)t_encA - (float)enc_A);
            if  ( fl > 30 )  TRAY_STEP_DECREASE;
            MSG2("t_encA: %d \t enc_A: %d\t diff: %0.1f\n ", t_encA, enc_A, fl);
            WR_TRAY_QTY(tray_level);
            TimeStop();
            
            if ( tray == EMPTY_TRAY )
            {
                HAL_Delay(300);    Sol_On_Off( 1,  SOL_ON, 200);
            }
        }
    }
    if (IR1_IN == 0)        // IR1 blocked
    {
        long int t_encA = 0;
        
        Z_RobotCheck_IR1 = zRobotIR1_Check();
        if ( Z_RobotCheck_IR1 == 1 && IR1_IN == 0 && IR2_IN == 0 && ECT1_IN == 0 )       // IR1 is broken (tray added) => MOVE DOWN
        {
            if ( tray == EMPTY_TRAY )
            {
                Sol_On_Off( 1,  SOL_OFF, 300);
            }
            
            t_encA = enc_A;
            MSG2("TRAY IN\t Motor DOWN\n");
            Reset_Duty();
            MOTOR_DOWN;
            Op_Init();
            HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
            TimeReset(100);
            while (1)      // while IR is broken
            {
                motor_current = measure_current();
                if (IR2_IN == 1)
                {                    
                    MSG2("Tray reached below IR2\n\n");
                    break;
                }
                if (LIM2_IN == 0)
                {
                    //tray_level = 100;
                    MSG2("Tray reached Bottom: %d\n\n", tray_level);
                    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
                    float fl = fabs((float)t_encA - (float)enc_A);
                    if  ( fl > 30 )  TRAY_STEP_INCREASE;
                    MSG2("t_encA: %d \t enc_A: %d\t diff: %0.1f\n ", t_encA, enc_A, fl);
                    WR_TRAY_QTY(tray_level);
                    TimeStop();
                    goto Op_Exit;
                    break;
                }
                if ( Check_OperationError() )   goto OpErr_Exit;
                if ( Timeout() )
                {                    
                    duty_change  = 1;
                    TimeReset(100);
                    if (curcnt++ >= 15){
                        curcnt = 0;
                        MSG2("mot cur: %0.2f A\n", motor_current);
                    }
                }
                Check_DoorOpen();
                if ( Check_OverCurrent() == 1 )   // if Over current stop and end
                {
                    goto OverCurrent;
                }
                if ( Check_EmergencyExit() )
                {
                    goto Emg_Exit;
                }
            }
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
            HAL_Delay(50);
            
            // move motor to level with IR2
            
            Reset_Duty();
            MOTOR_UP;
            Op_Init();
            HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
            TimeReset(100);
            while (1)      // while IR is unbroken
            {
                motor_current = measure_current();
                if (IR2_IN == 0)
                {                
                    break;
                }
                if (LIM1_IN == 0)
                {
                    MSG2("Tray reached Top\n\n");
                    break;
                }
                if ( Check_OperationError() )   goto OpErr_Exit;
                if ( Timeout() )
                {                    
                    duty_change  = 1;
                    TimeReset(100);
                    if (curcnt++ >= 15){
                        curcnt = 0;
                        MSG2("mot cur: %0.2f A\n", motor_current);
                    }
                }
                Check_DoorOpen();
                if (Check_OverCurrent() == 1)   // if Over current stop and end
                {
                    goto OverCurrent;
                }
                if ( Check_EmergencyExit() )
                {
                    goto Emg_Exit;
                }
            }
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);                    
            
            //LEVEL_IR2:
            float fl = fabs((float)t_encA - (float)enc_A);
            if  ( fl > 30 )  TRAY_STEP_INCREASE;
            WR_TRAY_QTY(tray_level);
            TimeStop();
            MSG2("t_encA: %d \t enc_A: %d\t diff: %0.1f\n ", t_encA, enc_A, fl);
            MSG2("Tray reached IR2: %d\n\n", tray_level);

            if ( tray == EMPTY_TRAY )
            {
                HAL_Delay(300);    Sol_On_Off( 1,  SOL_ON, 200);
            }
        }
    }
    else if (LIM1_IN == 0)          // Upper limit reached, do nothing
    {
    }
    Op_Exit:
    if (ECT1_IN == 1 && LIM1_IN == 1)           // Tray Empty, (DKT Sensor), => MOVE UP
    {
        tray_is_empty = true;
        if ( tray == EMPTY_TRAY )
        {
            Sol_On_Off( 1,  SOL_OFF, 200);
        }
                
        MSG2("tray empty\t Motor UP\n");
        HAL_Delay(500);
        MOTOR_UP;
        Reset_Duty();
        Op_Init();
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
        TimeReset(100);
        while( LIM1_IN == 1 && IR2_IN == 1 )      // Bottom limit switch contact
        {
            motor_current = measure_current();
            
            if (IR2_IN == 0)
            {                
                break;
            }
            if ( Check_OperationError() )   goto OpErr_Exit;
            if (Timeout())
            {                
                duty_change  = 1;
                TimeReset(100);
                if (curcnt++ >= 15){
                    curcnt = 0;
                    MSG2("mot cur: %0.2f A\n", motor_current);
                }
            }
            Check_DoorOpen();
            if (Check_OverCurrent() == 1)   // if Over current stop and end
            {
                goto OverCurrent;
            }
            if ( Check_EmergencyExit() )
            {
                goto Emg_Exit;
            }
        }
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
        MSG2("Tray reached Top\n");

        tray_level = 0;        // assume tray is filled empty irrespective
        WR_TRAY_QTY(tray_level);
        TimeStop();
    }
    return;
    OverCurrent:    
    Emg_Exit:
    OpErr_Exit:
    if ( tray == EMPTY_TRAY )
    {
        if (tray_is_empty == false)
        {
            HAL_Delay(300);    Sol_On_Off( 1,  SOL_ON, 200);
        }
    }
    //Op_Reset();
}