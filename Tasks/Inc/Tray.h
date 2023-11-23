#ifndef __TRAY_H__
#define __TRAY_H__

#include "main.h"

extern uint8_t prev_door;
extern uint8_t prev_LIM1;
extern uint8_t prev_LIM2;
extern uint8_t prev_ECT1;
extern float motor_current;
extern float curr_limit[2];
extern int tray_level;

/******************************* Tray function ********************************/
char zRobotIR1_Check(void);
void Op_Init(void);
char Check_OperationError(void);
char get_OperationStatus(void);
void Op_Reset(void);
char Check_DoorOpen(void);
char Check_EmergencyExit(void);
char Check_OverCurrent(void);
void Manual_TrayUP(void);
void Manual_TrayDown(char ir2_check);
void Manual_ModeEnd(void);
void Tray_Init(void);
void Feed_tray(void);
void Empty_tray(void);
void OKNG_tray(void);
    
#endif
