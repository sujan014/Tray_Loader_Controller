# Tray_Loader_Controller
C Program for autonomous tray loader system.
This program runs on STM32F4XX MCU.
This is program to control Tray loader system autonomously.
The system consiste of switch sensors, IR sensors, DC motor control , UArt in/out.
The motor runs depending on the state of the sensors and command from Uart.
The motor running mechanism is through pulse generation.
The MCU continuously sends state of motors, sensors and tray state to Uart.
The tray state is empoty, filled or tray count.
The tray motor is programmed to run with safety of user in mind. 
When the door sensor is detected, motor stops and resumes when door is closed.

