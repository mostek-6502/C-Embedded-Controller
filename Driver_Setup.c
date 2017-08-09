// *****************************************************************************
//
// Rick Faszold
//
// XEn, LLC
//
// Date: August 9th, 2017
//
// This module merely sets up all of the drivers for the various chip functions uses.
//     Serial Output - Mainly for Testing
//     I2C - Temperature and ADC Chips
//     PWM - Pump Motors
//     Various Timers
//
//
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>


#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/adc.h"

#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/drivers/GPIO.h>
#include <ti/sysbios/knl/Clock.h>
//#include <ti/ndk/inc/usertype.h>


/* TI-RTOS Header files */
//#include <ti/drivers/EMAC.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/PWM.h>
// #include <ti/drivers/SDSPI.h>
// #include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>
// #include <ti/drivers/USBMSCHFatFs.h>
// #include <ti/drivers/Watchdog.h>
// #include <ti/drivers/WiFi.h>

/* Board Header file */
#include "Board.h"


#include "constants.h"
#include "globals.h"

#include "Telemetry.h"
#include "Console_Interface.h"
#include "Temperature_Interface.h"


// from main.c
void Clock_Temperature_Hold(void);

void Timer_Startup(void);
void Timer_One_Second_System(void);
void Timer_LED_Blink(void);


int Create_The_One_Shot_Temperature_Clock(void)
{


	uint32_t uiResolution = g_s_EEPROM_Data.uiTemperatureResolution - TEMP_RES_BASE_OFFSET;
	if (uiResolution > TEMP_RESOLUTION_BITS_12)
	{
		Telemetry_Send_Output_Value("Driver_Setup()::Create_The_One_Shot_Temperature_Clock  Valid Index: [0-3]... [0] Is Now Used.  Old Value: ", uiResolution);
		uiResolution = TEMP_RESOLUTION_BITS_9;
	}



	uint32_t uint32Temperature_Clock_Delay = g_ui_Temperature_Clock_Delay[uiResolution];


	if (g_Clock_Temperature_OneShot_Handle)  // does this already exist?
	{
		uint32_t uiClock_Timeout = Clock_getTimeout(g_Clock_Temperature_OneShot_Handle);

		if (uiClock_Timeout == uint32Temperature_Clock_Delay)  // clocks equal??
		{
			//Telemetry_Dump_Clock_Settings("Create_The_One_Shot_Temperature_Clock / Update", g_Clock_Temperature_OneShot_Handle);
			return 0;  // if so, get out...
		}

		// if not, change the clocks...
		Clock_delete(&g_Clock_Temperature_OneShot_Handle);
		g_Clock_Temperature_OneShot_Handle = NULL;
	}



	Error_Block eb;
 	Error_init(&eb);

 	Clock_Params clockParams;

 	Clock_Params_init(&clockParams);
 	clockParams.period = 0;
 	clockParams.startFlag = FALSE;
 	g_Clock_Temperature_OneShot_Handle = Clock_create( (Clock_FuncPtr) Clock_Temperature_Hold, uint32Temperature_Clock_Delay, &clockParams, &eb);
 	if (!g_Clock_Temperature_OneShot_Handle)
 	{
 		Telemetry_Send_Output("Driver_Setup()::Create_The_One_Shot_Temperature_Clock()  Error: Unable To Create!...\n");
 		return 10;
 	}

 	//Telemetry_Dump_Clock_Settings("Create_The_One_Shot_Temperature_Clock", g_Clock_Temperature_OneShot_Handle);


 	return 0;
}


/*
int Create_The_One_Shot_ADC_Clock(void)
{
	Error_Block eb;
 	Error_init(&eb);

 	Clock_Params clockParams;

 	Clock_Params_init(&clockParams);
 	clockParams.period = 0;
 	clockParams.startFlag = FALSE;
 	g_Clock_ADC_OneShot_Handle = Clock_create( (Clock_FuncPtr) Clock_ADC_Hold, 96, &clockParams, &eb);
 	if (!g_Clock_ADC_OneShot_Handle)
 	{
 		Telemetry_Send_Output("Driver_Setup()::Create_The_One_Shot_ADC_Clock()  Error: Unable To Create!...\n");
 		return 10;
 	}

 	return NO_ERRORS;
}
*/


int Create_Timer_One_Second_System(void)
{
	Error_Block eb;
 	Error_init(&eb);

 	Clock_Params clockParams;

 	Clock_Params_init(&clockParams);
 	clockParams.period = 1000;
 	clockParams.startFlag = TRUE;
 	g_Clock_One_Second_Timer_Handle = Clock_create( (Clock_FuncPtr) Timer_One_Second_System, 1, &clockParams, &eb);
 	if (!g_Clock_One_Second_Timer_Handle)
 	{
 		Telemetry_Send_Output("Driver_Setup()::Create_Timer_One_Second_System()  Error: Unable To Create!...\n");
 		return 10;
 	}

 	return NO_ERRORS;
}


int Create_Timer_LED_Blink(void)
{
	Error_Block eb;
 	Error_init(&eb);

 	Clock_Params clockParams;

 	Clock_Params_init(&clockParams);
 	clockParams.period = 250;
 	clockParams.startFlag = TRUE;
 	g_Clock_One_Second_Timer_Handle = Clock_create( (Clock_FuncPtr) Timer_LED_Blink, 1, &clockParams, &eb);
 	if (!g_Clock_One_Second_Timer_Handle)
 	{
 		Telemetry_Send_Output("Driver_Setup()::Create_Timer_LED_Blink()  Error: Unable To Create!...\n");
 		return 10;
 	}

 	return NO_ERRORS;
}




void ReadCallBack(void)
{

	System_printf("Read Call Back!");
	System_flush();


}



int Driver_Setup(void)
{

	Telemetry_Send_Output("Driver_Setup()  Begin...\n");


	// Setup UARTS for Reporting and Console


	UART_Params      UART_Parameters;


	UART_Params_init(&UART_Parameters);
	UART_Parameters.baudRate  = 57600;
	UART_Parameters.dataLength = UART_LEN_8;
	UART_Parameters.stopBits = UART_STOP_ONE;
	UART_Parameters.parityType =UART_PAR_NONE;
	UART_Parameters.writeDataMode = UART_DATA_BINARY;
	UART_Parameters.readDataMode = UART_DATA_BINARY;
	UART_Parameters.readReturnMode = UART_RETURN_NEWLINE;
	UART_Parameters.readEcho = UART_ECHO_OFF;  // Console is Off
	UART_Parameters.readMode = UART_MODE_CALLBACK;
	UART_Parameters.writeMode = UART_MODE_BLOCKING;
	UART_Parameters.readCallback = (UART_Callback) &UART_CallBack_Read_Console;
	UART_Parameters.writeCallback = 0;

	g_UART_Handle_Console = UART_open(UART_CONSOLE, &UART_Parameters);
 	if (!g_UART_Handle_Console)
 	{
 		Telemetry_Send_Output("Driver_Setup()  Exit on UART Console Setup...\n");
 		return 10;
 	}


	UART_Parameters.readMode = UART_MODE_BLOCKING;
	UART_Parameters.writeMode = UART_MODE_BLOCKING;
	UART_Parameters.readCallback = 0;
	UART_Parameters.writeCallback = 0;
 	UART_Parameters.readEcho = UART_ECHO_OFF;  // Logger is Off
 	g_UART_Handle_Logger = UART_open(UART_LOGGER, &UART_Parameters);
 	if (!g_UART_Handle_Logger)
 	{
 		Telemetry_Send_Output("Driver_Setup()  Exit on UART Logger Setup...\n");
 		return 20;
 	}

 	/* TEST!!!!!!!!!!!!!!!! */
	UART_Parameters.readMode = UART_MODE_BLOCKING;
	UART_Parameters.writeMode = UART_MODE_BLOCKING;
	UART_Parameters.readCallback = 0;
	UART_Parameters.writeCallback = 0;
 	UART_Parameters.readEcho = UART_ECHO_OFF;  // Logger is Off
 	g_UART_Handle_Test_Logger = UART_open(UART_TEST, &UART_Parameters);
 	if (!g_UART_Handle_Test_Logger)
 	{
 		Telemetry_Send_Output("Driver_Setup()  Exit on UART TEST Setup...\n");
 		return 30;
 	}




 	// Setup I2C...
	I2C_Params I2C_Parameters;


 	I2C_Params_init(&I2C_Parameters);
 	I2C_Parameters.transferMode  = I2C_MODE_BLOCKING;
 	I2C_Parameters.transferCallbackFxn = NULL;   // Temperature_CallBack_Handler;
 	I2C_Parameters.bitRate = I2C_100kHz;
 	g_I2C_Handle_0_7 = I2C_open(Board_I2C0, &I2C_Parameters);
 	if (!g_I2C_Handle_0_7)
 	{
 		Telemetry_Send_Output("Driver_Setup()  Exit on I2C (Temperatures... 0-7) Setup...\n");
 		return 40;
 	}


 	I2C_Parameters.transferMode  = I2C_MODE_BLOCKING;
 	I2C_Parameters.transferCallbackFxn = NULL;   // Temperature_CallBack_Handler;
 	I2C_Parameters.bitRate = I2C_100kHz;
 	g_I2C_Handle_8_15 = I2C_open(Board_I2C1, &I2C_Parameters);
 	if (!g_I2C_Handle_8_15)
 	{
 		Telemetry_Send_Output("Driver_Setup()  Exit on I2C (Temperatures... 8-15) Setup...\n");
 		return 50;
 	}


 	I2C_Parameters.transferMode  = I2C_MODE_BLOCKING;
 	I2C_Parameters.transferCallbackFxn = NULL;   // ADC_CallBack_Handler;
 	I2C_Parameters.bitRate = I2C_100kHz;
 	g_I2C_ADC_Handle = I2C_open(Board_I2C2, &I2C_Parameters);
 	if (!g_I2C_ADC_Handle)
 	{
 		Telemetry_Send_Output("Driver_Setup()  Exit on I2C (ADC) Setup...\n");
 		return 60;
 	}



	int iRtn = Create_The_One_Shot_Temperature_Clock();
	if (iRtn)
	{
 		Telemetry_Send_Output("Driver_Setup()::Create_The_One_Shot_Temperature_Clock()   Error on Setup..\n");
 		return 70;
	}



	iRtn = Create_Timer_One_Second_System();
	if (iRtn)
	{
 		Telemetry_Send_Output("Driver_Setup()::Create_Timer_One_Second_System()   Error on Setup..\n");
 		return 80;
	}



	iRtn = Create_Timer_LED_Blink();
	if (iRtn)
	{
 		Telemetry_Send_Output("Driver_Setup()::Create_Timer_LED_Blink()   Error on Setup..\n");
 		return 90;
	}



	PWM_Params      params;

	PWM_Params_init(&params);
	params.period = PWM_PERIOD;					// Period in microseconds
	params.dutyMode = PWM_DUTY_TIME;		// Duty specified in microseconds
	g_PWM_Handle_Dish_Pump = PWM_open(Board_PWM3, &params);
	if (!g_PWM_Handle_Dish_Pump)
	{
		Telemetry_Send_Output("Driver_Setup()::Exit on PWM3 Setup (Dish Pump) ..\n");
		return 100;
	}
	PWM_setDuty(g_PWM_Handle_Dish_Pump, 0);


	params.period = PWM_PERIOD;					// Period in microseconds
	params.dutyMode = PWM_DUTY_TIME;		// Duty specified in microseconds
	g_PWM_Handle_Immediate_Pump = PWM_open(Board_PWM2, &params);
	if (!g_PWM_Handle_Immediate_Pump)
	{
		Telemetry_Send_Output("Driver_Setup()::Exit on PWM2 Setup (Immediate Pump)..\n");
		return 110;
	}
	PWM_setDuty(g_PWM_Handle_Immediate_Pump, 0);


	params.period = PWM_PERIOD;					// Period in microseconds
	params.dutyMode = PWM_DUTY_TIME;		// Duty specified in microseconds
	g_PWM_Handle_Hold_Pump = PWM_open(Board_PWM1, &params);
	if (!g_PWM_Handle_Hold_Pump)
	{
		Telemetry_Send_Output("Driver_Setup()::Exit on PWM1 Setup (Hold Pump)..\n");
		return 120;
	}
	PWM_setDuty(g_PWM_Handle_Hold_Pump, 0);


	params.period = PWM_PERIOD;					// Period in microseconds
	params.dutyMode = PWM_DUTY_TIME;		// Duty specified in microseconds
	g_PWM_Handle_AUX_Pump = PWM_open(Board_PWM0, &params);
	if (!g_PWM_Handle_AUX_Pump)
	{
		Telemetry_Send_Output("Driver_Setup()::Exit on PWM0 Setup (AUX Pump)..\n");
		return 130;
	}
	PWM_setDuty(g_PWM_Handle_AUX_Pump, 0);




 	Telemetry_Send_Output("Driver_Setup()  All Drivers Created Successfully!\n");



 	return 0;
}
