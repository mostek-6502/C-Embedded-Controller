//*****************************************************************************
//
// Rick Faszold
//
// XEn, LLC
//
// Date: August 9th, 2017
//
// Originally, thise code was written in C++ for a different platform.  At the time of adopting to
// TI's CodeComposer, their overall support for C++ was limited.  Therefore, I converted the code to
// straight C.  Although the conversion was not desired, the overall support from TI was excellent.
//
// This module is used to obtain temperature readings from a Dallas Semiconductor DS2482-800 chip.
// The DS2482-800 chip is a multi-channel chip that allows access to many other 1-Wire devices from
// that channel.  For my purposes, I used one DS18B20 Temperature chip (Probe) per DS2482 channel.
// I felt as if the time spent determining changes or addressing by ROM code could get very overly
// complex, thus I kept it simple; but, a bit more expensive.
//
// Up to 16 DS18B20 temperature chips can be used AND be dynamically configured for resolution.
// The chip allows 9,10,11 and 12 bits of resolution.
// In this case, the first 8-bits are the whole number part of the temperature.  For instance,
// if the chip reports, 22.5 degrees (C), the first 8 bits are 22.
// This leaves 1,2,3 and 4 bits of resolution for the fractional part of the teperature.
//    1-bit  .0 or .5                                 (94ms)
//    2-bits .00 .25  .50  .75                        (188ms)
//    3-bits .125  .250  .375  .500  .625  .750  .875 (375ms)
//    4-bits .0625  .125...                           (750ms)
//    For the purposes of the USER, I rounded all of these to the nearest 10th.
//    From the perspctive of the downstream code, all of the calcs are based off of the whole number.
//
// For speed purposes, I run the chips at the lowest resolution 9-bits n order to keep
// the dealy short.  Since I use TI-RTOS, I use Semaphores to signal when processing is over.
// This allows the chip to move on to other things while the temperatures are being read.
//
// The calculations for running the pump motors are essentially based on the whole number portion of this.
// The PWM setting depend on the various differentials the pump motors.
// Many of the temperature redaings are not used to support any down stream equipment, they are merely there
// for tracking purposes.  For instance, Ground Temp, Outside Air Temp etc...
//
// Finally, as you can see the two DS2482-800 chips reside at the same physical location.
// This means that I'm using two I2C ports on the board instead of adressing each as designed.
// I did this in order to avoid potential power surges and not being able to control
// current to the chip.  Therefore, it was easier to ground the chip to 0 (or 0x18) and use two ports.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


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
#include "driverlib/i2c.h"
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

#include <sys/socket.h>

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
#include "Driver_Setup.h"
#include "UDP_Utilities.h"
#include "EEPROM_Utilities.h"
#include "Task_Setups.h"
#include "Console_Interface.h"
#include "Semaphore_Setup.h"



// Addresses & Channels
#define DS2482_DEVICE_1_ADDRESS             0x18   // Base Address for Device 1 - Add Bit at end for R/W 0011000?
#define DS2482_DEVICE_2_ADDRESS             0x19   // Base Address for Device 1 - Add Bit at end for R/W 0011001?
#define DS2482_MAX_CHANNELS                 8      //

// DS2482 Commands
#define DS2482_DEVICE_RESET                 0xF0   // Status Reg Set,  Wait 800, Can Be Executed At Anytime */
#define DS2482_SET_READ_POINTER_COMMAND     0xE1   // Any Set,         Wait 0  , Can Be Executed At Anytime */
#define DS2482_WRITE_CONFIGURATION          0xD2   // Config Reg Set,  Wait 0  , 1-Wire Clear (Status Register 1WB == 0) */
#define DS2482_CHANNEL_SELECT_COMMAND       0xC3   // Channel Reg Set, Wait 0  , 1-Wire Clear (Status Register 1WB == 0) */
#define DS2482_ONE_WIRE_RESET               0xB4   // Status Reg Set,  Wait 600, 1-Wire Clear (Status Register 1WB == 0) */
#define DS2482_ONE_WIRE_WRITE_BYTE          0xA5   // Status Reg Set,  Wait 600, 1-Wire Clear (Status Register 1WB == 0) */
#define DS2482_ONE_WIRE_READ_BYTE           0x96   // Status Reg Set,  Wait 600, 1-Wire Clear (Status Register 1WB == 0) */

// DS2482 Register Pointer Codes
#define DS2482_STATUS_REGISTER              0xF0
#define DS2482_DATA_REGISTER                0xE1
#define DS2482_CHANNEL_SELECTION_REGISTER	0xD2
#define DS2482_CONFIGURATION_REGISTER       0xC3




// DS18B20 Commands
#define DS18B20_READ_ROM                    0x33
#define DS18B20_SKIP_ROM                    0xCC
#define DS18B20_CONVERT_TEMP                0x44
#define DS18B20_READ_SCRATCHPAD             0xBE
#define DS18B20_WRITE_SCRATCHPAD            0x4E
#define DS18B20_COPY_SCRATCHPAD             0x48
#define DS18B20_READ_POWER_SUPPLY           0xB4

#define DS2482_CONFIGURATION				0xF0
#define ONE_WIRE_BUSY_FLAG					0x01
#define ONE_WIRE_PPD						0x02


#define I2C_MASTER_INTERNAL_TIMEOUT 		16384


//uint8_t a_ui8_Slave_Addresses[MAX_TEMPERATURE_PROBES]    =      {0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19};
uint8_t a_ui8_Slave_Addresses[MAX_TEMPERATURE_PROBES]      = {0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18};
uint8_t a_ui8_Write_Channel_Array[MAX_TEMPERATURE_PROBES]  = {0xF0, 0xE1, 0xD2, 0xC3, 0xB4, 0xA5, 0x96, 0x87, 0xF0, 0xE1, 0xD2, 0xC3, 0xB4, 0xA5, 0x96, 0x87};
uint8_t a_ui8_Verify_Channel_Array[MAX_TEMPERATURE_PROBES] = {0xB8, 0xB1, 0xAA, 0xA3, 0x9C, 0x95, 0x8E, 0x87, 0xB8, 0xB1, 0xAA, 0xA3, 0x9C, 0x95, 0x8E, 0x87};

I2C_Handle a_h_I2C_Handle[MAX_TEMPERATURE_PROBES]          = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

													  //9     10    11    12
uint32_t a_uiConfigResBits[MAX_TEMP_RESOLUTIONS]  	= {0x1F, 0x3F, 0x5F, 0x7F};
uint32_t a_uiResolutionMask[MAX_TEMP_RESOLUTIONS] 	= {0x08, 0x0C, 0x0E, 0x0F};
uint32_t g_uiResolutionIndex;

uint32_t g_uiTemperatureIndex;
uint32_t g_ui32SrcClock;
uint32_t g_uiLoggingFlag;
uint32_t g_uiPresensePulseDetected;



void Temperature_Set_Logging_Flag(uint32_t uiSetLoggingFlag)
{
	g_uiLoggingFlag = uiSetLoggingFlag;
}

void Temperature_Log_Message(char* szMsg, uint32_t uiLocation, uint32_t ui32ErrorCode, uint32_t ui32Extended)
{

	// due to the fact that we have this information AND the routine was not setup to take a specific index.
	// I put the extended error code in array 0 and will try to glean information from it later...
	g_s_Temperature_Telemetry[g_uiTemperatureIndex].uiErrorFlag = uiLocation;


	if (g_uiLoggingFlag == false) return;

	char szTemp[128] = "\0";

	Telemetry_Send_Output(szMsg);

	char szIndex[12];
	char szLocation[12];
	char szErrorCode[12];
	char szExtended[12];


	ltoa(g_uiTemperatureIndex, szIndex);
	ltoa(uiLocation, szLocation);
	ltoa(ui32ErrorCode, szErrorCode);
	ltoa(ui32Extended, szExtended);

	strcpy(szTemp, " Index: ");
	strcat(szTemp, szIndex);
	strcat(szTemp, "   Location: ");
	strcat(szTemp, szLocation);
	strcat(szTemp, "   Error: ");
	strcat(szTemp, szErrorCode);
	strcat(szTemp, "   Extended: ");
	strcat(szTemp, szExtended);
	strcat(szTemp, "\n");

	Telemetry_Send_Output(szTemp);

	return;
}

void Temperature_Log_Message_Generic(char* szMsg)
{
	if (g_uiLoggingFlag == false) return;

	Telemetry_Send_Output(szMsg);
}


void Temperature_Set_Resolution(uint32_t uiResolution)
{

	if (uiResolution == g_uiResolutionIndex)
	{
		// no changes required, get out.
		return;
	}

	if (uiResolution <= TEMP_RESOLUTION_BITS_12)
	{
		g_uiResolutionIndex = uiResolution;
	}
	else
	{
		g_uiResolutionIndex = TEMP_RESOLUTION_BITS_9;  // default to 9 bits
	}

	// Set Flag to Reset the Temperature Resolution
	uint32_t i;
	for (i = 0; i < MAX_TEMPERATURE_PROBES; i++)
    {
		// s_Temperature_Telemetry[i].uiTemperature_Resolution = g_uiResolutionIndex;  // kind of a waste since the resolution is always the same
    	g_s_Temperature_Telemetry[i].uiProbe_Configuration_Flag = false;
    }
}



void Temperature_Initialize(uint32_t uiResolution)
{
	//g_HighestWaitCounter = 0;
	g_uiLoggingFlag = false;


	Temperature_Set_Resolution(uiResolution);


	uint32_t i, j;

    for (i = 0; i < MAX_TEMPERATURE_PROBES; i++)
    {
    	if (i <= 7)
    	{
    		a_h_I2C_Handle[i] = g_I2C_Handle_0_7;
    	}
    	else
    	{
    		a_h_I2C_Handle[i] = g_I2C_Handle_8_15;
    	}
    }


	// Init The Reading Structure
    for (i = 0; i < MAX_TEMPERATURE_PROBES; i++)
    {
    	g_s_Temperature_Telemetry[i].uiROM_Flag = false;
    	g_s_Temperature_Telemetry[i].uiProbe_Configuration_Flag = false;
    	for (j = 0; j < 8; j++)
    	{
    		g_s_Temperature_Telemetry[i].ucROM[j] = 0;
    	}
    }

    return;
}


uint32_t I2C_Receive(uint8_t* p_ui8Data)
{
	// 6000's

	I2C_Transaction Temperature_Transaction;

	Temperature_Transaction.slaveAddress = (unsigned char) a_ui8_Slave_Addresses[g_uiTemperatureIndex];
	Temperature_Transaction.writeBuf = NULL;
	Temperature_Transaction.writeCount = 0;
	Temperature_Transaction.readBuf = p_ui8Data;
	Temperature_Transaction.readCount = 1;
	Temperature_Transaction.arg = NULL;


	//bool bTransferOK = I2C_transfer(g_I2C_Handle_0_7, &Temperature_Transaction); /* Perform I2C transfer
	bool bTransferOK = I2C_transfer(a_h_I2C_Handle[g_uiTemperatureIndex], &Temperature_Transaction);

	if (bTransferOK)
	{
		return I2C_MASTER_ERR_NONE;
	}


	// oops, badness...
	uint32_t uiReturn = I2C_control(a_h_I2C_Handle[g_uiTemperatureIndex], I2C_MASTER_ERR_NONE, 0);


	Temperature_Log_Message("I2C_Receive()::I2C_Control()", 6001, uiReturn, 0);


	return uiReturn;

}



uint32_t Clear_1_Wire_Busy_Status(uint8_t uiCommand1)
{
	// 2000's

	// The DS2482-800 Register Must Be Pre Set to the STATUS Register!!!
	uint32_t ui32ErrorCode = 0;

	uint8_t ui8Data = 0;

	uint32_t uiCounter = 0;

	char szLocation[] = "Clear_1_Wire_Busy_Status";

	// these are the commands that do NOT require a wait for the 1WBusy Flag to Clear..  we can get out.
	if ((uiCommand1 == DS2482_SET_READ_POINTER_COMMAND) ||
			(uiCommand1 == DS2482_WRITE_CONFIGURATION) ||
			(uiCommand1 == DS2482_CHANNEL_SELECT_COMMAND))
	{
		return I2C_MASTER_ERR_NONE;
	}


	// we could constantly check for the Status Register Busy Flag....
	SysCtlDelay(g_ui_0001_Second);

	for (uiCounter = 0; uiCounter < 600; uiCounter++)
	{
		// Get The Status of the 1 WIRE RESET - Already Pointing at the Status Register
		ui32ErrorCode = I2C_Receive(&ui8Data);
		if (ui32ErrorCode != 0)
		{
			Temperature_Log_Message(szLocation, 2001, ui32ErrorCode, 0);
			return ui32ErrorCode;
		}



		if ((ui8Data & ONE_WIRE_BUSY_FLAG) == 0)  // clears 1 Wire Busy Status - Then, Ready to Go!!!!
		{
			if (uiCommand1 == DS2482_ONE_WIRE_RESET)		// look at the PPD on a 1 Wire Rest...
			{
				if ((ui8Data & ONE_WIRE_PPD) == 0) 			// this means that a Presense Pulse Was Not Detected on the Probe
				{
					Temperature_Log_Message(szLocation, 2005, 77, 0);
					return 2005;
				}
			}

			return I2C_MASTER_ERR_NONE;
		}


		SysCtlDelay(g_ui_0001_Second);
	}


	// OK, we errored out!!!!
	ui32ErrorCode = I2C_MASTER_INTERNAL_TIMEOUT;

	Temperature_Log_Message(szLocation, 2010, ui32ErrorCode, 0);

	return ui32ErrorCode;
}





//sends an I2C command to the specified slave
uint32_t I2C_SendCommand(uint8_t uiCommand1, uint8_t uiCommand2)
{
	// 3000's

	uint8_t a_txBuffer[2];

	I2C_Transaction Temperature_Transaction;


	a_txBuffer[0] = uiCommand1;
	a_txBuffer[1] = uiCommand2;

	Temperature_Transaction.slaveAddress = (unsigned char) a_ui8_Slave_Addresses[g_uiTemperatureIndex];
	Temperature_Transaction.writeBuf = a_txBuffer;
	Temperature_Transaction.writeCount = 1;
	if (uiCommand2)
	{
		Temperature_Transaction.writeCount = 2;
	}
	Temperature_Transaction.readBuf = NULL;
	Temperature_Transaction.readCount = 0;
 	Temperature_Transaction.arg = NULL;

 	bool bTransferOK;


	//bTransferOK = I2C_transfer(g_I2C_Handle_0_7, &Temperature_Transaction); /* Perform I2C transfer */
 	bTransferOK = I2C_transfer(a_h_I2C_Handle[g_uiTemperatureIndex], &Temperature_Transaction);


	if (bTransferOK)
	{
		return I2C_MASTER_ERR_NONE;
	}


	// oops, badness...
	uint32_t uiReturn = I2C_control(a_h_I2C_Handle[g_uiTemperatureIndex], I2C_MASTER_ERR_NONE, 0);


	Temperature_Log_Message("Sent Command()::I2C_Control()", 3010, uiReturn, 0);


	return uiReturn;

}


uint32_t I2C_SendCommand_Generic(uint8_t uiCommand1, uint8_t uiCommand2)
{
	// 4000's

	uint32_t ui32ErrorCode = 0;

	char szLocation[] = "I2C_SendCommand_Generic";

	ui32ErrorCode = I2C_SendCommand(uiCommand1, uiCommand2);

	if (ui32ErrorCode == I2C_MASTER_ERR_NONE)
	{

		ui32ErrorCode = Clear_1_Wire_Busy_Status(uiCommand1);
		if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
		{
			Temperature_Log_Message(szLocation, 4000, ui32ErrorCode, 0);
			return ui32ErrorCode;
		}

		return I2C_MASTER_ERR_NONE;
	}

	if (ui32ErrorCode == I2C_MASTER_INT_NACK)
	{
		return I2C_MASTER_INT_NACK;
	}


	Temperature_Log_Message(szLocation, 4010, ui32ErrorCode, 0);

	return ui32ErrorCode;
}


uint32_t Set_DS18B20_Configuration(void)
{
	// 5000's

	uint32_t ui32ErrorCode = 0;

	char szLocation[] = "Set_DS18B20_Configuration";


	ui32ErrorCode = I2C_SendCommand_Generic(DS2482_ONE_WIRE_RESET, 0);
	if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
	{
		Temperature_Log_Message(szLocation, 5000, ui32ErrorCode, 0);
		return ui32ErrorCode;
	}


	ui32ErrorCode = I2C_SendCommand_Generic(DS2482_ONE_WIRE_WRITE_BYTE, DS18B20_SKIP_ROM);
	if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
	{
		Temperature_Log_Message(szLocation, 5005, ui32ErrorCode, 0);
		return ui32ErrorCode;
	}

	ui32ErrorCode = I2C_SendCommand_Generic(DS2482_ONE_WIRE_WRITE_BYTE, DS18B20_WRITE_SCRATCHPAD);
	if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
	{
		Temperature_Log_Message(szLocation, 5010, ui32ErrorCode, 0);
		return ui32ErrorCode;
	}

	ui32ErrorCode = I2C_SendCommand_Generic(DS2482_ONE_WIRE_WRITE_BYTE, 0x4A);  // J
	if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
	{
		Temperature_Log_Message(szLocation, 5015, ui32ErrorCode, 0);
		return ui32ErrorCode;
	}

	ui32ErrorCode = I2C_SendCommand_Generic(DS2482_ONE_WIRE_WRITE_BYTE, 0x43);  // C
	if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
	{
		Temperature_Log_Message(szLocation, 5020, ui32ErrorCode, 0);
		return ui32ErrorCode;
	}



	uint8_t ui8ConfigBit = a_uiConfigResBits[g_uiResolutionIndex];  // 9, 10, 11, 12


	ui32ErrorCode = I2C_SendCommand_Generic(DS2482_ONE_WIRE_WRITE_BYTE, ui8ConfigBit);  // New Temp Config
	if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
	{
		Temperature_Log_Message(szLocation, 5025, ui32ErrorCode, 0);
		return ui32ErrorCode;
	}


	ui32ErrorCode = I2C_SendCommand_Generic(DS2482_ONE_WIRE_RESET, 0);
	if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
	{
		Temperature_Log_Message(szLocation, 5030, ui32ErrorCode, 0);
		return ui32ErrorCode;
	}


	ui32ErrorCode = I2C_SendCommand_Generic(DS2482_ONE_WIRE_WRITE_BYTE, DS18B20_SKIP_ROM);
	if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
	{
		Temperature_Log_Message(szLocation, 5035, ui32ErrorCode, 0);
		return ui32ErrorCode;
	}



	ui32ErrorCode = I2C_SendCommand_Generic(DS2482_ONE_WIRE_WRITE_BYTE, DS18B20_COPY_SCRATCHPAD);  // New Temp Config
	if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
	{
		Temperature_Log_Message(szLocation, 5040, ui32ErrorCode, 0);
		return ui32ErrorCode;
	}



	// just wait to make sure it copied all the way!!!!  Give it double time...
	SysCtlDelay(g_ui_001_Second);
	SysCtlDelay(g_ui_001_Second);


	ui32ErrorCode = I2C_SendCommand_Generic(DS2482_ONE_WIRE_RESET, 0);
	if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
	{
		Temperature_Log_Message(szLocation, 5045, ui32ErrorCode, 0);
		return ui32ErrorCode;
	}



	uint8_t ui8TempData = 0;
	uint32_t uiIndex;
	uint32_t bKeepProcessing = true;
	for (uiIndex = 0; uiIndex < 10000 && bKeepProcessing; uiIndex++)  // you could read the busy flag instead...!
	{
		// Set Register to Read
		ui32ErrorCode = I2C_SendCommand_Generic(DS2482_SET_READ_POINTER_COMMAND, DS2482_DATA_REGISTER);
		if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
		{
			Temperature_Log_Message(szLocation, 5050, ui32ErrorCode, 0);
			return ui32ErrorCode;
		}


		// Get the data
		ui8TempData = 0;
		ui32ErrorCode = I2C_Receive(&ui8TempData);
		if (ui32ErrorCode != 0)
		{
			Temperature_Log_Message(szLocation, 5055, ui32ErrorCode, 0);
			return ui32ErrorCode;
		}

		if (ui8TempData)
		{
			bKeepProcessing = false;
		}
	}


	if (bKeepProcessing == true)  // this means no data was returned from the probe
	{
		ui32ErrorCode = 5110;
		Temperature_Log_Message(szLocation, 5060, ui32ErrorCode, 0);
	}


	return ui32ErrorCode;
}






uint32_t I2C_Read_Data(uint8_t* ui8Data)
{
	// 7000's

	uint32_t ui32ErrorCode = 0;

	char szLocation[] = "I2C_Read_Data";

	ui32ErrorCode = I2C_SendCommand_Generic(DS2482_ONE_WIRE_READ_BYTE, 0);
	if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
	{
		Temperature_Log_Message(szLocation, 7000, ui32ErrorCode, 0);
		return ui32ErrorCode;
	}

	// Set Register to Read
	ui32ErrorCode = I2C_SendCommand_Generic(DS2482_SET_READ_POINTER_COMMAND, DS2482_DATA_REGISTER);
	if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
	{
		Temperature_Log_Message(szLocation, 7020, ui32ErrorCode, 0);
   		return ui32ErrorCode;
	}


	// Get the data
	uint8_t ui8TempData = 0;
	ui32ErrorCode = I2C_Receive(&ui8TempData);
	if (ui32ErrorCode != 0)
	{
		Temperature_Log_Message(szLocation, 7030, ui32ErrorCode, 0);
   		return ui32ErrorCode;
	}


	*ui8Data = ui8TempData;

	return I2C_MASTER_ERR_NONE;

}




uint32_t I2C_Reset_DS2482_And_Configure(uint32_t uiResetChip)
{
	// 8000's

	uint8_t ui8Data = 0;

	uint32_t ui32ErrorCode;

	char szLocation[] = "I2C_Reset_DS2482_And_Configure";

	if (uiResetChip == false)  // anything to do here?
	{
		return I2C_MASTER_ERR_NONE;
	}

	ui32ErrorCode = I2C_SendCommand_Generic(DS2482_DEVICE_RESET, 0);
	if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
	{
		Temperature_Log_Message(szLocation, 8000, ui32ErrorCode, 0);
		return ui32ErrorCode;
	}


	// Write the Configuration
	ui32ErrorCode = I2C_SendCommand_Generic(DS2482_WRITE_CONFIGURATION, DS2482_CONFIGURATION);
	if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
	{
		Temperature_Log_Message(szLocation, 8005, ui32ErrorCode, 0);
   		return ui32ErrorCode;
	}


	// Get the Configuration Data
	ui32ErrorCode = I2C_Receive(&ui8Data);
	if (ui32ErrorCode != 0)
	{
		Temperature_Log_Message(szLocation, 8010, ui32ErrorCode, 0);
   		return ui32ErrorCode;
	}


	// Check The Data
	if (ui8Data != (DS2482_CONFIGURATION & 0x0F))
	{
		Temperature_Log_Message("I2C_Reset_DS2482_And_Configure: Invalid Configuration", 8015, ui32ErrorCode, ui8Data);
   		return 101;
	}


	return I2C_MASTER_ERR_NONE;
}


uint32_t I2C_Set_Channel_Select(uint32_t ui8_Write_Channel, uint32_t ui8_Verify_Channel)
{

	// 9000's

	uint8_t ui8Data = 0;

	uint32_t ui32ErrorCode;

	char szLocation[] = "I2C_Set_Channel_Select";

	// Select The Channel to Use
	ui32ErrorCode = I2C_SendCommand_Generic(DS2482_CHANNEL_SELECT_COMMAND, ui8_Write_Channel);
	if (ui32ErrorCode != 0)
	{
		Temperature_Log_Message(szLocation, 9000, ui32ErrorCode, 0);
		return ui32ErrorCode;
	}


	// Get The Channel Data that is being pointed at
	ui32ErrorCode = I2C_Receive(&ui8Data);
	if (ui32ErrorCode != 0)
	{
		Temperature_Log_Message(szLocation, 9010, ui32ErrorCode, 0);
		return ui32ErrorCode;
	}


	if (ui8Data != ui8_Verify_Channel)
	{
		Temperature_Log_Message(szLocation, 9020, ui32ErrorCode, 0);
		return 103;
	}

	return I2C_MASTER_ERR_NONE;
}


uint32_t I2C_Calculate_ScratchPad_CRC(unsigned char* ucString, int iLen)
{
	// 10000

	//short int uCalcCRC = 0;

	uint8_t uCalcCRC = 0;

	int i = 0;
	for (i = 0; i < iLen; i++)
	{
		short int inbyte = ucString[i];
		int j = 0;
		for (j = 0; j < 8; j++)
		{
			short int mix = (uCalcCRC ^ inbyte) & 0x01;
			uCalcCRC >>= 1;

			if (mix)
			{
				uCalcCRC ^= 0x8C;
			}

			inbyte >>= 1;
		}
	}

	short int uCRC = 0;
	if (iLen == 7)  // ROM
	{
		uCRC = (short int) ucString[7];
	}
	else if (iLen == 8)  // ScratchPAD
	{
		uCRC = (short int) ucString[8];
	}

	if (uCRC == uCalcCRC)
	{
		return I2C_MASTER_ERR_NONE;
	}


	Temperature_Log_Message("I2C_Calculate_ScratchPad_CRC: CRC Error!", 10000, uCRC, uCalcCRC);

	return 105;
}



uint32_t I2C_Get_ROM_Codes(char* szROMCode)
{
	// 11000

	uint32_t ui32ErrorCode;

	uint32_t uiCounter = 0;

	uint8_t ui8Data = 0;

	char szLocation[] = "I2C_Get_ROM_Codes";

	// Reset the 1-Wire Device
	ui32ErrorCode = I2C_SendCommand_Generic(DS2482_ONE_WIRE_RESET, 0);
	if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
	{
		Temperature_Log_Message(szLocation, 11000, ui32ErrorCode, 0);
		return ui32ErrorCode;
	}


	// Get the Status Data
	ui32ErrorCode = I2C_Receive(&ui8Data);
	if (ui32ErrorCode != 0)
	{
		Temperature_Log_Message(szLocation, 11020, ui32ErrorCode, 0);
   		return ui32ErrorCode;
	}


	if ((ui8Data & 0x02) == 0)
	{
		Temperature_Log_Message("Get_ROM_Codes()->Status (NO PPD)", 11030, ui8Data, 0);
	}


	if ((ui8Data & 0x04) != 0)
	{
		Temperature_Log_Message("Get_ROM_Codes()->Status  (SHORT DETECTED)", 11040, ui8Data, 0);
	}


	// Set up for Read
	ui32ErrorCode = I2C_SendCommand_Generic(DS2482_ONE_WIRE_WRITE_BYTE, DS18B20_READ_ROM);
	if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
	{
		Temperature_Log_Message(szLocation, 11050, ui32ErrorCode, 0);
		return ui32ErrorCode;
	}


	for (uiCounter = 0; uiCounter < 8; uiCounter++)
	{
		ui32ErrorCode = I2C_Read_Data(&ui8Data);
		if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
		{
			Temperature_Log_Message(szLocation, 11070, ui32ErrorCode, 0);
			return ui32ErrorCode;
		}

		szROMCode[uiCounter] = (char) ui8Data;

		//Temperature_Log_Message("Going Nuts!", uiIndex, 11081);
	}


	ui32ErrorCode = I2C_Calculate_ScratchPad_CRC((unsigned char *) szROMCode, 7);
	if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
	{
		Temperature_Log_Message(szLocation, 11080, ui32ErrorCode, 0);
		return ui32ErrorCode;
	}


	return I2C_MASTER_ERR_NONE;

}



uint32_t I2C_Activate_The_Temperatures(void)
{
	// 13000

	uint32_t ui32ErrorCode;

	char szLocation[] = "I2C_Activate_The_Temperatures";

	// Reset the 1-Wire Device
	ui32ErrorCode = I2C_SendCommand_Generic(DS2482_ONE_WIRE_RESET, 0);
	if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
	{
		Temperature_Log_Message(szLocation, 13000, ui32ErrorCode, 0);
		return ui32ErrorCode;
	}


	ui32ErrorCode = I2C_SendCommand_Generic(DS2482_ONE_WIRE_WRITE_BYTE, DS18B20_SKIP_ROM);
	if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
	{
		Temperature_Log_Message(szLocation, 13020, ui32ErrorCode, 0);
		return ui32ErrorCode;
	}


	ui32ErrorCode = I2C_SendCommand_Generic(DS2482_ONE_WIRE_WRITE_BYTE, DS18B20_CONVERT_TEMP);
	if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
	{
		Temperature_Log_Message(szLocation, 13040, ui32ErrorCode, 0);
		return ui32ErrorCode;
	}

	return I2C_MASTER_ERR_NONE;
}


void I2C_Convert_Celcius_To_Farenheit(uint32_t uiTemperatureIndex)
{
	float fTemp;
	float fTemp2;

	fTemp = (float) g_s_Temperature_Telemetry[uiTemperatureIndex].ui8Whole_C + ((float) g_s_Temperature_Telemetry[uiTemperatureIndex].ui8Fraction_C / 10.0);

	if (g_s_Temperature_Telemetry[uiTemperatureIndex].ui8SignBit_C)
	{
		fTemp = fTemp *= -1;
	}

	// get the whole portion
	fTemp = (fTemp * 1.8) + 32;
	uint8_t ui8Temp_1 = (uint8_t) fTemp;
	g_s_Temperature_Telemetry[uiTemperatureIndex].ui8Whole_F = ui8Temp_1;

	// get the fraction
	fTemp2 = fTemp - (float) g_s_Temperature_Telemetry[uiTemperatureIndex].ui8Whole_F;
	fTemp2 *= 10;
	uint8_t ui8Temp_2 = (uint8_t) fTemp2;
	g_s_Temperature_Telemetry[uiTemperatureIndex].ui8Fraction_F = ui8Temp_2;

	// get the sign bit
	g_s_Temperature_Telemetry[uiTemperatureIndex].ui8SignBit_F = 0;
	if (fTemp < 0) g_s_Temperature_Telemetry[uiTemperatureIndex].ui8SignBit_F = 1;

}



uint32_t I2C_Retrieve_The_Temperatures(uint32_t uiTemperatureIndex)
{
	// 15000

	uint32_t ui32ErrorCode;

	uint8_t ui8Data;

	char szLocation[] = "Retrieve_The_Temperatures";

	// Reset the 1-Wire Device
	ui32ErrorCode = I2C_SendCommand_Generic(DS2482_ONE_WIRE_RESET, 0);
	if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
	{
		Temperature_Log_Message(szLocation, 15000, ui32ErrorCode, 0);
		return ui32ErrorCode;
	}


	// Get the Status Data
	ui32ErrorCode = I2C_Receive(&ui8Data);
	if (ui32ErrorCode != 0)
	{
		Temperature_Log_Message(szLocation, 15020, ui32ErrorCode, 0);
   		return ui32ErrorCode;
	}


	if ((ui8Data & 0x04) != 0)
	{
		Temperature_Log_Message(szLocation, 15030, ui32ErrorCode, 0);
		return 5;
	}


	ui32ErrorCode = I2C_SendCommand_Generic(DS2482_ONE_WIRE_WRITE_BYTE, DS18B20_SKIP_ROM);
	if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
	{
		Temperature_Log_Message(szLocation, 15040, ui32ErrorCode, 0);
		return ui32ErrorCode;
	}


	ui32ErrorCode = I2C_SendCommand_Generic(DS2482_ONE_WIRE_WRITE_BYTE, DS18B20_READ_SCRATCHPAD);
	if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
	{
		Temperature_Log_Message(szLocation, 15060, ui32ErrorCode, 0);
		return ui32ErrorCode;
	}


	uint8_t uiTempCode[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
	uint32_t uiIndex;

	for (uiIndex = 0; uiIndex < 9; uiIndex++)
	{
		ui32ErrorCode = I2C_Read_Data(&ui8Data);
		if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
		{
			Temperature_Log_Message(szLocation, 15080, ui32ErrorCode, 0);
			return ui32ErrorCode;
		}

		uiTempCode[uiIndex] = (uint32_t) ui8Data;
	}

	ui32ErrorCode = I2C_Calculate_ScratchPad_CRC((unsigned char *) uiTempCode, 8);
	if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
	{
		Temperature_Log_Message(szLocation, 15090, ui32ErrorCode, 0);
		return 107;
	}


	//char szMsg[128] = "\0";
	//sprintf(szMsg, "ScratchPad: %x-%x-%x-%x-%x-%x-%x-%x-%x \n", uiTempCode[0], uiTempCode[1], uiTempCode[2], uiTempCode[3], uiTempCode[4], uiTempCode[5], uiTempCode[6], uiTempCode[7], uiTempCode[8]);
	//UART_Logger(szMsg);

	uint8_t uiTempRes = uiTempCode[4];  // This is the Configuration Register
	uiTempRes = uiTempRes & 0x000000FF; // was FF
	// s_Temperature_Telemetry[uiTemperatureIndex].uiTemperature_Resolution = uiTempRes;


	// this is a check...
	// 1. Did the resolution match KNOWN and Possible Resolution Combinations?
	// 2. Did the global resolution Index match the index from array.  Does 0 = 0, 1 = 1, etc...
	uint32_t uiFlag = false;
	for (uiIndex = 0; uiIndex < MAX_TEMP_RESOLUTIONS; uiIndex++)
	{
		if (a_uiConfigResBits[uiIndex] == uiTempRes)  // 1.
		{
			if (g_uiResolutionIndex == uiIndex)  // 2.
			{
				uiFlag = true;
			}
		}
	}


	// let's see if the Config Register Returns the same thing that we sent it... :-)
	if (uiFlag == false)
	{
		ui32ErrorCode = 109;

		char szMsg[128];
		char szExpected[12];
		char szBits[12];
		char szReceived[12];

		ltoa(9 + g_uiResolutionIndex, szExpected);
		ltoa(a_uiConfigResBits[g_uiResolutionIndex], szBits);
		ltoa(uiTempRes, szReceived);

		strcpy(szMsg, "Invalid Resolution...Expected: ");
		strcat(szMsg, szExpected);
		strcpy(szMsg, "  Bits-[");
		strcat(szMsg, szBits);
		strcpy(szMsg, "]  Received: [");
		strcat(szMsg, szReceived);
		strcpy(szMsg, "]  \n");

		Temperature_Log_Message(szMsg, 15100, ui32ErrorCode, uiTempRes);
		return ui32ErrorCode;
	}


	//if (uiTempCode[2] != 0x4A)  // J
	//{
		//Temperature_Log_Message("Invalid Byte[2]-0x4A", ui32ErrorCode, (uint32_t) uiTempCode[2]);
		//return 111;
	//}

	//if (uiTempCode[3] != 0x43)  // C
	//{
		//Temperature_Log_Message("Invalid Byte[3]-0x43", ui32ErrorCode, (uint32_t) uiTempCode[3]);
		//return 112;
	//}



	//if (uiTemp > 200)
	//{
	//	ui8Data = 0;
	//}

	uint32_t uiTemp = uiTempCode[5];
	uiTemp = uiTemp & 0x000000FF;
	if (uiTemp != 0xFF)
	{
		Temperature_Log_Message("Invalid Byte[5]-0xFF", 15110, ui32ErrorCode, uiTemp);
		return 115;
	}


	uiTemp = uiTempCode[7];
	uiTemp = uiTemp & 0x000000FF;
	if (uiTemp != 0x10)
	{
		Temperature_Log_Message("Invalid Byte[7]-0x10", 15120, ui32ErrorCode, uiTemp);
		return 117;
	}


	int8_t i8Temperature;

	uint8_t ucFractionIndex = 0;
    uint8_t ucByte0 = uiTempCode[0];
    uint8_t ucByte1 = uiTempCode[1];


    //s_Temperature_Telemetry[uiTemperatureIndex].ui8Raw0 = ucByte0;
	//s_Temperature_Telemetry[uiTemperatureIndex].ui8Raw1 = ucByte1;


    // GET THE FRACTION
	// 9  bit = 1/2   or .5
	// 10 bit = 1/4   or .25
	// 11 bit = 1/8   or .125
	// 12 bit = 1/16  or .0625


    							// 0    1    2    3    4    5    6    7   8    9    A    B    C    D    E    F
                                // 0  .06  .12  .18  .25  .31  .38  .44  .5  .56  .62  .69  .75  .81  .88  .93
	uint8_t aFractionPos[16] =    {0,   1,   1,   2,   3,   3,   4,   4,  5,   6,   6,   7,   8,   8,   9,   9};

								// 0    1    2    3    4    5    6    7   8    9    A    B    C    D    E    F
								// 0  .93  .88  .81  .75  .69  .62  .56  .5  .44  .38  .31  .25  .18  .12  .06
	uint8_t aFractionNeg[16] =    {9,   9,   8,   7,   7,   6,   6,  5,   4,   4,   3,   3,   2,   1,   1,  0 };


	// the issue with 4 bit resolution is trying to represent .0625 with an integer, you can't or you
	// constantly have to recalculate it when you want to use it...
	//so, I am downshifting to .1, .2, .3, etc.  this still gives good resolution within a Whole Number


	// bTemp at this point has the resolution..
	// all is cleared away from the whole number portion
	// this is the index for the Fraction Lookup on the Table above!
	ucFractionIndex = ucByte0 & a_uiResolutionMask[g_uiResolutionIndex];




	// GET THE TEMPERATURE
	// Mask out the sign bit
	// uint8_t ucByteSignBit = ucByte1 & 0x08;


	ucByte0 = ucByte0 >> 4;   	// shifts bit7 through bit4 down to bit3 through bit0 - clears out the decimal portion
	ucByte0 = ucByte0 & 0x0F;    // clears out the upper bits

	ucByte1 = ucByte1 << 4;		// shifts bit0 through bit3 up to bit 7 through bit4 - clears out the sign bits
	ucByte1 = ucByte1 & 0xF0;   // clears out the lower bits

	i8Temperature = ucByte1 | ucByte0;


	uint8_t ui8_FractionValue = 0;


	// check the sign bit
	if ((i8Temperature & (int8_t) 0x80) == 0)
	{
		g_s_Temperature_Telemetry[uiTemperatureIndex].ui8SignBit_C = 0;
		g_s_Temperature_Telemetry[uiTemperatureIndex].ui8Whole_C = i8Temperature;

		ui8_FractionValue = aFractionPos[ucFractionIndex];
		g_s_Temperature_Telemetry[uiTemperatureIndex].ui8Fraction_C = ui8_FractionValue;
	}
	else
	{
		// flip everything around...!!!

		// the number is negative!!  Set Sign Bit.
		g_s_Temperature_Telemetry[uiTemperatureIndex].ui8SignBit_C = 1;

		// flip the bits of the Whole Number to get the negative number.
		g_s_Temperature_Telemetry[uiTemperatureIndex].ui8Whole_C = i8Temperature ^ 0xFF;

		ui8_FractionValue = aFractionNeg[ucFractionIndex];
		g_s_Temperature_Telemetry[uiTemperatureIndex].ui8Fraction_C = ui8_FractionValue;
	}



	I2C_Convert_Celcius_To_Farenheit(uiTemperatureIndex);


	return I2C_MASTER_ERR_NONE;
}


void Reset_ROM_Codes(void)
{
	g_s_Temperature_Telemetry[g_uiTemperatureIndex].uiROM_Flag = false;

	g_s_Temperature_Telemetry[g_uiTemperatureIndex].ucROM[0] = 0;
	g_s_Temperature_Telemetry[g_uiTemperatureIndex].ucROM[1] = 0;
	g_s_Temperature_Telemetry[g_uiTemperatureIndex].ucROM[2] = 0;
	g_s_Temperature_Telemetry[g_uiTemperatureIndex].ucROM[3] = 0;
	g_s_Temperature_Telemetry[g_uiTemperatureIndex].ucROM[4] = 0;
	g_s_Temperature_Telemetry[g_uiTemperatureIndex].ucROM[5] = 0;
	g_s_Temperature_Telemetry[g_uiTemperatureIndex].ucROM[6] = 0;
	g_s_Temperature_Telemetry[g_uiTemperatureIndex].ucROM[7] = 0;

}


void Reset_Temperatures(void)
{

	g_s_Temperature_Telemetry[g_uiTemperatureIndex].uiErrorFlag = 9999;

	g_s_Temperature_Telemetry[g_uiTemperatureIndex].ui8Whole_C = 0;
	g_s_Temperature_Telemetry[g_uiTemperatureIndex].ui8Fraction_C = 0;
	g_s_Temperature_Telemetry[g_uiTemperatureIndex].ui8SignBit_C = 0;

	g_s_Temperature_Telemetry[g_uiTemperatureIndex].ui8Whole_F = 0;
	g_s_Temperature_Telemetry[g_uiTemperatureIndex].ui8Fraction_F = 0;
	g_s_Temperature_Telemetry[g_uiTemperatureIndex].ui8SignBit_F = 0;
}

void Temperature_Initiate(void)
{

	// 16000
	uint32_t uiOK;
	uint32_t ui32ErrorCode;

	uint32_t a_ui32_Reset_Chip[MAX_TEMPERATURE_PROBES] = {true, false, false, false, false, false, false, false, true, false, false, false, false, false, false, false};


	char szLocation[] = "Temperature_Initiate";

	// set up the CHIP, The Configs, Get The ROMs and Ask the Probes to work on a Temp.


	for (g_uiTemperatureIndex = 0; g_uiTemperatureIndex < MAX_TEMPERATURE_PROBES; g_uiTemperatureIndex++)
	{
		Reset_Temperatures();

		if (g_uiTemperatureIndex == 1)
		{
			Clock_start(g_Clock_Temperature_OneShot_Handle);
		}



		uiOK = true;
		g_s_Temperature_Telemetry[g_uiTemperatureIndex].uiErrorFlag = I2C_MASTER_ERR_NONE;


		uiOK = true;

		ui32ErrorCode = I2C_Reset_DS2482_And_Configure(a_ui32_Reset_Chip[g_uiTemperatureIndex]);
		if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
		{
			Temperature_Log_Message(szLocation, 20, ui32ErrorCode, 0);
			Reset_ROM_Codes();
			uiOK = false;
		}


		if (uiOK)
		{
			ui32ErrorCode = I2C_Set_Channel_Select(a_ui8_Write_Channel_Array[g_uiTemperatureIndex], a_ui8_Verify_Channel_Array[g_uiTemperatureIndex]);
			if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
			{
				Temperature_Log_Message(szLocation, 30, ui32ErrorCode, 0);
				Reset_ROM_Codes();
				uiOK = false;
			}
		}


		if (uiOK)
		{
			if (g_s_Temperature_Telemetry[g_uiTemperatureIndex].uiROM_Flag == false)
			{
				char szROMCode[DS18B20_ROM_SIZE];
				ui32ErrorCode = I2C_Get_ROM_Codes(szROMCode);
				if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
				{
					Temperature_Log_Message(szLocation, 40, I2C_MASTER_ERR_NONE, 0);
					Reset_ROM_Codes();
					// uiOK is NOT set here because the ROM Codes are not critical to the temperature....
				}
				else
				{
					g_s_Temperature_Telemetry[g_uiTemperatureIndex].uiROM_Flag = true;

					g_s_Temperature_Telemetry[g_uiTemperatureIndex].ucROM[0] = szROMCode[0];
					g_s_Temperature_Telemetry[g_uiTemperatureIndex].ucROM[1] = szROMCode[1];
					g_s_Temperature_Telemetry[g_uiTemperatureIndex].ucROM[2] = szROMCode[2];
					g_s_Temperature_Telemetry[g_uiTemperatureIndex].ucROM[3] = szROMCode[3];
					g_s_Temperature_Telemetry[g_uiTemperatureIndex].ucROM[4] = szROMCode[4];
					g_s_Temperature_Telemetry[g_uiTemperatureIndex].ucROM[5] = szROMCode[5];
					g_s_Temperature_Telemetry[g_uiTemperatureIndex].ucROM[6] = szROMCode[6];
					g_s_Temperature_Telemetry[g_uiTemperatureIndex].ucROM[7] = szROMCode[7];
				}
			}
		}


		//UART_Logger("Past ROM\n");


		if (uiOK)
		{
			if (g_s_Temperature_Telemetry[g_uiTemperatureIndex].uiProbe_Configuration_Flag == false)
			{
				ui32ErrorCode = Set_DS18B20_Configuration();  // sets accuracy to 1 bit... much faster calculation!
				if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
				{
					Temperature_Log_Message(szLocation, 50, ui32ErrorCode, 0);
					Reset_ROM_Codes();
					uiOK = false;
				}
				else
				{
					g_s_Temperature_Telemetry[g_uiTemperatureIndex].uiProbe_Configuration_Flag = true;
				}
			}
		}



		if (uiOK)
		{
			ui32ErrorCode = I2C_Activate_The_Temperatures();
			if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
			{
				Temperature_Log_Message(szLocation, 60, ui32ErrorCode, 0);
				Reset_ROM_Codes();
				uiOK = false;
			}
		}

	}



	return;
}


void Temperature_Get(void)
{

	uint32_t uiOK;
	uint32_t ui32ErrorCode;


	char szLocation[] = "Temperature_Get";


	// Get The Temps
	for (g_uiTemperatureIndex = 0; g_uiTemperatureIndex < MAX_TEMPERATURE_PROBES; g_uiTemperatureIndex++)
	{
		uiOK = true;

		if (g_s_Temperature_Telemetry[g_uiTemperatureIndex].uiErrorFlag == I2C_MASTER_ERR_NONE)
		{

			ui32ErrorCode = I2C_Set_Channel_Select(a_ui8_Write_Channel_Array[g_uiTemperatureIndex], a_ui8_Verify_Channel_Array[g_uiTemperatureIndex]);
			if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
			{
				Temperature_Log_Message(szLocation, 85, ui32ErrorCode, 0);
				uiOK = false;
			}


			if (uiOK)
			{
				ui32ErrorCode = I2C_Retrieve_The_Temperatures(g_uiTemperatureIndex);
				if (ui32ErrorCode != I2C_MASTER_ERR_NONE)
				{
					Temperature_Log_Message(szLocation, 90, ui32ErrorCode, 0);
					uiOK = false;

				}
			}

		}
	}


	//Temperature_Log_Message("\n\nHighest Counter----------------------------------------------", g_HighestWaitCounter, g_HighestWaitCounter);

	return;
}





