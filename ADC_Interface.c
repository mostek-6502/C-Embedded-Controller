//*****************************************************************************
//
// Rick Faszold
//
// XEn, LLC
//
// Date: August 9th, 2017
//
// Originally, thise code was written in C++ for a different platform.  At the time of adopting to
// TI's CodeComposer, the overall support for C++ was limited.  Therefore, I converted the code to
// straight C.  Although the conversion was not desired, the overall support from TI was excellent.
//
// This module is the interface to a Linear Technology LTC2309 ADC chip using I2C for a
//     group of Voltage readings.
//
// Steady readings are difficult to obtain, therefore, I sample each channel MAX_ADC_SAMPLES times,
//     subtract out the high and the low and divide by MAX_ADC_SAMPLES - 2.
// Essentially, I'm trying to get a 'decent' steady state for each photo-resistor.
//
// The chip is setup for SINGLE ENDED, ODD Sign, UNIPOLAR with an I2C Adderess of 0.
//
// Notes:
// 	    Chip 1 - Dish Movement
// 	    Chip 2 - Motor Speed
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



int ADC_Get_Channel_Data(uint8_t ui8_ChipAddress, uint8_t ui8_Channel_Config, uint16_t *ui16_Voltage)
{
	// if we use sleep mode, there is a 200ms delay....  Right now, we are using nap mode....
	bool bTransferOK;

	I2C_Transaction ADC_Transaction;

	uint8_t ui8_ReadBuffer[2];
	uint8_t ui8_WriteBuffer[2];

	uint8_t ui8_Result_1;
	uint8_t ui8_Result_2;
	uint16_t uiTempVoltage;


	//char szMessage[80];
	//sprintf(szMessage, "Write - Chip Address: %x   Channel Selector: %x \n", ui8_ChipAddress, ui8_Channel_Config);
	//Telemetry_System_Printf(szMessage);



	ui8_WriteBuffer[0] = ui8_Channel_Config;
	ui8_WriteBuffer[1] = 0;


	ADC_Transaction.slaveAddress = (unsigned char) ui8_ChipAddress;
	ADC_Transaction.writeBuf = ui8_WriteBuffer;
	ADC_Transaction.writeCount = 1;
	ADC_Transaction.readBuf = NULL;
	ADC_Transaction.readCount = 0;
	ADC_Transaction.arg = NULL;

	bTransferOK = I2C_transfer(g_I2C_ADC_Handle, &ADC_Transaction); /* Perform I2C transfer */
	if (bTransferOK == false)
	{
		*ui16_Voltage = ERROR_VOLTAGE_VALUE;

		int iReturn = I2C_control(g_I2C_ADC_Handle, I2C_MASTER_ERR_NONE, 0);

		Telemetry_Send_Output_Value("ADC_Get_Channel_Data()::Write() ", iReturn);

		return iReturn;
	}

	// a little delay before reading the channel

	//sprintf(szMessage, "Read - Chip Address: %x   Channel Selector: %x \n", ui8_ChipAddress, ui8_Channel_Config);
	//Telemetry_System_Printf(szMessage);

	ui8_ReadBuffer[0] = 0;
	ui8_ReadBuffer[1] = 0;

	ADC_Transaction.slaveAddress = (unsigned char) ui8_ChipAddress;
	ADC_Transaction.writeBuf = ui8_WriteBuffer;
	ADC_Transaction.writeCount = 1;
	ADC_Transaction.readBuf = ui8_ReadBuffer;
	ADC_Transaction.readCount = 2;
	ADC_Transaction.arg = NULL;

	bTransferOK = I2C_transfer(g_I2C_ADC_Handle, &ADC_Transaction); /* Perform I2C transfer */
	if (bTransferOK == false)
	{
		*ui16_Voltage = ERROR_VOLTAGE_VALUE;

		int iReturn = I2C_control(g_I2C_ADC_Handle, I2C_MASTER_ERR_NONE, 0);

		Telemetry_Send_Output_Value("ADC_Get_Channel_Data()::Read() ", iReturn);

		return iReturn;
	}


	ui8_Result_1 = ui8_ReadBuffer[0];
	ui8_Result_2 = ui8_ReadBuffer[1];

	//if ((ui8_Result_1 == 0) && (ui8_Result_2 == 16))  // I'm dumping the LSB if that is the ONLY bit set
	//{
	//	ui8_Result_2 = 0;
	//}

	uiTempVoltage = ui8_Result_1 << 8;
	uiTempVoltage += ui8_Result_2;
	uiTempVoltage = uiTempVoltage & 0xFFF0;

	// normalize it to 12 bits...., take no chances and zero out various bit fields....
	uiTempVoltage = uiTempVoltage >> 4;
	uiTempVoltage = uiTempVoltage & 0x0FFF;

	*ui16_Voltage = uiTempVoltage;

	//sprintf(szMessage, "DATA - Voltage: %i \n", uiTempVoltage);
	//Telemetry_System_Printf(szMessage);

	return 0;

}



void ADC_Get_Data(void)
{
	int iRtn;

	// uint8_t a_ui8_ADC_Slave_Addresses[MAX_ADC_CHIPS]			= { 0x08, 0x0A, 0x1A };

	// this line needs to change right here to get the other chips online...
	uint8_t a_ui8_ADC_Chip_Slave_Addresses[MAX_ADC_CHIPS]			= { 0x08 };
	uint32_t a_ui32_Max_Channels_To_Use[MAX_ADC_CHIPS]				= { 8 };

																    // MSB is Channel Select, LSB (0x?8) is Configuration
	uint8_t a_ui8_Channel_Select[MAX_ADC_CHANNELS]					= { 0x88, 0xC8, 0x98, 0xD8, 0xA8, 0xE8, 0xB8, 0xF8 };


	uint32_t uiChipInUse;
	uint32_t uiMaxChips;
	uint8_t ui8_Chips_Addresses;

	uint32_t uiChannel_Index;

	uint32_t uiMaxChannels;
	uint8_t ui8_Channel_Selector;


	uint16_t ui16_Voltage = ERROR_VOLTAGE_VALUE;

	//char szMessage[128];

	// ths routine is 'programming' each chip to start data gathering
	//uiMaxChips = MAX_ADC_CHIPS;
	uiMaxChips = 1;
	for (uiChipInUse = 0; uiChipInUse < uiMaxChips; uiChipInUse++)
	{
		ui8_Chips_Addresses = a_ui8_ADC_Chip_Slave_Addresses[uiChipInUse];

		uiMaxChannels = a_ui32_Max_Channels_To_Use[uiChipInUse];

		for (uiChannel_Index = 0; uiChannel_Index < uiMaxChannels; uiChannel_Index++)
		{
			ui8_Channel_Selector = a_ui8_Channel_Select[uiChannel_Index];

			uint32_t uiLow = 0xFFFF;
			uint32_t uiHigh = 0;
			uint32_t uiAccumulator = 0;
			uint32_t uiAverageIndex = 0;

			while (uiAverageIndex < MAX_ADC_SAMPLES)
			{
				iRtn = ADC_Get_Channel_Data(ui8_Chips_Addresses, ui8_Channel_Selector, &ui16_Voltage);

				if (iRtn == 0)
				{
					uiAverageIndex++;

					if (ui16_Voltage < uiLow) uiLow = ui16_Voltage;
					if (ui16_Voltage > uiHigh) uiHigh = ui16_Voltage;

					uiAccumulator += ui16_Voltage;

					//char szMessage[128];
					//sprintf(szMessage, "Max Chips: %i  Chip Used: %i  Chip Address: %x   Max Channels: %i   Channel Index: %i   Channel Selector: %x   Voltage: %i \n",
					//		uiMaxChips,
					//		uiChipInUse,
					//		ui8_Chips_Addresses,
					//		uiMaxChannels,
					//		uiChannel_Index,
					//		ui8_Channel_Selector,
					//		ui16_Voltage);
					//Telemetry_System_Printf(szMessage);
				}
				else
				{
					Telemetry_Send_Output("ADC_Get_Data()::Invalid Return On Data \n");
				}
			}

			// scratch the high and the low and average it out...
			uiAccumulator -= uiLow;
			uiAccumulator -= uiHigh;
			uiAccumulator = uiAccumulator / (MAX_ADC_SAMPLES - 2);


			if (uiChipInUse == 0)
			{
				// there are 8 channels of of ADC data... the 1st four are for HORIZONTAL, then last four are for VERTICAL
				if (uiChannel_Index < MAX_PHOTORESISTOR_RLUP)
				{
					g_s_Dish_Movement_Telemetry.MT_a_ui32ADC_H_Data[uiChannel_Index] = uiAccumulator;
				}
				else
				{
					g_s_Dish_Movement_Telemetry.MT_a_ui32ADC_V_Data[uiChannel_Index - MAX_PHOTORESISTOR_RLUP] = uiAccumulator;
				}
			}
			else if (uiChipInUse == 1) // this is currently not used
			{
				if (uiChannel_Index == 0) g_s_Motor_Voltages.uiDishPump = ui16_Voltage;
				else if (uiChannel_Index == 1) g_s_Motor_Voltages.uiImmediateReseviorPump = uiAccumulator;
				else if (uiChannel_Index == 2) g_s_Motor_Voltages.uiHoldReseviorPump = uiAccumulator;
				else if (uiChannel_Index == 3) g_s_Motor_Voltages.uiAUXPump = uiAccumulator;
				else if (uiChannel_Index == 4) g_s_Motor_Voltages.uiHorizontalDishMotor = uiAccumulator;
				else if (uiChannel_Index == 5) g_s_Motor_Voltages.uiVerticalDishDishMotor = uiAccumulator;
			}
		}
	}


	g_s_Dish_Movement_Telemetry.MT_iH_ResultCalc =
						(g_s_Dish_Movement_Telemetry.MT_a_ui32ADC_H_Data[0] + g_s_Dish_Movement_Telemetry.MT_a_ui32ADC_H_Data[2]) -
						(g_s_Dish_Movement_Telemetry.MT_a_ui32ADC_H_Data[1] + g_s_Dish_Movement_Telemetry.MT_a_ui32ADC_H_Data[3]);


	g_s_Dish_Movement_Telemetry.MT_iV_ResultCalc =
						(g_s_Dish_Movement_Telemetry.MT_a_ui32ADC_V_Data[0] + g_s_Dish_Movement_Telemetry.MT_a_ui32ADC_V_Data[2]) -
						(g_s_Dish_Movement_Telemetry.MT_a_ui32ADC_V_Data[1] + g_s_Dish_Movement_Telemetry.MT_a_ui32ADC_V_Data[3]);





}





