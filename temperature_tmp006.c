//*****************************************************************************
//
// temperature_tmp006.c - Example to use of the SensorLib with the TMP006
//
// Copyright (c) 2013-2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.0.12573 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "sensorlib/hw_tmp006.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/tmp006.h"
#include "drivers/rgb.h"
/*Humidity*/
#include "sensorlib/hw_sht21.h"
#include "sensorlib/sht21.h"


//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Temperature Measurement with the TMP006 (temperature_tmp006)</h1>
//!
//! This example demonstrates the basic use of the Sensor Library, TM4C123G
//! LaunchPad and the SensHub BoosterPack to obtain ambient and object
//! temperature measurements with the Texas Instruments TMP006 sensor.
//!
//! Connect a serial terminal program to the LaunchPad's ICDI virtual serial
//! port at 115,200 baud.  Use eight bits per byte, no parity and one stop bit.
//! The raw sensor measurements are printed to the terminal.  The RGB LED
//! blinks at 1Hz once the initialization is complete and the example is
//! running.
//
//*****************************************************************************

//*****************************************************************************
//
// Define TMP006 I2C Address.
// Define SHT21 I2C Address.
//
//*****************************************************************************
#define TMP006_I2C_ADDRESS      0x41
#define SHT21_I2C_ADDRESS       0x40

//*****************************************************************************
//
// Global instance structure for the I2C master driver.
//
//*****************************************************************************
tI2CMInstance g_sI2CInst;


//*****************************************************************************
//
// Global instance structure for the TMP006 sensor driver.
// Global instance structure for the SHT21 sensor driver.
//
//*****************************************************************************
tTMP006 g_sTMP006Inst;
tSHT21 g_sSHT21Inst;

//*****************************************************************************
//
// Global new data flag to alert main that TMP006 data is ready.
//
//*****************************************************************************
volatile uint_fast8_t g_vui8DataFlag;

//*****************************************************************************
//
// Global new error flag to store the error condition if encountered.
//
//*****************************************************************************
volatile uint_fast8_t g_vui8ErrorFlag;

//*****************************************************************************
//
// Colors for the RGB LED.
//
//*****************************************************************************
uint32_t g_pui32Colors[3];

//*****************************************************************************
//
// Application function to capture ASSERT failures and other debug conditions.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// TMP006 Sensor callback function.  Called at the end of TMP006 sensor driver
// transactions. This is called from I2C interrupt context. Therefore, we just
// set a flag and let main do the bulk of the computations and display.
//
//*****************************************************************************
void
TMP006AppCallback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    //
    // If the transaction succeeded set the data flag to indicate to
    // application that this transaction is complete and data may be ready.
    //
    if(ui8Status == I2CM_STATUS_SUCCESS)
    {
        g_vui8DataFlag = 1;
    }

    //
    // Store the most recent status in case it was an error condition
    //
    g_vui8ErrorFlag = ui8Status;
}


//*****************************************************************************
//
// TMP006 Application error handler.
//
//*****************************************************************************
void
TMP006AppErrorHandler(char *pcFilename, uint_fast32_t ui32Line)
{
    //
    // Set terminal color to red and print error status and locations
    //
    UARTprintf("\033[31;1m");
    UARTprintf("Error: %d, File: %s, Line: %d\n"
               "See I2C status definitions in utils\\i2cm_drv.h\n",
               g_vui8ErrorFlag, pcFilename, ui32Line);

    //
    // Return terminal color to normal
    //
    UARTprintf("\033[0m");

    //
    // Set RGB Color to RED
    //
    g_pui32Colors[0] = 0xFFFF;
    g_pui32Colors[1] = 0;
    g_pui32Colors[2] = 0;
    RGBColorSet(g_pui32Colors);

    //
    // Increase blink rate to get attention
    //
    RGBBlinkRateSet(10.0f);

    //
    // Go to sleep wait for interventions.  A more robust application could
    // attempt corrective actions here.
    //
    while(1)
    {
        ROM_SysCtlSleep();
    }
}

//*****************************************************************************
//
// Called by the NVIC as a result of I2C3 Interrupt. I2C3 is the I2C connection
// to the TMP006.
//
//*****************************************************************************
void
TMP006I2CIntHandler(void)
{
    //
    // Pass through to the I2CM interrupt handler provided by sensor library.
    // This is required to be at application level so that I2CMIntHandler can
    // receive the instance structure pointer as an argument.
    //
    I2CMIntHandler(&g_sI2CInst);
}

//*****************************************************************************
//
// Function to wait for the SHT21 transactions to complete.
//
//*****************************************************************************
void
SHT21AppI2CWait(char *pcFilename, uint_fast32_t ui32Line)
{
    //
    // Put the processor to sleep while we wait for the I2C driver to
    // indicate that the transaction is complete.
    //
    while((g_vui8DataFlag == 0) && (g_vui8ErrorFlag == 0))
    {
        ROM_SysCtlSleep();
    }

    //
    // If an error occurred call the error handler immediately.
    //
    if(g_vui8ErrorFlag)
    {
        TMP006AppErrorHandler(pcFilename, ui32Line);
    }

    //
    // clear the data flag for next use.
    //
    g_vui8DataFlag = 0;
}

//*****************************************************************************
//
// Called by the NVIC as a result of GPIO port E interrupt event. For this
// application GPIO port E pin 0 is the interrupt line for the TMP006
//
//*****************************************************************************
void
IntGPIOe(void)
{
    uint32_t ui32Status;

    ui32Status = GPIOIntStatus(GPIO_PORTE_BASE, true);

    //
    // Clear all the pin interrupts that are set
    //
    GPIOIntClear(GPIO_PORTE_BASE, ui32Status);

    if(ui32Status & GPIO_PIN_0)
    {
        //
        // This interrupt indicates a conversion is complete and ready to be
        // fetched.  So we start the process of getting the data.
        //
        TMP006DataRead(&g_sTMP006Inst, TMP006AppCallback, &g_sTMP006Inst);
    }
}

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}
//Humidity
void
HumiditySht(void)
{
    float fTemperature, fHumidity;
    int32_t i32IntegerPart;
    int32_t i32FractionPart;
	 //
	        // Write the command to start a humidity measurement
	        //
	        SHT21Write(&g_sSHT21Inst, SHT21_CMD_MEAS_RH, g_sSHT21Inst.pui8Data, 0,
	                   TMP006AppCallback, &g_sSHT21Inst);

	        //
	        // Wait for the I2C transactions to complete before moving forward
	        //
	        SHT21AppI2CWait(__FILE__, __LINE__);

	        //
	        // Wait 33 milliseconds before attempting to get the result. Datasheet
	        // claims this can take as long as 29 milliseconds
	        //
	        ROM_SysCtlDelay(ROM_SysCtlClockGet() / (30 * 3));

	        //
	        // Get the raw data from the sensor over the I2C bus
	        //
	        SHT21DataRead(&g_sSHT21Inst, TMP006AppCallback, &g_sSHT21Inst);

	        //
	        // Wait for the I2C transactions to complete before moving forward
	        //
	        SHT21AppI2CWait(__FILE__, __LINE__);

	        //
	        // Get a copy of the most recent raw data in floating point format.
	        //
	        SHT21DataHumidityGetFloat(&g_sSHT21Inst, &fHumidity);

	        //
	        // Write the command to start a temperature measurement
	        //
	        SHT21Write(&g_sSHT21Inst, SHT21_CMD_MEAS_T, g_sSHT21Inst.pui8Data, 0,
	                   TMP006AppCallback, &g_sSHT21Inst);

	        //
	        // Wait for the I2C transactions to complete before moving forward
	        //
	        SHT21AppI2CWait(__FILE__, __LINE__);

	        //
	        // Wait 100 milliseconds before attempting to get the result. Datasheet
	        // claims this can take as long as 85 milliseconds
	        //
	        ROM_SysCtlDelay(ROM_SysCtlClockGet() / (10 * 3));

	        //
	        // Read the conversion data from the sensor over I2C.
	        //
	        SHT21DataRead(&g_sSHT21Inst, TMP006AppCallback, &g_sSHT21Inst);

	        //
	        // Wait for the I2C transactions to complete before moving forward
	        //
	        SHT21AppI2CWait(__FILE__, __LINE__);

	        //
	        // Get the most recent temperature result as a float in celcius.
	        //
	        SHT21DataTemperatureGetFloat(&g_sSHT21Inst, &fTemperature);

	        //
	        // Convert the floats to an integer part and fraction part for easy
	        // print. Humidity is returned as 0.0 to 1.0 so multiply by 100 to get
	        // percent humidity.
	        //
	        fHumidity *= 100.0f;
	        i32IntegerPart = (int32_t) fHumidity;
	        i32FractionPart = (int32_t) (fHumidity * 1000.0f);
	        i32FractionPart = i32FractionPart - (i32IntegerPart * 1000);
	        if(i32FractionPart < 0)
	        {
	            i32FractionPart *= -1;
	        }

	        //
	        // Print the humidity value using the integers we just created
	        //
	        UARTprintf("Humidity %3d.%03d\t", i32IntegerPart, i32FractionPart);

	        //
	        // Perform the conversion from float to a printable set of integers
	        //
	        i32IntegerPart = (int32_t) fTemperature;
	        i32FractionPart = (int32_t) (fTemperature * 1000.0f);
	        i32FractionPart = i32FractionPart - (i32IntegerPart * 1000);
	        if(i32FractionPart < 0)
	        {
	            i32FractionPart *= -1;
	        }

	        //
	        // Print the temperature as integer and fraction parts.
	        //
	        UARTprintf("Temperature %3d.%03d\n", i32IntegerPart, i32FractionPart);

	        //
	        // Delay for one second. This is to keep sensor duty cycle
	        // to about 10% as suggested in the datasheet, section 2.4.
	        // This minimizes self heating effects and keeps reading more accurate.
	        //
	        ROM_SysCtlDelay(ROM_SysCtlClockGet() / 3);
}

//*****************************************************************************
//
// Main 'C' Language entry point.
//
//*****************************************************************************
int
main(void)
{
    float fAmbient, fObject;
    int_fast32_t i32IntegerPart;
    int_fast32_t i32FractionPart;

    //
    // Setup the system clock to run at 40 Mhz from PLL with crystal reference
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    //
    // Enable the peripherals used by this example.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    //
    // Initialize the UART.
    //
    ConfigureUART();

    //
    // Print the welcome message to the terminal.
    //
    UARTprintf("\033[2J\033[1;1HTMP006 Example\n");

    //
    // Setup the color of the RGB LED.
    //
    g_pui32Colors[RED] = 0;
    g_pui32Colors[BLUE] = 0xFFFF;
    g_pui32Colors[GREEN] = 0;

    //
    // Initialize the RGB Driver and start RGB blink operation.
    //
    RGBInit(0);
    RGBColorSet(g_pui32Colors);
    RGBIntensitySet(0.5f);
    RGBEnable();

    //
    // The I2C3 peripheral must be enabled before use.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    //
    // Configure the pin muxing for I2C3 functions on port D0 and D1.
    // This step is not necessary if your part does not support pin muxing.
    //
    ROM_GPIOPinConfigure(GPIO_PD0_I2C3SCL);
    ROM_GPIOPinConfigure(GPIO_PD1_I2C3SDA);

    //
    // Select the I2C function for these pins.  This function will also
    // configure the GPIO pins pins for I2C operation, setting them to
    // open-drain operation with weak pull-ups.  Consult the data sheet
    // to see which functions are allocated per pin.
    //
    GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
    ROM_GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);

    //
    // Configure and Enable the GPIO interrupt. Used for DRDY from the TMP006
    //
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0);
    GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_0);
    ROM_GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_FALLING_EDGE);
    ROM_IntEnable(INT_GPIOE);

    //
    // Keep only some parts of the systems running while in sleep mode.
    // GPIOE is for the TMP006 data ready interrupt.
    // UART0 is the virtual serial port
    // TIMER0, TIMER1 and WTIMER5 are used by the RGB driver
    // I2C3 is the I2C interface to the TMP006
    //
    ROM_SysCtlPeripheralClockGating(true);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOE);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER0);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER1);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_I2C3);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_WTIMER5);

    //
    // Enable interrupts to the processor.
    //
    ROM_IntMasterEnable();

    //
    // Initialize I2C3 peripheral.
    //
    I2CMInit(&g_sI2CInst, I2C3_BASE, INT_I2C3, 0xff, 0xff,
             SysCtlClockGet());

    //
    // Initialize the TMP006
    //
    TMP006Init(&g_sTMP006Inst, &g_sI2CInst, TMP006_I2C_ADDRESS,
               TMP006AppCallback, &g_sTMP006Inst);
    SHT21Init(&g_sSHT21Inst, &g_sI2CInst, SHT21_I2C_ADDRESS,
              TMP006AppCallback, &g_sSHT21Inst);
    //
    // Put the processor to sleep while we wait for the I2C driver to
    // indicate that the transaction is complete.
    //
    while((g_vui8DataFlag == 0) && (g_vui8ErrorFlag == 0))
    {
        ROM_SysCtlSleep();
    }
    //
    // Wait for the I2C transactions to complete before moving forward
    //
    SHT21AppI2CWait(__FILE__, __LINE__);
    //
    // Delay for 20 milliseconds for SHT21 reset to complete itself.
    // Datasheet says reset can take as long 15 milliseconds.
    //
    ROM_SysCtlDelay(ROM_SysCtlClockGet() / (50 * 3));

    //
    // If an error occurred call the error handler immediately.
    //
    if(g_vui8ErrorFlag)
    {
        TMP006AppErrorHandler(__FILE__, __LINE__);
    }

    //
    // clear the data flag for next use.
    //
    g_vui8DataFlag = 0;

    //
    // Delay for 10 milliseconds for TMP006 reset to complete.
    // Not explicitly required. Datasheet does not say how long a reset takes.
    //
    ROM_SysCtlDelay(ROM_SysCtlClockGet() / (100 * 3));

    //
    // Enable the DRDY pin indication that a conversion is in progress.
    //
    TMP006ReadModifyWrite(&g_sTMP006Inst, TMP006_O_CONFIG,
                          ~TMP006_CONFIG_EN_DRDY_PIN_M,
                          TMP006_CONFIG_EN_DRDY_PIN, TMP006AppCallback,
                          &g_sTMP006Inst);

    //
    // Wait for the DRDY enable I2C transaction to complete.
    //
    while((g_vui8DataFlag == 0) && (g_vui8ErrorFlag == 0))
    {
        ROM_SysCtlSleep();
    }

    //
    // If an error occurred call the error handler immediately.
    //
    if(g_vui8ErrorFlag)
    {
        TMP006AppErrorHandler(__FILE__, __LINE__);
    }

    //
    // clear the data flag for next use.
    //
    g_vui8DataFlag = 0;

    //
    // Last thing before the loop start blinking to show we got this far and
    // the tmp006 is setup and ready for auto measure
    //
    RGBBlinkRateSet(1.0f);

    //
    // Loop Forever
    //
    while(1)
    {
        //
        // Put the processor to sleep while we wait for the TMP006 to
        // signal that data is ready.  Also continue to sleep while I2C
        // transactions get the raw data from the TMP006
        //
        while((g_vui8DataFlag == 0) && (g_vui8ErrorFlag == 0))
        {
            ROM_SysCtlSleep();
        }

        //
        // If an error occurred call the error handler immediately.
        //
        if(g_vui8ErrorFlag)
        {
            TMP006AppErrorHandler(__FILE__, __LINE__);
        }

        //
        // Reset the flag
        //
        g_vui8DataFlag = 0;

        //
        // Get a local copy of the latest data in float format.
        //
        TMP006DataTemperatureGetFloat(&g_sTMP006Inst, &fAmbient, &fObject);

        //
        // Convert the floating point ambient temperature  to an integer part
        // and fraction part for easy printing.
        //
        i32IntegerPart = (int32_t)fAmbient;
        i32FractionPart = (int32_t)(fAmbient * 1000.0f);
        i32FractionPart = i32FractionPart - (i32IntegerPart * 1000);
        if(i32FractionPart < 0)
        {
            i32FractionPart *= -1;
        }
        UARTprintf("Ambient %3d.%03d\t", i32IntegerPart, i32FractionPart);

        //
        // Convert the floating point ambient temperature  to an integer part
        // and fraction part for easy printing.
        //
        i32IntegerPart = (int32_t)fObject;
        i32FractionPart = (int32_t)(fObject * 1000.0f);
        i32FractionPart = i32FractionPart - (i32IntegerPart * 1000);
        if(i32FractionPart < 0)
        {
            i32FractionPart *= -1;
        }
        UARTprintf("Object %3d.%03d\n", i32IntegerPart, i32FractionPart);
        HumiditySht();
    }
}
