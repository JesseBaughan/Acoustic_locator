//*****************************************************************************
//
// usb_dev_bulk.c - Main routines for the generic bulk device example.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
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
// This is part of revision 2.1.4.178 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "usblib/usblib.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdbulk.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "usb_bulk_structs.h"

#include "driverlib/adc.h"
#include "driverlib/udma.h"

//*******************************************************************************
//UDMA VARIABLES
//*******************************************************************************
#define PWM_FREQUENCY (50)
#define PWM_MIN (30)
#define PWM_MAX (120)
#define PWM_ZERO (75)

#define ADC_O_SSFIFO1 (0x068) //ADC0 sample sequencer 1 result FIFO 
#define ADC_BUFFER_SIZE (32) //two 16-bit ADC sample from each of the ADC's 
#define ADC_BUFFER_WINDOW_COUNT (2) //This is how many windows we have, this is ping and pong = 2
#define NUM_CHANNELS (2) //This is the amount of data items to transfer
#define FILTER_SIZE (64)

static volatile uint32_t g_ui32DMAErrCount = 0;
void InitADC00(void);
void ConfigureUART(void);

#pragma DATA_ALIGN(g_adcPingPong0, 64) //Create ping pong buffer for ADC0
static int32_t g_adcPingPong0[ADC_BUFFER_SIZE*ADC_BUFFER_WINDOW_COUNT][NUM_CHANNELS]; //Setup an 2 dimensional array for ping-pong buffers A and B
static int32_t g_adcInIndex0 = 0;
static int32_t g_adcOutIndex0 = 0;
static volatile int32_t g_stall0 = 0;
static volatile int32_t g_adcPingPongStage0[ADC_BUFFER_WINDOW_COUNT] = { 0 };

#pragma DATA_ALIGN(g_adcPingPong1, 64) //Create ping pong buffer for ADC1
static int32_t g_adcPingPong1[ADC_BUFFER_SIZE*ADC_BUFFER_WINDOW_COUNT][NUM_CHANNELS]; 
static int32_t g_adcInIndex1 = 0;
static volatile int32_t g_stall1 = 0;
static volatile int32_t g_adcPingPongStage1[ADC_BUFFER_WINDOW_COUNT] = { 0 };

static uint32_t g_msg_index = 0;
static uint32_t g_usb_receive_size = 0;
static volatile int32_t g_usb_bytes_to_read = 0;

static uint32_t g_ui32Load;
static uint32_t g_ui32PWMClock;
static uint8_t g_ui8Adjust = PWM_ZERO;
static volatile uint8_t g_ui8AdjustPending = PWM_ZERO;

//static int32_t g_channelSum[NUM_CHANNELS] = { 0 };
//static int16_t g_channelOffset[NUM_CHANNELS] = { 0 };
//static int32_t g_channelOffsetCount = 0;
//static uint8_t g_fire = 0;

#pragma DATA_ALIGN(pui8ControlTable, 1024)
uint8_t pui8ControlTable[1024];

volatile uint32_t newtemp;
uint32_t HW_AVERAGE = 8;

//*****************************************************************************
//USB VARIABLES
//
//! \addtogroup example_list
//! <h1>USB Generic Bulk Device (usb_dev_bulk)</h1>
//!
//! This example provides a generic USB device offering simple bulk data
//! transfer to and from the host.  The device uses a vendor-specific class ID
//! and supports a single bulk IN endpoint and a single bulk OUT endpoint.
//! Data received from the host is assumed to be ASCII text and it is
//! echoed back with the case of all alphabetic characters swapped.
//!
//! A Windows INF file for the device is provided on the installation CD and
//! in the C:/ti/TivaWare-for-C-Series/windows_drivers directory of TivaWare C
//! series releases.  This INF contains information required to install the
//! WinUSB subsystem on Windowi16XP and Vista PCs.  WinUSB is a Windows
//! subsystem allowing user mode applications to access the USB device without
//! the need for a vendor-specific kernel mode driver.
//!
//! A sample Windows command-line application, usb_bulk_example, illustrating
//! how to connect to and communicate with the bulk device is also provided.
//! The application binary is installed as part of the ''Windows-side examples
//! for USB kits'' package (SW-USB-win) on the installation CD or via download
//! from http://www.ti.com/tivaware .  Project files are included to allow
//! the examples to be built using Microsoft VisualStudio 2008.  Source code
//! for this application can be found in directory
//! TivaWare-for-C-Series/tools/usb_bulk_example.
//
//*****************************************************************************

//*****************************************************************************
//
// The system tick rate expressed both as ticks per second and a millisecond
// period.
//
//*****************************************************************************
#define SYSTICKS_PER_SECOND     100
#define SYSTICK_PERIOD_MS       (1000 / SYSTICKS_PER_SECOND)

//*****************************************************************************
//
// The global system tick counter.
//
//*****************************************************************************
volatile uint32_t g_ui32SysTickCount = 0;

//*****************************************************************************
//
// Variables tracking transmit and receive counts.
//
//*****************************************************************************
volatile uint32_t g_ui32TxCount = 0;
volatile uint32_t g_ui32RxCount = 0;
#ifdef DEBUG
uint32_t g_ui32UARTRxErrors = 0;
#endif

//*****************************************************************************
//
// Debug-related definitions and declarations.
//
// Debug output is available via UART0 if DEBUG is defined during build.
//
//*****************************************************************************
#ifdef DEBUG
//*****************************************************************************
//
// Map all debug print calls to UARTprintf in debug builds.
//
//*****************************************************************************
#define DEBUG_PRINT UARTprintf

#else

//*****************************************************************************
//
// Compile out all debug print calls in release builds.
//
//*****************************************************************************
#define DEBUG_PRINT while(0) ((int (*)(char *, ...))0)
#endif

//*****************************************************************************
//
// Flags used to pass commands from interrupt context to the main loop.
//
//*****************************************************************************
#define COMMAND_PACKET_RECEIVED 0x00000001
#define COMMAND_STATUS_UPDATE   0x00000002

volatile uint32_t g_ui32Flags = 0;

//*****************************************************************************
//
// Global flag indicating that a USB configuration has been set.
//
//*****************************************************************************
static volatile bool g_bUSBConfigured = false;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    UARTprintf("Error at line %d of %s\n", ui32Line, pcFilename);
    while(1)
    {
    }
}
#endif

//******************************USB STUFF**************************************
//*****************************************************************************
//
// Interrupt handler for the system tick counter.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    //
    // Update our system tick counter.
    //
    g_ui32SysTickCount++;
}

//*****************************************************************************
//
// Receive new data and echo it back to the host.
//
// \param psDevice points to the instance data for the device whose data is to
// be processed.
// \param pui8Data points to the newly received data in the USB receive buffer.
// \param ui32NumBytes is the number of bytes of data available to be processed.
//
// This function is called whenever we receive a notification that data is
// available from the host. We read the data, byte-by-byte and swap the case
// of any alphabetical characters found then write it back out to be
// transmitted back to the host.
//
// \return Returns the number of bytes of data processed.
//
//*****************************************************************************
static uint32_t
EchoNewDataToHost(tUSBDBulkDevice *psDevice, uint8_t *pui8Data,
                  uint32_t ui32NumBytes)
{
    uint32_t ui32Loop, ui32Space, ui32Count;
    uint32_t ui32ReadIndex;
    uint32_t ui32WriteIndex;
    tUSBRingBufObject sTxRing;

    //
    // Get the current buffer information to allow us to write directly to
    // the transmit buffer (we already have enough information from the
    // parameters to access the receive buffer directly).
    //
    USBBufferInfoGet(&g_sTxBuffer, &sTxRing);

    //
    // How much space is there in the transmit buffer?
    //
    ui32Space = USBBufferSpaceAvailable(&g_sTxBuffer);

    //
    // How many characters can we process this time round?
    //
    ui32Loop = (ui32Space < ui32NumBytes) ? ui32Space : ui32NumBytes;
    ui32Count = ui32Loop;

    //
    // Update our receive counter.
    //
    g_ui32RxCount += ui32NumBytes;

    //
    // Dump a debug message.
    //
    DEBUG_PRINT("Received %d bytes\n", ui32NumBytes);

    //
    // Set up to process the characters by directly accessing the USB buffers.
    //
    ui32ReadIndex = (uint32_t)(pui8Data - g_pui8USBRxBuffer);
    ui32WriteIndex = sTxRing.ui32WriteIndex;

    while(ui32Loop)
    {
        //
        // Copy from the receive buffer to the transmit buffer converting
        // character case on the way.
        //

        //
        // Is this a lower case character?
        //
        if((g_pui8USBRxBuffer[ui32ReadIndex] >= 'a') &&
           (g_pui8USBRxBuffer[ui32ReadIndex] <= 'z'))
        {
            //
            // Convert to upper case and write to the transmit buffer.
            //
            g_pui8USBTxBuffer[ui32WriteIndex] =
                (g_pui8USBRxBuffer[ui32ReadIndex] - 'a') + 'A';
        }
        else
        {
            //
            // Is this an upper case character?
            //
            if((g_pui8USBRxBuffer[ui32ReadIndex] >= 'A') &&
               (g_pui8USBRxBuffer[ui32ReadIndex] <= 'Z'))
            {
                //
                // Convert to lower case and write to the transmit buffer.
                //
                g_pui8USBTxBuffer[ui32WriteIndex] =
                    (g_pui8USBRxBuffer[ui32ReadIndex] - 'Z') + 'z';
            }
            else
            {
                //
                // Copy the received character to the transmit buffer.
                //
                g_pui8USBTxBuffer[ui32WriteIndex] =
                        g_pui8USBRxBuffer[ui32ReadIndex];
            }
        }

        //
        // Move to the next character taking care to adjust the pointer for
        // the buffer wrap if necessary.
        //
        ui32WriteIndex++;
        ui32WriteIndex = (ui32WriteIndex == BULK_BUFFER_SIZE) ?
                         0 : ui32WriteIndex;

        ui32ReadIndex++;
        ui32ReadIndex = (ui32ReadIndex == BULK_BUFFER_SIZE) ?
                        0 : ui32ReadIndex;

        ui32Loop--;
    }

    //
    // We've processed the data in place so now send the processed data
    // back to the host.
    //
    USBBufferDataWritten(&g_sTxBuffer, ui32Count);

    DEBUG_PRINT("Wrote %d bytes\n", ui32Count);

    //
    // We processed as much data as we can directly from the receive buffer so
    // we need to return the number of bytes to allow the lower layer to
    // update its read pointer appropriately.
    //
    return(ui32Count);
}

//*****************************************************************************
//
// Handles bulk driver notifications related to the transmit channel (data to
// the USB host).
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the bulk driver to notify us of any events
// related to operation of the transmit data channel (the IN channel carrying
// data to the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
uint32_t
TxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
          void *pvMsgData)
{
    //
    // We are not required to do anything in response to any transmit event
    // in this example. All we do is update our transmit counter.
    //
    if(ui32Event == USB_EVENT_TX_COMPLETE)
    {
        g_ui32TxCount += ui32MsgValue;
    }

    //
    // Dump a debug message.
    //
    DEBUG_PRINT("TX complete %d\n", ui32MsgValue);

    return(0);
}

//*****************************************************************************
//
// Handles bulk driver notifications related to the receive channel (data from
// the USB host).
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the bulk driver to notify us of any events
// related to operation of the receive data channel (the OUT channel carrying
// data from the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
uint32_t
RxHandler(void *pvCBData, uint32_t ui32Event,
               uint32_t ui32MsgValue, void *pvMsgData)
{
    //
    // Which event are we being sent?
    //
    switch(ui32Event)
    {
        //
        // We are connected to a host and communication is now possible.
        //
        case USB_EVENT_CONNECTED:
        {
            g_bUSBConfigured = true;
            UARTprintf("Host connected.\n");

            //
            // Flush our buffers.
            //
            USBBufferFlush(&g_sTxBuffer);
            USBBufferFlush(&g_sRxBuffer);

            break;
        }

        //
        // The host has disconnected.
        //
        case USB_EVENT_DISCONNECTED:
        {
            g_bUSBConfigured = false;
            UARTprintf("Host disconnected.\n");
            break;
        }

        //
        // A new packet has been received.
        //
        case USB_EVENT_RX_AVAILABLE:
        {
            tUSBDBulkDevice *psDevice;

            //
            // Get a pointer to our instance data from the callback data
            // parameter.
            //
            psDevice = (tUSBDBulkDevice *)pvCBData;

            //
            // Read the new packet and echo it back to the host.
            //
            return(EchoNewDataToHost(psDevice, pvMsgData, ui32MsgValue));
        }

        //
        // Ignore SUSPEND and RESUME for now.
        //
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
        {
            break;
        }

        //
        // Ignore all other events and return 0.
        //
        default:
        {
            break;
        }
    }

    return(0);
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

//*****************************************************************************
//
// UDMA AND ADC STUFF
//
//*****************************************************************************
void uDMAErrorHandler(void)
{
	uint32_t ui32Status;
	ui32Status = uDMAErrorStatusGet();
	if(ui32Status)
	{
		uDMAErrorStatusClear();
		g_ui32DMAErrCount++;
	}
}

void UARTIntHandler(void)
{
//	uint32_t ui32Status;
//	ui32Status = UARTIntStatus(UART0_BASE, true); //get interrupt status
//	UARTIntClear(UART0_BASE, ui32Status); //clear the asserted interrupts

//	while(UARTCharsAvail(UART0_BASE)) //loop while there are chars
//	{
//		UARTCharPutNonBlocking(UART0_BASE, UARTCharGetNonBlocking(UART0_BASE));
//		//echo character
//		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2); //blink LED
//		SysCtlDelay(SysCtlClockGet() / (1000 * 3)); //delay ~1 msec
//		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0); //turn off LED
//	}
}

void ADC00IntHandler(void)
{
    uint32_t ui32ModeP; //Primary 
    uint32_t ui32ModeA; //Alternate 
		
	//Cler the interrupt flag
    ADCIntClear(ADC0_BASE, 0);
		
	//Get status of primary uDMA transfer
    ui32ModeP = uDMAChannelModeGet(UDMA_CHANNEL_ADC1 | UDMA_PRI_SELECT); 
		
	//uDMA transfer from ADC to In buffer is done if control structure indicates STOP, reset uDMA channel
    if(ui32ModeP == UDMA_MODE_STOP) 
    {
				//Setup the primary channel agains
        uDMAChannelTransferSet(UDMA_CHANNEL_ADC1 | UDMA_PRI_SELECT,
                                   UDMA_MODE_PINGPONG,
								   (void *) (ADC0_BASE + ADC_O_SSFIFO1),
								   &g_adcPingPong0[g_adcInIndex0][0], NUM_CHANNELS);				 
        uDMAChannelEnable(UDMA_CHANNEL_ADC1); //Must be re-enabled
									 
        g_adcInIndex0 = (g_adcInIndex0 + 1) % (ADC_BUFFER_SIZE*ADC_BUFFER_WINDOW_COUNT); //Increment buffer???
				//Something to do with waiting for the buffer to be empty and ready??
        if((g_adcInIndex0 % ADC_BUFFER_SIZE) == 2)
        {
        	int32_t ready_buffer = (g_adcInIndex0 / ADC_BUFFER_SIZE);
        	if(g_adcPingPongStage0[ready_buffer % ADC_BUFFER_WINDOW_COUNT] != 0)
        		g_stall0++;
       		ready_buffer = (ready_buffer - 1 + ADC_BUFFER_WINDOW_COUNT) % ADC_BUFFER_WINDOW_COUNT;
       		g_adcPingPongStage0[ready_buffer] = 1;
        }
    }
    else
    {
    	ui32ModeA = uDMAChannelModeGet(UDMA_CHANNEL_ADC1 | UDMA_ALT_SELECT);
		if(ui32ModeA == UDMA_MODE_STOP)
		{
			uDMAChannelTransferSet(UDMA_CHANNEL_ADC1 | UDMA_ALT_SELECT,
									   UDMA_MODE_PINGPONG,
									   (void *) (ADC0_BASE + ADC_O_SSFIFO1),
									   &g_adcPingPong0[g_adcInIndex0][0], NUM_CHANNELS); //IS THIS CORRECT??					 
			uDMAChannelEnable(UDMA_CHANNEL_ADC1);
										 
			g_adcInIndex0 = (g_adcInIndex0 + 1) % (ADC_BUFFER_SIZE*ADC_BUFFER_WINDOW_COUNT);
			if((g_adcInIndex0 % ADC_BUFFER_SIZE) == 2)
			{
				int32_t ready_buffer = (g_adcInIndex0 / ADC_BUFFER_SIZE);
				if(g_adcPingPongStage0[ready_buffer % ADC_BUFFER_WINDOW_COUNT] != 0)
					g_stall0++;
				ready_buffer = (ready_buffer - 1 + ADC_BUFFER_WINDOW_COUNT) % ADC_BUFFER_WINDOW_COUNT;
				g_adcPingPongStage0[ready_buffer] = 1;
			}
		}
    }
}

// ADC 1 interrupt handler. Called on completion of uDMA transfer
void ADC10IntHandler(void)
{
    uint32_t ui32ModeP;
    uint32_t ui32ModeA;

    ADCIntClear(ADC1_BASE, 0);

    ui32ModeP = uDMAChannelModeGet(UDMA_CHANNEL_SSI1TX | UDMA_PRI_SELECT);

    if(ui32ModeP == UDMA_MODE_STOP)
    {
        uDMAChannelTransferSet(UDMA_CHANNEL_SSI1TX | UDMA_PRI_SELECT,
                                   UDMA_MODE_PINGPONG,
								   (void *) (ADC1_BASE + ADC_O_SSFIFO1),
								   &g_adcPingPong1[g_adcInIndex1][0], NUM_CHANNELS);
        uDMAChannelEnable(UDMA_CHANNEL_SSI1TX);
									 
        g_adcInIndex1 = (g_adcInIndex1 + 1) % (ADC_BUFFER_SIZE*ADC_BUFFER_WINDOW_COUNT);
        if((g_adcInIndex1 % ADC_BUFFER_SIZE) == 2)
        {
					//If the amount of transfers is not equal to ADC buffer size (4) then stall for a bit
        	int32_t ready_buffer = (g_adcInIndex1 / ADC_BUFFER_SIZE);
        	if(g_adcPingPongStage1[ready_buffer % ADC_BUFFER_WINDOW_COUNT] != 0)
        		g_stall1++;
       		ready_buffer = (ready_buffer - 1 + ADC_BUFFER_WINDOW_COUNT) % ADC_BUFFER_WINDOW_COUNT;
       		g_adcPingPongStage1[ready_buffer] = 1;
        }
    }
    else
    {
    	ui32ModeA = uDMAChannelModeGet(UDMA_CHANNEL_SSI1TX | UDMA_ALT_SELECT);
		if(ui32ModeA == UDMA_MODE_STOP)
		{
			uDMAChannelTransferSet(UDMA_CHANNEL_SSI1TX | UDMA_ALT_SELECT,
									   UDMA_MODE_PINGPONG,
									   (void *) (ADC1_BASE + ADC_O_SSFIFO1),
									   &g_adcPingPong1[g_adcInIndex1][0], NUM_CHANNELS);
			uDMAChannelEnable(UDMA_CHANNEL_SSI1TX);
										 
			g_adcInIndex1 = (g_adcInIndex1 + 1) % (ADC_BUFFER_SIZE*ADC_BUFFER_WINDOW_COUNT);
			if((g_adcInIndex1 % ADC_BUFFER_SIZE) == 2)
			{
				int32_t ready_buffer = (g_adcInIndex1 / ADC_BUFFER_SIZE);
				if(g_adcPingPongStage1[ready_buffer % ADC_BUFFER_WINDOW_COUNT] != 0)
					g_stall1++;
				ready_buffer = (ready_buffer - 1 + ADC_BUFFER_WINDOW_COUNT) % ADC_BUFFER_WINDOW_COUNT;
				g_adcPingPongStage1[ready_buffer] = 1;
			}
		}
    }
}

void ConfigureADC (void) {
	
	//ADC 0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
	SysCtlDelay(3);
	//Configure sequencers for ADC 0 
	ADCHardwareOversampleConfigure(ADC0_BASE, HW_AVERAGE);
	ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_ALWAYS, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH2);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH1);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_TS);
	ADCSequenceStepConfigure(ADC0_BASE,1,3,ADC_CTL_TS|ADC_CTL_IE|ADC_CTL_END);
//Enable DMA for sample sequwencers; Allows DMA requests to be generated based on the FIFO level of the sample sequencer.
	ADCSequenceDMAEnable(ADC0_BASE, 1); 
	
	uDMAChannelAttributeDisable(UDMA_CHANNEL_ADC1,
                                    UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
                                    UDMA_ATTR_HIGH_PRIORITY |
                                    UDMA_ATTR_REQMASK);

   uDMAChannelControlSet(UDMA_CHANNEL_ADC1 | UDMA_PRI_SELECT,
                            UDMA_SIZE_32 | UDMA_SRC_INC_NONE | UDMA_DST_INC_32 |
                              UDMA_ARB_4); //Transfer data size is 32 bits, arbitration size is 4 which is same size as ADC FIFO

   uDMAChannelControlSet(UDMA_CHANNEL_ADC1 | UDMA_ALT_SELECT,
    						UDMA_SIZE_32 | UDMA_SRC_INC_NONE | UDMA_DST_INC_32 |
                              UDMA_ARB_4);

   uDMAChannelTransferSet(UDMA_CHANNEL_ADC1 | UDMA_PRI_SELECT,
                               UDMA_MODE_PINGPONG,
							   (void *) (ADC0_BASE + ADC_O_SSFIFO1),
							   &g_adcPingPong0[0][0], NUM_CHANNELS); //source is ADC0 FIFO, destination is adcPingPong0 - A

   uDMAChannelTransferSet(UDMA_CHANNEL_ADC1 | UDMA_ALT_SELECT,
                               UDMA_MODE_PINGPONG,
							   (void *) (ADC0_BASE + ADC_O_SSFIFO1),
							   &g_adcPingPong0[1][0], NUM_CHANNELS); //source is ADC0 FIFO, destination is adcPingPong0 - B
								 
   g_adcInIndex0 = 2;
	
								 
	//ADC 1
	//Configure sequencers for ADC 1
	ADCHardwareOversampleConfigure(ADC1_BASE, HW_AVERAGE);
	ADCSequenceConfigure(ADC1_BASE, 1, ADC_TRIGGER_ALWAYS, 0);
	ADCSequenceStepConfigure(ADC1_BASE, 1, 0, ADC_CTL_CH2);
	ADCSequenceStepConfigure(ADC1_BASE, 1, 1, ADC_CTL_CH1);
	ADCSequenceStepConfigure(ADC1_BASE, 1, 2, ADC_CTL_TS);
	ADCSequenceStepConfigure(ADC1_BASE,1,3,ADC_CTL_TS|ADC_CTL_IE|ADC_CTL_END);
	ADCPhaseDelaySet(ADC1_BASE, ADC_PHASE_180);
	
	//Enable DMA for sample sequwencers; Allows DMA requests to be generated based on the FIFO level of the sample sequencer.
	ADCSequenceDMAEnable(ADC1_BASE, 1);  //Send interrupt to uDMA when FIFO full - sequence is complete
								 
	uDMAChannelAttributeDisable(UDMA_CHANNEL_SSI1TX,
                                    UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
                                    UDMA_ATTR_HIGH_PRIORITY |
                                    UDMA_ATTR_REQMASK);

   uDMAChannelControlSet(UDMA_CHANNEL_SSI1TX | UDMA_PRI_SELECT,
                            UDMA_SIZE_32 | UDMA_SRC_INC_NONE | UDMA_DST_INC_32 |
                              UDMA_ARB_4);

   uDMAChannelControlSet(UDMA_CHANNEL_SSI1TX | UDMA_ALT_SELECT,
    						UDMA_SIZE_32 | UDMA_SRC_INC_NONE | UDMA_DST_INC_32 |
                              UDMA_ARB_4);

   uDMAChannelTransferSet(UDMA_CHANNEL_SSI1TX | UDMA_PRI_SELECT,
                               UDMA_MODE_PINGPONG,
							   (void *) (ADC1_BASE + ADC_O_SSFIFO1),
							   &g_adcPingPong1[0][0], NUM_CHANNELS);

   uDMAChannelTransferSet(UDMA_CHANNEL_SSI1TX | UDMA_ALT_SELECT,
                               UDMA_MODE_PINGPONG,
							   (void *) (ADC1_BASE + ADC_O_SSFIFO1),
							   &g_adcPingPong1[1][0], NUM_CHANNELS);
								 
   uDMAChannelAssign(UDMA_CH25_ADC1_1);
   g_adcInIndex1 = 2;
								 
	//ADC 0 and ADC 1
	uDMAChannelEnable(UDMA_CHANNEL_ADC1);
  uDMAChannelEnable(UDMA_CHANNEL_SSI1TX);

	// make sure interrupt flag is clear
	// enable the interrupt for the module and for the sequence
	ADCIntClear(ADC0_BASE, 1);
	ADCIntClear(ADC1_BASE, 1);
  ADCIntEnable(ADC0_BASE, 1);
	ADCIntEnable(ADC1_BASE, 1);
	IntEnable(INT_ADC0SS1);
	IntEnable(INT_ADC1SS1);
	
	ADCSequenceEnable(ADC0_BASE, 1);
	ADCSequenceEnable(ADC1_BASE, 1);
								 
}


//*****************************************************************************
//
// This is the main application entry function.
//
//*****************************************************************************
int
main(void)
{
    volatile uint32_t ui32Loop;
    uint32_t ui32TxCount;
    uint32_t ui32RxCount;
	
    //ROM_FPULazyStackingEnable();

    // Set the clocking to run from the PLL at 50MHz
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
		//SysCtlPWMClockSet(SYSCTL_PWMDIV_64); 
	
		//Configure USB ports
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_GPIOPinTypeUSBAnalog(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);
	
    // Enable the GPIO port that is used for the on-board LED.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_2);
	
		ConfigureUART();
	
		// gpio
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
		GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
		GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_7);
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, 0);
		
		//USB
		UARTprintf("Configuring USB\n");
    g_bUSBConfigured = false;		
    // Initialize the transmit and receive buffers.
    USBBufferInit(&g_sTxBuffer);
    USBBufferInit(&g_sRxBuffer);
    USBStackModeSet(0, eUSBModeForceDevice, 0); // Set the USB stack mode to Device mode with VBUS monitoring.  
    USBDBulkInit(0, &g_sBulkDevice); // Pass our device information to the USB library and place the device on the bus.
		
    UARTprintf("Waiting for host...\n");
	
		//uDMA
		SysCtlPeripheralClockGating(true);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
		SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UDMA);
		IntEnable(INT_UDMAERR);
		uDMAEnable();
		uDMAControlBaseSet(pui8ControlTable);
    
    // Clear our local byte counters. 
    //ui32RxCount = 0;
    //ui32TxCount = 0;
		
		// Enable the system tick
    ROM_SysTickPeriodSet(ROM_SysCtlClockGet() / SYSTICKS_PER_SECOND);
    ROM_SysTickIntEnable();
    ROM_SysTickEnable();
		
		//ConfigureADC();
		//IntMasterEnable(); //Enable all interrupts

    while(1)
    {
      
        // See if any data has been transferred.
        if((ui32TxCount != g_ui32TxCount) || (ui32RxCount != g_ui32RxCount))
        {
       
            // Has there been any transmit traffic since we last checked?
            if(ui32TxCount != g_ui32TxCount)
            {
            
                // Turn on the Green LED.
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);

      
                // Delay for a bit.
                for(ui32Loop = 0; ui32Loop < 150000; ui32Loop++)
                {
                }

         
                // Turn off the Green LED.
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);

   
                // Take a snapshot of the latest transmit count.      
                ui32TxCount = g_ui32TxCount;
            }

            //
            // Has there been any receive traffic since we last checked?
            //
            if(ui32RxCount != g_ui32RxCount)
            {
                //
                // Turn on the Blue LED.
                //
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

                //
                // Delay for a bit.
                //
                for(ui32Loop = 0; ui32Loop < 150000; ui32Loop++)
                {
                }

                //
                // Turn off the Blue LED.
                //
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

                //
                // Take a snapshot of the latest receive count.
                //
                ui32RxCount = g_ui32RxCount;
            }

            //
            // Update the display of bytes transferred.
            //
            //UARTprintf("\rTx: %d  Rx: %d", ui32TxCount, ui32RxCount);
        }
    }
}
