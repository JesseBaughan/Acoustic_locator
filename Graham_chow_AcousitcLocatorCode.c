#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_adc.h"
#include "driverlib/pin_map.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/rom.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/udma.h"
#include "driverlib/uart.h"
#include "usblib/usblib.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdbulk.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "usb_bulk_structs.h"
#include "driverlib/pwm.h"
#include "global_def.h"
/*
 * main.c
 */

#define PWM_FREQUENCY (50)
#define PWM_MIN (30)
#define PWM_MAX (120)
#define PWM_ZERO (75)

static volatile uint32_t g_ui32DMAErrCount = 0;
void InitADC00(void);
void ConfigureUART(void);

static volatile bool g_bUSBConfigured = false;

#pragma DATA_ALIGN(g_adcPingPong0, 64)
static int32_t g_adcPingPong0[ADC_BUFFER_SIZE*ADC_BUFFER_WINDOW_COUNT][NUM_CHANNELS];
static int32_t g_adcInIndex0 = 0;
static int32_t g_adcOutIndex0 = 0;
static volatile int32_t g_stall0 = 0;
static volatile int32_t g_adcPingPongStage0[ADC_BUFFER_WINDOW_COUNT] = { 0 };

#pragma DATA_ALIGN(g_adcPingPong1, 64)
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

static int32_t g_channelSum[NUM_CHANNELS] = { 0 };
static int16_t g_channelOffset[NUM_CHANNELS] = { 0 };
static int32_t g_channelOffsetCount = 0;
static uint8_t g_fire = 0;



static int32_t g_filter[FILTER_SIZE] =
{
		2158	,
		-3620	,
		-7100	,
		-4486	,
		1833	,
		2758	,
		-2365	,
		-3667	,
		2091	,
		4641	,
		-1618	,
		-5790	,
		807	,
		7030	,
		412	,
		-8308	,
		-2119	,
		9586	,
		4462	,
		-10802	,
		-7643	,
		11929	,
		12065	,
		-12913	,
		-18596	,
		13723	,
		29512	,
		-14331	,
		-53266	,
		14702	,
		166092	,
		247319	,
		166092	,
		14702	,
		-53266	,
		-14331	,
		29512	,
		13723	,
		-18596	,
		-12913	,
		12065	,
		11929	,
		-7643	,
		-10802	,
		4462	,
		9586	,
		-2119	,
		-8308	,
		412	,
		7030	,
		807	,
		-5790	,
		-1618	,
		4641	,
		2091	,
		-3667	,
		-2365	,
		2758	,
		1833	,
		-4486	,
		-7100	,
		-3620	,
		2158
};

// uDMA control table aligned to 1024-byte boundary
#pragma DATA_ALIGN(pui8ControlTable, 1024)
uint8_t pui8ControlTable[1024];


#if ((ADC_BUFFER_SIZE * (ADC_BUFFER_WINDOW_COUNT - 1)) < FILTER_SIZE)
error
#endif

int main(void)
{
	//uint32_t ui32Period;
	SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
	SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

	// configure USB
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_GPIOPinTypeUSBAnalog(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);

	ConfigureUART();

	// gpio
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_7);
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, 0);

	// configure pwm
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
	GPIOPinConfigure(GPIO_PB6_M0PWM0);
	g_ui32PWMClock = SysCtlClockGet() / 64;
	g_ui32Load = (g_ui32PWMClock / PWM_FREQUENCY) - 1;
	PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, g_ui32Load);

	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, g_ui8Adjust * g_ui32Load / 1000);
	PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
	PWMGenEnable(PWM0_BASE, PWM_GEN_0);

	// usb
	g_bUSBConfigured = false;
    USBBufferInit(&g_sTxBuffer);
    USBBufferInit(&g_sRxBuffer);
    USBStackModeSet(0, eUSBModeForceDevice, 0);
    USBDBulkInit(0, &g_sBulkDevice);

	// dma
	SysCtlPeripheralClockGating(true);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
	SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UDMA);
	IntEnable(INT_UDMAERR);
	uDMAEnable();
	uDMAControlBaseSet(pui8ControlTable);

	// adc
	InitADC00();

	IntMasterEnable();

	UARTprintf("ADC USB\n");

	tUSBRingBufObject sTxRing;
	uint32_t ui32Space = 0, ui32WriteIndex = 0, ui32Count = 0, ui32HeaderIndex = 0;
	bool header_reserved = false;
	bool overflow = false;

	USBBufferInfoGet(&g_sTxBuffer, &sTxRing);
	ui32WriteIndex = sTxRing.ui32WriteIndex % BULK_TX_BUFFER_SIZE;
	ui32Space = USBBufferSpaceAvailable(&g_sTxBuffer);


	UARTprintf("=============================\n");
	UARTprintf("begin: wi:%d sp:%d.\n", ui32WriteIndex, ui32Space);

	//int16_t debug = 1;
	while(true)
	{
		int32_t i, j, r0, r1;
		bool samples_added = false;

		///////////////////////////////////////////////////////////////////////
		// add header if necessary
		if((header_reserved == false) && (ui32Space > BULK_BLOCK_SIZE))
		{
			header_reserved = true;
			ui32Space -= BULK_BLOCK_SIZE;
			ui32HeaderIndex = ui32WriteIndex;
			ui32WriteIndex = (ui32WriteIndex + BULK_BLOCK_SIZE) % BULK_TX_BUFFER_SIZE;
		}

		///////////////////////////////////////////////////////////////////////
		// adc 0
		r0 = g_adcOutIndex0 / ADC_BUFFER_SIZE;
		int offset =((FILTER_SIZE - (ADC_BUFFER_SIZE-1))/ADC_BUFFER_SIZE)+2;
		r1 = (r0 + offset) % ADC_BUFFER_WINDOW_COUNT;

		if((g_adcPingPongStage0[r0]==2) && (g_adcPingPongStage0[r1]==2))
		{
			if(ui32Space > (BULK_BLOCK_SIZE * ADC_DECIMATED_BUFFER_SIZE))
			{
				if((overflow == false) || (g_usb_bytes_to_read > 0))
				{
					GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 4);
					for(j=ADC_DECIMATED_BUFFER_SIZE;j>0;j--)
					{
						// filter
						int32_t accum[NUM_CHANNELS] = { 0 };
						int32_t *filter = g_filter;
						int32_t *src = &g_adcPingPong0[g_adcOutIndex0][0];
						int32_t count = min(FILTER_SIZE, (ADC_BUFFER_SIZE*ADC_BUFFER_WINDOW_COUNT - g_adcOutIndex0));
						for(i=count;i>0;i--)
						{
							int32_t f = *filter;
							accum[0] += *src++ * f;
							accum[1] += *src++ * f;
							accum[2] += *src++ * f;
							accum[3] += *src++ * f;
							accum[4] += *src++ * f;
							accum[5] += *src++ * f;
							accum[6] += *src++ * f;
							accum[7] += *src++ * f;
							filter++;
						}
						int32_t next_count = FILTER_SIZE - count;
						src = &g_adcPingPong0[0][0];
						for(i=next_count;i>0;i--)
						{
							int32_t f = *filter;
							accum[0] += *src++ * f;
							accum[1] += *src++ * f;
							accum[2] += *src++ * f;
							accum[3] += *src++ * f;
							accum[4] += *src++ * f;
							accum[5] += *src++ * f;
							accum[6] += *src++ * f;
							accum[7] += *src++ * f;
							filter++;
						}

						// add to usb buffer
						int16_t *usb_ptr = (int16_t *)&g_pui8USBTxBuffer[ui32WriteIndex];
						*usb_ptr++ = (accum[0] >> 16);
						*usb_ptr++ = (accum[1] >> 16);
						*usb_ptr++ = (accum[2] >> 16);
						*usb_ptr++ = (accum[3] >> 16);
						*usb_ptr++ = (accum[4] >> 16);
						*usb_ptr++ = (accum[5] >> 16);
						*usb_ptr++ = (accum[6] >> 16);
						*usb_ptr++ = (accum[7] >> 16);
						ui32Count += BULK_BLOCK_SIZE;
						ui32Space -= BULK_BLOCK_SIZE;
						ui32WriteIndex = (ui32WriteIndex + BULK_BLOCK_SIZE) % BULK_TX_BUFFER_SIZE;
						g_adcOutIndex0 = (g_adcOutIndex0 + DECIMATION_INC) % (ADC_BUFFER_SIZE*ADC_BUFFER_WINDOW_COUNT);
					}
					samples_added = true;
					GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
				}
			}
			else
			{
				// run out of space, reset
				header_reserved = false;
	            USBBufferFlush(&g_sTxBuffer);
				USBBufferInfoGet(&g_sTxBuffer, &sTxRing);
				ui32Count = 0;
				ui32WriteIndex = sTxRing.ui32WriteIndex % BULK_TX_BUFFER_SIZE;
				ui32Space = USBBufferSpaceAvailable(&g_sTxBuffer);
				header_reserved = false;
				overflow = true;
				UARTprintf("overflow sp:%d\n", ui32Space);
			}
			if(samples_added == false)
			{
				g_adcOutIndex0 = (g_adcOutIndex0 + ADC_BUFFER_SIZE) % (ADC_BUFFER_SIZE*ADC_BUFFER_WINDOW_COUNT);
			}
			g_adcPingPongStage0[r0] = 0;

			if(g_ui8AdjustPending != g_ui8Adjust)
			{
				if(g_ui8AdjustPending < g_ui8Adjust)
				{
					g_ui8Adjust--;
					PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, g_ui8Adjust * g_ui32Load / 1000);
				}
				else if(g_ui8AdjustPending > g_ui8Adjust)
				{
					g_ui8Adjust++;
					PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, g_ui8Adjust * g_ui32Load / 1000);
				}
			}
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, g_fire ? GPIO_PIN_7 : 0);
		}
		else
		{
			for(j=0;j<ADC_BUFFER_WINDOW_COUNT;j++)
			{
				if((g_adcPingPongStage0[j]==1) && (g_adcPingPongStage1[j]==1))
				{
					int32_t *src0 = &g_adcPingPong0[j*ADC_BUFFER_SIZE][0];
					int32_t *src1 = &g_adcPingPong1[j*ADC_BUFFER_SIZE][0];
					for(i=ADC_BUFFER_SIZE;i>0;i--)
					{
						*src0 += *src1++ - 0x1000; g_channelSum[0] -= *src0; *src0++ += g_channelOffset[0];
						*src0 += *src1++ - 0x1000; g_channelSum[1] -= *src0; *src0++ += g_channelOffset[1];
						*src0 += *src1++ - 0x1000; g_channelSum[2] -= *src0; *src0++ += g_channelOffset[2];
						*src0 += *src1++ - 0x1000; g_channelSum[3] -= *src0; *src0++ += g_channelOffset[3];
						*src0 += *src1++ - 0x1000; g_channelSum[4] -= *src0; *src0++ += g_channelOffset[4];
						*src0 += *src1++ - 0x1000; g_channelSum[5] -= *src0; *src0++ += g_channelOffset[5];
						*src0 += *src1++ - 0x1000; g_channelSum[6] -= *src0; *src0++ += g_channelOffset[6];
						*src0 += *src1++ - 0x1000; g_channelSum[7] -= *src0; *src0++ += g_channelOffset[7];
					}
					g_adcPingPongStage0[j] = 2;
					g_adcPingPongStage1[j] = 0;
					g_channelOffsetCount++;
				}
			}
			if(g_channelOffsetCount > CHANNEL_OFFSET_AVERAGE_COUNT)
			{
				g_channelOffset[0] = (g_channelOffset[0] / 2) + (int16_t)(g_channelSum[0] / CHANNEL_OFFSET_AVERAGE_DIVISOR);
				g_channelOffset[1] = (g_channelOffset[1] / 2) + (int16_t)(g_channelSum[1] / CHANNEL_OFFSET_AVERAGE_DIVISOR);
				g_channelOffset[2] = (g_channelOffset[2] / 2) + (int16_t)(g_channelSum[2] / CHANNEL_OFFSET_AVERAGE_DIVISOR);
				g_channelOffset[3] = (g_channelOffset[3] / 2) + (int16_t)(g_channelSum[3] / CHANNEL_OFFSET_AVERAGE_DIVISOR);
				g_channelOffset[4] = (g_channelOffset[4] / 2) + (int16_t)(g_channelSum[4] / CHANNEL_OFFSET_AVERAGE_DIVISOR);
				g_channelOffset[5] = (g_channelOffset[5] / 2) + (int16_t)(g_channelSum[5] / CHANNEL_OFFSET_AVERAGE_DIVISOR);
				g_channelOffset[6] = (g_channelOffset[6] / 2) + (int16_t)(g_channelSum[6] / CHANNEL_OFFSET_AVERAGE_DIVISOR);
				g_channelOffset[7] = (g_channelOffset[7] / 2) + (int16_t)(g_channelSum[7] / CHANNEL_OFFSET_AVERAGE_DIVISOR);
				g_channelOffsetCount = 0;
				g_channelSum[0] = 0;
				g_channelSum[1] = 0;
				g_channelSum[2] = 0;
				g_channelSum[3] = 0;
				g_channelSum[4] = 0;
				g_channelSum[5] = 0;
				g_channelSum[6] = 0;
				g_channelSum[7] = 0;
			}
		}

		///////////////////////////////////////////////////////////////////////
		// process USB if necessary
		if(samples_added)
		{
			// flush usb buffer..
			int32_t usb_want = g_usb_bytes_to_read - BULK_BLOCK_SIZE;
			int32_t high_water = (USB_HIGH_WATER_MULTIPLIER * BULK_BLOCK_SIZE);
			int32_t target_size = min(usb_want, high_water);
			if((ui32Count == target_size) && (target_size > 0))
			{
				// add in header
				int8_t *usb_ptr = (int8_t *)&g_pui8USBTxBuffer[ui32HeaderIndex];
				// 16 byte header
				*usb_ptr++ = 0xAA;
				*usb_ptr++ = 0xAA;
				*usb_ptr++ = (target_size >> 8) & 0xFF;
				*usb_ptr++ = (target_size) & 0xFF;
				for(i=12;i>0;i--)
					*usb_ptr++ = 0;

				USBBufferDataWritten(&g_sTxBuffer, target_size + BULK_BLOCK_SIZE);

				header_reserved = false;
				overflow = false;
				IntMasterDisable();
				g_usb_bytes_to_read -= (target_size + BULK_BLOCK_SIZE);
				if(g_usb_bytes_to_read < 0)
					g_usb_bytes_to_read = 0;
				IntMasterEnable();
				// get usb status
				USBBufferInfoGet(&g_sTxBuffer, &sTxRing);
				ui32WriteIndex = sTxRing.ui32WriteIndex % BULK_TX_BUFFER_SIZE;
				ui32Space = USBBufferSpaceAvailable(&g_sTxBuffer);
				ui32Count = 0;
			}
		}
		else
		{
			if(g_stall0 > 0)
			{
				UARTprintf("stall 0:%d\n", g_stall0);
				g_stall0 = 0;
			}
			if(g_stall1 > 0)
			{
				UARTprintf("stall 1:%d\n", g_stall1);
				g_stall1 = 0;
			}
		}
	}
}

void Timer0IntHandler(void)
{
	// Clear the timer interrupt
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	// Read the current state of the GPIO pin and
	// write back the opposite state
	/*
	if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2))
	{
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
	}
	else
	{
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 4);
	}
	*/
}

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

// ADC interrupt handler. Called on completion of uDMA transfer
void ADC00IntHandler(void)
{
    uint32_t ui32ModeP;
    uint32_t ui32ModeA;

    ADCIntClear(ADC0_BASE, 0);

    ui32ModeP = uDMAChannelModeGet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT);
    if(ui32ModeP == UDMA_MODE_STOP)
    {
        uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT,
                                   UDMA_MODE_PINGPONG,
								   (void *) (ADC0_BASE + ADC_O_SSFIFO0),
								   &g_adcPingPong0[g_adcInIndex0][0], NUM_CHANNELS);
        uDMAChannelEnable(UDMA_CHANNEL_ADC0);
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
    else
    {
    	ui32ModeA = uDMAChannelModeGet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT);
		if(ui32ModeA == UDMA_MODE_STOP)
		{
			uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT,
									   UDMA_MODE_PINGPONG,
									   (void *) (ADC0_BASE + ADC_O_SSFIFO0),
									   &g_adcPingPong0[g_adcInIndex0][0], NUM_CHANNELS);
			uDMAChannelEnable(UDMA_CHANNEL_ADC0);
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

// ADC interrupt handler. Called on completion of uDMA transfer
void ADC10IntHandler(void)
{
    uint32_t ui32ModeP;
    uint32_t ui32ModeA;

    ADCIntClear(ADC1_BASE, 0);

    ui32ModeP = uDMAChannelModeGet(UDMA_CHANNEL_SSI1RX | UDMA_PRI_SELECT);

    if(ui32ModeP == UDMA_MODE_STOP)
    {
        uDMAChannelTransferSet(UDMA_CHANNEL_SSI1RX | UDMA_PRI_SELECT,
                                   UDMA_MODE_PINGPONG,
								   (void *) (ADC1_BASE + ADC_O_SSFIFO0),
								   &g_adcPingPong1[g_adcInIndex1][0], NUM_CHANNELS);
        uDMAChannelEnable(UDMA_CHANNEL_SSI1RX);
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
    else
    {
    	ui32ModeA = uDMAChannelModeGet(UDMA_CHANNEL_SSI1RX | UDMA_ALT_SELECT);
		if(ui32ModeA == UDMA_MODE_STOP)
		{
			uDMAChannelTransferSet(UDMA_CHANNEL_SSI1RX | UDMA_ALT_SELECT,
									   UDMA_MODE_PINGPONG,
									   (void *) (ADC1_BASE + ADC_O_SSFIFO0),
									   &g_adcPingPong1[g_adcInIndex1][0], NUM_CHANNELS);
			uDMAChannelEnable(UDMA_CHANNEL_SSI1RX);
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


// Initialize ADC 0 0 uDMA transfer
void InitADC00(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);

	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0); // 0, 1, 2, 3
	GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0); // 4, 5, 6, 7

	ADCHardwareOversampleConfigure(ADC0_BASE, HW_AVERAGE);
	ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_ALWAYS, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH1);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_TS);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_TS);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 4, ADC_CTL_TS);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 5, ADC_CTL_TS);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 6, ADC_CTL_TS);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 7, ADC_CTL_TS|ADC_CTL_IE|ADC_CTL_END);
	ADCSequenceDMAEnable(ADC0_BASE, 0);

	// adc 0
    uDMAChannelAttributeDisable(UDMA_CHANNEL_ADC0,
                                    UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
                                    UDMA_ATTR_HIGH_PRIORITY |
                                    UDMA_ATTR_REQMASK);

    uDMAChannelControlSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT,
                            UDMA_SIZE_32 | UDMA_SRC_INC_NONE | UDMA_DST_INC_32 |
                              UDMA_ARB_8);

    uDMAChannelControlSet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT,
    						UDMA_SIZE_32 | UDMA_SRC_INC_NONE | UDMA_DST_INC_32 |
                              UDMA_ARB_8);

    uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT,
                               UDMA_MODE_PINGPONG,
							   (void *) (ADC0_BASE + ADC_O_SSFIFO0),
							   &g_adcPingPong0[0][0], NUM_CHANNELS);

    uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT,
                               UDMA_MODE_PINGPONG,
							   (void *) (ADC0_BASE + ADC_O_SSFIFO0),
							   &g_adcPingPong0[1][0], NUM_CHANNELS);
    g_adcInIndex0 = 2;

    // adc 1
	ADCHardwareOversampleConfigure(ADC1_BASE, HW_AVERAGE);
	ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_ALWAYS, 1);
	ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_CH0);
	ADCSequenceStepConfigure(ADC1_BASE, 0, 1, ADC_CTL_CH1);
	ADCSequenceStepConfigure(ADC1_BASE, 0, 2, ADC_CTL_TS);
	ADCSequenceStepConfigure(ADC1_BASE, 0, 3, ADC_CTL_TS);
	ADCSequenceStepConfigure(ADC1_BASE, 0, 4, ADC_CTL_TS);
	ADCSequenceStepConfigure(ADC1_BASE, 0, 5, ADC_CTL_TS);
	ADCSequenceStepConfigure(ADC1_BASE, 0, 6, ADC_CTL_TS);
	ADCSequenceStepConfigure(ADC1_BASE, 0, 7, ADC_CTL_TS|ADC_CTL_IE|ADC_CTL_END);
	ADCPhaseDelaySet(ADC1_BASE, ADC_PHASE_180);
	ADCSequenceDMAEnable(ADC1_BASE, 0);
    uDMAChannelAttributeDisable(UDMA_CHANNEL_SSI1RX,
                                    UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
                                    UDMA_ATTR_HIGH_PRIORITY |
                                    UDMA_ATTR_REQMASK);

    uDMAChannelControlSet(UDMA_CHANNEL_SSI1RX | UDMA_PRI_SELECT,
                            UDMA_SIZE_32 | UDMA_SRC_INC_NONE | UDMA_DST_INC_32 |
                              UDMA_ARB_8);

    uDMAChannelControlSet(UDMA_CHANNEL_SSI1RX | UDMA_ALT_SELECT,
    						UDMA_SIZE_32 | UDMA_SRC_INC_NONE | UDMA_DST_INC_32 |
                              UDMA_ARB_8);

    uDMAChannelTransferSet(UDMA_CHANNEL_SSI1RX | UDMA_PRI_SELECT,
                               UDMA_MODE_PINGPONG,
							   (void *) (ADC1_BASE + ADC_O_SSFIFO0),
							   &g_adcPingPong1[0][0], NUM_CHANNELS);

    uDMAChannelTransferSet(UDMA_CHANNEL_SSI1RX | UDMA_ALT_SELECT,
                               UDMA_MODE_PINGPONG,
							   (void *) (ADC1_BASE + ADC_O_SSFIFO0),
							   &g_adcPingPong1[1][0], NUM_CHANNELS);
    uDMAChannelAssign(UDMA_CH24_ADC1_0);
    g_adcInIndex1 = 2;

    // both
    uDMAChannelEnable(UDMA_CHANNEL_ADC0);
    uDMAChannelEnable(UDMA_CHANNEL_SSI1RX);

	ADCIntClear(ADC0_BASE, 0);
	ADCIntClear(ADC1_BASE, 0);
	ADCIntEnable(ADC0_BASE, 0);
	ADCIntEnable(ADC1_BASE, 0);
	IntEnable(INT_ADC0SS0);
	IntEnable(INT_ADC1SS0);

	ADCSequenceEnable(ADC0_BASE, 0);
	ADCSequenceEnable(ADC1_BASE, 0);
}

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void ConfigureUART(void)
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
        //g_ui32TxCount += ui32MsgValue;
    }

    //
    // Dump a debug message.
    //
    //DEBUG_PRINT("TX complete %d\n", ui32MsgValue);

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
            IntMasterDisable();
            g_usb_bytes_to_read = 0;
            IntMasterEnable();

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
            int i;

            //
            // Read the command 0xAA
            //
            // byte 0: command = 0xAA read
            // byte 1: MSB buffer size
            // byte 2: LSB buffer size

            uint8_t *data = (uint8_t *)pvMsgData;
            uint8_t pwm;
			for(i = ui32MsgValue;i>0;i--)
			{
				switch(g_msg_index)
				{
				default:
				case 0:
					if(*data == 0xAA)
					{
						g_msg_index++;
					}
					break;
				case 1:
					if(*data == 0xAA)
						g_msg_index++;
					else
						g_msg_index = 0;
					break;
				case 2:
					g_usb_receive_size = (uint32_t)*data << 8;
					g_msg_index++;
					break;
				case 3:
					g_usb_receive_size += (uint32_t)*data;
					if(g_usb_receive_size > 0)
					{
						IntMasterDisable();
						g_usb_bytes_to_read += g_usb_receive_size;
						IntMasterEnable();
					}
					g_msg_index++;
					break;
				case 4:
					pwm = *data;
					pwm = min(pwm, PWM_MAX);
					pwm = max(pwm, PWM_MIN);
					g_ui8AdjustPending = pwm;
					//if(g_ui8AdjustPending != g_ui8Adjust)
					//{
					// cause stack overflow
					//UARTprintf("pwm:%d\n", g_ui8AdjustPending);
					//}
					g_msg_index++;
					break;
				case 5:
					g_fire = (*data);
					g_msg_index = 0;
					break;
				}
				data++;
				if(data >= &g_pui8USBRxBuffer[BULK_RX_BUFFER_SIZE])
				{
					data = &g_pui8USBRxBuffer[0];
				}

            }
			USBBufferDataRemoved(&g_sRxBuffer, ui32MsgValue);
            break;
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
