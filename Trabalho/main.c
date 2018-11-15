/* FreeRTOS.org includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Demo includes. */
#include "basic_io.h"

/* CMSIS INCLUDES */
#include "lpc17xx_pinsel.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_timer.h"

#include "light.h"
#include "oled.h"
#include "temp.h"
#include "acc.h"

static uint32_t msTicks = 0;
static uint8_t buf[10];

static uint8_t barPos = 2;

/* Constants */
#define LUMINOSIDADE_ACTIVATION_TIME 100
#define TRIMPOT_ACTIVATION_TIME 500

static void vLuminosidadeTask( void *pvParameters );
static void vTrimpotTask( void *pvParameters );
static void vConsumerTask( void *pvParameters );

xQueueHandle xQueue;

typedef struct Data {
	int value;
	int origin;
} data;

static void intToString(int value, uint8_t* pBuf, uint32_t len, uint32_t base) {
	static const char* pAscii = "0123456789abcdefghijklmnopqrstuvwxyz";
	int pos = 0;
	int tmpValue = value;

	if (pBuf == NULL || len < 2) {
		return;
	}

	if (base < 2 || base > 36) {
		return;
	}

	if (value < 0) {
		tmpValue = -tmpValue;
		value = -value;
		pBuf[pos++] = '-';
	}

	do {
		pos++;
		tmpValue /= base;
	} while (tmpValue > 0);

	if (pos > len) {
		// the len parameter is invalid.
		return;
	}

	pBuf[pos] = '\0';

	do {
		pBuf[--pos] = pAscii[value % base];
		value /= base;
	} while (value > 0);

	return;

}

void SysTick_Handler(void) {
	msTicks++;
}

static uint32_t getTicks(void) {
	return msTicks;
}

static void init_ssp(void) {
	SSP_CFG_Type SSP_ConfigStruct;
	PINSEL_CFG_Type PinCfg;

	/*
	 * Initialize SPI pin connect
	 * P0.7 - SCK;
	 * P0.8 - MISO
	 * P0.9 - MOSI
	 * P2.2 - SSEL - used as GPIO
	 */
	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 7;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 8;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 9;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);

	SSP_ConfigStructInit(&SSP_ConfigStruct);

	// Initialize SSP peripheral with parameter given in structure above
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

	// Enable SSP peripheral
	SSP_Cmd(LPC_SSP1, ENABLE);

}

static void init_i2c(void) {
	PINSEL_CFG_Type PinCfg;

	/* Initialize I2C2 pin connect */
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);

	// Initialize I2C peripheral
	I2C_Init(LPC_I2C2, 100000);

	/* Enable I2C1 operation */
	I2C_Cmd(LPC_I2C2, ENABLE);
}

static void init_adc(void) {
	PINSEL_CFG_Type PinCfg;

	/*
	 * Init ADC pin connect
	 * AD0.0 on P0.23
	 */
	PinCfg.Funcnum = 1;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Pinnum = 23;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);

	/* Configuration for ADC :
	 * 	Frequency at 1Mhz
	 *  ADC channel 0, no Interrupt
	 */
	ADC_Init(LPC_ADC, 1000000);
	ADC_IntConfig(LPC_ADC, ADC_CHANNEL_0, DISABLE);
	ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);
}

void init_stuff() {
	GPIO_SetDir(2, 1 << 0, 1);
	GPIO_SetDir(2, 1 << 1, 1);

	GPIO_SetDir(0, 1 << 27, 1);
	GPIO_SetDir(0, 1 << 28, 1);
	GPIO_SetDir(2, 1 << 13, 1);
	GPIO_SetDir(0, 1 << 26, 1);

	GPIO_ClearValue(0, 1 << 27); //LM4811-clk
	GPIO_ClearValue(0, 1 << 28); //LM4811-up/dn
	GPIO_ClearValue(2, 1 << 13); //LM4811-shutdn

	init_i2c();
	init_ssp();
	init_adc();

	oled_init();
	light_init();

	temp_init(&getTicks);

	if (SysTick_Config(SystemCoreClock / 1000)) {
		while (1);  // Capture error
	}

	light_enable();
	light_setRange(LIGHT_RANGE_4000);

	oled_clearScreen(OLED_COLOR_WHITE);

	oled_putString(1, 1, (uint8_t*) "Light  : ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
	oled_putString(1, 9, (uint8_t*) "Umidade: ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);

	int32_t temperature = 0;
	uint32_t luminosity = 0;
	uint32_t humidity = 0;

	uint8_t isSwitching = 0;
	uint8_t state = 0;
}

int main( void )
{
	init_stuff();

	LPC_GPIO0->FIODIR |= (1<<22);		// configura porta 1.22 como saída

    xQueue = xQueueCreate( 7, sizeof( data ) );

	if( xQueue != NULL )
	{
		xTaskCreate( vLuminosidadeTask, "Task Luminosidade", 240, NULL, 1, NULL );
		xTaskCreate( vTrimpotTask, "Task Trimpot", 240, NULL, 1, NULL );

		xTaskCreate( vConsumerTask, "Consumidor", 240, NULL, 1, NULL );

		vTaskStartScheduler();
	}

	for( ;; );
	return 0;
}
/*-----------------------------------------------------------*/

static void vLuminosidadeTask( void *pvParameters )
{
	portBASE_TYPE xStatus;
	const portTickType xTicksToWait = 100 / portTICK_RATE_MS;

	for( ;; )
	{
		vTaskDelay(LUMINOSIDADE_ACTIVATION_TIME/portTICK_RATE_MS);

		int luminosity = light_read();
		data d = {luminosity, 1};

		xStatus = xQueueSendToBack( xQueue, &d, 0 );
	}
}

static void vTrimpotTask( void *pvParameters )
{
	portBASE_TYPE xStatus;
	const portTickType xTicksToWait = 100 / portTICK_RATE_MS;

	for( ;; )
	{
		vTaskDelay(TRIMPOT_ACTIVATION_TIME/portTICK_RATE_MS);

		ADC_StartCmd(LPC_ADC, ADC_START_NOW);
		while (!(ADC_ChannelGetStatus(LPC_ADC, ADC_CHANNEL_0, ADC_DATA_DONE)));
		int humidity = ADC_ChannelGetData(LPC_ADC, ADC_CHANNEL_0);
		data d = {humidity, 2};

		xStatus = xQueueSendToBack( xQueue, &d, 0 );
	}
}

static void print_to_oled(int value, int line) {
	intToString(value, buf, 10, 10);
	oled_fillRect((1 + 9 * 6), 1 + line * 8, 80, 16, OLED_COLOR_WHITE);
	oled_putString((1 + 9 * 6), 1 + line * 8, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
}

static void vConsumerTask( void *pvParameters )
{
	portBASE_TYPE xStatus;
	int luminosidade = 1;
	int trimpot = 1;

	for( ;; )
	{
		 vTaskDelay(50/portTICK_RATE_MS);

		data d;
		xStatus = xQueueReceive( xQueue, &d, 0 );

		if (d.origin == 2) { 	   // trimpot
			trimpot = d.value;

			if (d.value > 3200) {  // > 80%
				LPC_GPIO0->FIOSET |= (1<<22);		// liga o LED
			} else {
				LPC_GPIO0->FIOCLR |= (1<<22);		// desliga o LED
			}
		} else {
			luminosidade = d.value;
		}

		print_to_oled(luminosidade, 1);
		print_to_oled(trimpot, 2);
	}
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* This function will only be called if an API call to create a task, queue
	or semaphore fails because there is too little heap RAM remaining. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	/* This function will only be called if a task overflows its stack.  Note
	that stack overflow checking does slow down the context switch
	implementation. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* This example does not use the idle hook to perform any processing. */
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
	/* This example does not use the tick hook to perform any processing. */
}
