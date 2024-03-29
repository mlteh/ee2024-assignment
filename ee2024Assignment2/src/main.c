#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_uart.h"
#include "light.h"
#include "temp.h"
#include "joystick.h"
#include "pca9532.h"
#include "acc.h"
#include "oled.h"
#include "rgb.h"
#include "led7seg.h"
#include "math.h"

#include <stdio.h>
#define PINSEL_EINT0	20
#define RGB_BOTH 0x5

//interrupt priority values
int32_t finalPriority, PG = 5, PPEINT3 = 0b11, SPEINT3 = 0b000;
int32_t PPEINT0 = 0b10, SPEINT0 = 0b000;

volatile uint32_t msTicks; // counter for 1ms SysTicks
volatile uint32_t prevMstVal = 0;
volatile uint32_t prevMst = 0;
volatile uint32_t brPrevMst = 0;
volatile uint32_t prevMstRed = 0;
volatile uint8_t butSW3 = 0;
uint32_t temperature = 0;
uint32_t light = 1;
uint8_t moved = 0;
uint8_t monitorMode = 0;

volatile uint32_t prevSW4Count = 0;
const uint32_t lowestLight = 0;
const uint32_t highestLight = 2000;
const uint32_t lightThreshold = 20;
const double tempThreshold = 30.0; //change for assessment
uint8_t lightWarning = 0;
uint8_t tempWarning = 0;
uint8_t moveWarning = 0;

uint8_t blinkBothLeds = 0;
uint8_t blinkRedLed = 0;
uint8_t blinkBlueLed = 0;

uint16_t led16Val = 0;
uint8_t numToShift = 16;

uint8_t val7Seg = 0;
uint32_t prevMstSecs = 0;

volatile uint32_t tempWaveCount = 0;
uint32_t t1 = 0;
uint32_t t2 = 0;
int32_t currentTemperature = 0;

//flags used for the blinking of RGB LEDs
volatile uint8_t blueLedFlag = 0;
volatile uint8_t redLedFlag = 0;
volatile uint8_t brLedFlag = 0;

uint32_t uartCount = 0;
uint8_t prevMstValUart = 0;
static char* msg = NULL;
static char* msgMovementDarkness = "Movement in darkness was Detected.\r\n";
static char* msgFire = "Fire was Detected.\r\n";
static char* msgEnterMonitor = "Entering MONITOR Mode.\r\n";

//*****function prototype declarations*****//

void blinkBothRGB();
void blinkBlueRGB();
void blinkRedRGB();
int32_t temp_interrupt(uint32_t, uint32_t);

// ****************
//  SysTick_Handler - just increment SysTick counter
void SysTick_Handler(void) {
	msTicks++;
}
uint32_t getMsTicks() {
	return msTicks;
}
// EINT3 Interrupt Handler
void EINT3_IRQHandler(void) {

	//NVIC_ClearPendingIRQ(EINT0_IRQn);
	if ((LPC_GPIOINT ->IO2IntStatF >> 5) & 0x1) {
		lightWarning = 1;
		LPC_GPIOINT ->IO2IntClr = (1 << 5);
	}

	if ((LPC_GPIOINT ->IO0IntStatF >> 24) & 0x1) {

		int pin1 = (GPIO_ReadValue(0) >> 24);
		int pin2 = (GPIO_ReadValue(0) >> 25);
		//printf( "1st: %d 2nd: %d \n", pin1 , pin2);

		if (((pin1 == 0 && pin2 == 0) || (pin1 == 4 && pin2 == 2))
				&& numToShift <= 15)
			numToShift++;
		else if (pin1 > pin2 && pin1 != 4 && numToShift > 0)
			numToShift--;

		LPC_GPIOINT ->IO0IntClr = (1 << 24);
	}

	if ((LPC_GPIOINT ->IO0IntStatF >> 2) & 0x1) {
		tempWaveCount++;
		if (tempWaveCount == 170) {
			t2 = msTicks;
			currentTemperature = temp_interrupt(t1, t2);
			tempWaveCount = 0;
			t1 = t2;
		}
		LPC_GPIOINT ->IO0IntClr = (1 << 2);
	}

}
void EINT0_IRQHandler(void) {
	// clear interrupt flag for eint0
	LPC_SC ->EXTINT = (1 << 0);
	butSW3 = 1;
}

static void monitor7Seg(uint32_t time) {
	switch (time) {

	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
	case 6:
	case 7:
	case 8:
	case 9:
		led7seg_setChar('0' + time, 0);
		break;
	case 10:
	case 11:
	case 12:
	case 13:
	case 14:
	case 15:
		led7seg_setChar('A' + time % 10, 0);
		break;
	}

}

void setRGB(uint8_t ledCol) {
	if (ledCol == RGB_BOTH) {
		GPIO_SetValue(2, (1 << 0));
		GPIO_SetValue(0, (1 << 26));
	} else {
		if (ledCol == RGB_RED) {
			GPIO_SetValue(2, (1 << 0));
		} else {
			GPIO_ClearValue(2, (1 << 0));
		}
		if (ledCol == RGB_BLUE) {
			GPIO_SetValue(0, (1 << 26));
		} else {
			GPIO_ClearValue(0, (1 << 26));
		}
	}
}

void blinkRGB() {
	if (lightWarning && moved) {
		blinkBlueLed = 1;
	} else if (tempWarning) {
		blinkRedLed = 1;
	}

	if (blinkRedLed && blinkBlueLed)
		blinkBothRGB();
	else if (blinkBlueLed)
		blinkBlueRGB();
	else if (blinkRedLed)
		blinkRedRGB();

}

void blinkBothRGB() {
	uint32_t currentMst = msTicks;
	if (currentMst - brPrevMst > 333) {
		brLedFlag = !brLedFlag;
		brPrevMst = currentMst;
	}
	if (brLedFlag) {
		setRGB(RGB_BOTH);
	} else {
		setRGB(8);
	}
}

void blinkBlueRGB() {
	uint32_t currentMst = msTicks;
	if (currentMst - prevMst > 333) {
		blueLedFlag = !blueLedFlag;
		prevMst = currentMst;
	}
	if (blueLedFlag)
		setRGB(RGB_BLUE);
	else
		setRGB(8);

}

void blinkRedRGB() {
	uint32_t currentMstRed = msTicks;
	if (currentMstRed - prevMstRed > 333) {
		redLedFlag = !redLedFlag;
		prevMstRed = currentMstRed;
	}

	if (redLedFlag)
		setRGB(RGB_RED);
	else
		setRGB(8);

}

void activate_supplement_lights() {
	led16Val = (65535 >> numToShift);
	pca9532_setLeds(led16Val, 0xffff);
}

void pinsel_uart3(void) {
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 0;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 1;
	PINSEL_ConfigPin(&PinCfg);
}
void init_uart(void) {
	UART_CFG_Type uartCfg;
	uartCfg.Baud_rate = 115200;
	uartCfg.Databits = UART_DATABIT_8;
	uartCfg.Parity = UART_PARITY_NONE;
	uartCfg.Stopbits = UART_STOPBIT_1;
	//pin select for uart3;
	pinsel_uart3();
	//supply power & setup working parameters for uart3
	UART_Init(LPC_UART3, &uartCfg);
	//enable transmit for uart3
	UART_TxCmd(LPC_UART3, ENABLE);
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

static void init_GPIO(void) {

	//Initialize button sw4
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 0;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 1;
	PinCfg.Pinnum = 31;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(1, 1 << 31, 0);

	//initialize sw5
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 24;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(1, 1 << 24, 0);
	PinCfg.Pinnum = 25;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(1, 1 << 25, 0);

	//Initialize "temp interrupt"
	PinCfg.Funcnum = 0;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(1, 1 << 2, 0);

	//Initialize button sw3
	PinCfg.Funcnum = 0;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 10;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2, 1 << 10, 0);

	//Initialize button loight
	PinCfg.Funcnum = 0;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 5;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2, 1 << 5, 0);

}

static void reset() {
	led7seg_setChar(255, FALSE);
	numToShift = 16;
	pca9532_setLeds(0, 0xffff);
	setRGB(8);
	lightWarning = 0;
	tempWarning = 0;
	blinkBlueLed = 0;
	blinkRedLed = 0;
	blinkBothLeds = 0;

}

static void updateOledDisplay(int32_t x, int32_t y, int32_t z, int32_t value) {
	unsigned char oledString[100] = "x";
	int light = light_read();
	double temp = currentTemperature / 10.0;

	//NNN_-_T*****_L*****_AX*****_AY*****_AZ*****\r\n
	if (value != prevMstVal) {

		sprintf(oledString, "L:%d     \n", light);
		oled_putString(1, 10, oledString, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		sprintf(oledString, "T:%0.1lfC\n", temp);
		oled_putString(40, 10, oledString, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		sprintf(oledString, "X:%d   \n", x);
		oled_putString(1, 20, oledString, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		sprintf(oledString, "Y:%d   \n", y);
		oled_putString(1, 30, oledString, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		sprintf(oledString, "Z:%d   \n", z);
		oled_putString(1, 40, oledString, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

		if (light < lightThreshold)
			oled_putString(20, 50, "Dark\n", OLED_COLOR_BLACK,
					OLED_COLOR_WHITE);
		else
			oled_putString(20, 50, "    \n", OLED_COLOR_WHITE,
					OLED_COLOR_BLACK);
		if (temp > tempThreshold)
			oled_putString(50, 50, "Hot\n", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
		else
			oled_putString(50, 50, "   \n", OLED_COLOR_WHITE, OLED_COLOR_BLACK);

	}
	prevMstVal = value;

}

static int checkSW4(uint32_t currentSW4Count) {
	int x = 1;
	if (currentSW4Count - prevSW4Count < 500) {
		x = 0;
	}
	prevSW4Count = currentSW4Count;
	return x;
}

int main(void) {

	/* Enable and setup SysTick Timer at a periodic rate 1 msec*/
	;
	SysTick_Config(SystemCoreClock / 1000);

	init_i2c();
	init_ssp();
	init_GPIO();
	int btn1 = 0;
	pca9532_init();

	temp_init(getMsTicks);
	pca9532_init();
	acc_init();
	init_uart();

	/***initialize for light sensor interrupt****/
	light_enable();
	light_setRange(LIGHT_RANGE_4000);
	light_setLoThreshold(lightThreshold);
	light_setHiThreshold(2000);
	light_setIrqInCycles(LIGHT_CYCLE_1);
	light_clearIrqStatus();

	LPC_GPIOINT ->IO2IntClr = 1 << 5;
	LPC_GPIOINT ->IO2IntEnF |= 1 << 5;
	/*** -----------------------------------****/

	// Enable GPIO Interrupt P0.2
	LPC_GPIOINT ->IO0IntEnF |= 1 << 2;

	//Enable sw5 interrupt**//
	LPC_GPIOINT ->IO0IntEnF |= 1 << 24;
	//LPC_GPIOINT ->IO0IntEnF |= 1 << 25;
	//*********************//

	//** Enabling interrupts for ENIT0 ** //
	LPC_SC ->EXTINT = (1 << 0);
	LPC_PINCON ->PINSEL4 = (1 << PINSEL_EINT0); /* Configure P2_10 as EINT0/1 */

	//Setting interrupt priorities
	NVIC_SetPriorityGrouping(5);
	finalPriority = NVIC_EncodePriority(PG, PPEINT3, SPEINT3);
	NVIC_SetPriority(EINT3_IRQn, finalPriority);
	finalPriority = NVIC_EncodePriority(PG, PPEINT0, SPEINT0);
	NVIC_SetPriority(EINT0_IRQn, finalPriority);
	NVIC_SetPriority(SysTick_IRQn, 0x00);

	// Enable EINT0 & EINT3 interrupt
	NVIC_ClearPendingIRQ(EINT3_IRQn);
	NVIC_ClearPendingIRQ(EINT0_IRQn);
	NVIC_EnableIRQ(EINT0_IRQn);
	NVIC_EnableIRQ(EINT3_IRQn);

	oled_init();
	rgb_init();
	led7seg_init();

	// Setup Accelerometer
	int32_t xoff = 0;
	int32_t yoff = 0;
	int32_t zoff = 0;

	int8_t x = 0;
	int8_t y = 0;
	int8_t z = 0;

	acc_read(&x, &y, &z);
	xoff = 0 - x;
	yoff = 0 - y;
	zoff = 64 - z;

	GPIO_SetDir(2, 1 << 0, 1); /**/

	int8_t moving = 0;
	int8_t lastmove = 64;
	int8_t delta = 0;

	while (1) {

		btn1 = (GPIO_ReadValue(1) >> 31) & 0x01;

		if (!btn1 && checkSW4(msTicks)) {
			if (monitorMode == 0) {
				oled_putString(1, 0, "MONITOR", OLED_COLOR_WHITE,
						OLED_COLOR_BLACK);
				UART_Send(LPC_UART3, (uint8_t *) msgEnterMonitor,
						strlen(msgEnterMonitor), BLOCKING);
				t1 = msTicks;
				light_read();	//simply just to meet assignment requirements

			}
			monitorMode = !monitorMode;
		}
		if (butSW3) {
			x = 0;
			y = 0;
			z = 0;
			butSW3 = 0;
			acc_read(&x, &y, &z);
			xoff = 0 - x;
			yoff = 0 - y;
			zoff = 64 - z;
			reset();
			val7Seg = 0;
			t1 = msTicks;
		}

		if (monitorMode) {

			if (blinkBlueLed) {
				activate_supplement_lights();
			}

			uint32_t currentMstValue = msTicks;
			light_clearIrqStatus();

			acc_read(&x, &y, &z);
			x = x + xoff;
			y = y + yoff;
			z = z + zoff;

			blinkRGB();
			lightWarning = 0;

			if (currentTemperature / (10.0) > tempThreshold) {
				tempWarning = 1;
			} else
				tempWarning = 0;

			moving = sqrt(x * x + y * y + z * z);
			delta = moving - lastmove;
			lastmove = moving;

			if (delta > 3 || delta < -3) {
				moved = 1;
			} else
				moved = 0;
			//printf ("%d\n",val7Seg);

			if (prevMstSecs != currentMstValue / 1000) {
				monitor7Seg(val7Seg);

				prevMstSecs = currentMstValue / 1000;

				if (val7Seg % 5 == 0 && val7Seg != 0) {
					updateOledDisplay(x, y, z, (currentMstValue / 1000));

					if (val7Seg == 15 ) {  //&& ((currentMstValue / 1000) != prevMstValUart

						if (blinkRedLed)
							UART_Send(LPC_UART3, (uint8_t *) msgFire,
									strlen(msgFire), BLOCKING);
						if (blinkBlueLed)
							UART_Send(LPC_UART3,
									(uint8_t *) msgMovementDarkness,
									strlen(msgMovementDarkness), BLOCKING);
						asprintf(&msg,
								"%03d_-_T%0.1lf_L%d_AX%03d_AY%03d_AZ%03d\r\n",
								uartCount++, currentTemperature / 10.0,
								light_read(), x, y, z);
						UART_Send(LPC_UART3, (uint8_t *) msg, strlen(msg),
								BLOCKING);

						prevMstValUart = currentMstValue / 1000;
					}
				}

				val7Seg = (val7Seg + 1) % 16;
			}

		} else {
			oled_clearScreen(OLED_COLOR_BLACK);
			reset();
		}

		Timer0_Wait(1);
	}

}

void check_failed(uint8_t *file, uint32_t line) {
	/* User can add his own implementation to report the file name and line number,
	 "ex: printf"("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
		;
}
