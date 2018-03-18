/*********
 *   A demo example using several of the peripherals on the base board
 *
 *   Copyright(C) 2011, EE2024
 *   All rights reserved.
 *
 **********/

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"
#include "led7seg.h"  //(Place aft pinsel.h as you need some pins to configured as SSP & SPI)

#include "joystick.h"
#include "pca9532.h"
#include "acc.h"
#include "oled.h"
#include "rgb.h"
#include "light.h"
#include "temp.h"
#include "math.h"
#include "lpc17xx_uart.h"

#define OBSTACLE_NEAR_THRESHOLD 3000
#define TEMP_HIGH_THRESHOLD 28
#define ACC_THRESHOLD 0.4
#define FORWARD 3
#define STATIONARY 2
#define REVERSE 1

volatile uint32_t msTicks = 0;

uint8_t RESET_INITIAL = 1;
uint32_t INITIAL_TIME = 0;
uint32_t CURRENT_TIME = 0;

int RED_LED = 0;
int BLUE_LED = 0;
int mode = 2;

uint8_t col_detect = 0;
uint8_t temp_high = 0;
uint8_t oled_disp[40] = {};
uint8_t uartmessage[200] = {};

uint8_t fwd_uart = 0;
uint8_t rev_uart = 0;
uint8_t obs_avd = 1;
uint8_t obstacle_near = 0;
uint8_t lock_status = 0;

static uint8_t barPos = 2;

static void moveBar(uint8_t steps, uint8_t dir)
{
    uint16_t ledOn = 0;

    if (barPos == 0)
        ledOn = (1 << 0) | (3 << 14);
    else if (barPos == 1)
        ledOn = (3 << 0) | (1 << 15);
    else
        ledOn = 0x07 << (barPos-2);

    barPos += (dir*steps);
    barPos = (barPos % 16);

    pca9532_setLeds(ledOn, 0xffff);
}


static void drawOled(uint8_t joyState)
{
    static int wait = 0;
    static uint8_t currX = 48;
    static uint8_t currY = 32;
    static uint8_t lastX = 0;
    static uint8_t lastY = 0;

    if ((joyState & JOYSTICK_CENTER) != 0) {
        oled_clearScreen(OLED_COLOR_BLACK);
        return;
    }

    if (wait++ < 3)
        return;

    wait = 0;

    if ((joyState & JOYSTICK_UP) != 0 && currY > 0) {
        currY--;
    }

    if ((joyState & JOYSTICK_DOWN) != 0 && currY < OLED_DISPLAY_HEIGHT-1) {
        currY++;
    }

    if ((joyState & JOYSTICK_RIGHT) != 0 && currX < OLED_DISPLAY_WIDTH-1) {
        currX++;
    }

    if ((joyState & JOYSTICK_LEFT) != 0 && currX > 0) {
        currX--;
    }

    if (lastX != currX || lastY != currY) {
        oled_putPixel(currX, currY, OLED_COLOR_WHITE);
        lastX = currX;
        lastY = currY;
    }
}


#define NOTE_PIN_HIGH() GPIO_SetValue(0, 1<<26);
#define NOTE_PIN_LOW()  GPIO_ClearValue(0, 1<<26);




static uint32_t notes[] = {
        2272, // A - 440 Hz
        2024, // B - 494 Hz
        3816, // C - 262 Hz
        3401, // D - 294 Hz
        3030, // E - 330 Hz
        2865, // F - 349 Hz
        2551, // G - 392 Hz
        1136, // a - 880 Hz
        1012, // b - 988 Hz
        1912, // c - 523 Hz
        1703, // d - 587 Hz
        1517, // e - 659 Hz
        1432, // f - 698 Hz
        1275, // g - 784 Hz
};

static void playNote(uint32_t note, uint32_t durationMs) {

    uint32_t t = 0;

    if (note > 0) {

        while (t < (durationMs*1000)) {
            NOTE_PIN_HIGH();
            Timer0_us_Wait(note / 2);
            //delay32Us(0, note / 2);

            NOTE_PIN_LOW();
            Timer0_us_Wait(note / 2);
            //delay32Us(0, note / 2);

            t += note;
        }

    }
    else {
    	Timer0_Wait(durationMs);
        //delay32Ms(0, durationMs);
    }
}

static uint32_t getNote(uint8_t ch)
{
    if (ch >= 'A' && ch <= 'G')
        return notes[ch - 'A'];

    if (ch >= 'a' && ch <= 'g')
        return notes[ch - 'a' + 7];

    return 0;
}

static uint32_t getDuration(uint8_t ch)
{
    if (ch < '0' || ch > '9')
        return 400;

    /* number of ms */

    return (ch - '0') * 200;
}

static uint32_t getPause(uint8_t ch)
{
    switch (ch) {
    case '+':
        return 0;
    case ',':
        return 5;
    case '.':
        return 20;
    case '_':
        return 30;
    default:
        return 5;
    }
}

static void playSong(uint8_t *song) {
    uint32_t note = 0;
    uint32_t dur  = 0;
    uint32_t pause = 0;

    /*
     * A song is a collection of tones where each tone is
     * a note, duration and pause, e.g.
     *
     * "E2,F4,"
     */

    while(*song != '\0') {
        note = getNote(*song++);
        if (*song == '\0')
            break;
        dur  = getDuration(*song++);
        if (*song == '\0')
            break;
        pause = getPause(*song++);

        playNote(note, dur);
        //delay32Ms(0, pause);
        Timer0_Wait(pause);

    }
}

static uint8_t * song = (uint8_t*)"C2.C2,D4,C4,F4,E8,";
        //"C2.C2,D4,C4,F4,E8,C2.C2,D4,C4,G4,F8,C2.C2,c4,A4,F4,E4,D4,A2.A2,H4,F4,G4,F8,";
        //"D4,B4,B4,A4,A4,G4,E4,D4.D2,E4,E4,A4,F4,D8.D4,d4,d4,c4,c4,B4,G4,E4.E2,F4,F4,A4,A4,G8,";



static void init_ssp(void)
{
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

static void init_i2c(void)
{
	PINSEL_CFG_Type PinCfg;

	/* Initialize I2C2 pin connect */
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);  //P0.10
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);  //P0.11

	// Initialize I2C peripheral, speed = 100000
	I2C_Init(LPC_I2C2, 100000);

	/* Enable I2C1 operation */
	I2C_Cmd(LPC_I2C2, ENABLE);
}

static void init_GPIO(void)
{
	// Initialize button
	/* Line 261 */	PINSEL_CFG_Type PinCfg;
	/* Line 262 */
	/* Line 263 */	    PinCfg.Funcnum = 0;
	/* Line 264 */	    PinCfg.OpenDrain = 0;
	/* Line 265 */	    PinCfg.Pinmode = 0;
	/* Line 266 */	    PinCfg.Portnum = 1;
	/* Line 267 */	    PinCfg.Pinnum = 31;
                     	PINSEL_ConfigPin(&PinCfg);  // SW4 P1.31
                     	GPIO_SetDir(1, 1<<31, 0);

                     	PinCfg.Funcnum = 1;
                     	PinCfg.Portnum = 2;
			            PinCfg.Pinnum = 10;
	             	    PINSEL_ConfigPin(&PinCfg);  //SW3 P0.4(BL_EN) or P2.10(PIO2_9)
	                    GPIO_SetDir(2, 1<<10, 0);


	                    PinCfg.Funcnum = 0;
	                    PinCfg.Portnum = 2;
	                    PinCfg.Pinnum = 0;
					    PINSEL_ConfigPin(&PinCfg);  //Red LED P2.0
					    GPIO_SetDir(2, 1<<0, 1);

					    PinCfg.Portnum = 0;
					    PinCfg.Pinnum = 26;
					    PINSEL_ConfigPin(&PinCfg);  //Blue LED P0.26
					    GPIO_SetDir(0, 1<<26, 1);

					    PinCfg.Portnum = 2;
					    PinCfg.Pinnum = 1;
					    PINSEL_ConfigPin(&PinCfg);  //Green LED P2.1
					    GPIO_SetDir(2, 1<<1, 1);


                     	PinCfg.Portnum = 0;
			            PinCfg.Pinnum = 24;
	             	    PINSEL_ConfigPin(&PinCfg);  //Rotary Switch P0.24 or P0.25
//	                    GPIO_SetDir(0, 1<<24, 0);

}

void ready_uart(void)
{
	// PINSEL Configuration
	PINSEL_CFG_Type CPin;
	    CPin.OpenDrain = 0;
	    CPin.Pinmode = 0;
	    CPin.Funcnum = 2;
	    CPin.Pinnum = 0;   //TXD3
	    CPin.Portnum = 0;
	PINSEL_ConfigPin(&CPin);
	    CPin.Pinnum = 1;
	    CPin.Portnum = 0;   //RXD3
	PINSEL_ConfigPin(&CPin);

	// Initialise and enable the UART. Not enabling the UART will lead to a hard fault
	UART_CFG_Type UCfg;
	    UCfg.Baud_rate = 115200;
	    UCfg.Databits = UART_DATABIT_8;
	    UCfg.Parity = UART_PARITY_NONE;
	    UCfg.Stopbits = UART_STOPBIT_1;

	// supply power & setup working parameters for UART3
	UART_Init(LPC_UART3, &UCfg);

	// enable transmit for uart3
	UART_TxCmd(LPC_UART3, ENABLE);

	// FIFO configuration- For system enhancements only
	//
}

void SysTick_Handler()
{
	msTicks++;
}

uint32_t getTicks(void)

{
	return msTicks;
}

static void DISPSEG (int i)
{
  char dispseg [16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', '8', 'C', '0', 'E', 'F'};
  led7seg_setChar(dispseg[i], FALSE);
}
static void BLINK_RED (uint32_t x)
{
	CURRENT_TIME = getTicks();
	if (RESET_INITIAL)
	{
		if (CURRENT_TIME - INITIAL_TIME >= x)
		{
			INITIAL_TIME = CURRENT_TIME;
			RESET_INITIAL = 0;
			GPIO_SetValue(2,1<<0);
		}
	}
	else
	{
		if (CURRENT_TIME - INITIAL_TIME >= x)
		{
			INITIAL_TIME = CURRENT_TIME;
			RESET_INITIAL = 1;
			GPIO_ClearValue(2,1<<0);
		}
	}
//	CURRENT_TIME = getTicks();
//	INITIAL_TIME = getTicks();
//	if(RESET_INITIAL)
//	{
//		GPIO_SetValue(2,1<<0);
//		while(CURRENT_TIME - INITIAL_TIME <= x)
//		{
//			CURRENT_TIME = getTicks();
//		}
//		RESET_INITIAL = 0;
//	}
//	else
//	{
//		GPIO_ClearValue(2,1<<0);
//		while(CURRENT_TIME - INITIAL_TIME <= x)
//		{
//			CURRENT_TIME = getTicks();
//		}
//		RESET_INITIAL = 1;
//	}

//	    	if (RESET_INITIAL)
//	    	{
//	    		if (CURRENT_TIME - INITIAL_TIME <= x)
//	    		{
//	    			GPIO_SetValue(2,1<<0);
//	    		}
//
//	    		else
//	    		{
//	    			RESET_INITIAL = 0;
//	    			INITIAL_TIME = CURRENT_TIME;
//	    			GPIO_ClearValue(2,1<<0);
//	    		}
//	    	}
//	    	else
//	    	{
//	    		if (CURRENT_TIME - INITIAL_TIME <= x)
//	    		{
//	    			GPIO_ClearValue(2,1<<0);
//	    		}
//	    		else
//	    		{
//	    			RESET_INITIAL = 1;
//	    			INITIAL_TIME = CURRENT_TIME;
//	    			GPIO_SetValue(2,1<<0);
//	    		}
//	    	}

}

static void BLINK_BLUE (uint32_t x)
{

	CURRENT_TIME = getTicks();
		if (RESET_INITIAL)
		{
			if (CURRENT_TIME - INITIAL_TIME >= x)
			{
				INITIAL_TIME = CURRENT_TIME;
				RESET_INITIAL = 0;
				GPIO_SetValue(0,1<<26);
			}
		}
		else
		{
			if (CURRENT_TIME - INITIAL_TIME >= x)
			{
				INITIAL_TIME = CURRENT_TIME;
				RESET_INITIAL = 1;
				GPIO_ClearValue(0,1<<26);
			}
		}


//	CURRENT_TIME = getTicks();
//
//	    	if (RESET_INITIAL)
//	    	{
//	    		if (CURRENT_TIME - INITIAL_TIME <= x)
//	    		{
//	    			GPIO_SetValue(0,1<<26);
//	    		}
//
//	    		else
//	    		{
//	    			RESET_INITIAL = 0;
//	    			INITIAL_TIME = CURRENT_TIME;
//	    			GPIO_ClearValue(0,1<<26);
//	    		}
//	    	}
//	    	else
//	    	{
//	    		if (CURRENT_TIME - INITIAL_TIME <= x)
//	    		{
//	    			GPIO_ClearValue(0,1<<26);
//	    		}
//	    		else
//	    		{
//	    			RESET_INITIAL = 1;
//	    			INITIAL_TIME = CURRENT_TIME;
//	    			GPIO_SetValue(0,1<<26);
//	    		}
//	    	}


}
static void INTERRUPT_PRIORYTY (void)
{
	NVIC_SetPriority(SysTick_IRQn, 1); //System Timer has highest priority

	// Enable EINT3 interrupt for Light Sensor has higher priority than button.
	NVIC_ClearPendingIRQ(EINT3_IRQn);
	NVIC_SetPriority(EINT3_IRQn, 2);
	NVIC_EnableIRQ(EINT3_IRQn);

	// Enable EINT0 interrupt for SW3
	LPC_SC->EXTINT = 1;
	NVIC_ClearPendingIRQ(EINT0_IRQn);
	NVIC_SetPriority(EINT0_IRQn, 3);
	NVIC_EnableIRQ(EINT0_IRQn);
}


static void Interrupt_Initialisation (void)
{
	//For Light Sensor GPIO Interrupt
	light_clearIrqStatus();
	LPC_GPIOINT->IO2IntClr |= 1<<5;
	LPC_GPIOINT->IO2IntEnF|= 1<<5;
	NVIC_EnableIRQ(EINT3_IRQn);

	//For SW3 Interrupt
	LPC_SC->EXTMODE |= 1;
	LPC_SC->EXTPOLAR &~ 1;
	NVIC_EnableIRQ(EINT0_IRQn);
}

static void LIGHT_INTERRUPT_CONFIG (void)
{
	light_setRange(LIGHT_RANGE_4000);
	light_setIrqInCycles(LIGHT_CYCLE_1);
	light_setLoThreshold(0);
	light_setHiThreshold(OBSTACLE_NEAR_THRESHOLD);
}
void COLLISION_DETECTION(int8_t x)
{
	if (x/64.0 > ACC_THRESHOLD)
	{
	  BLUE_LED = 1;
	  col_detect = 1;
	  sprintf(oled_disp,"Air bag release");
      oled_putString(0, 30,(uint8_t*)oled_disp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    }
}

void BATTERY_CONDITION_MONITORING (int32_t temp)
{

	if ( temp/10.0 > TEMP_HIGH_THRESHOLD)
	{
	  RED_LED = 1;
	  temp_high = 1;
	  sprintf(oled_disp,"Temp. too high");
	  oled_putString(0, 40,(uint8_t*)oled_disp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
   }
}

void OBSTACLE_AVOIDANCE(uint32_t light)
{
	int n;
	if(light <= OBSTACLE_NEAR_THRESHOLD)
	{
		n = (int) light/187.5;
	    pca9532_setLeds(pow(2,n)-1,pow(2,16-n)-1);
	}
}
void OBSTACLE_NEAR(void)
{
    pca9532_setLeds(0xFFFF,0);
	sprintf(oled_disp,"Obstacle near");
	oled_putString(0, 30,(uint8_t*)oled_disp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	if (obs_avd)
	{
	sprintf(uartmessage,"Obstacle near\r\n");
	UART_Send(LPC_UART3, (uint8_t *)uartmessage, strlen(uartmessage), BLOCKING);
	obs_avd = 0;
	}
	obstacle_near = 0;

}

void EINT3_IRQHandler(void)
{
	if ((LPC_GPIOINT->IO2IntStatF) >> 5 & 1)
	{
		obstacle_near = 1;
		obs_avd = 1;
		LPC_GPIOINT->IO2IntClr = 1<<5;
		light_clearIrqStatus();
	}
}

uint8_t pressCount = 0;



void EINT0_IRQHandler(void)
{
	if ((LPC_SC->EXTINT) >> 0 & 1)
	{
		LPC_SC->EXTINT = 1 << 0;

		//printf("SW3 Pressed\n");

		pressCount++;
	}
}


int ModeControl(int currentMode)
{
	int newMode = currentMode;

	uint32_t startTime = 0;
	uint32_t currentTime = 0;

	if(pressCount > 0)
	{
		if ((currentMode == REVERSE) || (currentMode == FORWARD))
		{
			oled_clearScreen(OLED_COLOR_BLACK);
			newMode = STATIONARY;
		}
		else
		{
			startTime = getTicks();
			currentTime = getTicks();

			while (currentTime - startTime <= 1000)
			{
				currentTime = getTicks();
			}

			if (pressCount == 1)
			{
				oled_clearScreen(OLED_COLOR_BLACK);
				fwd_uart = 1;
				newMode = FORWARD;
			}
			else
			{
				oled_clearScreen(OLED_COLOR_BLACK);
				rev_uart = 1;
				newMode = REVERSE;
			}
		}
	}

	pressCount = 0;
	return newMode;
}


uint8_t RESET_INITIAL_MAIN = 1;
uint32_t INITIAL_TIME_MAIN = 0;
uint32_t CURRENT_TIME_MAIN = 0;

int main (void) {
	uint8_t state = 0;

    int32_t xoff = 0;
    int32_t yoff = 0;
    int32_t zoff = 0;

    int8_t x = 0;

    int8_t y = 0;
    int8_t z = 0;
    uint8_t dir = 1;
    uint8_t wait = 0;

//    uint8_t SW4 = 1;
    int counter = 0;

    init_i2c();
    init_ssp();
    init_GPIO();

    pca9532_init();
    joystick_init();
    acc_init();
    oled_init();
    led7seg_init();
    light_enable();
    SysTick_Config(SystemCoreClock/1000); //1000Hz = 1ms, interrupt occurs every 1ms
    temp_init(&getTicks);

    /*
     * Assume base board in zero-g position when reading first value.
     */
    acc_read(&x, &y, &z);
    xoff = 0-x;
    yoff = 0-y;
    zoff = 64-z;

    Interrupt_Initialisation();
    INTERRUPT_PRIORYTY();
    LIGHT_INTERRUPT_CONFIG();

    /* ---- Speaker ------> */

    GPIO_SetDir(2, 1<<0, 1);
    GPIO_SetDir(2, 1<<1, 1);

    GPIO_SetDir(0, 1<<27, 1);
    GPIO_SetDir(0, 1<<28, 1);
    GPIO_SetDir(2, 1<<13, 1);
    GPIO_SetDir(0, 1<<26, 1);

    GPIO_ClearValue(0, 1<<27); //LM4811-clk
    GPIO_ClearValue(0, 1<<28); //LM4811-up/dn
    GPIO_ClearValue(2, 1<<13); //LM4811-shutdn

    /* <---- Speaker ------ */

    moveBar(1, dir);
    oled_clearScreen(OLED_COLOR_BLACK);

    uint32_t my_light_value;

    uint32_t my_temp_value;
    uint32_t SW4_press_count = 0;
    uint8_t SW4_PRESSED = 0;


    // Main program initialisation
    static char* msg = NULL;
    uint32_t XXX = 0;



    ready_uart();



    while (1)
    {

    	 if(pressCount > 0)
		{
		  mode = ModeControl(mode);
		}
    	 if (BLUE_LED)
		{BLINK_BLUE(333);}
		if (RED_LED)
		{BLINK_RED(333);}
//		SW4 = (GPIO_ReadValue(1) >> 31) & 0x01;

    	if (mode == REVERSE)
    	{
    		CURRENT_TIME_MAIN = getTicks();
			if (RESET_INITIAL_MAIN)
			{
				 RESET_INITIAL_MAIN = 0;
				 INITIAL_TIME_MAIN = CURRENT_TIME_MAIN;
			}
			else
			{
				if (CURRENT_TIME_MAIN - INITIAL_TIME_MAIN >= 1000)
				{
					if (rev_uart)
					{
						sprintf(uartmessage,"Entering Reverse mode.\r\n");
						UART_Send(LPC_UART3, (uint8_t *)uartmessage, strlen(uartmessage), BLOCKING);
						rev_uart = 0;
					}

					sprintf(oled_disp,"MODE:REVERSE");
					oled_putString(0, 0,(uint8_t*)oled_disp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
					sprintf(oled_disp,"MODE:REVERSE");
					oled_putString(0, 0,(uint8_t*)oled_disp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
                    if(obstacle_near)
                    {OBSTACLE_NEAR();}
					my_light_value = light_read();
					OBSTACLE_AVOIDANCE(my_light_value);
					if (lock_status)
					{
						sprintf(oled_disp,"Car Locked  ");
						oled_putString(0, 50,(uint8_t*)oled_disp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
					}
					else
					{
						sprintf(oled_disp,"Car Unlocked");
						oled_putString(0, 50,(uint8_t*)oled_disp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
					}
					state = joystick_read();
								if (state == 2)
								{

									lock_status = 1;
								}
								else if (state == 4)
								{
									lock_status = 0;
								}
					RESET_INITIAL_MAIN = 1;
				}
			}
    }

    	else if  (mode == STATIONARY)
    	{
    		BLUE_LED = 0;
			GPIO_ClearValue(0,1<<26);
			RED_LED = 0;
			GPIO_ClearValue(2,1<<0);
			counter = 0;
			led7seg_setChar(0, FALSE);
			pca9532_setLeds(0,0xFFFF);
			sprintf(oled_disp,"MODE:STATIONARY");
			oled_putString(0, 0,(uint8_t*)oled_disp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
			if (lock_status)
			{
				sprintf(oled_disp,"Car Locked  ");
				oled_putString(0, 50,(uint8_t*)oled_disp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
			}
			else
			{
				sprintf(oled_disp,"Car Unlocked");
			    oled_putString(0, 50,(uint8_t*)oled_disp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
			}

			state = joystick_read();
			if (state == 2)
			{

				lock_status = 1;
			}
			else if (state == 4)
			{
				lock_status = 0;
			}

    	}
    	else if (mode == FORWARD)
    	{
    		if (fwd_uart)
			{
			sprintf(uartmessage,"Entering Forward mode.\r\n");
			UART_Send(LPC_UART3, (uint8_t *)uartmessage, strlen(uartmessage), BLOCKING);
			fwd_uart = 0;
			}

    		if (lock_status)
			{
				sprintf(oled_disp,"Car Locked  ");
				oled_putString(0, 50,(uint8_t*)oled_disp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
			}
			else
			{
				sprintf(oled_disp,"Car Unlocked");
				oled_putString(0, 50,(uint8_t*)oled_disp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
			}
    		state = joystick_read();
			if (state == 2)
			{

				lock_status = 1;
			}
			else if (state == 4)
			{
				lock_status = 0;
			}


			CURRENT_TIME_MAIN = getTicks();

			if (RESET_INITIAL_MAIN)
			{
				 RESET_INITIAL_MAIN = 0;
				 INITIAL_TIME_MAIN = CURRENT_TIME_MAIN;
			}
			else
			{
				if (CURRENT_TIME_MAIN - INITIAL_TIME_MAIN >= 1000)
				{
					sprintf(oled_disp,"MODE:FORWARD");
					oled_putString(0, 0,(uint8_t*)oled_disp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

					acc_read(&x, &y, &z);
					x = x+xoff;
					y = y+yoff;
					z = z+zoff;

					if (y < 0) {
						dir = 1;
						y = -y;
					}
					else {
						dir = -1;
					}

					if (y > 1 && wait++ > (40 / (1 + (y/10))))
					{
						moveBar(1, dir);
						wait = 0;
					}
					if ((counter == 5) || (counter == 10) || (counter == 15))
				   {
					  sprintf(oled_disp,"TEMP: %2.2f C",my_temp_value/10.0);
					  oled_putString(0, 10,(uint8_t*)oled_disp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
					  sprintf(oled_disp,"AX[g]: %02.2f",x/64.0);
					  oled_putString(0, 20,(uint8_t*)oled_disp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
				   }
					if (counter == 15)
					{
						if (col_detect && temp_high)
						{
							sprintf(uartmessage,"Temperature too high.\r\n");
							UART_Send(LPC_UART3, (uint8_t *)uartmessage, strlen(uartmessage), BLOCKING);
							temp_high = 0;
							sprintf(uartmessage,"Collision has been detected.\r\n");
							UART_Send(LPC_UART3, (uint8_t *)uartmessage, strlen(uartmessage), BLOCKING);
							col_detect = 0;
						}
						else if (col_detect)
						{
							sprintf(uartmessage,"Collision has been detected.\r\n");
							UART_Send(LPC_UART3, (uint8_t *)uartmessage, strlen(uartmessage), BLOCKING);
							col_detect = 0;
						}
						else if (temp_high)
						{
							sprintf(uartmessage,"Temperature too high.\r\n");
							UART_Send(LPC_UART3, (uint8_t *)uartmessage, strlen(uartmessage), BLOCKING);
							temp_high = 0;
						}

						sprintf(uartmessage,"%03d_Temp_%2.2f_ACC_%2.2f\r\n ", XXX,my_temp_value/10.0,x/64.0);
						UART_Send(LPC_UART3, (uint8_t *)uartmessage, strlen(uartmessage), BLOCKING);
						XXX++;
					}

					if (counter < 16)
					{
						DISPSEG(counter);
						counter ++;
					}
					else
					{
						counter = 0;
						DISPSEG(counter);
					}
					COLLISION_DETECTION(x);
					my_temp_value = temp_read();
					BATTERY_CONDITION_MONITORING(my_temp_value);

					pca9532_setLeds(0,0xFFFF);
					RESET_INITIAL_MAIN = 1;

				}
			}
    	}

        /* ####### Joystick and OLED  ###### */
        /* # */
//
        // This is to read current value of P1.31 to see if SW4 is pressed
        //Reads 32-bit value of Port 1(P1.31),right shift by 31, bitwise &
//        // if LSB = 0 pressed LSB = 1 not pressed
//        state = joystick_read();
//        if (state != 0)
//            drawOled(state);

        /* # */
        /* ############################################# */

//        Timer0_Wait(1); // Milliseconds
    }
}






void check_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while(1);
}

// handle drive mode changes


