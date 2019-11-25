//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.
// Course: ECE 355 "Microprocessor-Based Systems".
// This is template code for Part 2 of Introductory Lab.
//
// See "system/include/cmsis/stm32f0xx.h" for register/bit definitions.
// See "system/src/cmsis/vectors_stm32f0xx.c" for handler declarations.
// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"
#include "stm32f0xx_spi.h"


// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via $(trace)).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)

void myGPIOA_Init(void);
void myGPIOB_Init(void);
void myTIM2_Init(void);
void myEXTI_Init(void);
void myADC1_Init(void);
void myDAC1_Init(void);
void mySPI1_Init(void);
void myLCD_Init();

void LCD_Delay();
void LCD_Transmit(uint8_t data);
void send_instr(uint8_t instr);
void send_var(uint8_t var);
void LCD_WriteStr(const char* string, unsigned int line);

void testADC(void);
void testLCD(void);

void print_Res();
void print_Fre();

// Your global variables...


main(int argc, char* argv[])

{

	trace_printf("This is the Final Part of Introductory Lab...\n");
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	myGPIOA_Init();		/* Initialize I/O port PA */
	myGPIOB_Init();		/* Initialize I/O port PB */
	myTIM2_Init();		/* Initialize timer TIM2 */
	myEXTI_Init();		/* Initialize EXTI */
	myADC1_Init();		/* Initialize ADC  */
	myDAC1_Init();
	mySPI1_Init();
	myLCD_Init();


	//testLCD();
	while (1)
	{

	//testADC();

	print_Res();

	}

	return 0;

}
void myADC1_Init(){

	RCC->APB2ENR |= RCC_APB2ENR_ADCEN; /*Enable ADC clock*/
	//calibrate ADC
	trace_printf("Starting ADC Calibration... \n");
	ADC1->CR = ADC_CR_ADCAL;
	while(ADC1->CR == ADC_CR_ADCAL){}		//wait until calibration is complete(ADCAL flag)
	trace_printf("ADC Calibration Complete \n");
	/* ADC Configurations:
	 * Enable continuous conversion mode to always be converting pot value
	 * With continuous conversion, may run into data overrun events (converted data was not read
	 * in time). Therefore we enable overrun mode to overwrite unread data with new converted data.
	 * allows for accurate update to LCD */
	 ADC1->CFGR1 |= ADC_CFGR1_CONT;		//continuous mode
	 ADC1->CFGR1 |= ADC_CFGR1_OVRMOD;	// overrun mode
	 //Select channel 0 for PA0 (Pot)
	 ADC1->CHSELR |= ADC_CHSELR_CHSEL0;
	//enable ADC, check for flag before using
	ADC1->CR |= ADC_CR_ADEN;
	while(!(ADC1->ISR & ADC_ISR_ADRDY)) {};
	trace_printf("ADC Ready for operation \n");
}
void testADC(){
	//Start ADC
	ADC1->CR |= ADC_CR_ADSTART;
	//Read ACD data register to int32
	uint32_t adc_val = ADC1->DR;
	//do math to get resister value of pot
	uint32_t res = (adc_val*5000)/4095;
	//trace_printf("ADC Value %u \n", adc_val);
	trace_printf("Resistance %u Ohms \n", res);
}
void print_Res(){
	//start ADC conversion
	ADC1->CR |= ADC_CR_ADSTART;
	//Read ACD data register to integer
	uint32_t adc_val = ADC1->DR;
	//do math to get resister value of pot
	uint32_t res = (adc_val*5000)/4095;
	//generate resistance string
	char res_string[9];
	snprintf(res_string, sizeof(res_string),"R:%uOh",res);
	LCD_WriteStr(res_string,1);
}
void print_Fre(){

}
void LCD_WriteStr(const char* string, unsigned int line){
	//check which line to be written to
	uint8_t instr = 0x80;
	if(line==1){
		instr |=0x40;
	}
	send_instr(instr);
	unsigned int q;
	for(q=0;string[q]!=0;q++){
		send_var(string[q]);
	}
	for(;q<9;q++){
		send_var(0x20);
	}
}
void myDAC1_Init(){
	/* NOTES:
	*  DAC Output Voltage = VDDA * DOR/4095
	*/
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;		//enable clock for DAC operation, then turn on
	DAC->CR &= ~(DAC_CR_BOFF1);				//enable output buffer
	DAC->CR |= DAC_CR_EN1;
}
void mySPI1_Init(){										//COMMENT
	// enable SPI1 clock
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	// create instance of SPI struct
	SPI_InitTypeDef SPI_InitStructInfo;
	// create pointer to instance of SPI struct
	SPI_InitTypeDef* SPI_InitStruct = &SPI_InitStructInfo;

	// set SPI struct parameters
	SPI_InitStruct->SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStruct->SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct->SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct->SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct->SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct->SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStruct->SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct->SPI_CRCPolynomial = 7;

	// Initialize SPI1 based on struct parameters
	SPI_Init(SPI1, SPI_InitStruct);
	// Enable SPI1
	SPI_Cmd(SPI1, ENABLE);

}
void LCD_Delay(){

	// volatile int so delay doesn't get optimized out
	volatile unsigned int n;
	// for loop to 5000 for delay effect
	for(n=0;n<5000;n++);

}
void LCD_Transmit(uint8_t data){				//COMMENT
	// force lck to 0
	GPIOB->BSRR |= GPIO_BSRR_BR_4;

	// wait for SPI ready
	while((SPI1->SR & SPI_SR_BSY) != 0x0000);

	// send 8 bits over MOSI
	SPI_SendData8(SPI1, data);

	// wait for SPI not busy
	while((SPI1->SR & SPI_SR_BSY) != 0x0000);

	// force lck to 1
	GPIOB->BSRR |= GPIO_BSRR_BS_4;
}
void send_instr(uint8_t instr){							//COMMENT
	//split 8 bit instructions into two 4bit (high, low)
	uint8_t H_half = (instr & 0xF0) >> 4;
	uint8_t L_half = (instr & 0x0F);

	// transmit upper half of instruction to LCD
	LCD_Transmit(0x00 | H_half);
	LCD_Transmit(0x80 | H_half);
	LCD_Transmit(0x00 | H_half);
	LCD_Delay();

	// transmit lower half of instruction to LCD
	LCD_Transmit(0x00 | L_half);
	LCD_Transmit(0x80 | L_half);
	LCD_Transmit(0x00 | L_half);
	LCD_Delay();

}
void send_var(uint8_t var){									//COMMENT
	//split 8 bit instructions into two 4bit (high, low)
	// split character into upper and lower 4 bits
	uint8_t H_half = (var & 0xF0) >> 4;
	uint8_t L_half = (var & 0x0F);

	// transmit upper half of character to LCD
	LCD_Transmit(0x40 | H_half);
	LCD_Transmit(0xC0 | H_half);
	LCD_Transmit(0x40 | H_half);
	LCD_Delay();

	// transmit lower half of character to LCD
	LCD_Transmit(0x40 | L_half);
	LCD_Transmit(0xC0 | L_half);
	LCD_Transmit(0x40 | L_half);
	LCD_Delay();

}
void myLCD_Init(){
	//set function sent 3 times
	LCD_Delay();

	LCD_Transmit(0x03);
	LCD_Transmit(0x03|0x80);
	LCD_Transmit(0x03);
	LCD_Delay();

	LCD_Transmit(0x03);
	LCD_Transmit(0x03|0x80);
	LCD_Transmit(0x03);
	LCD_Delay();

	LCD_Transmit(0x03);
	LCD_Transmit(0x03|0x80);
	LCD_Transmit(0x03);
	LCD_Delay();

	//set 4bit mode
	LCD_Transmit(0x02);
	LCD_Transmit(0x02|0x80);
	LCD_Transmit(0x02);
	LCD_Delay();

	//send instructions to enable proper LCD operation
	send_instr(0x28);		//0010 1000 (Dl = 0, N = 1, F = 0)
	send_instr(0x0C);		//0000 1100 (D = 1, C = 0, B = 0)
	send_instr(0x06);		//0000 0110 (I/D = 1, S = 0)
	send_instr(0x01);		//0000 0001 (clears display)
}
void testLCD(){																		//REMOVE
	char* topline = "123456789";
	char* botline = "123456789  ";

	send_instr(0x80);
	for(uint8_t i=0;i<8;i++){
		send_var(topline[i]);
	}
	send_instr(0xC0);
	for(uint8_t i=0;i<8;i++){
		send_var(botline[i]);
	}
}
void myGPIOA_Init()
{
	/* Enable clock for GPIOA peripheral (Relevant register: RCC->AHBENR) */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	// Configure PA0 as analog input for pot
	GPIOA->MODER &= ~(GPIO_MODER_MODER0);
	/* Ensure no pull-up/pull-down for PA0 */
	GPIOA->MODER &= ~(GPIO_PUPDR_PUPDR0);

	/* Configure PA1 as input (Relevant register: GPIOA->MODER)*/
	GPIOA->MODER &= ~(GPIO_MODER_MODER1);
	/* Ensure no pull-up/pull-down for PA1 (Relevant register: GPIOA->PUPDR) */
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);

	// Configure PA4 as analog input for DAC
	GPIOA->MODER |= GPIO_MODER_MODER4;
	/* Ensure no pull-up/pull-down for PA4 */
	GPIOA->MODER &= ~(GPIO_PUPDR_PUPDR4);
}
void myGPIOB_Init(){
	/* Enable clock for GPIOB peripheral (Relevant register: RCC->AHBENR) */
		RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	/* Configure PB3 for MOSI via alternative function 0 */
		GPIOB->MODER = (GPIOB->MODER & (~GPIO_MODER_MODER3)) | GPIO_MODER_MODER3_1;	// clear GPOIB->MODER before setting bits
	/* Configure PB5 for SCK via alternative function 0 */
		GPIOB->MODER = (GPIOB->MODER & (~GPIO_MODER_MODER5)) | GPIO_MODER_MODER5_1;
	/* Configure PB4 for 74HC595 output */
		GPIOB->MODER |= GPIO_MODER_MODER4_0;
	//set alternative function to 0 for PB3, PB5
		GPIOB->AFR[0] &= ~(GPIO_AFRL_AFR3);
		GPIOB->AFR[0] &= ~(GPIO_AFRL_AFR5);
	//Ensure no pull-up/pull-down for PB3, PB5
		GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR3);
		GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR5);
}
void myTIM2_Init()
{
	/* Enable clock for TIM2 peripheral (Relevant register: RCC->APB1ENR) */
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	// Relevant register: TIM2->CR1
	TIM2->CR1 = ((uint16_t)0x008C);
	//TIM2->CR1 |= TIM_CR1_APRE 	//buffer auto-reload
	//TIM2->CR1 &= ~(TIM_CR1_DIR); 	//counter used as upcounter
	//TIM2->CR1 |= TIM_CR1_URS;		//stop on overflow
	//TIM2->CR1 |=

	/* Set clock pre-scaler value */
	TIM2->PSC = myTIM2_PRESCALER;
	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD;

	/* Update timer registers (Relevant register: TIM2->EGR) */
	TIM2->EGR = ((uint16_t)0x0001);

	/* Assign TIM2 interrupt priority = 0 in NVIC (Relevant register: NVIC->IP[3], or use NVIC_SetPriority)*/
	NVIC_SetPriority(TIM2_IRQn, 0);

	/* Enable TIM2 interrupts in NVIC (Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ) */
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Enable update interrupt generation (Relevant register: TIM2->DIER) */
	TIM2->DIER |= TIM_DIER_UIE;
}
void myEXTI_Init()
{
	/* Map EXTI1 line to PA1 */
	// Relevant register: SYSCFG->EXTICR[0]
	SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI1_PA;

	/* EXTI1 line interrupts: set rising-edge trigger */
	// Relevant register: EXTI->RTSR
	EXTI->RTSR |= EXTI_RTSR_TR1;

	/* Unmask interrupts from EXTI1 line */
	// Relevant register: EXTI->IMR
	EXTI->IMR |= EXTI_IMR_MR1;
	/* Assign EXTI1 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[1], or use NVIC_SetPriority
	NVIC_SetPriority(EXTI0_1_IRQn, 0);

	/* Enable EXTI1 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(EXTI0_1_IRQn);
}
/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler()
{
	/* Check if update interrupt flag is indeed set */
	if ((TIM2->SR & TIM_SR_UIF) != 0)
	{
		trace_printf("\n*** Overflow! ***\n");

		/* Clear update interrupt flag */
		// Relevant register: TIM2->SR
		TIM2->SR &= ~(TIM_SR_UIF);

		/* Restart stopped timer */
		// Relevant register: TIM2->CR1
		TIM2->CR1 |= TIM_CR1_CEN;
	}
}


/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI0_1_IRQHandler()
{
	/* Check if EXTI1 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR1) != 0)
	{
		//
		// 1. If this is the first edge:

		if(!(TIM2->CR1 & TIM_CR1_CEN)){

			//	- Start timer (TIM2->CR1).
			TIM2->CR1 |= TIM_CR1_CEN;
		}
		//    Else (this is the second edge):
		else {
			//	- Stop timer (TIM2->CR1).
			TIM2->CR1 &= ~(TIM_CR1_CEN);
			//clear count
			TIM2->CNT &= 0x00000000;
		}

		// 2. Clear EXTI1 interrupt pending flag (EXTI->PR).
		EXTI->PR |= EXTI_PR_PR1;
	}
}


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
