/*----------------------------------------------------------------------------
 * Name:    Blinky.c
 * Purpose: LED Flasher
 * Note(s): 
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2013 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <stdio.h>
#include "STM32L1xx.h"
#include "LED.h"

#include "stm32l1xx_i2c.h"
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_rcc.h"
#include "stm32l1xx_adc.h"
#include "stm32l1xx_dma.h"
#include "stm32l1xx_dac.h"
#include "stm32l1xx_opamp.h"

#define  SLAVE_ADDRESS  0x0D
//#define  I2C_AD5934
//#define  ADC1_OPAMP1

/* Private define ------------------------------------------------------------*/
#define ADC1_DR_ADDRESS       ((uint32_t)0x40012458)
static __IO uint16_t ADC_ConvertedValue;

volatile uint32_t msTicks;                      /* counts 1ms timeTicks       */
/*----------------------------------------------------------------------------
  SysTick_Handler
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void) {
  msTicks++;
}

/*----------------------------------------------------------------------------
  delays number of tick Systicks (happens every 1 ms)
 *----------------------------------------------------------------------------*/
void Delay (uint32_t dlyTicks) {                                              
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks);
}


/*----------------------------------------------------------------------------
  Function that initializes Button pins
 *----------------------------------------------------------------------------*/
void BTN_Init(void) {

  RCC->AHBENR |=  (1UL <<  0);                  /* Enable GPIOA clock         */


  GPIOA->MODER   &=  ~(0x00000003);             /* General purpose output mode*/
  GPIOA->OSPEEDR &=  ~(0x00000003);
  GPIOA->OSPEEDR |=   (0x00000002);             /* 40 MHz Medium speed        */
  GPIOA->PUPDR   &=  ~(0x00000003);             /* No pull-up, pull-down      */
}

/*----------------------------------------------------------------------------
  Function that read Button pins
 *----------------------------------------------------------------------------*/
uint32_t BTN_Get(void) {

 return (GPIOA->IDR & (1UL << 0));
}


/*----------------------------------------------------------------------------
  set HSI as SystemCoreClock (HSE is not populated on STM32L-Discovery board)
 *----------------------------------------------------------------------------*/
void SystemCoreClockSetHSI(void) {

  RCC->CR |= ((uint32_t)RCC_CR_HSION);                      /* Enable HSI                        */ 
  while ((RCC->CR & RCC_CR_HSIRDY) == 0);                   /* Wait for HSI Ready                */

  FLASH->ACR |= FLASH_ACR_ACC64;                            /* Enable 64-bit access              */
  FLASH->ACR |= FLASH_ACR_PRFTEN;                           /* Enable Prefetch Buffer            */
  FLASH->ACR |= FLASH_ACR_LATENCY;                          /* Flash 1 wait state                */

  RCC->APB1ENR |= RCC_APB1ENR_PWREN;                        /* Enable the PWR APB1 Clock         */
  PWR->CR = PWR_CR_VOS_0;                                   /* Select the Voltage Range 1 (1.8V) */
  while((PWR->CSR & PWR_CSR_VOSF) != 0);                    /* Wait for Voltage Regulator Ready  */

  RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;                /* HCLK = SYSCLK 16MHz               */
  RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;               /* PCLK2 = HCLK                      */
  RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV1;               /* PCLK1 = HCLK                      */
    
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI; /* HSI is system clock               */
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);   /* Wait for HSI used as system clock */
}

/* ----------------------------------------------------------------------------
  I2Cx system initializarion start from here ...
  ----------------------------------------------------------------------------*/

#define Timed(x) Timeout = 0xFFFFF; while (x) \
 { if (Timeout-- == 0) goto errReturn; } \
 bStaus++;

ErrorStatus I2C_Read(
 I2C_TypeDef* I2Cx,
 uint8_t *buf,
 uint32_t nbyte,
 uint8_t SlaveAddress)
{
  __IO uint32_t Timeout = 0;
	uint8_t bStaus = 0;

	if (!nbyte)
		return SUCCESS;

  // Wait for idle I2C interface
  Timed(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));

	// Enable Acknowledgment, clear POS flag
  I2C_AcknowledgeConfig(I2Cx, ENABLE);
	I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Current);

	// Intiate Start Sequence (wait for EV5)
  I2C_GenerateSTART(I2Cx, ENABLE);
	Timed(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

	// Send Address
  I2C_Send7bitAddress(I2Cx, SlaveAddress, I2C_Direction_Receiver);

	// EV6
  Timed(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_ADDR));

  if (nbyte == 1) {

		/* read 1 byte */
		// Clear Ack bit
    I2C_AcknowledgeConfig(I2Cx, DISABLE);

		// EV6_1 -- must be atomic -- Clear ADDR, generate STOP
    __disable_irq();
    (void) I2Cx->SR2; I2C_GenerateSTOP(I2Cx,ENABLE);
		__enable_irq();

		// Receive data EV7
    Timed(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_RXNE));
		*buf++ = I2C_ReceiveData(I2Cx);
	}
  else if (nbyte == 2) {

	  /* read 2 bytes */
    // Set POS flag
    I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Next);

	  // EV6_1 -- must be atomic and in this order

    __disable_irq();
    (void) I2Cx->SR2; // Clear ADDR flag
    I2C_AcknowledgeConfig(I2Cx, DISABLE); // Clear Ack bit
		__enable_irq();

		// EV7_3 -- Wait for BTF, program stop, read data twice

    Timed(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF));

		__disable_irq();
		I2C_GenerateSTOP(I2Cx,ENABLE);
		*buf++ = I2Cx->DR;
		__enable_irq();

    *buf++ = I2Cx->DR;
	}
  else {

		/* read 3 or more bytes */
		(void) I2Cx->SR2; // Clear ADDR flag
		while (nbyte-- != 3)
		{
			//  EV7 -- cannot guarantee 1 transfer completion time,s
			// wait for BTF instead of RXNE
			Timed(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF));
		  *buf++ = I2C_ReceiveData(I2Cx);
		}

		Timed(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF));

		// EV7_2 -- Figure 1 has an error, doesn't read N-2 !
		I2C_AcknowledgeConfig(I2Cx, DISABLE); // clear ack bit

		__disable_irq();
    *buf++ = I2C_ReceiveData(I2Cx); // receive byte N-2
		I2C_GenerateSTOP(I2Cx,ENABLE); // program stop
		__enable_irq();

		*buf++ = I2C_ReceiveData(I2Cx); // receive byte N-1

		// wait for byte N

    Timed(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
		*buf++ = I2C_ReceiveData(I2Cx);

		nbyte = 0;
	}

  // Wait for stop
  Timed(I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF));

	return SUCCESS;

errReturn:
	return ERROR;
}

ErrorStatus I2C_Write(I2C_TypeDef* I2Cx,
 const uint8_t* buf,
 uint32_t nbyte,
 uint8_t SlaveAddress)
{
  __IO uint32_t Timeout = 0;
	uint8_t bStaus = 0;
	uint32_t lsEvent = 0;

  if (nbyte) {

		  Timed(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));

		  // Intiate Start Sequence
      I2C_GenerateSTART(I2Cx, ENABLE);

		  Timed(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

		  // Send Address EV5
      I2C_Send7bitAddress(I2Cx, SlaveAddress, I2C_Direction_Transmitter);
		  Timed(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

		  // EV6 Write first byte EV8_1
      I2C_SendData(I2Cx, *buf++);

		  while (--nbyte) {
        // wait on BTF
        Timed(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF));
			  I2C_SendData(I2Cx, *buf++);
			}

			Timed(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF));

		  I2C_GenerateSTOP(I2Cx, ENABLE);

			Timed(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF));
   }

	 return SUCCESS;

errReturn:
   lsEvent = I2C_GetLastEvent(I2Cx);

	 return ERROR;
}



void I2C_LowLevel_Init(I2C_TypeDef* I2Cx, int ClockSpeed, int OwnAddress)
{
        GPIO_InitTypeDef GPIO_InitStructure;
        I2C_InitTypeDef I2C_InitStructure;

        // Enable GPIOB clocks
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

        // Configure I2C clock and GPIO
        GPIO_StructInit(&GPIO_InitStructure);

				if (I2Cx == I2C1){
					 /* I2C1 clock enable */
           RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

					 /* I2C1 SDA and SCL configuration */
           GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_8 | GPIO_Pin_9);
					 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
					 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
					 GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
					 //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
					 GPIO_Init(GPIOB, &GPIO_InitStructure);

					 // Connect I2C1 pins to AF
					 GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);  // SCL
					 GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);  // SDA

           /* I2C1 Reset */
					 RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
					 RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);
        }
        else {
					// I2C2 ...
				}

				Delay(10);

				/* Configure I2Cx */
        I2C_StructInit(&I2C_InitStructure);
			  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
			  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
				I2C_InitStructure.I2C_OwnAddress1 = OwnAddress;
			  I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;
			  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
			  I2C_InitStructure.I2C_ClockSpeed = ClockSpeed;

			  I2C_Init(I2Cx, &I2C_InitStructure);
				I2C_Cmd(I2Cx, ENABLE);
}

//
// Set MOC(Microcontroller Output Clock) selected SYSCLK
//
void OutputClockSetSysclock(void)
{
	/* MOC SYSCLK selected */
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_MCOPRE) | RCC_CFGR_MCOSEL_0;

	/* Enable AHB GPIOA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOAEN, ENABLE);

	/* GPIOA PA8 Alternate function mode */
	GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER8) | GPIO_MODER_MODER8_1;
	GPIOA->AFR[1] = (GPIOA->AFR[1] & ~GPIO_AFRH_AFRH8);

	/* 40 MHz High speed */
  GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR8);
  GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR8_0|GPIO_OSPEEDER_OSPEEDR8_1);
}

//
// OPAMP configuration.
//
void OPAMP_Config(void)
{
  //GPIO_InitTypeDef   GPIO_InitStructure;

  /* GPIOA and GPIOB Peripheral clock enable */
  //RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);

  /* Configure PB0 (OPAMP2 output) in analog mode */
  //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  //GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Configure PA6 (OPAMP2 positive input) in analog mode */
  //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  //GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* COMP Peripheral clock enable: COMP and OPAMP share the same Peripheral clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_COMP, ENABLE);

  /* Enable OPAMP1 */
  OPAMP_Cmd(OPAMP_Selection_OPAMP1, ENABLE);

  /* Close S4 and S5 swicthes to make an internal follower */
  OPAMP_SwitchCmd(OPAMP_OPAMP1Switch4 | OPAMP_OPAMP1Switch5, ENABLE);
}

//
// ADC1 Configuration
//
void ADC_LowLeve_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	// Configure ADC on ADC1_IN1, PIN PA1 PA2 & PA3
	// These two PA1 PA2 uesd as input pins to an opamp
  // PA3 is an output of opamp
  GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*------------------------ OPAMP configuration ------------------------------*/

	OPAMP_Config();

  /*------------------------ DMA1 configuration ------------------------------*/

  /* Enable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  /* DMA1 channel1 configuration */
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_ADDRESS;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_ConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);

  /* Enable DMA1 channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);

	/*----------------- ADC1 configuration with DMA enabled --------------------*/

  /* Enable ADC1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

  /* ADC1 Configuration -----------------------------------------------------*/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = 0;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

	/* Enable temperature sensor and Vref */
  //ADC_TempSensorVrefintCmd(ENABLE);

  /* ADC1 regular channel 3 configuration */
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_24Cycles);
  /* ADC1 regular channel configuration */
  //ADC_RegularChannelConfig(ADC1, UBAT_ADC_CHANNEL,       1, ADC_SampleTime_24Cycles);
  //ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 2, ADC_SampleTime_24Cycles);
  //ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint,    3, ADC_SampleTime_24Cycles);

  /* Enable the request after last transfer for DMA Circular mode */
  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

  /* Define delay between ADC1 conversions */
  ADC_DelaySelectionConfig(ADC1, ADC_DelayLength_7Cycles);

  /* Enable ADC1 Power Down during Delay */
  ADC_PowerDownCmd(ADC1, ADC_PowerDown_Idle_Delay, ENABLE);

  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);

  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Wait until the ADC1 is ready */
  while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS));

  /* Start ADC1 Software Conversion */
  ADC_SoftwareStartConv(ADC1);
}
//
// @brief  Power periodic task
// @param  Period: Task execution period in milliseconds
//
void Power_Task(uint32_t Period)
{
  uint16_t tmp = 0;

	Delay(Period);

	/* Conversion done ? */
  if (DMA_GetFlagStatus(DMA1_FLAG_TC1))
  {
    tmp = ADC_ConvertedValue;

    /* Start next ADC1 Software Conversion */
    ADC_SoftwareStartConv(ADC1);
  }
}

//
// DAC LowLevel Init
//
void DAC_LowLevel_Init()
{
  GPIO_InitTypeDef	GPIO_InitStructure;
	DAC_InitTypeDef	DAC_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	DAC_DeInit();
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	DAC_Init(DAC_Channel_2, &DAC_InitStructure);
	DAC_Cmd(DAC_Channel_2,ENABLE);

	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);
	DAC_Cmd(DAC_Channel_1,ENABLE);

	DAC_SetChannel1Data(DAC_Align_12b_R, 0x0000);
	DAC_SetChannel2Data(DAC_Align_12b_R, 0x0000);
}

//
// External Funcs Headfile
//
void ad5933_probe(void);
int ad5933_ring_postdisable(void);
int ad5933_ring_postenable(void);
int ad5933_ring_preenable(void);

/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {
//  int32_t num = -1;
//  int32_t dir =  1;
//  uint32_t btns = 0;
//  uint8_t cmdBuf[2] = {0x80, 0x81};
//  uint8_t ptBuf[2] = {0xB0, 0x80};
//  uint8_t regBuf[1] = {0xff};

  SystemCoreClockSetHSI();

  SystemCoreClockUpdate();                      /* Get Core Clock Frequency   */
  if (SysTick_Config(SystemCoreClock / 1000)) { /* SysTick 1 msec interrupts  */
    while (1);                                  /* Capture error              */
  }

#if defined (I2C_AD5934)
	//
	// PA8 (Microcontroller Output Clock)
	// output 16MHz and to be used as AD5934 MCLK
	//
	OutputClockSetSysclock();

	//
	// I2C1 hardware init
	//
  I2C_LowLevel_Init(I2C1, 400000, 0xfb);

	//I2C_Write(I2C1, cmdBuf, 2, 0x0D << 1);
  //I2C_Write(I2C1, ptBuf, 2, 0x0D << 1);
	//I2C_Read(I2C1, regBuf, 1, 0x0D << 1);

	//
	// AD5934 init task
	//
	ad5933_probe();
  ad5933_ring_preenable();
#endif

  ADC_LowLeve_Init();

//  LED_Init();
//  BTN_Init();
//
  while(1) {                                      /* Loop forever               */
//    btns = BTN_Get();                           /* Read button states         */

//    if (btns != (1UL << 0)) {
//      /* Calculate 'num': 0,1,...,LED_NUM-1,LED_NUM-1,...,1,0,0,...  */
//      num += dir;
//      if (num == LED_NUM) { dir = -1; num =  LED_NUM-1; }
//      else if   (num < 0) { dir =  1; num =  0;         }

//      LED_On (num);
//      Delay( 50);                               /* Delay 50ms                 */
//      LED_Off(num);
//      Delay(450);                               /* Delay 450ms                */
//    }
//    else {
//      LED_Out (0x03);
//      Delay(10);                                /* Delay 10ms                 */
//    }

#if defined (I2C_AD5934)
    //
    // AD5934 Main Task
    //
    ad5933_ring_postenable();
#endif

#if defined (ADC1_OPAMP1)
    //
    // ADC1, OMAMP1 Main Task
    //
    Power_Task(10);
#endif
  }
}
