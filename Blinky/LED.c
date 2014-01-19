/*----------------------------------------------------------------------------
 * Name:    LED.c
 * Purpose: low level LED functions
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

#include "STM32L1xx.h"
#include "LED.h"

const unsigned long led_mask[] = {1UL << 6, 1UL << 7};

/*----------------------------------------------------------------------------
  initialize LED Pins
 *----------------------------------------------------------------------------*/
void LED_Init (void) {

  RCC->AHBENR |=  (1UL <<  1);                  /* Enable GPIOB clock         */

  GPIOB->MODER   &=  ~(0x0000F000);
  GPIOB->MODER   |=   (0x00005000);             /* General purpose output mode*/
  GPIOB->OTYPER  &=  ~(0x000000C0);             /* Output push-pull           */
  GPIOB->OSPEEDR &=  ~(0x0000F000);
  GPIOB->OSPEEDR |=   (0x00005000);             /* 2 MHz Low speed            */
  GPIOB->PUPDR   &=  ~(0x0000F000);             /* No pull-up, pull-down      */
}

/*----------------------------------------------------------------------------
  Function that turns on requested LED
 *----------------------------------------------------------------------------*/
void LED_On (unsigned int num) {

  if (num < LED_NUM) {
    GPIOB->BSRRL = led_mask[num];
  }
}

/*----------------------------------------------------------------------------
  Function that turns off requested LED
 *----------------------------------------------------------------------------*/
void LED_Off (unsigned int num) {

  if (num < LED_NUM) {
    GPIOB->BSRRH = led_mask[num];
  }
}

/*----------------------------------------------------------------------------
  Function that outputs value to LEDs
 *----------------------------------------------------------------------------*/
void LED_Out(unsigned int value) {
  int i;

  for (i = 0; i < LED_NUM; i++) {
    if (value & (1<<i)) {
      LED_On (i);
    } else {
      LED_Off(i);
    }
  }
}
