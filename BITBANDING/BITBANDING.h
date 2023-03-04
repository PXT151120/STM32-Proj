/*
 * BITBANDING.h
 *
 *  Created on: Jan 18, 2023
 *      Author: ASUS
 */

#ifndef INC_BITBANDING_H_
#define INC_BITBANDING_H_

#include "stm32f103xb.h"
#include <stdint.h>
//Peripheral address to alias region


#define ALIAS_PERI_BASE 0x42000000

//Bit banding GPIO_PORTA
#define PINA0 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOA->ODR) * 32 + 0 * 4))
#define PINA1 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOA->ODR) * 32 + 1 * 4))
#define PINA2 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOA->ODR) * 32 + 2 * 4))
#define PINA3 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOA->ODR) * 32 + 3 * 4))
#define PINA4 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOA->ODR) * 32 + 4 * 4))
#define PINA5 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOA->ODR) * 32 + 5 * 4))
#define PINA6 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOA->ODR) * 32 + 6 * 4))
#define PINA7 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOA->ODR) * 32 + 7 * 4))
#define PINA8 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOA->ODR) * 32 + 8 * 4))
#define PINA9 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOA->ODR) * 32 + 9 * 4))
#define PINA10 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOA->ODR) * 32 + 10 * 4))
#define PINA11 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOA->ODR) * 32 + 11 * 4))
#define PINA12 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOA->ODR) * 32 + 12 * 4))
#define PINA13 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOA->ODR) * 32 + 13 * 4))
#define PINA14 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOA->ODR) * 32 + 14 * 4))
#define PINA15 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOA->ODR) * 32 + 15 * 4))
#define PINA16 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOA->ODR) * 32 + 16 * 4))

//Bit banding GPIO_PORTB
#define PINB0 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOB->ODR) * 32 + 0 * 4))
#define PINB1 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOB->ODR) * 32 + 1 * 4))
#define PINB2 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOB->ODR) * 32 + 2 * 4))
#define PINB3 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOB->ODR) * 32 + 3 * 4))
#define PINB4 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOB->ODR) * 32 + 4 * 4))
#define PINB5 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOB->ODR) * 32 + 5 * 4))
#define PINB6 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOB->ODR) * 32 + 6 * 4))
#define PINB7 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOB->ODR) * 32 + 7 * 4))
#define PINB8 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOB->ODR) * 32 + 8 * 4))
#define PINB9 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOB->ODR) * 32 + 9 * 4))
#define PINB10 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOB->ODR) * 32 + 10 * 4))
#define PINB11 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOB->ODR) * 32 + 11 * 4))
#define PINB12 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOB->ODR) * 32 + 12 * 4))
#define PINB13 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOB->ODR) * 32 + 13 * 4))
#define PINB14 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOB->ODR) * 32 + 14 * 4))
#define PINB15 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOB->ODR) * 32 + 15 * 4))
#define PINB16 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOB->ODR) * 32 + 16 * 4))

//Bit banding GPIO_PORTC
#define PINC0 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOC->ODR) * 32 + 0 * 4))
#define PINC1 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOC->ODR) * 32 + 1 * 4))
#define PINC2 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOC->ODR) * 32 + 2 * 4))
#define PINC3 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOC->ODR) * 32 + 3 * 4))
#define PINC4 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOC->ODR) * 32 + 4 * 4))
#define PINC5 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOC->ODR) * 32 + 5 * 4))
#define PINC6 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOC->ODR) * 32 + 6 * 4))
#define PINC7 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOC->ODR) * 32 + 7 * 4))
#define PINC8 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOC->ODR) * 32 + 8 * 4))
#define PINC9 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOC->ODR) * 32 + 9 * 4))
#define PINC10 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOC->ODR) * 32 + 10 * 4))
#define PINC11 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOC->ODR) * 32 + 11 * 4))
#define PINC12 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOC->ODR) * 32 + 12 * 4))
#define PINC13 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOC->ODR) * 32 + 13 * 4))
#define PINC14 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOC->ODR) * 32 + 14 * 4))
#define PINC15 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOC->ODR) * 32 + 15 * 4))
#define PINC16 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOC->ODR) * 32 + 16 * 4))

//Bit banding GPIO_PORTD
#define PIND0 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOD->ODR) * 32 + 0 * 4))
#define PIND1 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOD->ODR) * 32 + 1 * 4))
#define PIND2 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOD->ODR) * 32 + 2 * 4))
#define PIND3 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOD->ODR) * 32 + 3 * 4))
#define PIND4 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOD->ODR) * 32 + 4 * 4))
#define PIND5 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOD->ODR) * 32 + 5 * 4))
#define PIND6 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOD->ODR) * 32 + 6 * 4))
#define PIND7 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOD->ODR) * 32 + 7 * 4))
#define PIND8 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOD->ODR) * 32 + 8 * 4))
#define PIND9 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOD->ODR) * 32 + 9 * 4))
#define PIND10 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOD->ODR) * 32 + 10 * 4))
#define PIND11 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOD->ODR) * 32 + 11 * 4))
#define PIND12 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOD->ODR) * 32 + 12 * 4))
#define PIND13 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOD->ODR) * 32 + 13 * 4))
#define PIND14 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOD->ODR) * 32 + 14 * 4))
#define PIND15 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOD->ODR) * 32 + 15 * 4))
#define PIND16 (*(volatile uint32_t *)(ALIAS_PERI_BASE + (uint32_t)(&GPIOD->ODR) * 32 + 16 * 4))
#endif /* INC_BITBANDING_H_ */
