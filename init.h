#ifndef __init_H
#define __init_H

#include "stm32l476xx.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define NO_OF_EDGES   1001
#define BufferSize 32



void System_Clock_Init(void);
void gpio_init(void);
void Delay(uint32_t ms);
void USART_Delay(uint32_t us);
void UART2_Init(void);
void UART2_GPIO_Init(void);
void USART_Init (USART_TypeDef * USARTx);
uint8_t USART_Read (USART_TypeDef * USARTx);
void USART_Write(USART_TypeDef * USARTx, uint8_t *buffer, uint32_t nBytes);
void TIM4_init(void);
void TIM2_init(void);
void TIM4_IRQHandler(void);
int POST(void);
void capture(void);
void process(void);
void reset(void);
char* itoa(int val, int base);
void get_UI_limit(void);
#endif
