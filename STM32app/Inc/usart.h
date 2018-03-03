/*
 * usart.h
 *
 *  Created on: 20 gru 2017
 *      Author: DELL
 */

#ifndef USART_H_
#define USART_H_

#include <inttypes.h>
#include "stm32f4xx.h"

void usartSendByte(USART_TypeDef * usart, uint8_t byte);

void usartWriteString(USART_TypeDef * usart, const char *string);

void usartWriteNumber(USART_TypeDef * usart, uint32_t number);

#endif /* USART_H_ */
