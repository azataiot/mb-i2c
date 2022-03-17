/*
 * serial.h
 *
 *  Created on: Mar 16, 2022
 *      Author: Azat
 */

#ifndef INC_SERIAL_H_
#define INC_SERIAL_H_

#include "main.h"
#include "azt.h"
#include "string.h"
#include <stdio.h>

HAL_StatusTypeDef print(UART_HandleTypeDef *huart, const char *msg);

HAL_StatusTypeDef println(UART_HandleTypeDef *huart, const char *msg);

// blocking (poll) uart transmission.
HAL_StatusTypeDef uart_tx(UART_HandleTypeDef *huart, const uint8_t *msg);

// blocking (poll) uart transmission with an expiration time in ms.
HAL_StatusTypeDef uart_tx_exp(UART_HandleTypeDef *huart, const uint8_t *msg, uint32_t expire_ms);

// non-blocking uart transmission with interrupt.
/**
 * In the interrupt method data transmission takes place in the background or in non-blocking
 * mode. when the transmission is complete, HAL_UART_TxCpltCallback is called,
 * and we can write further instruction inside that function.
 */


#endif /* INC_SERIAL_H_ */
