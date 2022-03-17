/*
 * serial.c
 *
 *  Created on: Mar 16, 2022
 *      Author: Azat
 */




#include "serial.h"


HAL_StatusTypeDef print(UART_HandleTypeDef *huart, const char *msg){
	return HAL_UART_Transmit(huart,(uint8_t *)msg, strlen(( char * )msg), HAL_MAX_DELAY);
}

HAL_StatusTypeDef println(UART_HandleTypeDef *huart, const char *msg){
	char data[strlen(( char * )msg)+1];
	sprintf(data,"%s\n",( char * )msg);
	return HAL_UART_Transmit(huart,(uint8_t *)data, strlen(data), HAL_MAX_DELAY);
}

HAL_StatusTypeDef uart_tx(UART_HandleTypeDef *huart, const uint8_t *msg){
	return HAL_UART_Transmit(huart,(uint8_t *)msg, strlen(( char * )msg), HAL_MAX_DELAY);
}

HAL_StatusTypeDef uart_tx_exp(UART_HandleTypeDef *huart, const uint8_t *msg, uint32_t expire_ms){
	return HAL_UART_Transmit(huart,(uint8_t *)msg, strlen(( char * )msg), expire_ms);
}
