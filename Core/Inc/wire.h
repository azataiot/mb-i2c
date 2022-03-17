/*
 * wire.h
 *
 *  Created on: Mar 17, 2022
 *      Author: Azat
 */

#ifndef INC_WIRE_H_
#define INC_WIRE_H_

#include "azt.h"



void i2c_scan(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);

status device_ready(I2C_HandleTypeDef *hi2c,uint16_t addr);

/******************************************************************************
function:
	I2C Write and Read
******************************************************************************/
status i2c_write_byte(I2C_HandleTypeDef *hi2c, byte dev_addr, byte mem_addr,
		byte data);

status i2c_write_word(I2C_HandleTypeDef *hi2c, byte dev_addr, byte mem_addr,
		word data);

byte i2c_read_byte(I2C_HandleTypeDef *hi2c, byte dev_addr, byte mem_addr);

word i2c_read_word(I2C_HandleTypeDef *hi2c, byte dev_addr,byte mem_addr);

#endif /* INC_WIRE_H_ */
