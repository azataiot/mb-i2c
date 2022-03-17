/*
 * wire.c
 *
 *  Created on: Mar 17, 2022
 *      Author: Azat
 */

#include "wire.h"
#include "serial.h"

#define I2C_TIMEOUT HAL_MAX_DELAY
//#define I2C_TIMEOUT 0x10

void i2c_scan(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart) {
	// i2c scan
	char buff[64];
	println(huart, "\nStarting the I2C Scanner...\n");
	// scan for device address from 0 to 127
	byte var, ret = 0;
	for (var = 0; var < 127; var++) {
		ret = HAL_I2C_IsDeviceReady(hi2c, var << 1, 3, I2C_TIMEOUT);
		if (ret == HAL_OK) {
			sprintf(buff, "Found an i2c device at 0x%X \n", var);
			print(huart, buff);
		}
	}
	println(huart, "\nI2C device scanning process finished.");
	// end of i2c scan
}

status device_ready(I2C_HandleTypeDef *hi2c, uint16_t addr) {
	return HAL_I2C_IsDeviceReady(hi2c, addr << 1, 3, I2C_TIMEOUT);
}

/******************************************************************************
 function:
 I2C Write and Read
 ******************************************************************************/

status i2c_write_byte(I2C_HandleTypeDef *hi2c, byte dev_addr, byte mem_addr,
		byte data) {
	byte buf[1] = { 0 };
	buf[0] = data;
	return HAL_I2C_Mem_Write(hi2c, dev_addr, mem_addr, sizeof(byte), buf,
			1, I2C_TIMEOUT);
}

status i2c_write_word(I2C_HandleTypeDef *hi2c, byte dev_addr, byte mem_addr,
		word data) {
	byte buf[2] = { 0 };
	buf[0] = data >> 8;
	buf[1] = data;
	return HAL_I2C_Mem_Write(hi2c, dev_addr, mem_addr, sizeof(byte), buf,
			2, I2C_TIMEOUT);
}

byte i2c_read_byte(I2C_HandleTypeDef *hi2c, byte dev_addr, byte mem_addr) {
	byte buf[1] = { mem_addr };
	HAL_I2C_Mem_Read(hi2c, dev_addr, mem_addr, sizeof(byte), buf, 1,
	I2C_TIMEOUT);
	return buf[0];
}

word i2c_read_word(I2C_HandleTypeDef *hi2c, byte dev_addr, byte mem_addr) {
	byte buf[2] = { 0, 0 };
	HAL_I2C_Mem_Read(hi2c, dev_addr, mem_addr, sizeof(byte), buf, 2,
	I2C_TIMEOUT);
	return ((buf[1] << 8) | (buf[0] & 0xff));
}
