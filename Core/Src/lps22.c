/*
 * lps22.c
 *
 *  Created on: Mar 17, 2022
 *      Author: Azat
 */

#include "string.h"
#include "lps22.h"
#include "serial.h"

void lps22_reset(I2C_HandleTypeDef *hi2c) {
	unsigned char buf;
	buf = i2c_read_word(hi2c, LPS22_I2C_ADDRESS, LPS_CTRL_REG2);
	buf |= 0x04;
	i2c_write_byte(hi2c, LPS22_I2C_ADDRESS, LPS_CTRL_REG2, buf); //SWRESET Set 1
	while (buf) {
		buf = i2c_read_word(hi2c, LPS22_I2C_ADDRESS,
		LPS_CTRL_REG2);
		buf &= 0x04;
	}
}

void lps22_start_oneshot(I2C_HandleTypeDef *hi2c) {
	unsigned char buf;
	buf = i2c_read_word(hi2c, LPS22_I2C_ADDRESS, LPS_CTRL_REG2);
	buf |= 0x01;                                         //ONE_SHOT Set 1
	i2c_write_byte(hi2c, LPS22_I2C_ADDRESS, LPS_CTRL_REG2, buf);
}

unsigned char lps22_init(I2C_HandleTypeDef *hi2c) {
	if (i2c_read_byte(hi2c, LPS22_I2C_ADDRESS, LPS_WHO_AM_I) != LPS_ID)
		return 0;    //Check device ID
	lps22_reset(hi2c);                              //Wait for reset to complete
	i2c_write_byte(hi2c, LPS22_I2C_ADDRESS, LPS_CTRL_REG1, 0x02); //Low-pass filter disabled , output registers not updated until MSB and LSB have been read , Enable Block Data Update , Set Output Data Rate to 0
	return 1;
}

void check_lps22(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart) {
	// check lps22
	if (device_ready(hi2c, LPS22_I2C_ADDRESS) == ok) {
		println(huart, "\nLPS22 successfully initialized!");
	} else {
		println(huart, "\nLPS22 device not found!!");
	};
}

void lps22_print(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart) {
	float PRESS_DATA = 0;
	float TEMP_DATA = 0;
	unsigned char u8Buf[3];
	char output[64];

	lps22_start_oneshot(hi2c); //Trigger one shot data acquisition
	if ((i2c_read_byte(hi2c, LPS22_I2C_ADDRESS, LPS_STATUS) & 0x01) == 0x01) { //a new pressure data is generated
		u8Buf[0] = i2c_read_byte(hi2c, LPS22_I2C_ADDRESS >> 1,
		LPS_PRESS_OUT_XL);
		u8Buf[1] = i2c_read_byte(hi2c, LPS22_I2C_ADDRESS >> 1,
		LPS_PRESS_OUT_L);
		u8Buf[2] = i2c_read_byte(hi2c, LPS22_I2C_ADDRESS >> 1,
		LPS_PRESS_OUT_H);
		PRESS_DATA = (float) ((u8Buf[2] << 16) + (u8Buf[1] << 8) + u8Buf[0])
				/ 4096.0f;
	}
	if ((i2c_read_byte(hi2c, LPS22_I2C_ADDRESS, LPS_STATUS) & 0x02) == 0x02) { // a new pressure data is generated
		u8Buf[0] = i2c_read_byte(hi2c, LPS22_I2C_ADDRESS, LPS_TEMP_OUT_L);
		u8Buf[1] = i2c_read_byte(hi2c, LPS22_I2C_ADDRESS, LPS_TEMP_OUT_H);
		TEMP_DATA = (float) ((u8Buf[1] << 8) + u8Buf[0]) / 100.0f;
	}
	sprintf(output, "Pressure = %6.2f hPa , Temperature = %6.2f'C", PRESS_DATA,
			TEMP_DATA);
	println(huart, output);
}
