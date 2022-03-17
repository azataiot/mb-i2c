/*
 * lps22.h
 *
 *  Created on: Mar 17, 2022
 *      Author: Azat
 */

#ifndef INC_LPS22_H_
#define INC_LPS22_H_

#include "azt.h"
#include "wire.h"
#include "main.h"

//i2c address
#define LPS22_I2C_ADDRESS		0x5C
#define LPS22_I2C_ADDRESS7		0x5C << 1
//
#define LPS_ID                  0xB1
//Register
#define LPS_INT_CFG             0x0B        //Interrupt register
#define LPS_THS_P_L             0x0C        //Pressure threshold registers
#define LPS_THS_P_H             0x0D
#define LPS_WHO_AM_I            0x0F        //Who am I
#define LPS_CTRL_REG1           0x10        //Control registers
#define LPS_CTRL_REG2           0x11
#define LPS_CTRL_REG3           0x12
#define LPS_FIFO_CTRL           0x14        //FIFO configuration register
#define LPS_REF_P_XL            0x15        //Reference pressure registers
#define LPS_REF_P_L             0x16
#define LPS_REF_P_H             0x17
#define LPS_RPDS_L              0x18        //Pressure offset registers
#define LPS_RPDS_H              0x19
#define LPS_RES_CONF            0x1A        //Resolution register
#define LPS_INT_SOURCE          0x25        //Interrupt register
#define LPS_FIFO_STATUS         0x26        //FIFO status register
#define LPS_STATUS              0x27        //Status register
#define LPS_PRESS_OUT_XL        0x28        //Pressure output registers
#define LPS_PRESS_OUT_L         0x29
#define LPS_PRESS_OUT_H         0x2A
#define LPS_TEMP_OUT_L          0x2B        //Temperature output registers
#define LPS_TEMP_OUT_H          0x2C
#define LPS_RES                 0x33        //Filter reset register

void lps22_reset(I2C_HandleTypeDef *hi2c);
void lps22_start_oneshot(I2C_HandleTypeDef *hi2c);
unsigned char lps22_init(I2C_HandleTypeDef *hi2c);

void check_lps22(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);
void lps22_print(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);

#endif /* INC_LPS22_H_ */
