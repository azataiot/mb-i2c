/*
 * lsm6d.h
 *
 *  Created on: Mar 17, 2022
 *      Author: Azat
 */

#ifndef INC_LSM6D_H_
#define INC_LSM6D_H_

#include "azt.h"
#include "wire.h"
#include "serial.h"
#include "lsm6ds3_reg.h"
#include <string.h>
#include <stdio.h>

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME   20 //ms

/* Private variables ---------------------------------------------------------*/


/** Testing the LSM6DS3TR 3D sensor.
 * 	functions provided in the c file.
 *  you need to also include all azt* related h files.
 */

void lsm6d_print(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);

//static void tx_com(UART_HandleTypeDef *huart, uint8_t *tx_buffer, uint16_t len );

static void platform_delay(uint32_t ms);

static void platform_init(void);






#endif /* INC_LSM6D_H_ */
