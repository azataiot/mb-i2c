/*
 * lsm6d.c
 *
 *  Created on: Mar 17, 2022
 *      Author: Azat
 */

#include "lsm6d.h"

static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_temperature;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float temperature_degC;
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];

void lsm6d_print(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart) {
	/* Initialize mems driver interface */
	stmdev_ctx_t dev_ctx;
	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;
	dev_ctx.handle = hi2c;

	// init the platform
	platform_init();

	/* Wait sensor boot time */
	platform_delay(BOOT_TIME);

	/* Check device ID */
//	lsm6ds3_device_id_get(&dev_ctx, &whoamI);
//
//	sprintf(tx_buffer, "0x%X", lsm6ds3_device_id_get(&dev_ctx, &whoamI));
////
//	println(huart, tx_buffer);
//	sprintf(tx_buffer, "0x%X", lsm6ds3_device_id_get(&dev_ctx, &whoamI) >> 1);
//	println(huart, tx_buffer);
//
//	sprintf(tx_buffer, "0x%X", lsm6ds3_device_id_get(&dev_ctx, &whoamI) << 1);
//		println(huart, tx_buffer);

//	if (whoamI != LSM6DS3_ID)
//		while (1) {
//			/* here device not found */
////			println(huart, "\nLSM6dS Not Found!!");
//		}

	/* Restore default configuration */
	lsm6ds3_reset_set(&dev_ctx, PROPERTY_ENABLE);

	do {
		lsm6ds3_reset_get(&dev_ctx, &rst);
	} while (rst);

	/*  Enable Block Data Update */
	lsm6ds3_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

	/* Set full scale */
	lsm6ds3_xl_full_scale_set(&dev_ctx, LSM6DS3_2g);
	lsm6ds3_gy_full_scale_set(&dev_ctx, LSM6DS3_2000dps);

	/* Set Output Data Rate for Acc and Gyro */
	lsm6ds3_xl_data_rate_set(&dev_ctx, LSM6DS3_XL_ODR_12Hz5);
	lsm6ds3_gy_data_rate_set(&dev_ctx, LSM6DS3_GY_ODR_12Hz5);

	uint8_t reg;

	/* Read output only if new value is available */
	lsm6ds3_xl_flag_data_ready_get(&dev_ctx, &reg);

	if (reg) {
		/* Read acceleration field data */
		memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
		lsm6ds3_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
		acceleration_mg[0] = lsm6ds3_from_fs2g_to_mg(data_raw_acceleration[0]);
		acceleration_mg[1] = lsm6ds3_from_fs2g_to_mg(data_raw_acceleration[1]);
		acceleration_mg[2] = lsm6ds3_from_fs2g_to_mg(data_raw_acceleration[2]);
		sprintf((char*) tx_buffer, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
				acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
		uart_tx(huart, tx_buffer);
//		tx_com(huart, tx_buffer, strlen((char const*) tx_buffer));
	}

	lsm6ds3_gy_flag_data_ready_get(&dev_ctx, &reg);

	if (reg) {
		/* Read angular rate field data */
		memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
		lsm6ds3_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
		angular_rate_mdps[0] = lsm6ds3_from_fs2000dps_to_mdps(
				data_raw_angular_rate[0]);
		angular_rate_mdps[1] = lsm6ds3_from_fs2000dps_to_mdps(
				data_raw_angular_rate[1]);
		angular_rate_mdps[2] = lsm6ds3_from_fs2000dps_to_mdps(
				data_raw_angular_rate[2]);
		sprintf((char*) tx_buffer,
				"Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
				angular_rate_mdps[0], angular_rate_mdps[1],
				angular_rate_mdps[2]);
		uart_tx(huart, tx_buffer);
	}

	lsm6ds3_temp_flag_data_ready_get(&dev_ctx, &reg);

	if (reg) {
		/* Read temperature data */
		memset(&data_raw_temperature, 0x00, sizeof(int16_t));
		lsm6ds3_temperature_raw_get(&dev_ctx, &data_raw_temperature);
		temperature_degC = lsm6ds3_from_lsb_to_celsius(data_raw_temperature);
		sprintf((char*) tx_buffer, "Temperature [degC]:%6.2f\r\n",
				temperature_degC);
		uart_tx(huart, tx_buffer);
	}
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
		uint16_t len) {
	HAL_I2C_Mem_Write(handle, LSM6DS3_I2C_ADD_L, reg,
	I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
	return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len) {
	HAL_I2C_Mem_Read(handle, LSM6DS3_I2C_ADD_L, reg,
	I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	return 0;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
//static void tx_com(UART_HandleTypeDef *huart, uint8_t *tx_buffer, uint16_t len) {
//	HAL_UART_Transmit(huart, tx_buffer, len, 1000);
//}
/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
void platform_delay(uint32_t ms) {
	HAL_Delay(ms);
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
void platform_init(void) {

}

