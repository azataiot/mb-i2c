/*
 * azt.c
 *
 *  Created on: Mar 16, 2022
 *      Author: Azat
 */

#include "azt.h"
#include "string.h"
#include "stdlib.h"


PIN pin2hal(char *pin) {
	PIN gpio;
	char bank[2];
	char num[2];
	strncpy(bank, pin, 2);
	strncpy(num, &pin[2], 2);
	bank[2] = '\0';
	num[2] = '\0';
	// get bank
	if (strcmp(bank, "PA") == 0) {
		gpio.BANK = GPIOA;
	} else if (strcmp(bank, "PB") == 0) {
		gpio.BANK = GPIOB;

	} else if (strcmp(bank, "PC") == 0) {
		gpio.BANK = GPIOC;

	} else if (strcmp(bank, "PD") == 0) {
		gpio.BANK = GPIOD;

	} else if (strcmp(bank, "PE") == 0) {
		gpio.BANK = GPIOE;

	} else if (strcmp(bank, "PF") == 0) {
		gpio.BANK = GPIOF;

	} else if (strcmp(bank, "PG") == 0) {
		gpio.BANK = GPIOG;

	} else if (strcmp(bank, "PH") == 0) {
		gpio.BANK = GPIOH;

	} else if (strcmp(bank, "PI") == 0) {
		gpio.BANK = GPIOI;
	}

	// get pin number
	int num_nature;
	num_nature = atoi(num);
	switch (num_nature) {
	case 0:
		gpio.NUM = GPIO_PIN_0;
		break;
	case 1:
		gpio.NUM = GPIO_PIN_1;
		break;
	case 2:
		gpio.NUM = GPIO_PIN_2;
		break;
	case 3:
		gpio.NUM = GPIO_PIN_3;
		break;
	case 4:
		gpio.NUM = GPIO_PIN_4;
		break;
	case 5:
		gpio.NUM = GPIO_PIN_5;
		break;
	case 6:
		gpio.NUM = GPIO_PIN_6;
		break;
	case 7:
		gpio.NUM = GPIO_PIN_7;
		break;
	case 8:
		gpio.NUM = GPIO_PIN_8;
		break;
	case 9:
		gpio.NUM = GPIO_PIN_9;
		break;
	case 10:
		gpio.NUM = GPIO_PIN_10;
		break;
	case 11:
		gpio.NUM = GPIO_PIN_11;
		break;
	case 12:
		gpio.NUM = GPIO_PIN_12;
		break;
	case 13:
		gpio.NUM = GPIO_PIN_13;
		break;
	case 14:
		gpio.NUM = GPIO_PIN_14;
		break;
	case 15:
		gpio.NUM = GPIO_PIN_15;
		break;
	}

	// return the pin
	return gpio;
}

// delay
void delay(uint32_t ms){
	HAL_Delay(ms);
}

void digitalWrite(char *pin, uint8_t val) {
	PIN gpio_pin = pin2hal(pin);
	HAL_GPIO_WritePin(gpio_pin.BANK, gpio_pin.NUM, val);
}

void togglePin(char *pin){
	PIN gpio_pin = pin2hal(pin);
	HAL_GPIO_TogglePin(gpio_pin.BANK, gpio_pin.NUM);
}

GPIO_PinState digitalRead(char *pin) {
	PIN gpio_pin = pin2hal(pin);
	return HAL_GPIO_ReadPin(gpio_pin.BANK, gpio_pin.NUM);
}
