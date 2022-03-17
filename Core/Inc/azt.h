/*
 * azt.h
 *
 *  Created on: Mar 16, 2022
 *      Author: Azat
 */

#ifndef INC_AZT_H_
#define INC_AZT_H_

// custom definitions
#define  HIGH 1
#define LOW 0
#define High 1
#define Low 0
#define high 1
#define low 0

#include "main.h"
#include "string.h"
#include "stdio.h"

// custom definitions
#define delay_ms(__xms)	HAL_Delay(__xms)

// custom typedef
typedef uint8_t byte;
typedef uint16_t word;
typedef uint32_t longword;

typedef enum
{
  ok       = 0x00U,
  error    = 0x01U,
  busy     = 0x02U,
  timeout  = 0x03U
} status;

struct PINType {
	GPIO_TypeDef *BANK;
	uint16_t NUM;
};

// basic pin type
typedef struct PINType PIN;

// get HAL pin definitions from a string.
PIN pin2hal(char *pin);

// delay
void delay(uint32_t ms);

// write value to the pin
void digitalWrite(char *pin, uint8_t val);

void togglePin(char *pin);

// read value from the pin
GPIO_PinState digitalRead(char *pin);

#endif /* INC_AZT_H_ */
