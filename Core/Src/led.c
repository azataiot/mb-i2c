/*
 * toggleLED.c
 *
 *  Created on: Mar 16, 2022
 *      Author: Azat
 */

#include <led.h>


void toggleLED(uint32_t ms){
	togglePin("PE3");
	delay(ms);
}
