/*
 * button.h
 *
 *  Created on: Nov 16, 2016
 *      Author: fwidmaierlocal
 */

#ifndef SRC_BUTTON_H_
#define SRC_BUTTON_H_

#include "sw/drivers/gpio/src/32b/f28x/f2806x/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines

#define GPIO_BUTTON GPIO_Number_26


// **************************************************************************
// the functions

inline bool BUTTON_isPressed(GPIO_Handle gpioHandle)
{
	// Note that the pin is high by default and pulled to low when the button is
	// pressed
	return GPIO_read(gpioHandle, GPIO_BUTTON) == LOW;
}

#ifdef __cplusplus
}
#endif // extern "C"

#endif /* SRC_BUTTON_H_ */
