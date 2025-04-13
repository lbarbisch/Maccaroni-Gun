/*
 * application.h
 *
 *  Created on: Aug 6, 2024
 *      Author: Lukas
 */

#ifndef INC_APPLICATION_H_
#define INC_APPLICATION_H_

#include "main.h"
#include "motorcontroller.h"

extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;

void setup();
void loop();

#endif /* INC_APPLICATION_H_ */
