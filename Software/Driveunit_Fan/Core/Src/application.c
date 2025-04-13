/*
 * application.c
 *
 *  Created on: Aug 6, 2024
 *      Author: Lukas
 */

#include "application.h"
#include "motorcontroller.h"

extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;

extern Controller moco;

bool oldTriggerState = 1;
bool stateMachine = 0;

void setup()
{
	initMotorControl();
	//calibrateOffset(64);
	setMode(POSITION_MODE);
	update();
	enableMotor();
}

int32_t travel = 1000;

void loop()
{
	update();
	if (stateMachine == 1)
	{
		if (abs(moco.position - travel) < 10)
		{
			stateMachine = 0;
			moco.target = 0;
		}
	}
	if ((oldTriggerState == 1) && (HAL_GPIO_ReadPin(Trigger_GPIO_Port, Trigger_Pin) == 0))
	{
		stateMachine = 1;
		moco.target = travel;
	}
	oldTriggerState = HAL_GPIO_ReadPin(Trigger_GPIO_Port, Trigger_Pin);
}
