/*
 * application.c
 *
 *  Created on: Aug 6, 2024
 *      Author: Lukas
 */

#include "application.h"
#include "motorcontroller.h"

extern Controller moco;
extern volatile bool newData;

bool oldTriggerState = 1;
uint8_t stateMachine = 0;

int32_t targetA = 0;
int32_t targetB = -600;


void setup()
{
	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);

	initMotorControl();

	moco.position = get_meas_angle();
	moco.old_angle = moco.position;
	moco.target = targetA;

	setnewData();
	update();

	while (moco.position > 1024) moco.position -= 2048;
	while (moco.position < -1024) moco.position += 2048;

	//setnewData();
	//update();
	enableMotor();
	//calibrateOffset(64);  // broken :/
}
void loop()
{
	// update control loop
	update();

	// start sequence
	if (stateMachine == 1)
	{
		if (abs(moco.position - targetA) < 10)
		{
			stateMachine = 2;
			moco.target = targetB;
		}
	}

	// start sequence
	if (stateMachine == 2)
	{
		if (abs(moco.position - targetB) < 10)
		{
			stateMachine = 0;
			moco.target = targetA;
		}
	}

	if ((oldTriggerState == 1) && (HAL_GPIO_ReadPin(Trigger_GPIO_Port, Trigger_Pin) == 0))
	{
		stateMachine = 1;
		moco.target = targetA;
	}

	// logic to detect falling edge of button
	oldTriggerState = HAL_GPIO_ReadPin(Trigger_GPIO_Port, Trigger_Pin);
}


