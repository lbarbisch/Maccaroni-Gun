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

#define SERVO_WAIT_MS 400
#define BLOCK_WAIT_MS 500

uint32_t servoWaitStart = 0;
uint32_t blockWaitStart = 0;

uint8_t rapidFireCount = 1;
uint8_t fireCount = 0;

void setServoSignal(bool active)
{
	// dummy implementation, fill in actual servo control later
}


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

	// detect falling edge of trigger button -> start pew
	if ((oldTriggerState == 1) && (HAL_GPIO_ReadPin(Trigger_GPIO_Port, Trigger_Pin) == 0) && (stateMachine == 0))
	{
		stateMachine = 1;
		disableFlywheels();
		servoWaitStart = HAL_GetTick();
		fireCount = rapidFireCount;
	}

	// wait period for flywheels to spin up
	if (stateMachine == 1)
	{
		if ((HAL_GetTick() - servoWaitStart) >= SERVO_WAIT_MS)
		{
			stateMachine = 2;
			moco.target = targetA;
			blockWaitStart = HAL_GetTick();
		}
	}

	// make sure motor is at starting position
	if (stateMachine == 2)
	{
		if ((abs(moco.position - targetA) < 10) || ((HAL_GetTick() - blockWaitStart) >= BLOCK_WAIT_MS))
		{
			stateMachine = 3;
			moco.target = targetB;
			blockWaitStart = HAL_GetTick();
		}
	}

	// move motor to positionB
	if (stateMachine == 3)
	{
		if ((abs(moco.position - targetB) < 10) || ((HAL_GetTick() - blockWaitStart) >= BLOCK_WAIT_MS))
		{

			moco.target = targetA;
			fireCount--;
			if (fireCount > 0)
			{
				// shoot once more
				stateMachine = 2;
			}
			else
			{
				// stop shooting
				stateMachine = 4;
			}
		}
	}

	// move motor back and after return disable flywheels
	if (stateMachine == 4)
	{
		if (abs(moco.position - targetA) < 10)
		{
			enableFlywheels();
			stateMachine = 0;
		}
	}

	// logic to detect falling edge of button
	oldTriggerState = HAL_GPIO_ReadPin(Trigger_GPIO_Port, Trigger_Pin);
}

void enableFlywheels()
{
	htim16.Instance->CCR1 = 1300;
}

void disableFlywheels()
{
	htim16.Instance->CCR1 = 540;
}
