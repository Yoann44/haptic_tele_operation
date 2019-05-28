/*
 * Copyright (C) 2017 EPFL-LSRO (Laboratoire de Systemes Robotiques).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "stdio.h"
#include "haptic_controller.h"
#include "communication.h"
#include "drivers/adc.h"
#include "drivers/incr_encoder.h"
#include "drivers/hall.h"
#include "drivers/callback_timers.h"
#include "lib/utils.h"
#include "torque_regulator.h"
#include "drivers/ext_uart.h"

#define DEFAULT_HAPTIC_CONTROLLER_PERIOD 	350 // Default control loop period [us].
#define HALL_TO_ANGLE_COEF_1				49.6388
#define HALL_TO_ANGLE_COEF_2 				-125.3859

#define MAX_SAMPLE_FOR_AVERAGE 				100

#define CENTER_OF_MASS_TO_ROTATION 			0.01988	//Distance between the center of mass and the rotation joint [m]
#define PADDLE_WEIGHT						0.075	//Weight of the paddle [kg]
#define GRAVITY_ACC							9.81

#define POSITIVE_DRY_FRICTION 				0.001563
#define NEGATIVE_DRY_FRICTION 				-0.001703
#define VISCIOUS_FRICTION					8.33e-7

#define VIRTUAL_WALL_ANGLE					15.0	// [deg]

#define SEND_TORQUE_PERIOD 					0.05	//50ms
#define RECV_POS_TIMEOUT 					0.2 //200ms

float32_t paddleAngleTable[MAX_SAMPLE_FOR_AVERAGE];
float32_t paddleSpeedTable[MAX_SAMPLE_FOR_AVERAGE];

volatile uint32_t  hapt_timestamp; // Time base of the controller, also used to timestamp the samples sent by streaming [us].
volatile float32_t hapt_motorTorque = 0.0; // Motor torque [N.m].

volatile float32_t hapt_paddleAngle;
volatile float32_t hapt_paddleVelocity;

volatile uint32_t hapt_dryFrictionComp = 1;
volatile uint32_t hapt_visFrictionComp = 1;
volatile uint32_t hapt_gravityComp = 1;
volatile uint32_t hapt_virtualWall = 0;

volatile uint32_t hapt_positionInput = 3;

volatile float32_t hapt_virtualWallK = -0.05;
volatile float32_t hapt_virtualWallB = 0.0001;

volatile float32_t hapt_encoderPaddleAngle; // Paddle angle measured by the incremental encoder [deg].
volatile float32_t hapt_hallVoltage; // Hall sensor output voltage [V].
volatile float32_t hapt_hallPaddleAngle;
volatile float32_t hapt_hallPaddleAngleFiltered;

volatile uint32_t hapt_filterType = 0;
volatile uint32_t hapt_filterBandwidth = 5;
volatile uint32_t hapt_averageAngleSample = 1;
volatile uint32_t hapt_averageSpeedSample = 50;

volatile float32_t hapt_velocityNoise = 1.0f;

volatile float32_t friction;
volatile float32_t viscous;
volatile float32_t gravity;
volatile float32_t virtualWall;

volatile float32_t hapt_receivedPosition = 0.0;

volatile float32_t integrator = 0.0;
volatile float32_t i_max = 0.0;
volatile float32_t error_vel = 0.0;
volatile float32_t gain_p = 0.0;
volatile float32_t gain_d = 0.0;
volatile float32_t gain_i = 0.0;

void hapt_Update(void);

/**
  * @brief Initializes the haptic controller.
  */
void hapt_Init(void)
{
    hapt_timestamp = 0;
    hapt_motorTorque = 0.0f;

    // Make the timers call the update function periodically.
    cbt_SetHapticControllerTimer(hapt_Update, DEFAULT_HAPTIC_CONTROLLER_PERIOD);

    // Share some variables with the computer.
    comm_monitorUint32Func("Timestep [us]", cbt_GetHapticControllerPeriod,
                           cbt_SetHapticControllerPeriod);
    comm_monitorFloat("Motor_torque [N.m]", (float32_t*)&hapt_motorTorque, READWRITE);
    comm_monitorFloat("Paddle_angle [deg]", (float32_t*)&hapt_paddleAngle, READONLY);
    comm_monitorFloat("Paddle_velocity [deg/s]", (float32_t*)&hapt_paddleVelocity, READONLY);

    //MODES
    comm_monitorUint32("Mode (0:Disabled, 1:Hall, 2:Hal filtered, 3:Encoder)", &hapt_positionInput, READWRITE);

    comm_monitorUint32("Compensate dry friction (0:Disabled, 1:Active)", &hapt_dryFrictionComp, READWRITE);
    comm_monitorUint32("Compensate viscious friction (0:Disabled, 1:Active)", &hapt_visFrictionComp, READWRITE);
    comm_monitorUint32("Compensate gravity (0:Disabled, 1:Active)", &hapt_gravityComp, READWRITE);
    comm_monitorUint32("Simulate wall (0:Disabled, 1:Active)", &hapt_virtualWall, READWRITE);

    comm_monitorFloat("Virtual wall stiffness [N.m/deg]", (float32_t*)&hapt_virtualWallK, READWRITE);
    comm_monitorFloat("Virtual wall damping [N.m/(deg/s)]", (float32_t*)&hapt_virtualWallB, READWRITE);

    comm_monitorUint32("Filter type (0:MA, 1:RON)", &hapt_filterType, READWRITE);
    comm_monitorFloat("Filter bandwidth [Hz]", (float32_t*)&hapt_filterBandwidth, READWRITE);
    comm_monitorUint32("Number of samples to average angle (max 100)", &hapt_averageAngleSample, READWRITE);
    comm_monitorUint32("Number of samples to average speed (max 100)", &hapt_averageSpeedSample, READWRITE);

    comm_monitorFloat("Noise on velocity [deg/s]", (float32_t*)&hapt_velocityNoise, READWRITE);

    comm_monitorFloat("Dry friction [Nm]", (float32_t*)&friction, READONLY);
    comm_monitorFloat("Viscous friction [Nm]", (float32_t*)&viscous, READONLY);
    comm_monitorFloat("Gravity [Nm]", (float32_t*)&gravity, READONLY);

    comm_monitorFloat("Receive torque [Nm]", (float32_t*)&hapt_receivedPosition, READONLY);
    comm_monitorFloat("Encoder_pos [deg]", (float32_t*)&hapt_encoderPaddleAngle, READONLY);
    comm_monitorFloat("Hall_pos [deg]", (float32_t*)&hapt_hallPaddleAngle, READONLY);
    comm_monitorFloat("Hall_pos_filtered [deg]", (float32_t*)&hapt_hallPaddleAngleFiltered, READONLY);

    comm_monitorFloat("hall_voltage [V]", (float32_t*)&hapt_hallVoltage, READONLY);

	comm_monitorFloat("PID intergrator", (float32_t*)&integrator, READONLY);
	comm_monitorFloat("PID error velocity", (float32_t*)&error_vel, READONLY);
	comm_monitorFloat("PID I max", (float32_t*)&i_max, READWRITE);
	comm_monitorFloat("PID P gain", (float32_t*)&gain_p, READWRITE);
	comm_monitorFloat("PID I gain", (float32_t*)&gain_i, READWRITE);
	comm_monitorFloat("PID D gain", (float32_t*)&gain_d, READWRITE);


    //comm_monitorFloat("Delta position", (float32_t*)&delta_position, READONLY);
    //comm_monitorFloat("Speed", (float32_t*)&speed, READONLY);
    //comm_monitorFloat("Speed Hall", (float32_t*)&speed_hall, READONLY);
    //comm_monitorFloat("Acc Hall", (float32_t*)&acc_hall, READONLY);
    //comm_monitorFloat("Acc", (float32_t*)&acc, READONLY);
    //comm_monitorFloat("Gain", (float32_t*)&gain, READWRITE);
    //comm_monitorFloat("Rest position", (float32_t*)&rest_position, READWRITE);

    exuart_Init(115200);

}

void getPaddleAngle();
void getPaddleSpeed();

float compensateDryFriction();
float compensateVisciousFriction();
float compensateGravity();
float simulteVirtualWall();
float goToPosition(float setpoint);

float wrap_pi(float x);
float degrees(float radians);
float radians(float degrees);
float constrain(float val, float min, float max);

void send(float data);
bool receive(float* output);

/**
  * @brief Updates the haptic controller state.
  */
/*
#define DELTA 0.1f
void hapt_Update()
{
	float32_t motorShaftAngle; // [deg].

	static uint32_t last_mode;
	static float32_t last_angle;

	//Get the encoder angle
	motorShaftAngle = enc_GetPosition();
	hapt_encoderPaddleAngle = motorShaftAngle / REDUCTION_RATIO;

	if(mode == 1) {
		if(last_angle + DELTA > hapt_encoderPaddleAngle && last_angle - DELTA < hapt_encoderPaddleAngle)
			hapt_motorTorque += increment_value;
	}
	else if(mode == 2) {
		if(last_angle + DELTA > hapt_encoderPaddleAngle && last_angle - DELTA < hapt_encoderPaddleAngle)
			hapt_motorTorque -= increment_value;
	}
	else {
		hapt_motorTorque = 0.0f;
		last_angle = hapt_encoderPaddleAngle;
	}

	torq_SetTorque(hapt_motorTorque);
}
*/
/**
  * @brief Updates the haptic controller state.
  */
void hapt_Update()
{
	hapt_motorTorque = 0;

	getPaddleAngle();
	getPaddleSpeed();

	if(hapt_dryFrictionComp) {
		friction = compensateDryFriction();
		hapt_motorTorque += friction;
	}

	if(hapt_visFrictionComp) {
		viscous = compensateVisciousFriction();
		hapt_motorTorque += viscous;
	}

	if(hapt_gravityComp) {
		gravity = compensateGravity();
		hapt_motorTorque += gravity;
	}

	if(hapt_virtualWall) {
		virtualWall = simulteVirtualWall();
		hapt_motorTorque += virtualWall;
	}

	float recv_position;
	static float32_t dt_recv_position = RECV_POS_TIMEOUT;
	if(receive(&recv_position)) {
		hapt_receivedPosition = recv_position;
		dt_recv_position = 0;
	}
	else {
		dt_recv_position += ((float32_t)cbt_GetHapticControllerPeriod()) / 1000000.0f;
	}

	if(dt_recv_position < RECV_POS_TIMEOUT) {
		hapt_motorTorque += hapt_receivedPosition;
	}
	else {
		hapt_receivedPosition = 0.0;
	}

	static float32_t dt_send_torque = SEND_TORQUE_PERIOD;
	if(dt_send_torque > SEND_TORQUE_PERIOD) {
		send(hapt_receivedPosition);
		dt_send_torque = 0;
	}
	else {
		dt_send_torque += ((float32_t)cbt_GetHapticControllerPeriod()) / 1000000.0f;
	}



	torq_SetTorque(hapt_motorTorque);

	//exuart_SendByteAsync();
	//exuart_GetByte();
}

void send(float data) {
	char buffer[32];
	sprintf(buffer, "#%.2f$", data);
	exuart_SendBytesAsync((uint8_t *) buffer, strlen(buffer));
}

bool receive(float *output) {
	static char buffer[32];
	static uint8_t i = 0;
	while(exuart_ReceivedBytesCount()) {
		uint8_t data = exuart_GetByte();
		if(data == '#') {
			i = 0;
		}
		else if(data == '$' && i != 0) {
			float output_temp;
			buffer[i] = '/0';
			if(sscanf(buffer, "%f", &output_temp) == 1) {
				*output = output_temp;
				return true;
			}
		}
		else if(i > 30) {
			i = 0;
		}
		else {
			buffer[i] = data;
			i++;
		}
	}
	return false;
}

float goToPosition(float setpoint) {
	// Compute the dt (uncomment if you need it).
	float32_t dt = ((float32_t)cbt_GetHapticControllerPeriod()) / 1000000.0f; // [s].

	static float32_t last_error = 0.0f;
	float32_t error = setpoint - hapt_paddleAngle;

	integrator += error * gain_i * dt;
	if(integrator > i_max) {
		integrator = i_max;
	}
	else if(integrator < -i_max) {
		integrator = -i_max;
	}

	error_vel = (error - last_error) / dt;

	last_error = error;

	return error * gain_p + integrator + error_vel * gain_d;
}

float simulteVirtualWall() {
	if(hapt_paddleAngle > VIRTUAL_WALL_ANGLE) {
		return (hapt_paddleAngle - VIRTUAL_WALL_ANGLE) * hapt_virtualWallK + hapt_paddleVelocity * hapt_virtualWallB;
	}
	else if(hapt_paddleAngle < -VIRTUAL_WALL_ANGLE){
		return (hapt_paddleAngle + VIRTUAL_WALL_ANGLE) * hapt_virtualWallK + hapt_paddleVelocity * hapt_virtualWallB;
	}
	else {
		return 0.0;
	}
}

float compensateGravity() {
	return sin(radians(hapt_paddleAngle)) * CENTER_OF_MASS_TO_ROTATION * PADDLE_WEIGHT * GRAVITY_ACC / REDUCTION_RATIO;
}

float compensateDryFriction() {
	if(hapt_paddleVelocity > hapt_velocityNoise * 2.0f) {
		return POSITIVE_DRY_FRICTION;
	}
	else if(hapt_paddleVelocity < -hapt_velocityNoise * 2.0f) {
		return NEGATIVE_DRY_FRICTION;
	}
	else {
		return 0.0f;
	}
}

float compensateVisciousFriction() {
	return VISCIOUS_FRICTION * radians(hapt_paddleVelocity);
}

void getPaddleAngle() {
	//Get the encoder angle
	float motorShaftAngle = enc_GetPosition();
	hapt_encoderPaddleAngle = motorShaftAngle / REDUCTION_RATIO;

    // Get the Hall sensor angle
    hapt_hallVoltage = hall_GetVoltage();
    hapt_hallPaddleAngle = HALL_TO_ANGLE_COEF_1 * hapt_hallVoltage + HALL_TO_ANGLE_COEF_2;

    if(hapt_averageAngleSample > MAX_SAMPLE_FOR_AVERAGE) {
    	hapt_averageAngleSample = MAX_SAMPLE_FOR_AVERAGE;
    }

    static int raw_hall_angle_index = 0;
    raw_hall_angle_index++;
    if(raw_hall_angle_index >= hapt_averageAngleSample) {
    	raw_hall_angle_index = 0;
    }

    paddleAngleTable[raw_hall_angle_index] = hapt_hallPaddleAngle;

    int i = 0;
    hapt_hallPaddleAngleFiltered = 0;
    for(i = 0; i < hapt_averageAngleSample; i++) {
    	hapt_hallPaddleAngleFiltered += paddleAngleTable[i];
    }

    hapt_hallPaddleAngleFiltered /= hapt_averageAngleSample;

	if(hapt_positionInput == 1) {
		hapt_paddleAngle = hapt_hallPaddleAngle;
	}
	else if(hapt_positionInput == 2) {
		hapt_paddleAngle = hapt_hallPaddleAngleFiltered;
	}
	else if(hapt_positionInput == 3) {
		hapt_paddleAngle = hapt_encoderPaddleAngle;
	}
	else {
		hapt_paddleAngle = 0.0f;
	}
}

void getPaddleSpeed() {
	static float32_t last_angle = 0;
	float32_t dt = ((float32_t)cbt_GetHapticControllerPeriod()) / 1000000.0f;

	if(hapt_averageSpeedSample > MAX_SAMPLE_FOR_AVERAGE) {
		hapt_averageSpeedSample = MAX_SAMPLE_FOR_AVERAGE;
	}

	static int speed_index = 0;
	speed_index++;
	if(speed_index >= hapt_averageSpeedSample) {
		speed_index = 0;
	}

	paddleSpeedTable[speed_index] = (hapt_paddleAngle - last_angle) / dt;
	last_angle = hapt_paddleAngle;

	if(hapt_filterType == 1) {
		float alpha = (2.0 * M_PI * dt * hapt_filterBandwidth) / (2.0 * M_PI * dt * hapt_filterBandwidth + 1.0);
		hapt_paddleVelocity = hapt_paddleVelocity * (1.0 -alpha) + paddleSpeedTable[speed_index] * alpha;
	}
	else {
		int i = 0;
		hapt_paddleVelocity = 0;
		for(i = 0; i < hapt_averageSpeedSample; i++) {
			hapt_paddleVelocity += paddleSpeedTable[i];
		}
		hapt_paddleVelocity /= hapt_averageSpeedSample;
	}
}

float constrain(float val, float min, float max)
{
	return (val < min) ? min : ((val > max) ? max : val);
}

float radians(float degrees)
{
	return (degrees / 180.0f) * M_PI;
}

float degrees(float radians)
{
	return (radians * 180.0f) / M_PI;
}

float wrap_pi(float x)
{
    int c = 0;

    while (x >= M_PI) {
        x -= 2 * M_PI;

        if (c++ > 100) {
            return 0.0f;
        }
    }

    c = 0;

    while (x < -M_PI) {
        x += 2 * M_PI;

        if (c++ > 100) {
            return INFINITY;
        }
    }

    return x;
}



