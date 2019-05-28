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

#include "mpu_6050.h"

#include <limits.h>

#include "i2c.h"
#include "../lib/utils.h"

#define GRAVITY_INTENSITY 9.81f
#define SINT16_MAX_F ((float32_t)SHRT_MAX)

#define SLAVE_ADDRESS 0x68 ///< 0x68 if the AD0 pin is low, 0x69 otherwise.

#define REG_CONFIG       0x1A ///< Configuration register.
#define REG_GYRO_CONFIG  0x1B ///< Gyroscope configuration register.
#define REG_ACCEL_CONFIG 0x1C ///< Accelerometer configuration register.
#define REG_SMPLRT_DIV   0x19 ///< Sample rate divider register.
#define REG_ACCEL_VALUES 0x3b ///< Accelerometer measurements register.
#define REG_TEMP_VALUES  0x41 ///< Temperature measurements register.
#define REG_GYRO_VALUES  0x43 ///< Gyroscope measurements register.
#define REG_SIG_PATH_RST 0x68 ///< Signal path reset register.
#define REG_USER_CTRL    0x6A ///< User control register.
#define REG_POWER_MGMT_1 0x6B ///< Power management 1 register.
#define REG_WHO_AM_I     0x75 ///< Who am I register.

#define WHO_AM_I_VAL 0x68 ///< Expected value of the REG_WHO_AM_I register.

/// Gets the actual range value from a mpu_AccelRange enum value.
const float32_t MPU_ACCEL_RANGE_REG_TO_CONV_FACTOR[] =
{
     2.0f / SINT16_MAX_F * GRAVITY_INTENSITY,
     4.0f / SINT16_MAX_F * GRAVITY_INTENSITY,
     8.0f / SINT16_MAX_F * GRAVITY_INTENSITY,
    16.0f / SINT16_MAX_F * GRAVITY_INTENSITY
};

/// Gets the actual range value from a mpu_GyroRange enum value.
const float32_t MPU_GYRO_RANGE_REG_TO_CONV_FACTOR[] =
{
     250.0f / SINT16_MAX_F,
     500.0f / SINT16_MAX_F,
    1000.0f / SINT16_MAX_F,
    2000.0f / SINT16_MAX_F
};

bool mpu_initialized = false; ///< True if the MPU-6050 is initialized, false otherwise.

float32_t accelFactor, ///< Converts a raw 16-bit integer value to an acceleration [m/s^2].
          gyroFactor; ///< Converts a raw 16-bit integer value to an angular speed [deg/s].

/**
 * @brief Initializes the MPU-6050 device.
 * @param accelRange accelerometer range.
 * @param gyroRange gyroscope range.
 * @param bandwidth bandwith of the accelerometer and gyroscope.
 * @return true if the initialization was successful, false otherwise.
 */
bool mpu_Init(mpu_AccelRange accelRange, mpu_GyroRange gyroRange,
              mpu_Bandwidth bandwidth)
{
    uint16_t id;
    
    //
    accelFactor = MPU_ACCEL_RANGE_REG_TO_CONV_FACTOR[accelRange];
    gyroFactor = MPU_GYRO_RANGE_REG_TO_CONV_FACTOR[gyroRange];

    // Reset the MPU-6050.
    i2c_WriteRegister(SLAVE_ADDRESS, REG_POWER_MGMT_1, (1<<7)|(1<<6), NULL); // Device reset.
    utils_DelayMs(100);

    // Test the communication with the chip.
    id = i2c_ReadRegister(SLAVE_ADDRESS, REG_WHO_AM_I, NULL);

    if(id != WHO_AM_I_VAL) // Error.
        return false;

    // Reset the MPU-6050 again.
    i2c_WriteRegister(SLAVE_ADDRESS, REG_POWER_MGMT_1,
                      (1<<7)|(1<<6), NULL); // Device reset.
    utils_DelayMs(100);

    i2c_WriteRegister(SLAVE_ADDRESS, REG_SIG_PATH_RST,
                      (1<<2)|(1<<1)|(1<<0), NULL); // Signal path reset.
    utils_DelayMs(100);

    // Setup the MPU-60X0 registers.
    i2c_WriteRegister(SLAVE_ADDRESS, REG_USER_CTRL,
                      (0<<6) | // Disable FIFO.
                      (0<<5) | // Disable master mode for the external I2C sensor.
                      (0<<4) | // Do not disable the I2C slave interface (mandatory for the MPU-6050).
                      (0<<2) | // Do not reset the FIFO.
                      (0<<1) | // Do not reset the I2C.
                      (0<<0),  // Do not reset the signal paths for the sensors.
                      NULL);

    i2c_WriteRegister(SLAVE_ADDRESS, REG_POWER_MGMT_1,
                      (0<<7) | // Do not reset the device.
                      (0<<6) | // Disable sleep.
                      (0<<5) | // Disable the "cycle" mode.
                      (0<<3) | // Do not disable the temperature sensor.
                      (3<<0),  // PLL with Z axis gyro ref.
                      NULL);

    i2c_WriteRegister(SLAVE_ADDRESS, REG_GYRO_CONFIG,
                      (0<<7) | // Do not perform self-test for axis X.
                      (0<<6) | // Do not perform self-test for axis Y.
                      (0<<5) | // Do not perform self-test for axis Z.
                      (gyroRange<<3), // Gyroscope range.
                      NULL);

    i2c_WriteRegister(SLAVE_ADDRESS, REG_ACCEL_CONFIG,
                      (0<<7) | // Do not perform self-test for axis X.
                      (0<<6) | // Do not perform self-test for axis Y.
                      (0<<5) | // Do not perform self-test for axis Z.
                      (accelRange<<3), // Accelerometer range.
                      NULL);

    i2c_WriteRegister(SLAVE_ADDRESS, REG_CONFIG,
                      (0<<3) | // Disable synchronisation with FSYNC pin.
                      (bandwidth<<0), // Accelerometer and gyroscope bandwidth.
                      NULL);

    if(bandwidth == MPU_DLPF_BW_256HZ)
        i2c_WriteRegister(SLAVE_ADDRESS, REG_SMPLRT_DIV, 7, NULL);
    else
        i2c_WriteRegister(SLAVE_ADDRESS, REG_SMPLRT_DIV, 0, NULL);
        
    //
    mpu_initialized = true;
        
    return true;
}

/**
 * @brief Acquires the value of a single axis, from the IMU.
 * @param axis the IMU axis to get (see mpu_Axis).
 * @param value variable to write the value on. The unit depends on the selected
 * axis: [m/s^2] for an acceleration, [deg/s] for an angular speed, and
 * [celsius] for the temperature.
 * @remark if the MPU-6050 is not initialized (mpu_Init() was not called
 * successfully), then this function returns 0.0f.
 * @remark If the operation failed, then *value is not modified.
 */
void mpu_GetAxis(mpu_Axis axis, float32_t *value)
{
    bool ok;
    uint8_t tempData[2];
    float32_t decimalValue;

    if(!mpu_initialized)
        return;

    i2c_ReadMultiBytesRegister(SLAVE_ADDRESS, axis, &tempData[0], 2, &ok);

    if(!ok)
        return;

    decimalValue = ((float32_t)(int16_t)((tempData[0]<<8) | tempData[1]));

    switch(axis)
    {
    case MPU_AXIS_ACCEL_X:
    case MPU_AXIS_ACCEL_Y:
    case MPU_AXIS_ACCEL_Z:
        *value = decimalValue * accelFactor;
        break;

    case MPU_AXIS_TEMPERATURE:
        *value =  decimalValue / 340.0f + 36.53f; // Coefs from the registers map spec.
        break;

    case MPU_AXIS_GYRO_X:
    case MPU_AXIS_GYRO_Y:
    case MPU_AXIS_GYRO_Z:
        *value =  decimalValue * gyroFactor;
        break;

    default:
        break;
    }
}

/**
 * @brief Acquires the acceleration from the IMU.
 * @param ax variable to write the X-axis acceleration on [m/s^2].
 * @param ay variable to write the Y-axis acceleration on [m/s^2].
 * @param az variable to write the Z-axis acceleration on [m/s^2].
 * @remark ax, ay, and az pointer must be valid (they can't be NULL).
 * @remark if the MPU-6050 is not initialized (mpu_Init() was not called
 * successfully), then this function does nothing.
 * @remark If the operation failed, then *ax, *ay and *az are not modified.
 */
void mpu_GetAcceleration(float32_t *ax, float32_t *ay, float32_t *az)
{   
    bool ok;
    uint8_t accelData[6];
    
    if(!mpu_initialized)
        return;

    i2c_ReadMultiBytesRegister(SLAVE_ADDRESS, REG_ACCEL_VALUES,
                               &accelData[0], 6, &ok);
                               
    if(!ok)
        return;

    *ax = ((float32_t)(int16_t)((accelData[0]<<8) | accelData[1]))
            * accelFactor;
    *ay = ((float32_t)(int16_t)((accelData[2]<<8) | accelData[3]))
            * accelFactor;
    *az = ((float32_t)(int16_t)((accelData[4]<<8) | accelData[5]))
            * accelFactor;
}

/**
 * @brief Acquires the angular speed from the IMU.
 * @param gx variable to write the X-axis angular speed on [deg/s].
 * @param gy variable to write the Y-axis angular speed on [deg/s].
 * @param gz variable to write the Z-axis angular speed on [deg/s].
 * @remark gx, gy, and gz pointer must be valid (they can't be NULL).
 * @remark if the MPU-6050 is not initialized (mpu_Init() was not called
 * successfully), then this function does nothing.
 * @remark If the operation failed, then *ax, *ay and *az are not modified.
 */
void mpu_GetAngularSpeed(float32_t *gx, float32_t *gy, float32_t *gz)
{
    bool ok;
    uint8_t gyroData[6];
    
    if(!mpu_initialized)
        return;
    
    i2c_ReadMultiBytesRegister(SLAVE_ADDRESS, REG_GYRO_VALUES,
                               &gyroData[0], 6, &ok);
                               
    if(!ok)
        return;

    *gx = ((float32_t)(int16_t)((gyroData[0]<<8) | gyroData[1])) * gyroFactor;
    *gy = ((float32_t)(int16_t)((gyroData[2]<<8) | gyroData[3])) * gyroFactor;
    *gz = ((float32_t)(int16_t)((gyroData[4]<<8) | gyroData[5])) * gyroFactor;
}

/**
 * @brief Acquires the temperature from the IMU.
 * @return the measured temperature [celsius], or 0 if the operation failed.
 * @remark if the MPU-6050 is not initialized (mpu_Init() was not called
 * successfully), then this function returns 0.0f.
 */
float32_t mpu_GetTemperature(void)
{
    bool ok;
    uint8_t tempData[2];
    
    if(!mpu_initialized)
        return 0.0f;

    i2c_ReadMultiBytesRegister(SLAVE_ADDRESS, REG_TEMP_VALUES,
                               &tempData[0], 2, &ok);
                               
    if(!ok)
        return 0.0f;

    return ((float32_t)(int16_t)((tempData[0]<<8) | tempData[1]))
            / 340.0f + 36.53f; // Coefs from the registers map spec.
}
