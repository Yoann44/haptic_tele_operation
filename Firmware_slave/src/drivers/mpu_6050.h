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

#ifndef __MPU_6050_H
#define __MPU_6050_H

#include "../main.h"

/** @defgroup MPU-6050 Driver / MPU-6050
  * @brief Driver for the MPU-6050, a 6-axis IMU from Invensense.
  *
  * This driver uses the I2C bus of the digital extension connector.
  *
  * Call mpu_Init() first in the initialization code. Then, call mpu_Get() to
  * get the accelerations and angular speeds.
  *
  * @note This driver setup the MPU-6050 sampling rate to 1 kHz. Calling
  * mpu_Get() more often will return the same values.
  *
  * @addtogroup MPU-6050
  * @{
  */

/**
 * @brief Enumeration of the possible accelerometer ranges.
 */
typedef enum
{
    MPU_ACCEL_RANGE_2G  = 0, ///< +-2 G (+-19.62 m/s^2).
    MPU_ACCEL_RANGE_4G  = 1, ///< +-4 G (+-39.24 m/s^2).
    MPU_ACCEL_RANGE_8G  = 2, ///< +-8 G (+-78.48 m/s^2).
    MPU_ACCEL_RANGE_16G = 3  ///< +-16 G (+-156.96 m/s^2).
} mpu_AccelRange;

/**
  * @brief Enumeration of the possible gyrometer ranges.
  */
typedef enum
{
    MPU_GYRO_RANGE_250DPS  = 0, ///< +- 250 deg/s.
    MPU_GYRO_RANGE_500DPS  = 1, ///< +- 500 deg/s.
    MPU_GYRO_RANGE_1000DPS = 2, ///< +- 1000 deg/s.
    MPU_GYRO_RANGE_2000DPS = 3 ///< +- 2000 deg/s.
} mpu_GyroRange;

/**
 * @brief Enumeration of the possible bandwidths.
 */
typedef enum
{
    MPU_DLPF_BW_256HZ = 0, ///< Cut-off freq: 256 Hz. IMU sampling freq: 8kHz.
    MPU_DLPF_BW_188HZ = 1, ///< Cut-off freq: 188 Hz. IMU sampling freq: 1kHz.
    MPU_DLPF_BW_98HZ  = 2, ///< Cut-off freq: 98 Hz. IMU sampling freq: 1kHz.
    MPU_DLPF_BW_42HZ  = 3, ///< Cut-off freq: 42 Hz. IMU sampling freq: 1kHz.
    MPU_DLPF_BW_20HZ  = 4, ///< Cut-off freq: 20 Hz. IMU sampling freq: 1kHz.
    MPU_DLPF_BW_10HZ  = 5, ///< Cut-off freq: 10 Hz. IMU sampling freq: 1kHz.
    MPU_DLPF_BW_5HZ   = 6  ///< Cut-off freq: 5 Hz. IMU sampling freq: 1kHz.
} mpu_Bandwidth;

/**
 * @brief Enumeration of all the measurement registers.
 */
typedef enum
{
    MPU_AXIS_ACCEL_X     = 0x3b,
    MPU_AXIS_ACCEL_Y     = 0x3d,
    MPU_AXIS_ACCEL_Z     = 0x3f,
    MPU_AXIS_TEMPERATURE = 0x41,
    MPU_AXIS_GYRO_X      = 0x43,
    MPU_AXIS_GYRO_Y      = 0x45,
    MPU_AXIS_GYRO_Z      = 0x47
} mpu_Axis;

bool mpu_Init(mpu_AccelRange accelRange, mpu_GyroRange gyroRange,
              mpu_Bandwidth bandwidth);
void mpu_GetAxis(mpu_Axis axis, float32_t *value);
void mpu_GetAcceleration(float32_t *ax, float32_t *ay, float32_t *az);
void mpu_GetAngularSpeed(float32_t *gx, float32_t *gy, float32_t *gz);
float mpu_GetTemperature(void);

/**
  * @}
  */

#endif
