/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file combo_amg_sensor_hw.h
 * @brief Describes the fxos8700 driver interface and structures.
 */

#ifndef DRIVER_COMBO_AMG_SENSOR_HW_H_
#define DRIVER_COMBO_AMG_SENSOR_HW_H_


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <Arduino.h>
#include "driver_sensors_types.h"

// Library dependencies (Adafruit BNO055 in the case) defined here
#include "Adafruit_I2CDevice.h"
#include "Adafruit_I2CRegister.h"
#include "Adafruit_BNO055.h"
#include "Wire.h"

using OR_sensor = Adafruit_BNO055 ; //try to generalize interface vs. library/sensor specific

typedef enum or_sensor_ss_t {
 // System Status (see section 4.3.58)
     Idle = 0,
     System_Error = 1,
     Initializing_Peripherals =2,
     System_Iniitalization = 3,
     Executing_Self_Test = 4,
     Sensor_fusio_algorithm_running = 5,
     System_running_without_fusion_algorithms = 6
} or_sensor_ss_t;

/* Self Test Results to be tested as "all good" and passed as it is
   for futher analysis by bit masking
     1 = test passed, 0 = test failed

     Bit 0 = Accelerometer self test
     Bit 1 = Magnetometer self test
     Bit 2 = Gyroscope self test
     Bit 3 = MCU self test

     0x0F = all good!
   */

typedef enum or_sensor_se_t {
    // System Error (see section 4.3.59)
     No_error = 0x00,
     Peripheral_initialization_error = 0x01,
     System_initialization_error = 0x02,
     Self_test_result_failed = 0x03,
     Register_map_value_out_of_range = 0x04,
     Register_map_address_out_of_range =0x05,
     Register_map_write_error = 0x06,
     BNO_low_power_mode_not_available_for_selected_operation_mode = 0x07,
     Accelerometer_power_mode_not_available = 0x08,
     Fusion_algorithm_configuration_error = 0x09,
     Sensor_configuration_error = 0x0A
} or_sensor_se_t;

// end of library dependencies

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief This defines the sensor specific information for I2C.
 */
typedef struct
{
    registerDeviceInfo_t deviceInfo; /*!< I2C device context. */
    bool isInitialized;              /*!< whether sensor is intialized or not.*/
    uint16_t slaveAddress;           /*!< slave address.*/
} combo_amg_sensor_i2c_sensorhandle_t;

/*! @brief This structure defines the amg_sensor raw data buffer.*/
typedef struct
{
    uint32_t timestamp; /*! The time this sample was recorded.  */
    int16_t accel[3];   /*!< The accel data */
    int16_t mag[3];     /*!< The mag data */
    int16_t gyro[3];     /*!< The gyro data */
} combo_amg_sensor_data_t;

/*! @brief This structure defines the amg_sensor raw accel data buffer.*/
typedef struct
{
    uint32_t timestamp; /*! The time this sample was recorded.  */
    int16_t accel[3];   /*!< The accel data */
} combo_amg_sensor_acceldata_t;

/*! @def    COMBO_SPI_MAX_MSG_SIZE
 *  @brief  The MAX size of SPI message. */
#define COMBO_AMG_SPI_MAX_MSG_SIZE (64)

/*! @def    COMBO_SPI_CMD_LEN
 *  @brief  The size of the Sensor specific SPI Header. */
#define COMBO_AMG_SPI_CMD_LEN (2)

/*! @def    COMBO_SS_ACTIVE_VALUE
 *  @brief  Is the Slave Select Pin Active Low or High. */
#define COMBO_AMG_SS_ACTIVE_VALUE SPI_SS_ACTIVE_LOW

/*******************************************************************************
 * APIs
 ******************************************************************************/
//TODO - can toss the Idle functions

/*! @brief       The interface function to initialize the sensor.
 *  @details     This function initializes the sensor and sensor handle.
 *  @param[in]   pSensorHandle  handle to the sensor.
 *  @param[in]   index          the I2C device number.
 *  @param[in]   sAddress       slave address of the device on the bus.
 *  @param[in]   whoami         WHO_AM_I value of the device.
 *  @constraints This should be the first API to be called.
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::Combo_AMG__I2C_Initialize() returns the status .
 */
int32_t Combo_AMG_I2C_Initialize(
    combo_amg_sensor_i2c_sensorhandle_t *pSensorHandle, uint8_t index, uint16_t sAddress, uint8_t whoAmi);

/*! @brief      :  The interface function to set the I2C Idle Task.
 *  @param[in]  :  combo_amg_sensor_i2c_sensorhandle_t *pSensorHandle, handle to the sensor handle.
 *  @param[in]  :  registeridlefunction_t idleTask, function pointer to the function to execute on I2C Idle Time.
 *  @param[in]  :  void *userParam, the pointer to the user idle ftask parameters.
 *  @return        void.
 *  @constraints   This can be called any number of times only after FXOS8700_I2C_Initialize().
 *                 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant    No
 */
void Combo_AMG_I2C_SetIdleTask(combo_amg_sensor_i2c_sensorhandle_t *pSensorHandle,
                              registeridlefunction_t idleTask,
                              void *userParam);

/*! @brief       The interface function to configure the sensor.
 *  @details     This function configure the sensor with requested ODR, Range and registers in the regsiter pair array.
 *  @param[in]   pSensorHandle      handle to the sensor.
 *  @param[in]   pRegWriteList      pointer to the register list.
 *  @constraints This can be called any number of times only after Combo_AMG_I2C_Initialize().
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::COMBO_I2C_Configure() returns the status .
 */
int32_t Combo_AMG_I2C_Configure(combo_amg_sensor_i2c_sensorhandle_t *pSensorHandle, const registerwritelist_t *pRegWriteList);

/*! @brief       The interface function to read the sensor data.
 *  @details     This function read the sensor data out from the device and returns raw data in a byte stream.
 *  @param[in]   pSensorHandle  handle to the sensor.
 *  @param[in]   pReadList      pointer to the list of device registers and values to read.
 *  @param[out]  pBuffer        buffer which holds raw sensor data.This buffer may be back to back databuffer based
 *                              command read in the list.
 *  @constraints This can be called any number of times only after Combo_AMG_I2C_Initialize().
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::COMBO_I2C_ReadData() returns the status .
 */
int32_t Combo_AMG_I2C_ReadData(combo_amg_sensor_i2c_sensorhandle_t *pSensorHandle,
                              const registerReadlist_t *pReadList,
                              uint8_t *pBuffer);

/*! @brief       The interface function to De Initialize sensor..
 *  @details     This function made sensor in a power safe state and de initialize its handle.
 *  @param[in]   pSensorHandle      handle to the sensor.
 *  @constraints This can be called only after Combo_AMG_I2C_Initialize().
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::COMBO_I2C_Deinit() returns the status .
 */
int32_t Combo_AMG_I2C_Deinit(combo_amg_sensor_i2c_sensorhandle_t *pSensorHandle);

#ifdef __cplusplus
}
#endif

#endif // DRIVER_COMBO_AMG_SENSOR_HW
