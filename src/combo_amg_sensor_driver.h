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


/*
#ifdef __cplusplus
extern "C" {
#endif */

#include <stdint.h>
#include <Arduino.h>
#include "driver_sensors_types.h"
#include "sensor_fusion_class.h"
#ifdef F_USING_SMARTSENSOR
#include "Adafruit_BNO055.h"
#endif
#ifdef __cplusplus
extern "C" {
#endif
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief This defines the sensor specific information for I2C.
 */
typedef struct
{
    registerDeviceInfo_t deviceInfo; /*!< I2C device context. */
    boolean isInitialized;              /*!< whether sensor is intialized or not.*/
    uint16_t slaveAddress;           /*!< slave address.*/
} i2c_sensorhandle_t;

/*! @brief This structure defines the amg_sensor raw data buffer.*/
typedef struct
{
    uint32_t timestamp; /*! The time this sample was recorded.  */
    int16_t accel[3];   /*!< The accel data */
    int16_t mag[3];     /*!< The mag data */
    int16_t gyro[3];     /*!< The gyro data */
} amg_sensor_data_t;

/*! @brief This structure defines the amg_sensor raw accel data buffer.*/
typedef struct
{
    uint32_t timestamp; /*! The time this sample was recorded.  */
    int16_t accel[3];   /*!< The accel data */
} acceldata_t;

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

bool I2CInitialize(int pin_i2c_sda, int pin_i2c_scl);

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
    i2c_sensorhandle_t *pSensorHandle, uint8_t index, uint16_t sAddress, uint8_t whoAmi);

/*! @brief      :  The interface function to set the I2C Idle Task.
 *  @param[in]  :  combo_amg_sensor_i2c_sensorhandle_t *pSensorHandle, handle to the sensor handle.
 *  @param[in]  :  registeridlefunction_t idleTask, function pointer to the function to execute on I2C Idle Time.
 *  @param[in]  :  void *userParam, the pointer to the user idle ftask parameters.
 *  @return        void.
 *  @constraints   This can be called any number of times only after FXOS8700_I2C_Initialize().
 *                 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant    No
 */
void Combo_AMG_I2C_SetIdleTask(i2c_sensorhandle_t *pSensorHandle,
                              registeridlefunction_t idleTask,
                              void *userParam);

/*! @brief       The interface function to configure the sensor.
 *  @details     This function configure the sensor with requested ODR, Range and registers in the regsiter pair array.
 *      #IFDEF F_USING_SMARTSENSOR configures also the spatial position of the sensor chip
 *                                 to have the correct axis remapping
 *  @param[in]   pSensorHandle      handle to the sensor.
 *  @param[in]   pRegWriteList      pointer to the register list.
 *  @constraints This can be called any number of times only after Combo_AMG_I2C_Initialize().
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::COMBO_I2C_Configure() returns the status .
 */
int32_t Combo_AMG_I2C_Configure(i2c_sensorhandle_t *pSensorHandle, const registerwritelist_t *pRegWriteList);

/*! @brief       The interface function to get sensors calibration/offset data from IMU internal registers.
 *  @details     This function configure the sensor with requested ODR, Range and registers in the regsiter pair array.
 *  @param[in]   pSensorHandle      handle to the sensor.
 *  @param[in]   pCalib_data        pointer to an array of uint8_t [] .
 *  @constraints This can be called any number of times only after Combo_AMG_I2C_Initialize().
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::COMBO_I2C_GetCalibrationData() returns the status .
 */
int32_t Combo_AMG_I2C_GetCalibrationData(i2c_sensorhandle_t *pSensorHandle, uint8_t *pCalib_data);

/*! @brief       The interface function to set sensors calibration/offset data into IMU internal registers.
 *  @details     This function configure the sensor with requested ODR, Range and registers in the regsiter pair array.
 *  @param[in]   pSensorHandle      handle to the sensor.
 *  @param[in]   pCalib_data        pointer to an array of uint8_t [] .
 *  @constraints This can be called any number of times only after Combo_AMG_I2C_Initialize().
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::COMBO_I2C_SetCalibrationData() returns the status .
 */
int32_t Combo_AMG_I2C_SetCalibrationData(i2c_sensorhandle_t *pSensorHandle, uint8_t *pCalib_data);

/*! @brief       The interface function to set the IMU in 9DOF operation mode.
 *  @details     This function configure the sensor with requested ODR, Range and registers in the regsiter pair array.
 *  @param[in]   pSensorHandle      handle to the sensor.
 *  @constraints This can be called any number of times only after Combo_AMG_I2C_Initialize().
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::COMBO_I2C_SetCalibrationData() returns the status .
 */
int32_t Combo_AMG_I2C_SetModeFusion(i2c_sensorhandle_t *pSensorHandle);

/*! @brief       The interface function to get the IMU in 9DOF operation mode.
 *  @details     This function configure the sensor with requested ODR, Range and registers in the regsiter pair array.
 *  @param[in]   pSensorHandle      handle to the sensor.
 *  @param[out]  pmode              pointer to the resulted mode.
 *  @constraints This can be called any number of times only after Combo_AMG_I2C_Initialize().
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::COMBO_I2C_SetCalibrationData() returns the status .
 */
int32_t Combo_AMG_I2C_GetModeFusion(i2c_sensorhandle_t *pSensorHandle, uint8_t* pmode);

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
int32_t Combo_AMG_I2C_ReadData(i2c_sensorhandle_t *pSensorHandle,
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
int32_t Combo_AMG_I2C_Deinit(i2c_sensorhandle_t *pSensorHandle);

// MEDIUM LEVEL (DRIVER) INTERFACE FUNCTIONS

int8_t Combo_AMG_sensor_Init(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);
/*! @brief       The interface function to init generic Combined Accel/mag/gyro sensor.
 *  @details     This function implements an abstraction layer versus hardware/library for this type of sensor.
 *               In case of Combo smart sensors it will call the init function of the whore combo/smart.
 *  @param[in]   sensor         pointer to the sensor data structure.
 *  @param[in]   sfg            pointer to the sensor fusion globals data structure.
 *  @constraints 
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::Accel_Init() returns the status .
 */
int8_t Accel_Init(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);

/*! @brief       The interface function to init generic magnetic sensor (logical or physical).
 *  @details     This function implements an abstraction layer versus hardware/library for this type of sensor.
 *               In case of Combo smart sensors it will call the init function of the whore combo/smart.
 *  @param[in]   sensor         pointer to the sensor data structure.
 *  @param[in]   sfg            pointer to the sensor fusion globals data structure.
 *  @constraints 
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::Mag_Init() returns the status .
 */
int8_t Mag_Init(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);

/*! @brief       The interface function to init generic gyroscope sensor (logical or physical).
 *  @details     This function implements an abstraction layer versus hardware/library for this type of sensor.
 *               In case of Combo smart sensors it will call the init function of the whore combo/smart.
 *  @param[in]   sensor         pointer to the sensor data structure.
 *  @param[in]   sfg            pointer to the sensor fusion globals data structure.
 *  @constraints 
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::Gyro_Init() returns the status .
 */
int8_t Gyro_Init(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);

/*! @brief       The interface function to init generic temperature sensor (logical or physical).
 *  @details     This function implements an abstraction layer versus hardware/library for this type of sensor.
 *               In case of Combo smart sensors it will call the init function of the whore combo/smart.
 *  @param[in]   sensor         pointer to the sensor data structure.
 *  @param[in]   sfg            pointer to the sensor fusion globals data structure.
 *  @constraints 
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::Therm_Init() returns the status .
 */
int8_t Therm_Init(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);

/*! @brief       The interface function to init generic Accelerometer/Magnetometer combo sensor (logical or physical).
 *  @details     This function implements an abstraction layer versus hardware/library for this type of sensor.
 *               In case of Combo smart sensors it will call the init function of the whore combo/smart.
 *  @param[in]   sensor         pointer to the sensor data structure.
 *  @param[in]   sfg            pointer to the sensor fusion globals data structure.
 *  @constraints 
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::Combo_Accel_Mag_Init() returns the status .
 */
int8_t Combo_Accel_Mag_Init(PhysicalSensor *sensor, SensorFusionGlobals *sfg);

/*! @brief       The interface function to init generic Accelerometer/Magnetometer/Gyroscope combo sensor (logical or physical).
 *  @details     This function implements an abstraction layer versus hardware/library for this type of sensor.
 *               In case of Combo smart sensors it will call the init function of the whore combo/smart.
 *  @param[in]   sensor         pointer to the sensor data structure.
 *  @param[in]   sfg            pointer to the sensor fusion globals data structure.
 *  @constraints 
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::Combo_Accel_Mag_Gyro_Init() returns the status .
 */
int8_t Combo_Accel_Mag_Gyro_Init(PhysicalSensor *sensor, SensorFusionGlobals *sfg);

/*! @brief       The interface function to init generic accelerometer sensor data (logical or physical).
 *  @details     This function implements an abstraction layer versus hardware/library for this type of sensor.
 *               In case of Combo smart sensors it will call the init function of the whore combo/smart.
 *  @param[in]   sensor         pointer to the sensor data structure.
 *  @param[in]   sfg            pointer to the sensor fusion globals data structure.
 *  @constraints This can be called only after Combo_AMG_I2C_Initialize().
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::Accel_Read() returns the status .
 */
int8_t Accel_Read(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);

/*! @brief       The interface function to read generic magnetic sensor data (logical or physical).
 *  @details     This function implements an abstraction layer versus hardware/library for this type of sensor.
 *               In case of Combo smart sensors it will call the init function of the whore combo/smart.
 *  @param[in]   sensor         pointer to the sensor data structure.
 *  @param[in]   sfg            pointer to the sensor fusion globals data structure.
 *  @constraints This can be called only after Combo_AMG_I2C_Initialize().
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::Mag_Read() returns the status .
 */
int8_t Mag_Read(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);

/*! @brief       The interface function to read generic gyroscope sensor data (logical or physical).
 *  @details     This function implements an abstraction layer versus hardware/library for this type of sensor.
 *               In case of Combo smart sensors it will call the init function of the whore combo/smart.
 *  @param[in]   sensor         pointer to the sensor data structure.
 *  @param[in]   sfg            pointer to the sensor fusion globals data structure.
 *  @constraints This can be called only after Combo_AMG_I2C_Initialize().
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::Gyro_Read() returns the status .
 */
int8_t Gyro_Read(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);

/*! @brief       The interface function to read generic temperature sensor data (logical or physical).
 *  @details     This function implements an abstraction layer versus hardware/library for this type of sensor.
 *               In case of Combo smart sensors it will call the init function of the whore combo/smart.
 *  @param[in]   sensor         pointer to the sensor data structure.
 *  @param[in]   sfg            pointer to the sensor fusion globals data structure.
 *  @constraints This can be called only after Combo_AMG_I2C_Initialize().
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::Therm_Read() returns the status .
 */
int8_t Therm_Read(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);

/*! @brief       The interface function to read generic Accelerometer/Magnetometer combo sensor data (logical or physical).
 *  @details     This function implements an abstraction layer versus hardware/library for this type of sensor.
 *               In case of Combo smart sensors it will call the init function of the whore combo/smart.
 *  @param[in]   sensor         pointer to the sensor data structure.
 *  @param[in]   sfg            pointer to the sensor fusion globals data structure.
 *  @constraints This can be called only after Combo_AMG_I2C_Initialize().
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::Combo_Accel_Mag_Read() returns the status .
 */
int8_t Combo_Accel_Mag_Read(PhysicalSensor *sensor, SensorFusionGlobals *sfg);

/*! @brief       The interface function to read generic Accelerometer/Magnetometer/Gyroscope combo sensor (logical or physical).
 *  @details     This function implements an abstraction layer versus hardware/library for this type of sensor.
 *               In case of Combo smart sensors it will call the init function of the whore combo/smart.
 *  @param[in]   sensor         pointer to the sensor data structure.
 *  @param[in]   sfg            pointer to the sensor fusion globals data structure.
 *  @constraints This can be called only after Combo_AMG_I2C_Initialize().
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::Combo_Accel_Mag_Gyro_Read() returns the status .
 */
int8_t Combo_Accel_Mag_Gyro_Read(PhysicalSensor *sensor, SensorFusionGlobals *sfg);

/*! @brief       The interface function to save to IMU smartsensor the calib data (calculater or initial offsets)
                 for accel/mag/gyro sensors
 *  @details     This function implements an abstraction layer versus hardware/library for this type of sensor.
 *               
 *  @param[in]   sfg            pointer to the sensor fusion globals data structure.
 *  @constraints This can be called only after Combo_AMG_I2C_Initialize().
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      void
 */
void SaveMagneticCalib(SensorFusionGlobals* sfg);
/*! @brief       The interface function to set to default (zero) IMU smartsensor calib data (offsets)
                 for accel/mag/gyro sensors
 *  @details     This function implements an abstraction layer versus hardware/library for this type of sensor.
 *               
 *  @param[in]   sfg            pointer to the sensor fusion globals data structure.
 *  @constraints This can be called only after Combo_AMG_I2C_Initialize().
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      void
 */
void EraseMagneticCalib(SensorFusionGlobals* sfg);
/*! @brief       The interface function to chage spatial reference system 
                 for accel/mag/gyro sensors (depending on physical position
                 the sensor is installed into)
 *  @details     This function implements an abstraction layer versus hardware/library for this type of sensor.
 *               
 *  @param[in]   sfg            pointer to the sensor fusion globals data structure.
 *  @param[in]   pos            spatial position (according to relative enum type)
 *  @constraints This can be called only after Combo_AMG_I2C_Initialize().
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      void
 */
void Get_NDOF_Calib(SensorFusionGlobals* sfg);
void SetAxisremap(SensorFusionGlobals* sfg, physical_sensor_position_t pos);
void setStatus(SensorFusionGlobals *sfg, fusion_status_t status);
fusion_status_t getStatus(SensorFusionGlobals *sfg);
void queueStatus(SensorFusionGlobals *sfg, fusion_status_t status);
void updateStatus(SensorFusionGlobals *sfg);
void testStatus(SensorFusionGlobals *sfg);
/// installSensor is used to instantiate a physical sensor driver into the
/// sensor fusion system. It doesn't actually communicate with the sensor.
/// This function is normally invoked via the "sfg." global pointer.
int8_t installSensor(
                     SensorFusionGlobals *sfg,      ///< top level fusion structure
                     PhysicalSensor *pSensor,       ///< pointer to structure describing physical sensor
                     uint16_t addr,                 ///< I2C address for sensor (if applicable)
                     uint16_t schedule,             ///< Sensor is read each time loop_count % schedule == 0
                     registerDeviceInfo_t *busInfo, ///< information required for bus power management
                     SensorType        sensor_type, ///< required to manage order of initialization
                     initializeSensor_t *initialize,///< pointer to sensor initialization function
                     readSensor_t *read);           ///< pointer to sensor read function

#ifdef __cplusplus
}
#endif

#endif // DRIVER_COMBO_AMG_SENSOR_HW
