/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright (c) 2016-2017 NXP
 * Copyright (c) 2020 Bjarne Hansen
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! \file combo_amg_sensor_driver.c
    \brief Defines commands to perform various tasks (e.g. read Device ID, Initialize, Read Data) 
    for the FXOS8700 6-axis accelerometer plus magnetometer. Actual I2C interface functions are
    found in sensor_io_i2c files.
*/

#include "build.h"
#include "combo_amg_sensor_driver.h"      // Generic ComboAMG_Sensor hardware interface
#include "driver_sensors.h"             // prototypes for *_Init() and *_Read() methods
#include "driver_sensors_types.h"
#include "sensor_fusion.h"
#include "sensor_fusion_class.h"
#include "Wire.h"
#undef EXTERN_BARO_THERM_SENSOR

//using OR_sensor = Adafruit_BNO055 ;
OR_sensor *Combo_AMG_sensor;

#define COMBO_I2C_ADDR (BNO055_ADDRESS_B) //due to particular breakout the default I2C address is this instead of chip default
#define COMBO_WHO_AM_I (BNO055_ID)

// Each entry in a RegisterWriteList is composed of: register address, value to write, bit-mask to apply to write (0 enables)
const registerwritelist_t   AMG_FULL_IDLE[] =
{
  // Set ACTIVE = other bits unchanged
  { COMBO_I2C_ADDR, 0x00, 0x01 }, //TO BE VERIFIED
    __END_WRITE_DATA__
};

/* ABSTRACT BUS LEVEL FUNCTIONs */
bool I2CInitialize(int pin_i2c_sda, int pin_i2c_scl) {
    return Wire.setPins (pin_i2c_sda,pin_i2c_sda);
}
int32_t Sensor_I2C_Read(registerDeviceInfo_t *devinfo, uint16_t addr,registerReadlist_t read_reg, uint8_t* I2C_Buffer
                            ){
     //placeholder unused because using BNO055 higher level abstraction interface
}
/* END OF BUS LEVEL FUNCTIONS*/

/* LOWER LEVEL FUNCTIONS TO INTERFACE GIVEN SENSOR DRIVER LIBRARY*/
int32_t Combo_AMG_I2C_Initialize(   combo_amg_sensor_i2c_sensorhandle_t *pSensorHandle, uint8_t index, uint16_t sAddress,
                                    uint8_t whoAmi){
    uint8_t status, test_result, error;
    or_sensor_se_t or_sensor_error;
    Combo_AMG_sensor = new (OR_sensor);
    if (!Combo_AMG_sensor->begin(OPERATION_MODE_NDOF)) {
        Combo_AMG_sensor->getSystemStatus(&status, &test_result, &error);
        or_sensor_error = (or_sensor_se_t) error;
        if (or_sensor_error != No_error) {
            
            return error;
        }
    } return 0x0;
    }

void Combo_AMG_I2C_SetIdleTask(combo_amg_sensor_i2c_sensorhandle_t *pSensorHandle, registeridlefunction_t idleTask,
                              void *userParam){
//placeholder
}

int32_t Combo_AMG_I2C_Configure(combo_amg_sensor_i2c_sensorhandle_t *pSensorHandle, const registerwritelist_t *pRegWriteList){
//placeholder
}

int32_t Combo_AMG_I2C_ReadData(combo_amg_sensor_i2c_sensorhandle_t *pSensorHandle, const registerReadlist_t *pReadList,
                              uint8_t *pBuffer){
//placeholder
}

int32_t Combo_AMG_I2C_Deinit(combo_amg_sensor_i2c_sensorhandle_t *pSensorHandle){
//placeholder
}
/* END OF LOWER LEVEL INTERFACE FUNCTIONS */

/* FUSION DATA STRUCTURE RELATED FUNCTIONS*/
void fInitializeFusion (SensorFusionGlobals *sfg) {
     //placeholder
}
/* END OF FUSION DATA STRUCTURE RELATED FUNCTIONS*/

/* ABSTRACT SENSOR INTERFACE FUNCTIONS*/
void fInitializeMagCalibration (SensorFusionGlobals *sfg) {
     //placeholder
}
int8_t Combo_AMG_sensor_Init(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg){
    return (int8_t) 
}


#if F_USING_ACCEL
int8_t Combo_AMG_Accel_Read(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg) {
    uint8_t                     I2C_Buffer[6 * ACCEL_FIFO_SIZE];    // I2C read buffer
    int32_t                     status;         // I2C transaction status
    int8_t                      j;              // scratch
    uint8_t                     fifo_packet_count;
    int16_t                     sample[3];

    if(!(sensor->isInitialized & F_USING_ACCEL)) {
       return SENSOR_ERROR_INIT;
    }
/* IMPLEMENTATION HW FX8700 dependent 
    // read the F_STATUS register (mapped to STATUS) and extract number of
    // measurements available (lower 6 bits)
    status = Sensor_I2C_Read(&sensor->deviceInfo,
                             sensor->addr, FXOS8700_F_STATUS_READ, I2C_Buffer);
    if (status == SENSOR_ERROR_NONE) {
#ifdef SIMULATOR_MODE
      fifo_packet_count = 1;
#else
      fifo_packet_count = I2C_Buffer[0] & FXOS8700_F_STATUS_F_CNT_MASK;
#endif
      // return if there are no measurements in the sensor FIFO.
      // this will only occur when the calling frequency equals or exceeds
      // ACCEL_ODR_HZ
      if (fifo_packet_count == 0) {
        return (SENSOR_ERROR_READ);
      }
    } else {
      return (status);
    }

    // Steady state when fusing at 40 Hz is 5 packets per cycle to read (accel
    // updates at 200 Hz). Noticed that I2C reads > 126 bytes don't work, so
    // limit the number of FIFO packets per burst read. With the address
    // auto-increment and wrap turned on, the registers are read
    // 0x01,0x02,...0x05,0x06,0x01,0x02,...  So we read 6 bytes per packet.
#define MAX_FIFO_PACKETS_PER_READ 15  // for max of 90 bytes per I2C xaction.
    FXOS8700_DATA_READ[0].readFrom = FXOS8700_OUT_X_MSB;  
    while ((fifo_packet_count > 0) && (status == SENSOR_ERROR_NONE)) {
      if (MAX_FIFO_PACKETS_PER_READ < fifo_packet_count) {
        FXOS8700_DATA_READ[0].numBytes = 6 * MAX_FIFO_PACKETS_PER_READ;
        fifo_packet_count -= MAX_FIFO_PACKETS_PER_READ;
      } else {
        FXOS8700_DATA_READ[0].numBytes = 6 * fifo_packet_count;
        fifo_packet_count = 0;
      }
      status = Sensor_I2C_Read(&sensor->deviceInfo,
                               sensor->addr, FXOS8700_DATA_READ, I2C_Buffer);
      if (status == SENSOR_ERROR_NONE) {
        for (j = 0; j < FXOS8700_DATA_READ[0].numBytes; j+=6) {
            // place the measurements read into the accelerometer buffer structure 
            sample[CHX] = (I2C_Buffer[j + 0] << 8) | (I2C_Buffer[j + 1]); 
            sample[CHY] = (I2C_Buffer[j + 2] << 8) | (I2C_Buffer[j + 3]); 
            sample[CHZ] = (I2C_Buffer[j + 4] << 8) | (I2C_Buffer[j + 5]);
            conditionSample(sample);  //truncate negative values to -32767
            // place the 6 bytes read into the 16 bit accelerometer structure 
            addToFifo((union FifoSensor*) &(sfg->Accel), ACCEL_FIFO_SIZE, sample);
        } // end transfer all bytes from each packet
      } // end processing a burst read
    } // end emptying all packets from FIFO
    return (status);*/
}  // end Combo_AMG_Accel_Read()
#endif
#if F_USING_MAG
// read FXOS8700 magnetometer over I2C
int8_t Combo_AMG_Mag_Read(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg) {
    uint8_t                     I2C_Buffer[6];  // I2C read buffer
    int32_t                     status;         // I2C transaction status
    int16_t                     sample[3];

    if(!(sensor->isInitialized & F_USING_MAG))
    {
        return SENSOR_ERROR_INIT;
    }
/*  OLD IMPLEMENTATION depending on FXOS7800
    // read the six sequential magnetometer output bytes
    FXOS8700_DATA_READ[0].readFrom = FXOS8700_M_OUT_X_MSB;
    FXOS8700_DATA_READ[0].numBytes = 6;
    status =  Sensor_I2C_Read(&sensor->deviceInfo, sensor->addr, FXOS8700_DATA_READ, I2C_Buffer );
    if (status==SENSOR_ERROR_NONE) {
        // place the 6 bytes read into the magnetometer structure
        sample[CHX] = (I2C_Buffer[0] << 8) | I2C_Buffer[1];
        sample[CHY] = (I2C_Buffer[2] << 8) | I2C_Buffer[3];
        sample[CHZ] = (I2C_Buffer[4] << 8) | I2C_Buffer[5];
        conditionSample(sample);  // truncate negative values to -32767
        addToFifo((union FifoSensor*) &(sfg->Mag), MAG_FIFO_SIZE, sample);
    }*/
    return status;
}//end FXOS8700_ReadMagData()
#endif  //F_USING_MAG

// read temperature register over I2C
int8_t Combo_AMG_Therm_Read(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg) {
    int8_t                      I2C_Buffer;     // I2C read buffer
    int32_t                     status;         // I2C transaction status

    if(!(sensor->isInitialized)) {
        return SENSOR_ERROR_INIT;
    }
    /*  OLD IMPLEMENTATION depending on FXOS7800
    // read the Temperature register 0x51
    FXOS8700_DATA_READ[0].readFrom = FXOS8700_TEMP;
    FXOS8700_DATA_READ[0].numBytes = 1;
    status =  Sensor_I2C_Read(&sensor->deviceInfo, sensor->addr, FXOS8700_DATA_READ, (uint8_t*)(&I2C_Buffer) );
    if (status==SENSOR_ERROR_NONE) {
        // convert the byte to temperature and place in sfg structure
        sfg->Temp.temperatureC = (float)I2C_Buffer * 0.96; //section 14.3 of manual says 0.96 degC/LSB
    }
    */
    return status;
}//end Combo_AMG_Therm_Read()

// read Combo_AMG gyro data  over I2C
int8_t Combo_AMG_Gyro_Read(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg)
{
    uint8_t     I2C_Buffer[6 * GYRO_FIFO_SIZE]; // I2C read buffer
    uint8_t      j;                              // scratch
    uint8_t     fifo_packet_count = 1;
    int32_t     status;
    int16_t     sample[3];

     if (sensor->isInitialized != F_USING_GYRO) {
      return SENSOR_ERROR_INIT;
    }
/* OLD IMPLEMENTATION DEPENDING ON FXAS2100
     // read the F_STATUS register (mapped to STATUS) and extract number of measurements available (lower 6 bits)
    status =  Sensor_I2C_Read(&sensor->deviceInfo, sensor->addr, FXAS21002_F_STATUS_READ, I2C_Buffer );
//    status = SENSOR_ERROR_NONE;
    if (status == SENSOR_ERROR_NONE) {
#ifdef SIMULATOR_MODE
        fifo_packet_count = 1;
#else
        fifo_packet_count = I2C_Buffer[0] & FXAS21002_F_STATUS_F_CNT_MASK ;
#endif
        // return if there are no measurements in the FIFO.
        // this will only occur when the calling frequency equals or exceeds GYRO_ODR_HZ
        if (fifo_packet_count == 0) return(SENSOR_ERROR_READ);
    } else {
      return (status);
    }
          // at this point there must be at least one measurement in the FIFO
          // available to read. handle the FXAS21000 and FXAS21002 differently
          // because only FXAS21002 supports WRAPTOONE feature.
          if (sfg->Gyro.iWhoAmI == FXAS21002_WHO_AM_I_WHOAMI_OLD_VALUE) {
//    if (true) {
            // read six sequential gyro output bytes
            FXAS21002_DATA_READ[0].readFrom = FXAS21002_OUT_X_MSB;
            FXAS21002_DATA_READ[0].numBytes = 6;

            // for FXAS21000, perform sequential 6 byte reads
            for (j = 0; j < fifo_packet_count; j++) {
              // read one set of measurements totalling 6 bytes
              status = Sensor_I2C_Read(&sensor->deviceInfo,
                                       sensor->addr, FXAS21002_DATA_READ,
                                       I2C_Buffer);

              if (status == SENSOR_ERROR_NONE) {
                // place the measurements read into the gyroscope buffer structure
                sample[CHX] = (I2C_Buffer[0] << 8) | I2C_Buffer[1];
                sample[CHY] = (I2C_Buffer[2] << 8) | I2C_Buffer[3];
                sample[CHZ] = (I2C_Buffer[4] << 8) | I2C_Buffer[5];
                conditionSample(sample);  // truncate negative values to -32767
                addToFifo((union FifoSensor*) &(sfg->Gyro), GYRO_FIFO_SIZE, sample);
            }
        }
    }   // end of FXAS21000 FIFO read
    else
    {  //Steady state when fusing at 40 Hz is 10 packets per cycle to read (gyro updates at 400 Hz). Takes 4 ms to read.
        // for FXAS21002, clear the FIFO in burst reads using WRAPTOONE feature, which decreases read time to 2 ms. 
        //Noticed that I2C reads > 126 bytes don't work, so limit the number of FIFO packets per burst read.
#define MAX_FIFO_PACKETS_PER_READ 11        
        FXAS21002_DATA_READ[0].readFrom = FXAS21002_OUT_X_MSB;
        while( (fifo_packet_count > 0)  && (status==SENSOR_ERROR_NONE)) {
            if( MAX_FIFO_PACKETS_PER_READ < fifo_packet_count ) {
               FXAS21002_DATA_READ[0].numBytes = MAX_FIFO_PACKETS_PER_READ * 6;
               fifo_packet_count -= MAX_FIFO_PACKETS_PER_READ;
            }else {
                FXAS21002_DATA_READ[0].numBytes = fifo_packet_count * 6;
                fifo_packet_count = 0;
            }
            status = Sensor_I2C_Read(&sensor->deviceInfo,
                                     sensor->addr, FXAS21002_DATA_READ,
                                     I2C_Buffer);
            if (status==SENSOR_ERROR_NONE) {
                for (j = 0; j < FXAS21002_DATA_READ[0].numBytes; j+=6) {
                    // place the measurements read into the gyroscope buffer structure
                    sample[CHX] = (I2C_Buffer[j + 0] << 8) | I2C_Buffer[j + 1];
                    sample[CHY] = (I2C_Buffer[j + 2] << 8) | I2C_Buffer[j + 3];
                    sample[CHZ] = (I2C_Buffer[j + 4] << 8) | I2C_Buffer[j + 5];
                    conditionSample(sample);  // truncate negative values to -32767
                    addToFifo((union FifoSensor*) &(sfg->Gyro), GYRO_FIFO_SIZE, sample);
                }
            }
        }
    }   // end of optimized FXAS21002 FIFO read
*/
    return status;
        }


// This is the composite read function that handles both accel and mag and Gyro portions of the Combo_AMG
// It returns the first failing status flag
int8_t Combo_AMG_Read(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg) {
    int8_t  sts1 = 0;
    int8_t  sts2 = 0;
    int8_t  sts3 = 0;
    /*  OLD IMPLEMENTATION depending on FXOS7800
#if F_USING_ACCEL
        sts1 = FXOS8700_Accel_Read(sensor, sfg);
#endif

#if F_USING_MAG
        sts2 = FXOS8700_Mag_Read(sensor, sfg);
        sts3 = FXOS8700_Therm_Read(sensor, sfg);
#endif
*/
    return (sts1 + sts2 + sts3);
} // end Combo_AMG_Read()

// Combo_AMG_Idle places the entire sensor into STANDBY mode (wakeup time = 1/ODR+1ms)
// This driver is all-on or all-off. It does not support single subsensor only.
// If you want that functionality, you can write your own using the initialization
// function in this file as a starting template.  We've chosen not to cover all
// permutations in the interest of simplicity.
int8_t Combo_AMG_Idle(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg) {
    int32_t     status;
    if(sensor->isInitialized == (F_USING_ACCEL|F_USING_MAG)) {
//status = Sensor_I2C_Write_List(&sensor->deviceInfo, sensor->addr, FXOS8700_FULL_IDLE ); //OLD IMPLEMENTATION depending on FXOS7800
        sensor->isInitialized = 0;
#if F_USING_ACCEL
        sfg->Accel.isEnabled = false;
#endif
#if F_USING_MAG
        sfg->Mag.isEnabled = false;
#endif
    } else {
      return SENSOR_ERROR_INIT;
    }
    return status;
} // end Combo_AMG_Idle()


// IMPORT SNIPPET from   sensor_fusion/sensor_fusion.c

/// Poor man's inheritance for status subsystem setStatus command
/// This function is normally invoked via the "sfg." global pointer.
void setStatus(SensorFusionGlobals *sfg, fusion_status_t status)
{
    sfg->pStatusSubsystem->set(sfg->pStatusSubsystem, status);
}

/// Poor man's inheritance for status subsystem getStatus command
/// This function is normally invoked via the "sfg." global pointer.
fusion_status_t getStatus(SensorFusionGlobals *sfg)
{
  return sfg->pStatusSubsystem->get(sfg->pStatusSubsystem);
}

/// Poor man's inheritance for status subsystem queueStatus command.
/// This function is normally involved via the "sfg." global pointer.
void queueStatus(SensorFusionGlobals *sfg, fusion_status_t status)
{
    sfg->pStatusSubsystem->queue(sfg->pStatusSubsystem, status);
}

/// Poor man's inheritance for status subsystem updateStatus command.
/// This function is normally involved via the "sfg." global pointer.
void updateStatus(SensorFusionGlobals *sfg)
{
    sfg->pStatusSubsystem->update(sfg->pStatusSubsystem);
}

void testStatus(SensorFusionGlobals *sfg)
{
    sfg->pStatusSubsystem->test(sfg->pStatusSubsystem);
}

/// utility function to insert default values in the top level structure
void initSensorFusionGlobals(SensorFusionGlobals *sfg,
                             StatusSubsystem *pStatusSubsystem /*,
                             ControlSubsystem *pControlSubsystem*/)
{
    sfg->iFlags = // all of the following defines are either 0x0000 or a 1-bit value (2, 4, 8 ...) and are defined in build.h
                F_USING_ACCEL           |
                F_USING_MAG             |
                F_USING_GYRO            |
                F_USING_PRESSURE        |
                F_USING_TEMPERATURE     |
                F_ALL_SENSORS           |       // refers to all applicable sensor types for the given physical unit
                F_1DOF_P_BASIC	        |       // 1DOF pressure (altitude) and temperature: (pressure)
                F_3DOF_G_BASIC	        |	// 3DOF accel tilt: (accel)
                F_3DOF_B_BASIC	        |	// 3DOF mag eCompass (vehicle): (mag)
                F_3DOF_Y_BASIC	        |	// 3DOF gyro integration: (gyro)
                F_6DOF_GB_BASIC	        |	// 6DOF accel and mag eCompass)
                F_6DOF_GY_KALMAN        |	// 6DOF accel and gyro (Kalman): (accel + gyro)
                F_9DOF_GBY_KALMAN	;	// 9DOF accel, mag and gyro (Kalman): (accel + mag + gyro)

    //sfg->pControlSubsystem = pControlSubsystem;
    sfg->pStatusSubsystem = pStatusSubsystem;
    sfg->loopcounter = 0;                     // counter incrementing each iteration of sensor fusion (typically 25Hz)
    sfg->systick_I2C = 0;                     // systick counter to benchmark I2C reads
    sfg->systick_Spare = 0;                   // systick counter for counts spare waiting for timing interrupt
    sfg->iPerturbation = 0;                   // no perturbation to be applied
    sfg->installSensor = installSensor;       // function for installing a new sensor into the structures
    sfg->initializeFusionEngine = initializeFusionEngine;   // initializes fusion variables
    sfg->readSensors = readSensors;           // function for reading a sensor
    sfg->runFusion = runFusion;               // function for running fusion algorithms
    sfg->applyPerturbation = ApplyPerturbation; // function used for step function testing
    sfg->conditionSensorReadings = conditionSensorReadings; // function does averaging, HAL adjustments, etc.
    sfg->clearFIFOs = clearFIFOs;             // function to clear FIFO flags    sfg->applyPerturbation = ApplyPerturbation; // function used for step function testing
    sfg->setStatus = setStatus;               // function to immediately set status change
    sfg->getStatus = getStatus;               // function to report status
    sfg->queueStatus = queueStatus;           // function to queue status change
    sfg->updateStatus = updateStatus;         // function to promote queued status change
    sfg->testStatus = testStatus;             // function for unit testing the status subsystem
    sfg->pSensors = NULL;                     // pointer to linked list of physical sensors
//  put error value into whoAmI as initial value
#if F_USING_ACCEL
    sfg->Accel.iWhoAmI = 0;
#endif
#if F_USING_MAG
    sfg->Mag.iWhoAmI = 0;
#endif
#if F_USING_GYRO
    sfg->Gyro.iWhoAmI = 0;
#endif
#if F_USING_PRESSURE
    sfg->Pressure.iWhoAmI = 0;
#endif
} // end initSensorFusionGlobals()

/// installSensor is used to instantiate a physical sensor driver into the
/// sensor fusion system. It doesn't actually communicate with the sensor.
/// This function is normally invoked via the "sfg." global pointer.
int8_t installSensor(
                     SensorFusionGlobals *sfg,  ///< top level fusion structure
                     struct PhysicalSensor *pSensor,    ///< pointer to structure describing physical sensor
                     uint16_t addr,             ///< I2C address for sensor (if applicable)
                     uint16_t schedule,         ///< Sensor is read each time loop_count % schedule == 0
                     registerDeviceInfo_t *busInfo, ///< information required for bus power management
                     initializeSensor_t *initialize,    ///< pointer to sensor initialization function
                     readSensor_t *read)        ///< pointer to sensor read function
{
    if (sfg && pSensor && initialize && read)
    {
    /* was  pSensor->deviceInfo.deviceInstance = busInfo->deviceInstance;
        pSensor->deviceInfo.functionParam = busInfo->functionParam;
        pSensor->deviceInfo.idleFunction = busInfo->idleFunction;
        but these aren't used. Instead of changing structs everywhere, 
        we just set to zero. */
        pSensor->deviceInfo.deviceInstance = 0;
        pSensor->deviceInfo.functionParam = NULL;
        pSensor->deviceInfo.idleFunction = NULL;

        pSensor->initialize = initialize;       // The initialization function is responsible for putting the sensor
                                                // into the proper mode for sensor fusion.
        pSensor->read = read;                   // The read function is responsible for taking sensor readings and
                                                // loading them into the sensor fusion input structures.
        pSensor->addr = addr;                   // I2C address if applicable
        pSensor->schedule = schedule;
        // Now add the new sensor at the head of the linked list
        pSensor->next = sfg->pSensors;
        sfg->pSensors = pSensor;
        return (0);
    }
    else
    {
        return (1);
    }
} // end installSensor()

// The initializeSensors function traverses the linked list of physical sensor
// types and calls the initialization function for each one.
int8_t initializeSensors(SensorFusionGlobals *sfg)
{
    struct PhysicalSensor  *pSensor;
    int8_t          s;
    int8_t          status = 0;
    for (pSensor = sfg->pSensors; pSensor != NULL; pSensor = pSensor->next)
    {
        s = pSensor->initialize(pSensor, sfg);
        if (status == 0) status = s;            // will return 1st error flag, but try all sensors
    }
    return (status);
} // end initializeSensors()

// process<Sensor>Data routines do post processing for HAL and averaging.  They
// are called from the readSensors() function below.
#if F_USING_ACCEL
void processAccelData(SensorFusionGlobals *sfg)
{
    int32_t iSum[3];		        // channel sums
    int16_t i, j;			        // counters
    /*if (sfg->Accel.iFIFOExceeded > 0) {
      sfg->setStatus(sfg, SOFT_FAULT);
    }

    ApplyAccelHAL(&(sfg->Accel));     // This function is board-dependent

    // calculate the average HAL-corrected measurement
    for (j = CHX; j <= CHZ; j++) iSum[j] = 0;
    for (i = 0; i < sfg->Accel.iFIFOCount; i++)
        for (j = CHX; j <= CHZ; j++) iSum[j] += sfg->Accel.iGsFIFO[i][j];
    if (sfg->Accel.iFIFOCount > 0)
    {
        for (j = CHX; j <= CHZ; j++)
        {
            sfg->Accel.iGs[j] = (int16_t)(iSum[j] / (int32_t) sfg->Accel.iFIFOCount);
            sfg->Accel.fGs[j] = (float)sfg->Accel.iGs[j] * sfg->Accel.fgPerCount;
        }
    }

    // apply precision accelerometer calibration (offset V, inverse gain invW and rotation correction R^T)
    // to map fGs onto fGc (g), iGc (counts)
    fInvertAccelCal(&(sfg->Accel), &(sfg->AccelCal));

    // update the precision accelerometer data buffer
    fUpdateAccelBuffer(&(sfg->AccelCal),
                       &(sfg->AccelBuffer),
                       &(sfg->Accel),
                       &(sfg->pControlSubsystem->AccelCalPacketOn));
    */return;
} // end processAccelData()
#endif

#if F_USING_MAG
void processMagData(SensorFusionGlobals *sfg)
{
    int32_t iSum[3];		        // channel sums
    int16_t i, j;			        // counters
/*
    if (sfg->Mag.iFIFOExceeded > 0) {
      sfg->setStatus(sfg, SOFT_FAULT);
    }

    ApplyMagHAL(&(sfg->Mag));         // This function is board-dependent

    // calculate the average HAL-corrected measurement
    for (j = CHX; j <= CHZ; j++) iSum[j] = 0;
    for (i = 0; i < sfg->Mag.iFIFOCount; i++)
	for (j = CHX; j <= CHZ; j++) iSum[j] += sfg->Mag.iBsFIFO[i][j];
    if (sfg->Mag.iFIFOCount > 0)
    {
      for (j = CHX; j <= CHZ; j++)
      {
          sfg->Mag.iBs[j] = (int16_t)(iSum[j] / (int32_t) sfg->Mag.iFIFOCount);
          sfg->Mag.fBs[j] = (float)sfg->Mag.iBs[j] * sfg->Mag.fuTPerCount;
      }
    }

    // remove hard and soft iron terms from fBs (uT) to get calibrated data fBc (uT), iBc (counts) and
    // update magnetic buffer avoiding a write while a magnetic calibration is in progress.
    // run one iteration of the time sliced magnetic calibration
    fInvertMagCal(&(sfg->Mag), &(sfg->MagCal));
    if (!sfg->MagCal.iMagBufferReadOnly)
        iUpdateMagBuffer(&(sfg->MagBuffer), &(sfg->Mag), sfg->loopcounter);
    fRunMagCalibration(&(sfg->MagCal), &(sfg->MagBuffer), &(sfg->Mag),
                           sfg->loopcounter);
*/
    return;
} // end processMagData()
#endif

#if F_USING_GYRO
void processGyroData(SensorFusionGlobals *sfg)
{
    int32_t iSum[3];		        // channel sums
    int16_t i, j;			        // counters
/*
    if (sfg->Gyro.iFIFOExceeded > 0) {
      sfg->setStatus(sfg, SOFT_FAULT);
    }

    ApplyGyroHAL(&(sfg->Gyro));       // This function is board-dependent

    // calculate the average HAL-corrected measurement.  This is used for offset
    // initialization, display purposes and in the 3-axis gyro-only algorithm.
    // The Kalman filters both do the full incremental rotation integration
    // right in the filters themselves.
    for (j = CHX; j <= CHZ; j++) iSum[j] = 0;
    for (i = 0; i < sfg->Gyro.iFIFOCount; i++)
        for (j = CHX; j <= CHZ; j++)
          iSum[j] += sfg->Gyro.iYsFIFO[i][j];
    if (sfg->Gyro.iFIFOCount > 0)
    {
        for (j = CHX; j <= CHZ; j++)
        {
            sfg->Gyro.iYs[j] = (int16_t)(iSum[j] / (int32_t) sfg->Gyro.iFIFOCount);
            sfg->Gyro.fYs[j] = (float)sfg->Gyro.iYs[j] * sfg->Gyro.fDegPerSecPerCount;
        }
    }
*/
    return;
} // end processGyroData()
#endif

/// readSensors traverses the linked list of physical sensors, calling the
/// individual read functions one by one.
/// This function is normally invoked via the "sfg." global pointer.
/// If a sensor is flagged as uninitialized, an attempt is made to initialize it.
/// If a sensor does not respond, it is marked as unintialized.
int8_t readSensors(
    SensorFusionGlobals *sfg,   ///< pointer to global sensor fusion data structure
    uint8_t read_loop_counter  ///< current loop counter (used for multirate processing)
    ) 
{
    struct PhysicalSensor  *pSensor;
    int8_t          s;
    int8_t          status = SENSOR_ERROR_NONE;

    pSensor = sfg->pSensors;

    for (pSensor = sfg->pSensors; pSensor != NULL; pSensor = pSensor->next)
    {   if (pSensor->isInitialized) {
            if ( 0 == (read_loop_counter % pSensor->schedule)) {
                //read the sensor if it is its turn (per loop_counter)
                s = pSensor->read(pSensor, sfg);
                if(s != SENSOR_ERROR_NONE) {
                    //sensor reported error, so mark it uninitialized.
                    //If it becomes reinitialized next loop, init function will set flag back to sensor type
                    pSensor->isInitialized = F_USING_NONE; 
                }
                if (status == SENSOR_ERROR_NONE) status = s; // will return 1st error flag, but try all sensors
            }
        }else {
            //sensor not initialized. Make one attempt to init it.
            //If init succeeds, next time through a sensor read will be attempted
            s = pSensor->initialize(pSensor, sfg);
            if (s != SENSOR_ERROR_NONE) {
              //note that there is still an error
              status = s;
            }
        }
    }
    if (status == SENSOR_ERROR_NONE) {
        //change (or keep) status to NORMAL on next regular status update
        sfg->queueStatus(sfg, NORMAL);
    } else {
      // flag that we have problem reading sensor, which may clear later
      sfg->setStatus(sfg, SOFT_FAULT);
    }
    return (status);
} // end readSensors()

/// conditionSensorReadings() transforms raw software FIFO readings into forms that
/// can be consumed by the sensor fusion engine.  This include sample averaging
/// and (in the case of the gyro) integrations, applying hardware abstraction layers,
/// and calibration functions.
/// This function is normally invoked via the "sfg." global pointer.
void conditionSensorReadings(SensorFusionGlobals *sfg) {
#if F_USING_ACCEL
    if (sfg->Accel.isEnabled) processAccelData(sfg);
#endif

#if F_USING_MAG
    if (sfg->Mag.isEnabled) processMagData(sfg);
#endif

#if F_USING_GYRO
    if (sfg->Gyro.isEnabled) processGyroData(sfg);
#endif
    return;
} // end conditionSensorReadings()

void zeroArray(StatusSubsystem *pStatus, void* data, uint16_t size, uint16_t numElements, uint8_t check) {
  uint16_t i;
  uint8_t *d8;
  uint16_t *d16;
  uint32_t *d32;
  switch(size) {
  case 8:
    d8 = (uint8_t *) data;
    for (i=0; i<numElements; i++) d8[i]=0;
    break;
  case 16:
    d16 = (uint16_t *) data;
    for (i=0; i<numElements; i++) d16[i]=0;
    break;
  case 32:
    d32 = (uint32_t *) data;
    for (i=0; i<numElements; i++) d32[i]=0;
    break;
  default:
    pStatus->set(pStatus, HARD_FAULT);
  }
  if (check) {
    switch(size) {
    case 8:
      d8 = (uint8_t *) data;
      for (i=0; i<numElements; i++)
        if (d8[i]!=0) pStatus->set(pStatus, HARD_FAULT);
      break;
    case 16:
      d16 = (uint16_t *) data;
      for (i=0; i<numElements; i++)
        if (d16[i]!=0) pStatus->set(pStatus, HARD_FAULT);
      break;
    case 32:
      d32 = (uint32_t *) data;
      for (i=0; i<numElements; i++)
        if (d32[i]!=0) pStatus->set(pStatus, HARD_FAULT);
      break;
    }
    return;
  }
} // end zeroArray()

/// Function to clear FIFO at the end of each fusion computation
void clearFIFOs(SensorFusionGlobals *sfg) {
  // We only clear FIFOs if the sensors are enabled.  This allows us
  // to continue to use these values when we've shut higher power consumption
  // sensors down during periods of no activity.
/*
#if F_USING_ACCEL
    sfg->Accel.iFIFOCount=0;
    sfg->Accel.iFIFOExceeded = false;
#endif
#if F_USING_MAG
    sfg->Mag.iFIFOCount=0;
    sfg->Mag.iFIFOExceeded = false;
#endif
#if F_USING_GYRO
    sfg->Gyro.iFIFOCount=0;
    sfg->Gyro.iFIFOExceeded = false;
#endif
*/
} // end clearFIFOs()



// All sensor drivers and initialization functions have a similar prototype
// sensor = pointer to linked list element used by the sensor fusion subsystem to specify required sensors
// sfg = pointer to top level data structure for sensor fusion

int8_t Accel_Init(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg) {
    //Use the same init function for thermometer, magnetometer and accelererometer - it will
    //end up being called moree times, but that's OK
    //TODO - can move the accel stuff in here, and mag stuff following...
  return Combo_AMG_sensor_Init(sensor, sfg);
}

int8_t Mag_Init(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg) {
    //Use the same init function for thermometer, magnetometer and accelererometer - it will
    //end up being called more times, but that's OK
  return Combo_AMG_sensor_Init(sensor, sfg);
}

int8_t Gyro_Init(PhysicalSensor *sensor, SensorFusionGlobals *sfg) {
    //Use the same init function for thermometer, magnetometer and accelererometer - it will
    //end up being called more times, but that's OK
  return Combo_AMG_sensor_Init(sensor, sfg);
}

int8_t Therm_Init(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg) {
#ifdef EXTERN_BARO_THERM_SENSOR
  return Baro_Therm_Init(sensor,sfg);
#endif
#ifndef EXTERN_BARO_THERM_SENSOR  
    //Use the same init function for thermometer, magnetometer and accelererometer - it will
    //end up being called more times, but that's OK
  return Combo_AMG_sensor_Init(sensor, sfg);
#endif
}
#ifdef EXTERN_BARO_THERM_SENSOR
int8_t Baro_Init(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg) {
  return Baro_Therm_Init(sensor,sfg);
}
#endif
int8_t Combo_Accel_Mag_Init(PhysicalSensor *sensor, SensorFusionGlobals *sfg) {
        //Use the same init function for thermometer, magnetometer and accelererometer - it will
    //end up being called more times, but that's OK
  return Combo_AMG_sensor_Init(sensor, sfg);
}

int8_t Combo_Accel_Mag_Gyro_Init(PhysicalSensor *sensor, SensorFusionGlobals *sfg) {
        //Use the same init function for thermometer, magnetometer and accelererometer - it will
    //end up being called more times, but that's OK
  return Combo_AMG_sensor_Init(sensor, sfg);
}

int8_t Accel_Read(PhysicalSensor *sensor, SensorFusionGlobals *sfg) {
  return Combo_AMG_Read(sensor, sfg);
}

int8_t Mag_Read(PhysicalSensor *sensor, SensorFusionGlobals *sfg) {
  return Combo_AMG_Read(sensor, sfg);
}

int8_t Therm_Read(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg) {
#ifdef EXTERN_BARO_THERM_SENSOR
  return Baro_Therm_Read(sensor,sfg);
#endif
#ifndef EXTERN_BARO_THERM_SENSOR  
    //Use the same init function for thermometer, magnetometer and accelererometer - it will
    //end up being called more times, but that's OK
  return Combo_AMG_Read(sensor, sfg);
#endif
}
#ifdef EXTERN_BARO_THERM_SENSOR
int8_t Baro_Read(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg) {
  return Baro_Therm_Read(sensor,sfg);
}
#endif
int8_t Gyro_Read(PhysicalSensor *sensor, SensorFusionGlobals *sfg) {
  return Combo_AMG_Read(sensor, sfg);
}

int8_t Combo_Accel_Mag_Read(PhysicalSensor *sensor, SensorFusionGlobals *sfg) {
  return Combo_AMG_Read(sensor, sfg);
}

int8_t Combo_Accel_Mag_Gyro_Read(PhysicalSensor *sensor, SensorFusionGlobals *sfg) {
  return Combo_AMG_Read(sensor, sfg);
}

int8_t Gyro_Idle(PhysicalSensor *sensor, SensorFusionGlobals *sfg) {
    return Combo_AMG_Idle (sensor,sfg);
}

int8_t Combo_Accel_Mag_Idle(PhysicalSensor *sensor, SensorFusionGlobals *sfg) {
    return Combo_AMG_Idle (sensor,sfg);
}

int8_t Combo_Accel_Mag_Gyro_Idle(PhysicalSensor *sensor, SensorFusionGlobals *sfg) {
    return Combo_AMG_Idle (sensor,sfg);
}


/// runFusion the top level call that actually runs the sensor fusion.
/// This is a utility function which manages the various defines in build.h.
/// You should feel free to drop down a level and implement only those portions
/// of fFuseSensors() that your application needs.
/// This function is normally involved via the "sfg." global pointer.
void runFusion(SensorFusionGlobals *sfg)
{
    struct SV_1DOF_P_BASIC *pSV_1DOF_P_BASIC;
    struct SV_3DOF_G_BASIC *pSV_3DOF_G_BASIC;
    struct SV_3DOF_B_BASIC *pSV_3DOF_B_BASIC;
    struct SV_3DOF_Y_BASIC *pSV_3DOF_Y_BASIC;
    struct SV_6DOF_GB_BASIC *pSV_6DOF_GB_BASIC;
    struct SV_6DOF_GY_KALMAN *pSV_6DOF_GY_KALMAN;
    struct SV_9DOF_GBY_KALMAN *pSV_9DOF_GBY_KALMAN;
    struct AccelSensor *pAccel;
    struct MagSensor *pMag;
    struct GyroSensor *pGyro;
    struct PressureSensor *pPressure;
    struct MagCalibration *pMagCal;
#if F_1DOF_P_BASIC
    pSV_1DOF_P_BASIC = &(sfg->SV_1DOF_P_BASIC);
#else
    pSV_1DOF_P_BASIC = NULL;
#endif
#if F_3DOF_G_BASIC
    pSV_3DOF_G_BASIC = &(sfg->SV_3DOF_G_BASIC)  ;
#else
    pSV_3DOF_G_BASIC = NULL;
#endif
#if F_3DOF_B_BASIC
    pSV_3DOF_B_BASIC = &(sfg->SV_3DOF_B_BASIC);
#else
    pSV_3DOF_B_BASIC = NULL;
#endif
#if F_3DOF_Y_BASIC
    pSV_3DOF_Y_BASIC = &(sfg->SV_3DOF_Y_BASIC);
#else
    pSV_3DOF_Y_BASIC = NULL;
#endif
#if F_6DOF_GB_BASIC
    pSV_6DOF_GB_BASIC = &(sfg->SV_6DOF_GB_BASIC);
#else
    pSV_6DOF_GB_BASIC = NULL;
#endif
#if F_6DOF_GY_KALMAN
    pSV_6DOF_GY_KALMAN = &(sfg->SV_6DOF_GY_KALMAN);
#else
    pSV_6DOF_GY_KALMAN = NULL;
#endif
#if F_9DOF_GBY_KALMAN
    pSV_9DOF_GBY_KALMAN = &(sfg->SV_9DOF_GBY_KALMAN);
#else
    pSV_9DOF_GBY_KALMAN = NULL;
#endif
#if F_USING_ACCEL
    pAccel =  &(sfg->Accel);
#else
    pAccel = NULL;
#endif
#if F_USING_MAG
    pMag = &(sfg->Mag);
    pMagCal = &(sfg->MagCal);
#else
    pMag = NULL;
    pMagCal = NULL;
#endif
#if F_USING_GYRO
    pGyro = &(sfg->Gyro);
#else
    pGyro = NULL;
#endif
#if F_USING_PRESSURE
    pPressure = &(sfg->Pressure);
#else
    pPressure = NULL;
#endif

    // conditionSensorReadings(sfg);  must be called prior to this function
    // fuse the sensor data
/*
    fFuseSensors(pSV_1DOF_P_BASIC, pSV_3DOF_G_BASIC,
                 pSV_3DOF_B_BASIC, pSV_3DOF_Y_BASIC,
                 pSV_6DOF_GB_BASIC, pSV_6DOF_GY_KALMAN,
                 pSV_9DOF_GBY_KALMAN, pAccel, pMag, pGyro,
                 pPressure, pMagCal);
*/                 
    clearFIFOs (sfg);
} // end runFusion()

/// This function is responsible for initializing the system prior to starting
/// the main fusion loop. I2C is initted, sensors configured, calibrations loaded.
/// This function is normally invoked via the "sfg." global pointer.
/// Fusion system status is set to:
///   INITIALIZING at the start of this function,
///   HARD_FAULT if a problem occurs initializing the I2C hardware,
///   SOFT_FAULT if a sensor doesn't initialize (it could be corrected later),
///   NORMAL when function ends, assuming no problem occurred
void initializeFusionEngine(SensorFusionGlobals *sfg, int pin_i2c_sda, int pin_i2c_scl)
{
    int16_t status = SENSOR_ERROR_NONE;
    //struct ControlSubsystem    *pComm;
    //pComm = sfg->pControlSubsystem;

    sfg->setStatus(sfg, INITIALIZING);
    if( ! I2CInitialize(pin_i2c_sda, pin_i2c_scl) ) {
        sfg->setStatus(sfg, HARD_FAULT);  // Never returns
    }
    status = initializeSensors(sfg);
    if (status!=SENSOR_ERROR_NONE) {  // fault condition found - will try again later
        sfg->setStatus(sfg, SOFT_FAULT);
    }
    /* communication structure no more supported because sustituted by SignalK comm
    // recall: typedef enum quaternion {Q3, Q3M, Q3G, Q6MA, Q6AG, Q9} quaternion_type;
    // Set the default quaternion to the most sophisticated supported by this build
    pComm->DefaultQuaternionPacketType = Q3;
    if (sfg->iFlags & F_3DOF_B_BASIC) pComm->DefaultQuaternionPacketType = Q3M;
    if (sfg->iFlags & F_3DOF_Y_BASIC) pComm->DefaultQuaternionPacketType = Q3G;
    if (sfg->iFlags & F_6DOF_GB_BASIC) pComm->DefaultQuaternionPacketType = Q6MA;
    if (sfg->iFlags & F_6DOF_GY_KALMAN) pComm->DefaultQuaternionPacketType = Q6AG;
    if (sfg->iFlags & F_9DOF_GBY_KALMAN) pComm->DefaultQuaternionPacketType = Q9;
    pComm->QuaternionPacketType = pComm->DefaultQuaternionPacketType ;
    */
    // initialize the sensor fusion algorithms
    fInitializeFusion(sfg);

    // reset the loop counter to zero for first iteration
    sfg->loopcounter = 0;

    // initialize the magnetic calibration and magnetometer data buffer
#if F_USING_MAG
    fInitializeMagCalibration(sfg);
#endif
}
//END OF IMPORT from sensor_fusion/sensor_fusion.c


/* OLD IMPLEMENTATION FX8700 dependent
int8_t Combo_AMG_sensor_Init(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg) {
    int32_t status;
    uint8_t reg;

    status = Sensor_I2C_Read_Register(&sensor->deviceInfo, sensor->addr, FXOS8700_WHO_AM_I, 1, &reg);

    if (status==SENSOR_ERROR_NONE) {
#if F_USING_ACCEL
       sfg->Accel.iWhoAmI = reg;
       sfg->Accel.iCountsPerg = FXOS8700_COUNTSPERG;
       sfg->Accel.fgPerCount = 1.0F / FXOS8700_COUNTSPERG;
#endif
#if F_USING_MAG
       sfg->Mag.iWhoAmI = reg;
       sfg->Mag.iCountsPeruT = FXOS8700_COUNTSPERUT;
       sfg->Mag.fCountsPeruT = (float) FXOS8700_COUNTSPERUT;
       sfg->Mag.fuTPerCount = 1.0F / FXOS8700_COUNTSPERUT;
#endif
       if (reg != FXOS8700_WHO_AM_I_PROD_VALUE) {
          return SENSOR_ERROR_INIT;  // The whoAmI did not match
       }
    } else {
        // whoAmI will retain default value of zero
        // return with error
        return status;
    }

    // Configure and start the fxos8700 sensor.  This does multiple register writes
    // (see FXOS8700_Initialization definition above)
    status = Sensor_I2C_Write_List(&sensor->deviceInfo, sensor->addr, FXOS8700_Initialization );
    sensor->isInitialized = F_USING_ACCEL | F_USING_MAG;
#if F_USING_ACCEL
    sfg->Accel.isEnabled = true;
#endif
#if F_USING_MAG
    sfg->Mag.isEnabled = true;
#endif

    return (status);
} // end FXOS8700_Init() */

/* OLD IMPLEMENTATION FX8700 dependent
#if F_USING_MAG
// read FXOS8700 magnetometer over I2C
int8_t FXOS8700_Mag_Read(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg) {
    uint8_t                     I2C_Buffer[6];  // I2C read buffer
    int32_t                     status;         // I2C transaction status
    int16_t                     sample[3];

    if(!(sensor->isInitialized & F_USING_MAG))
    {
        return SENSOR_ERROR_INIT;
    }

    // read the six sequential magnetometer output bytes
    FXOS8700_DATA_READ[0].readFrom = FXOS8700_M_OUT_X_MSB;
    FXOS8700_DATA_READ[0].numBytes = 6;
    status =  Sensor_I2C_Read(&sensor->deviceInfo, sensor->addr, FXOS8700_DATA_READ, I2C_Buffer );
    if (status==SENSOR_ERROR_NONE) {
        // place the 6 bytes read into the magnetometer structure
        sample[CHX] = (I2C_Buffer[0] << 8) | I2C_Buffer[1];
        sample[CHY] = (I2C_Buffer[2] << 8) | I2C_Buffer[3];
        sample[CHZ] = (I2C_Buffer[4] << 8) | I2C_Buffer[5];
        conditionSample(sample);  // truncate negative values to -32767
        addToFifo((union FifoSensor*) &(sfg->Mag), MAG_FIFO_SIZE, sample);
    }
    return status;
}//end FXOS8700_ReadMagData()
#endif  //F_USING_MAG
*/
/* OLD IMPLEMENTATION FX8700 dependent
// read temperature register over I2C
int8_t Therm_Read(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg) {
    int8_t                      I2C_Buffer;     // I2C read buffer
    int32_t                     status;         // I2C transaction status

    if(!(sensor->isInitialized)) {
        return SENSOR_ERROR_INIT;
    }

    // read the Temperature register 0x51
    FXOS8700_DATA_READ[0].readFrom = FXOS8700_TEMP;
    FXOS8700_DATA_READ[0].numBytes = 1;
    status =  Sensor_I2C_Read(&sensor->deviceInfo, sensor->addr, FXOS8700_DATA_READ, (uint8_t*)(&I2C_Buffer) );
    if (status==SENSOR_ERROR_NONE) {
        // convert the byte to temperature and place in sfg structure
        sfg->Temp.temperatureC = (float)I2C_Buffer * 0.96; //section 14.3 of manual says 0.96 degC/LSB
    }
    return status;
}//end FXOS8700_Therm_Read()
*/
/* OLD IMPLEMENTATION FX8700 dependent
// This is the composite read function that handles both accel and mag portions of the FXOS8700
// It returns the first failing status flag
int8_t FXOS8700_Read(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg) {
    int8_t  sts1 = 0;
    int8_t  sts2 = 0;
    int8_t  sts3 = 0;
#if F_USING_ACCEL
        sts1 = FXOS8700_Accel_Read(sensor, sfg);
#endif

#if F_USING_MAG
        sts2 = FXOS8700_Mag_Read(sensor, sfg);
        sts3 = FXOS8700_Therm_Read(sensor, sfg);
#endif

    return (sts1 + sts2 + sts3);
} // end FXOS8700_Read()
*/

/* OLD IMPLEMENTATION FX8700 dependent
// FXOS8700_Idle places the entire sensor into STANDBY mode (wakeup time = 1/ODR+1ms)
// This driver is all-on or all-off. It does not support mag or accel only.
// If you want that functionality, you can write your own using the initialization
// function in this file as a starting template.  We've chosen not to cover all
// permutations in the interest of simplicity.
int8_t FXOS8700_Idle(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg) {
    int32_t     status;
    if(sensor->isInitialized == (F_USING_ACCEL|F_USING_MAG)) {
        status = Sensor_I2C_Write_List(&sensor->deviceInfo, sensor->addr, FXOS8700_FULL_IDLE );
        sensor->isInitialized = 0;
#if F_USING_ACCEL
        sfg->Accel.isEnabled = false;
#endif
#if F_USING_MAG
        sfg->Mag.isEnabled = false;
#endif
    } else {
      return SENSOR_ERROR_INIT;
    }
    return status;
} // end FXOS8700_Idle()
*/
