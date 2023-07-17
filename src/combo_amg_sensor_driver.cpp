/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright (c) 2016-2017 NXP
 * Copyright (c) 2020 Bjarne Hansen
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! \file combo_amg_sensor_driver.cpp
    \brief Defines commands to perform various tasks (e.g. read Device ID, Initialize, Read Data) 
    for the FXOS8700 6-axis accelerometer plus magnetometer. Actual I2C interface functions are
    found in sensor_io_i2c files.
*/

#include "build.h"
#include <Arduino.h>
#include "combo_amg_sensor_driver.h"      // Generic ComboAMG_Sensor hardware interface
#include "driver_sensors.h"             // prototypes for *_Init() and *_Read() methods
#include "driver_sensors_types.h"
#include "sensor_fusion.h"
#include "sensor_fusion_class.h"
#include "Wire.h"
#ifdef F_USING_SMARTSENSOR
#include "Adafruit_BNO055.h"
#endif
#undef EXTERN_BARO_THERM_SENSOR
class OR_sensor_: public Adafruit_BNO055 {
  public:  
    OR_sensor_(int32_t sensorID, uint8_t address,
                           TwoWire* theWire) : Adafruit_BNO055(sensorID, address,
                                                               theWire) {}
    /*!
     *  @brief  Checks of all cal status values are set to 3 (fully calibrated)
     *  @return status of calibration
     */
    bool isFullyCalibrated()
    {
        uint8_t system, gyro, accel, mag;
        adafruit_bno055_opmode_t _mode = OR_sensor_::getMode();
        OR_sensor_::getCalibration(&system, &gyro, &accel, &mag);
        switch (_mode)
        {
        case OPERATION_MODE_ACCONLY:
            return (accel == 3);
        case OPERATION_MODE_MAGONLY:
            return (mag == 3);
        case OPERATION_MODE_GYRONLY:
        case OPERATION_MODE_M4G: /* No magnetometer calibration required. */
            return (gyro == 3);
        case OPERATION_MODE_ACCMAG:
        case OPERATION_MODE_COMPASS:
            return (accel == 3 && mag == 3);
        case OPERATION_MODE_ACCGYRO:
        case OPERATION_MODE_IMUPLUS:
            return (accel == 3 && gyro == 3);
        case OPERATION_MODE_MAGGYRO:
            return (mag == 3 && gyro == 3);
        default:
            // return (system == 3 && gyro == 3 && accel == 3 && mag == 3); // Implementation in the parent class requiring full calibration
            //  of all sensor, even for the Accelerometer that's long to achieve
            return (system == 3 && gyro == 3 && accel >= 1 && mag == 3); // Accel full calibration may be difficult to achieve
                                                                         // the "FullyCalibrated" bool true is required
                                                                         // by the library to read/write calibration(offset) regs
        }
    }
};

#define COMBO_I2C_ADDR (BNO055_ADDRESS_B) //due to particular breakout the default I2C address is this instead of chip default
#define COMBO_WHO_AM_I (BNO055_ID)
#define MIN_BUF_LEN 22 // 6 triple values plus 4 values for quaternion = 22 int16_t = 44 bytes - for data read from chip

// Each entry in a RegisterWriteList is composed of: register address, value to write, bit-mask to apply to write (0 enables)
const registerwritelist_t AMG_FULL_IDLE[] =
    {
        // Set ACTIVE = other bits unchanged
        {COMBO_I2C_ADDR, 0x00, 0x01}, // TO BE VERIFIED
        __END_WRITE_DATA__};

// utility typedefs for data area (to not be smashed by heap or stack limits)
typedef uint16_t word_readbuffer[MIN_BUF_LEN];
typedef uint8_t byte_readbuffer[MIN_BUF_LEN/2];
typedef struct vec_t {
                        float accel_x;
                        float accel_y;
                        float accel_z;
                        float mag_x;
                        float mag_y;
                        float mag_z;
                        float gyro_x;
                        float gyro_y;
                        float gyro_z;
                        float euler_yaw;
                        float euler_pitch;
                        float euler_roll;
                        float lia_x;
                        float lia_y;
                        float lia_z;
                        float gravity_x;
                        float gravity_y;
                        float gravity_z;
                        double q0;
                        double q1;
                        double q2;
                        double q3;
                    } vec_t;
typedef union vectors_t {
                        word_readbuffer w;
                        byte_readbuffer b;
                        vec_t           v;

} vectors_t;

// Global Data Area
vectors_t vectors;
adafruit_bno055_offsets_t offsets;
imu::Vector<3> xyz = {0};           //where data are read from IMU
imu::Quaternion wxyz(1, 0, 0, 0);   //
OR_sensor_::adafruit_vector_type_t vec_type[6] = {OR_sensor_::VECTOR_ACCELEROMETER, // Raw but calibrated
                                                  OR_sensor_::VECTOR_MAGNETOMETER,  // Raw but calibrated  
                                                  OR_sensor_::VECTOR_GYROSCOPE,     // Raw but calibrated
                                                  OR_sensor_::VECTOR_EULER,         // from Fusion
                                                  OR_sensor_::VECTOR_LINEARACCEL,   // from Fusion
                                                  OR_sensor_::VECTOR_GRAVITY};      // from Fusion
uint8_t  Buffer [sizeof(int16_t) * MIN_BUF_LEN];                                               
uint8_t* pBuffer = &Buffer[0];
registerReadlist_t ReadList;
auto* pReadList = &ReadList;
registerwritelist_t WriteList;
auto* pWriteList = &WriteList;
calib_index_t Calib;
auto* calib = &Calib;

// utility functions

#ifdef F_USING_SMARTSENSOR
void inline printvector (imu::Vector<3> xyz, OR_sensor_::adafruit_vector_type_t vec_type)
{
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  switch (vec_type){
    case OR_sensor_::VECTOR_ACCELEROMETER:
     Serial.print("Accl:");
     break;
    case OR_sensor_::VECTOR_MAGNETOMETER:
     Serial.print("Mag:");
     break;
    case OR_sensor_::VECTOR_GYROSCOPE:
     Serial.print("Gyro:");
     break;
     /*
    case SENSOR_TYPE_ROTATION_VECTOR:
     Serial.print("Rot:");
     break;
     */
    case OR_sensor_::VECTOR_EULER:
     Serial.print("Orient:");
     break;
    case OR_sensor_::VECTOR_LINEARACCEL:
     Serial.print("Linear:");
     break;
    case OR_sensor_::VECTOR_GRAVITY:
     Serial.print("Gravity:");
     break;
    default:
     Serial.print("Unk:");
  }
  Serial.print("\tx= ");
  Serial.print(xyz[0]);
  Serial.print(" |\ty= ");
  Serial.print(xyz[1]);
  Serial.print(" |\tz= ");
  Serial.println(xyz[2]);
}
#endif


/* ABSTRACT BUS LEVEL FUNCTIONs */
bool I2CInitialize(int pin_i2c_sda, int pin_i2c_scl) {
    return Wire.setPins (pin_i2c_sda,pin_i2c_scl);
}

int32_t Sensor_I2C_Read(registerDeviceInfo_t *devinfo, uint16_t addr,registerReadlist_t read_reg, uint8_t* I2C_Buffer
                            ){
     return 0; //placeholder unused because using BNO055 higher level abstraction interface
}
/* END OF BUS LEVEL FUNCTIONS*/

/* LOWER LEVEL FUNCTIONS TO INTERFACE GIVEN SENSOR DRIVER LIBRARY*/
int32_t Combo_AMG_I2C_Initialize(   i2c_sensorhandle_t *pSensorHandle, uint8_t index, uint16_t sAddress,
                                    uint8_t whoAmi){
    uint8_t sstatus, error, test_result;
    sstatus = error = test_result = 0x0F; // set !=0 the ouput var you want, 0 what you don't
    or_sensor_se_t or_sensor_error;
    OR_sensor_* Combo_AMG_sensor;
    if (! pSensorHandle->isInitialized) {                               // do nothing when already initialized
                                                                        // in case of second call
            Combo_AMG_sensor = new OR_sensor_(-1, index, &Wire);

        // Ther's no need to test if WHO_AM_I is correct because it is tested inside Begin function
        if (!Combo_AMG_sensor->begin(OPERATION_MODE_CONFIG)) {                 // default mode
                                                                               // prior of setting in OPERATION_MODE_NDOF
                                                                               // SensorFusion should initialize
                                                                               // offsets and axis remap recovered from SPIFFS
                                                                               // (if any). setMode(OPERATION_MODE_NDOF) will
                                                                               // be the last issued command at the end of 
                                                                               // SensorFusion initialization
            Combo_AMG_sensor->getSystemStatus(&sstatus, &test_result, &error);
            or_sensor_error = (or_sensor_se_t) error;
            if ( sstatus == 1 /* SYSTEM ERROR */ && error != No_error) {
                  pSensorHandle->isInitialized = false;
                  //delete pSensorHandle->deviceInfo.functionParam;
                  pSensorHandle->deviceInfo.functionParam = NULL;
                  delete Combo_AMG_sensor;
                  Serial.println("Combo_AMG_I2C_Initialize -- PANIC! - SYSTEM ERROR");
                  Serial.print("Sensor SelfTest:\t\t"); Serial.println(test_result, HEX);
                  Serial.print("Sensor SysStatus:\t\t"); Serial.println(sstatus,DEC);
                  Serial.print("Sensor init error:\t\t"); Serial.println (error,DEC);
                  while(true) {};
            }
            delete Combo_AMG_sensor;
            return (int32_t) ((sstatus <<8) || error);
        } else 
        {   
            Combo_AMG_sensor->getSystemStatus(&sstatus, &test_result, &error);
            if (sstatus != 1 /* there isn't SYSTEM ERROR - Tested error before when sstatus ==1 (Sys Error is on)*/ ) {
            pSensorHandle->isInitialized = true;
            pSensorHandle->deviceInfo.functionParam = Combo_AMG_sensor; // Use spare field 'functionParam' to store
                                                                        // pointer to OR_sensor object
            return SENSOR_ERROR_NONE;
            }
            //TODO : what if perfect condition (fusion running and no_error) is not reached?
            // delay and retry? throw an error (what?) on return?
            Serial.println("Combo_AMG_I2C_Initialize -- PANIC! - SYSTEM ERROR");
            Serial.print("Sensor SelfTest:\t\t"); Serial.println(test_result, HEX);
            Serial.print("Sensor SysStatus:\t\t"); Serial.println(sstatus,DEC);
            Serial.print("Sensor init error:\t\t"); Serial.println (error,DEC);
            while(true) {};
             
        }
    }
    return SENSOR_ERROR_NONE;
}

void Combo_AMG_I2C_SetIdleTask(i2c_sensorhandle_t *pSensorHandle, registeridlefunction_t idleTask,
                              void *userParam){
    
//placeholder
}

int32_t Combo_AMG_I2C_Configure(i2c_sensorhandle_t *pSensorHandle, const registerwritelist_t *pRegWriteList){
    #ifndef F_USING_SMARTSENSOR
    #endif
    #ifdef F_USING_SMARTSENSOR
    // Since the SMARTSENSOR is not directly interfaced via I2C bus
    // but via an higher level sensor library the second function parameter
    // is used to pass the position parameter of physical_sensor_position_t 
    // as "value" datum of the registerwritelist_t *.
    if (pSensorHandle->isInitialized){
        physical_sensor_position_t pos_ = (physical_sensor_position_t) pRegWriteList->value;
        
        OR_sensor_ *Combo_AMG_sensor = (OR_sensor_ *)pSensorHandle->deviceInfo.functionParam; // recover reference to OR_sensor object
        
        OR_sensor_::adafruit_bno055_axis_remap_config_t remap_; 
        OR_sensor_::adafruit_bno055_axis_remap_sign_t sign_;
        switch (pos_)
        {
        case P0 :
            remap_  = OR_sensor_::REMAP_CONFIG_P0;
            sign_   = OR_sensor_::REMAP_SIGN_P0;
            break;
        case P1 :
            remap_  = OR_sensor_::REMAP_CONFIG_P1;
            sign_   = OR_sensor_::REMAP_SIGN_P1;
            break;
        case P2 :
            remap_  = OR_sensor_::REMAP_CONFIG_P2;
            sign_   = OR_sensor_::REMAP_SIGN_P2;
            break;
        case P3 :
            remap_  = OR_sensor_::REMAP_CONFIG_P3;
            sign_   = OR_sensor_::REMAP_SIGN_P3;
        case P4 :
            remap_  = OR_sensor_::REMAP_CONFIG_P4;
            sign_   = OR_sensor_::REMAP_SIGN_P4;
            break;
        case P5:
            remap_  = OR_sensor_::REMAP_CONFIG_P5;
            sign_   = OR_sensor_::REMAP_SIGN_P5;
            break;
        case P6 :
            remap_  = OR_sensor_::REMAP_CONFIG_P6;
            sign_   = OR_sensor_::REMAP_SIGN_P6;
            break;
        case P7 :
            remap_  = OR_sensor_::REMAP_CONFIG_P7;
            sign_   = OR_sensor_::REMAP_SIGN_P7;
            break;
        default:
            remap_  = OR_sensor_::REMAP_CONFIG_P6;
            sign_   = OR_sensor_::REMAP_SIGN_P6;
            break;
        }
        Combo_AMG_sensor->setAxisRemap(remap_);
        Combo_AMG_sensor->setAxisSign(sign_);

        uint8_t sstatus, test_result, error;
        sstatus = test_result = error = 0xFF;
        Combo_AMG_sensor->getSystemStatus(&sstatus, &test_result, &error);
        if (sstatus == 1) {
            Serial.println("Combo_AMG_I2C_Configure --  sstatus = 1");
            Serial.print("Position requested:\t\t"); Serial.println(pos_,DEC);
            Serial.print("Sensor SelfTest:\t\t"); Serial.println(test_result, HEX);
            Serial.print("Sensor SysStatus:\t\t"); Serial.println(sstatus,DEC);
            Serial.print("Sensor init error:\t\t"); Serial.println (error,DEC);
            delay (1000);
        }
        return SENSOR_ERROR_NONE; 
    }  else   return SENSOR_ERROR_INIT;

    #endif
    return 0; //placeholder just to avoid compiler warnings
}

int32_t Combo_AMG_I2C_GetCalibration(i2c_sensorhandle_t *pSensorHandle, calib_index_t *pCalib_index){
    if (pSensorHandle->isInitialized){
        OR_sensor_ *Combo_AMG_sensor = (OR_sensor_ *)pSensorHandle->deviceInfo.functionParam; // recover reference to OR_sensor object
                                                                                              // from initialized sensor handle
        Combo_AMG_sensor->getCalibration( &pCalib_index->IMU_Calib_index,
                                          &pCalib_index->Gyro_Calib_index,
                                          &pCalib_index->Accel_Calib_index,
                                          &pCalib_index->Mag_Calib_index
        );
        return SENSOR_ERROR_NONE; 
    }    
    return SENSOR_ERROR_INIT;
}

int32_t Combo_AMG_I2C_GetCalibrationData(i2c_sensorhandle_t *pSensorHandle, uint8_t *pCalib_data){
    if (pSensorHandle->isInitialized){
        OR_sensor_ *Combo_AMG_sensor = (OR_sensor_ *)pSensorHandle->deviceInfo.functionParam; // recover reference to OR_sensor object
                                                                                              // from initialized sensor handle
        if (!Combo_AMG_sensor->getSensorOffsets(pCalib_data))
            return SENSOR_ERROR_READ;

        return SENSOR_ERROR_NONE; 
    }    
    return SENSOR_ERROR_INIT;
}

int32_t Combo_AMG_I2C_SetCalibrationData(i2c_sensorhandle_t *pSensorHandle, uint8_t *pCalib_data){
    
    offsets.accel_offset_x = uint16_t ((pCalib_data[1]) <<8) | uint16_t (pCalib_data[0]);
    offsets.accel_offset_y = uint16_t ((pCalib_data[3]) <<8) | uint16_t (pCalib_data[2]);
    offsets.accel_offset_z = uint16_t ((pCalib_data[5]) <<8) | uint16_t (pCalib_data[4]);

    offsets.mag_offset_x   = uint16_t ((pCalib_data[7]) <<8) | uint16_t (pCalib_data[6]);
    offsets.mag_offset_y   = uint16_t ((pCalib_data[9]) <<8) | uint16_t (pCalib_data[8]);
    offsets.mag_offset_z   = uint16_t ((pCalib_data[11]) <<8) | uint16_t (pCalib_data[10]);

    offsets.gyro_offset_x   = uint16_t ((pCalib_data[13]) <<8) | uint16_t (pCalib_data[12]);
    offsets.gyro_offset_y   = uint16_t ((pCalib_data[15]) <<8) | uint16_t (pCalib_data[14]);
    offsets.gyro_offset_z   = uint16_t ((pCalib_data[17]) <<8) | uint16_t (pCalib_data[16]);

    offsets.accel_radius    = uint16_t ((pCalib_data[19]) <<8) | uint16_t (pCalib_data[18]);
    offsets.mag_radius      = uint16_t ((pCalib_data[21]) <<8) | uint16_t (pCalib_data[20]);

    if (pSensorHandle->isInitialized){
        OR_sensor_ *Combo_AMG_sensor = (OR_sensor_ *)pSensorHandle->deviceInfo.functionParam; // recover reference to OR_sensor object
                                                                                              // from initialized sensor handle
        Combo_AMG_sensor->setSensorOffsets(offsets);  
        return SENSOR_ERROR_NONE; 
    }    
    return SENSOR_ERROR_INIT;
}

#ifdef F_USING_SMARTSENSOR
int32_t Combo_AMG_I2C_SetModeFusion(i2c_sensorhandle_t *pSensorHandle){
    if (pSensorHandle->isInitialized){
        OR_sensor_ *Combo_AMG_sensor = (OR_sensor_ *)pSensorHandle->deviceInfo.functionParam; // recover reference to OR_sensor object
                                                                                              // from initialized sensor handle
        Combo_AMG_sensor->setMode(OPERATION_MODE_NDOF);
        return SENSOR_ERROR_NONE;
    }    
    return SENSOR_ERROR_INIT;
}
#endif
#ifdef F_USING_SMARTSENSOR 
int32_t Combo_AMG_I2C_GetModeFusion(i2c_sensorhandle_t *pSensorHandle, uint8_t* pmode){
    if (pSensorHandle->isInitialized){
        OR_sensor_ *Combo_AMG_sensor = (OR_sensor_ *)pSensorHandle->deviceInfo.functionParam; // recover reference to OR_sensor object                                                                                     // from initialized sensor handle
        *pmode = Combo_AMG_sensor->getMode();
        return SENSOR_ERROR_NONE;
    }    
    return SENSOR_ERROR_INIT;
}
#endif

int32_t Combo_AMG_I2C_ReadData(i2c_sensorhandle_t *pSensorHandle, const registerReadlist_t *pReadList,
                              uint8_t *pBuffer)
{
        if (pSensorHandle->isInitialized)
        { 
            OR_sensor_ *Combo_AMG_sensor;
            Combo_AMG_sensor = (OR_sensor_ *)pSensorHandle->deviceInfo.functionParam;      // recover reference to OR_sensor_ object
                                                                                           // from initialized sensor handle 
                    xyz = Combo_AMG_sensor->getVector(OR_sensor_::VECTOR_ACCELEROMETER);
                    vectors.v.accel_x = xyz[0];
                    vectors.v.accel_y = xyz[1];
                    vectors.v.accel_z = xyz[2];
                    xyz = Combo_AMG_sensor->getVector(OR_sensor_::VECTOR_MAGNETOMETER);
                    vectors.v.mag_x = xyz[0];
                    vectors.v.mag_y = xyz[1];
                    vectors.v.mag_z = xyz[2];
                    xyz = Combo_AMG_sensor->getVector(OR_sensor_::VECTOR_GYROSCOPE);
                    vectors.v.gyro_x = xyz[0];
                    vectors.v.gyro_y = xyz[1];
                    vectors.v.gyro_z = xyz[2];
                    xyz = Combo_AMG_sensor->getVector(OR_sensor_::VECTOR_EULER);
                    vectors.v.euler_yaw = xyz[0];
                    vectors.v.euler_roll = xyz[1];
                    vectors.v.euler_pitch = xyz[2];
                    /*
                    printvector(xyz,OR_sensor_::VECTOR_EULER);
                    delay(1000);
                    */
                    xyz = Combo_AMG_sensor->getVector(OR_sensor_::VECTOR_LINEARACCEL);
                    vectors.v.lia_x = xyz[0];
                    vectors.v.lia_y = xyz[1];
                    vectors.v.lia_z = xyz[2];
                    xyz = Combo_AMG_sensor->getVector(OR_sensor_::VECTOR_GRAVITY);
                    vectors.v.gravity_x = xyz[0];
                    vectors.v.gravity_y = xyz[1];
                    vectors.v.gravity_z = xyz[2];
                    wxyz = Combo_AMG_sensor->getQuat();
                    vectors.v.q0 = wxyz.w();
                    vectors.v.q1 = wxyz.x();
                    vectors.v.q2 = wxyz.y();
                    vectors.v.q3 = wxyz.z();
                    uint8_t mode;
                    Combo_AMG_I2C_GetModeFusion(pSensorHandle, &mode);
                    if (mode != OPERATION_MODE_NDOF) {pWriteList->value = P4; //set as default position 4 (chip normally  up/down on breadboard)
                                                      Combo_AMG_I2C_Configure(pSensorHandle, pWriteList);
                                                      Combo_AMG_I2C_SetModeFusion;}
                    return SENSOR_ERROR_NONE;
        }else 
          return SENSOR_ERROR_INIT;     
    } 

 


int32_t Combo_AMG_I2C_Deinit(i2c_sensorhandle_t *pSensorHandle){
  return 0;   //placeholder
}
/* END OF LOWER LEVEL INTERFACE FUNCTIONS */

/* FUSION DATA STRUCTURE RELATED FUNCTIONS*/
void fInitializeFusion (SensorFusionGlobals *sfg) {
#ifdef F_USING_SMARTSENSOR
        PhysicalSensor * pSensor_;
        int32_t res;
        for (pSensor_ = sfg->pSensors; pSensor_ != NULL; pSensor_ = pSensor_->next){
            if (pSensor_->isInitialized & F_USING_ACCEL|F_USING_MAG|F_USING_GYRO) {
            i2c_sensorhandle_t* pSensorHandle_ = (i2c_sensorhandle_t*) pSensor_->deviceInfo.functionParam;
            if ( res = Combo_AMG_I2C_SetModeFusion(pSensorHandle_) == SENSOR_ERROR_NONE)
                return;
                Serial.print("Errore in Combo_AMG_I2C_SetModeFusion - SENSOR_ERROR = ");
                Serial.println(res, DEC);
                while (true)
            {}
            }
        }
#endif
#ifndef F_USING_SMARTSENSOR
// TO DO: recover code from original repo
#endif
}
/* END OF FUSION DATA STRUCTURE RELATED FUNCTIONS*/

/* ABSTRACT SENSOR INTERFACE FUNCTIONS*/
void fInitializeMagCalibration (SensorFusionGlobals *sfg) {
    //read stored calibration data an put inside sfg proper fV vectors and radiuses
    //GetConfigCalibrationData ()    //Copies from value stored in SPIFFS config
                                     //should be placed in signalk personalizations of this library
    SaveMagneticCalib(sfg);
} // end fInitializeMagCalibration () 

#define FXOS8700_COUNTSPERG     8192        //assumes +/-4 g range on accelerometer
#define FXOS8700_COUNTSPERUT    10
#define FXAS21000_COUNTSPERDEGPERSEC    20      // 1600dps range
#define FXAS21002_COUNTSPERDEGPERSEC    16      // for 2000dps=32000 counts

int8_t Combo_AMG_sensor_Init(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg){
#ifdef F_USING_SMARTSENSOR
    int32_t status = SENSOR_ERROR_NONE;
    uint8_t reg;
    i2c_sensorhandle_t* pSensorHandle_ = (i2c_sensorhandle_t*) sensor->deviceInfo.functionParam; //recover  handle pointer 
                                                                                                 //should be here if senspr installed
    if ( !pSensorHandle_->isInitialized){   //to avoid unuseful/dangerous reinits of data structures
    
    pSensorHandle_->deviceInfo = sensor->deviceInfo ;
    reg= (uint8_t) sensor->addr;
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
    #if F_USING_GYRO
        sfg->Gyro.iWhoAmI = reg;
        sfg->Gyro.iCountsPerDegPerSec = FXAS21002_COUNTSPERDEGPERSEC;
        sfg->Gyro.fDegPerSecPerCount = 1.0F / FXAS21002_COUNTSPERDEGPERSEC;
    #endif
       /*if (reg != COMBO_I2C_ADDR) {
          return SENSOR_ERROR_INIT;  // The whoAmI did not match *****No need to control because I2C_Initialize checks!!
       }*/
       // Configure and start the AMG sensor.
        status = Combo_AMG_I2C_Initialize(pSensorHandle_, reg , 0x0000, COMBO_WHO_AM_I);
        if (status == SENSOR_ERROR_NONE) {
            sensor->deviceInfo.functionParam = pSensorHandle_ ; //store sensor handle pointer in deviceinfo functionParam
    #if F_USING_ACCEL
            sfg->Accel.isEnabled = true;
    #endif
    #if F_USING_MAG
            sfg->Mag.isEnabled = true;
    #endif
    #if F_USING_GYRO
            sfg->Gyro.isEnabled = true;
    #endif
            sensor->isInitialized = F_USING_ACCEL | F_USING_MAG | F_USING_GYRO;
            pSensorHandle_->isInitialized = true;
        } else {
            /// Throw error and reset flags 
            sensor->isInitialized = F_USING_NONE ;
            pSensorHandle_->isInitialized = false;
        }
    } else {
        /// To be decided if give a signal that init was attempted on an already initialized sensor e.g throwing SENSOR_ERROR_PARAM 
            status = SENSOR_ERROR_NONE;
            
    }

    return (status);
#endif
#ifndef F_USING_SMARTSENSOR
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
#   endif
    #if F_USING_MAG
    sfg->Mag.isEnabled = true;
    #endif

    return (status);
 // end FXOS8700_Init() */
#endif    
}


int8_t Combo_AMG_therm_Init (struct PhysicalSensor *sensor, SensorFusionGlobals *sfg){
#ifdef F_USING_SMARTSENSOR
    int32_t status = SENSOR_ERROR_NONE;
    i2c_sensorhandle_t* pSensorHandle = (i2c_sensorhandle_t*) sensor->deviceInfo.functionParam; //recover  handle pointer 
                                                                                                 //should be here if senspr installed
    if ( !pSensorHandle->isInitialized){   //to avoid unuseful/dangerous reinits of data structures

       // Configure and start the TEMP sensor. (TODO. to be changed when extern BARO_TERM sensor intead of internal of AMG sensor)
       // if Temp sensor is internal you have to look inside sfg->pSensors list to find the pSensorHandle containing the pointer
       //  to OR_sensor obj (hopefully) if Combo_AMG (Accel/Mag/Gyro) physical sensor is already initialized 
    
        PhysicalSensor *pSensor_;
        status = SENSOR_ERROR_INVALID_PARAM;
        // looking for the Combo sensor offering the internal temp sensor
        //uint8_t i = 0;
        for (pSensor_=sfg->pSensors; pSensor_ != NULL ; pSensor_ = pSensor_->next){
            if (pSensor_->isInitialized == F_USING_ACCEL|F_USING_MAG|F_USING_GYRO 
            && pSensor_->sensor_type == kMagnetometerAccelerometerGyroscope)
             { //FOUND COMBO ALREADY INITIALIZED!!
            status =SENSOR_ERROR_NONE;
            break;
            }
        //    i++;
        }  
        if (status == SENSOR_ERROR_NONE) {

            
            i2c_sensorhandle_t* pSensorHandle_= (i2c_sensorhandle_t*) pSensor_->deviceInfo.functionParam;
            OR_sensor_* Combo_AMG_sensor = (OR_sensor_*) pSensorHandle_->deviceInfo.functionParam;
            pSensorHandle->deviceInfo.functionParam = Combo_AMG_sensor;
            sfg->Temp.isEnabled = true;
            sensor->isInitialized = F_USING_TEMPERATURE;
            pSensorHandle->isInitialized = true; 
        } else {
            /// Throw error and reset flags 
            sensor->isInitialized = F_USING_NONE ;
            pSensorHandle->isInitialized = false;
        }
    } else {
        /// To be decided if give a signal that init was attempted on an already initialized sensor e.g throwing SENSOR_ERROR_PARAM 
            status = SENSOR_ERROR_NONE;
            
    }

    return (status);
#endif
#ifndef F_USING_SMARTSENSOR
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
#   endif
    #if F_USING_MAG
    sfg->Mag.isEnabled = true;
    #endif

    return (status);
 // end FXOS8700_Init() */
#endif    
}


#if F_USING_ACCEL
int8_t Combo_AMG_Accel_Read(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg) {
    int32_t                     status;         // I2C transaction status
    if(!(sensor->isInitialized & F_USING_ACCEL) || !sfg->Accel.isEnabled) {
       return SENSOR_ERROR_INIT;
    }
   #ifdef F_USING_SMARTSENSOR
    const uint8_t retry = 3;
    uint8_t sstatus, error, k;
    sstatus = error = 0xFF;      // use a value different from 0 to trigger specific datum reading
    uint8_t test_result = k = 0x00; // set to 0 to avoid test execution (that could be very long with respect to the read frequency)
    OR_sensor_ *Combo_AMG_sensor;
    i2c_sensorhandle_t *pSensorHandle = (i2c_sensorhandle_t*) sensor->deviceInfo.functionParam;
    if (pSensorHandle->isInitialized)
        {
            Combo_AMG_sensor = (OR_sensor_ *)pSensorHandle->deviceInfo.functionParam; // recover reference to OR_sensor_ object
                                                                                     // from initialized sensor handle

            
            while (k < retry) {
            Combo_AMG_sensor->getSystemStatus(&sstatus, &test_result, &error);
            if (sstatus != 1 /*not SYSTEM ERROR*/)
                {
                xyz = Combo_AMG_sensor->getVector(OR_sensor_::VECTOR_ACCELEROMETER); // reads accel vector data from IMU chip
                for (int8_t i=0; i<3; i++) sfg->Accel.fGc[i] = sfg->Accel.fGs[i] = (double)xyz[i]/double(GTOMSEC2);  //sensor library returns
                                                                                                             //SI units in double precision
                return SENSOR_ERROR_NONE;
                }

            else {
                    delay (100); //try retry times then bail
                    k++;
                }    
            }
            return ((int32_t(status) << 8)) | (int32_t(error)); //for debug 
        }
        else
            return SENSOR_ERROR_INIT;
   #endif
   #ifndef F_USING_SMARTSENSOR   
    uint8_t                     I2C_Buffer[6 * ACCEL_FIFO_SIZE];    // I2C read buffer
    int8_t                      j;              // scratch
    uint8_t                     fifo_packet_count;
    int16_t                     sample[3];

    /*
    // IMPLEMENTATION HW FX8700 dependent 
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
    */
   #endif
    return (status);
}  // end Combo_AMG_Accel_Read()
#endif
#if F_USING_MAG
// read magnetometer data over I2C or library interface
int8_t Combo_AMG_Mag_Read(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg) {
    int32_t                     status;         // I2C transaction status
    if(!(sensor->isInitialized & F_USING_MAG) || !sfg->Mag.isEnabled) {
       return SENSOR_ERROR_INIT;
    }
  #ifdef F_USING_SMARTSENSOR
    const uint8_t retry = 3;
    uint8_t sstatus, error, k;
    sstatus = error = 0xFF;      // use a value different from 0 to trigger specific datum reading
    uint8_t test_result = k = 0x00; // set to 0 to avoid test execution (that could be very long with respect to the read frequency)
    OR_sensor_ *Combo_AMG_sensor;
    i2c_sensorhandle_t *pSensorHandle = (i2c_sensorhandle_t*) sensor->deviceInfo.functionParam;
    if (pSensorHandle->isInitialized)
        {
            Combo_AMG_sensor = (OR_sensor_ *)pSensorHandle->deviceInfo.functionParam; // recover reference to OR_sensor_ object
                                                                                     // from initialized sensor handle            
            while (k < retry) {
            Combo_AMG_sensor->getSystemStatus(&sstatus, &test_result, &error);
            if (sstatus != 1 /*not SYSTEM ERROR*/)
                {
                xyz = Combo_AMG_sensor->getVector(OR_sensor_::VECTOR_MAGNETOMETER); // reads mag vector data from IMU chip
                for (int8_t i=0; i<3; i++) sfg->Mag.fBc[i] = sfg->Mag.fBs[i] = (float)xyz[i];  
                return SENSOR_ERROR_NONE;
                }

            else {
                    delay (100); //try retry times then bail
                    k++;
                }    
            }
            return ((int32_t(status) << 8)) | (int32_t(error)); //for debug 
        }
        else
            return SENSOR_ERROR_INIT;
  #endif  
  #ifndef F_USING_SMARTSENSOR  
    uint8_t                     I2C_Buffer[6];  // I2C read buffer
    int16_t                     sample[3];

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
  #endif
    return status;
}//end Combo_AMG_Mag_Read()
#endif  //F_USING_MAG

// read temperature register over I2C
int8_t Combo_AMG_Therm_Read(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg) {
    int32_t                     status;         // I2C transaction status
 
    if(!(sensor->isInitialized & F_USING_TEMPERATURE)) {
       return SENSOR_ERROR_INIT;
    }
  #ifdef F_USING_SMARTSENSOR  
    const uint8_t retry =3;
    uint8_t sstatus, error, k;
    sstatus = error = 0xFF;         // use a value different from 0 to trigger specific datum reading
    uint8_t test_result = k = 0x00; // set to 0 to avoid test execution (that could be very long with respect to the read frequency)
    OR_sensor_ *Combo_AMG_sensor;
    i2c_sensorhandle_t *pSensorHandle = (i2c_sensorhandle_t*) sensor->deviceInfo.functionParam;

    if (pSensorHandle->isInitialized)
        {
            Combo_AMG_sensor = (OR_sensor_ *)pSensorHandle->deviceInfo.functionParam; // recover reference to OR_sensor_ object
                                                                                      // from initialized sensor handle
                sfg->Temp.temperatureC = (float)Combo_AMG_sensor->getTemp(); //
                return SENSOR_ERROR_NONE;
        }
        else
            return SENSOR_ERROR_INIT;
   #endif
   #ifndef F_USING_SMARTSENSOR
    int8_t                      I2C_Buffer;     // I2C read buffer
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
    #endif
    return status;
}//end Combo_AMG_Therm_Read()

// read Combo_AMG gyro data  over I2C
int8_t Combo_AMG_Gyro_Read(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg)
{
    int32_t     status;
    if(!(sensor->isInitialized & F_USING_GYRO) || !sfg->Gyro.isEnabled) {
       return SENSOR_ERROR_INIT;
    }
#ifdef F_USING_SMARTSENSOR
    const uint8_t retry = 3;
    uint8_t sstatus, error, k;
    sstatus = error = 0xFF;      // use a value different from 0 to trigger specific datum reading
    uint8_t test_result = k = 0x00; // set to 0 to avoid test execution (that could be very long with respect to the read frequency)
    OR_sensor_ *Combo_AMG_sensor;
    i2c_sensorhandle_t *pSensorHandle = (i2c_sensorhandle_t*) sensor->deviceInfo.functionParam;
    if (pSensorHandle->isInitialized)
        {
            Combo_AMG_sensor = (OR_sensor_ *)pSensorHandle->deviceInfo.functionParam; // recover reference to OR_sensor_ object
                                                                                     // from initialized sensor handle            
            while (k < retry) {
            Combo_AMG_sensor->getSystemStatus(&sstatus, &test_result, &error);
            if (error == No_error && sstatus == 5 /*fusion running*/)
                {
                xyz = Combo_AMG_sensor->getVector(OR_sensor_::VECTOR_GYROSCOPE); // reads gyro vector data from IMU chip
                for (int8_t i=0; i<3; i++) sfg->Gyro.fYs[i] = xyz[i];  
                return SENSOR_ERROR_NONE;
                }

            else {
                    delay (100); //try retry times then bail
                    k++;
                }    
            }
            return ((int32_t(status) << 8)) | (int32_t(error)); //for debug 
        }
        else
            return SENSOR_ERROR_INIT;
  #endif  
#ifndef F_USING_SMARTSENSOR 
    uint8_t     fifo_packet_count = 1;
    int16_t     sample[3];
    uint8_t     I2C_Buffer[6 * GYRO_FIFO_SIZE]; // I2C read buffer
    uint8_t      j;                              // scratch
     
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
#endif
    return status;
        }


// This is the composite read function that handles both accel, mag and gyro portions of the Combo_AMG
// It returns the first failing status flag
int8_t Combo_AMG_Read(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg) {
    int8_t  sts1 = SENSOR_ERROR_NONE;
    int8_t  sts2 = SENSOR_ERROR_NONE;
    int8_t  sts3 = SENSOR_ERROR_NONE;
    double doubleres;
    uint16_t word16;
  #ifdef F_USING_SMARTSENSOR  
    i2c_sensorhandle_t* pSensorHandle_ = (i2c_sensorhandle_t*) sensor->deviceInfo.functionParam;
    if (pSensorHandle_->isInitialized) {
        uint8_t mode;
        sts3 = Combo_AMG_I2C_GetModeFusion(pSensorHandle_, &mode);
        if (sfg->isFusionStarted && mode != OPERATION_MODE_NDOF) Combo_AMG_I2C_SetModeFusion(pSensorHandle_);
        pReadList->numBytes = sizeof(int16_t)*MIN_BUF_LEN;
        sts1 = Combo_AMG_I2C_ReadData(pSensorHandle_, pReadList, (uint8_t*) &vectors);
        sts2 = Combo_AMG_I2C_GetCalibration (pSensorHandle_, calib);
        if (sts1==SENSOR_ERROR_NONE && sts2==SENSOR_ERROR_NONE)
            {
  #if F_USING_ACCEL
                sfg->Accel.fGs[0] = sfg->Accel.fGc[0]= vectors.v.accel_x/double(GTOMSEC2);  //sensor library retur
                sfg->Accel.fGs[1] = sfg->Accel.fGc[1]= vectors.v.accel_y/double(GTOMSEC2);  //SI units
                sfg->Accel.fGs[2] = sfg->Accel.fGc[2]= vectors.v.accel_z/double(GTOMSEC2);

  #endif          
  #if F_USING_MAG
                sfg->Mag.fBs[0] = sfg->Mag.fBc[0]= vectors.v.mag_x;  
                sfg->Mag.fBs[1] = sfg->Mag.fBc[1]= vectors.v.mag_y;  
                sfg->Mag.fBs[2] = sfg->Mag.fBc[2]= vectors.v.mag_z;
  #endif
  #if F_USING_GYRO
                sfg->Gyro.fYs[0] = vectors.v.gyro_x;
                sfg->Gyro.fYs[1] = vectors.v.gyro_y;
                sfg->Gyro.fYs[2] = vectors.v.gyro_z;
  #endif 
  #if F_9DOF_GBY_KALMAN
                sfg->SV_9DOF_GBY_KALMAN.fRVecPl[0] = vectors.v.euler_yaw;
                sfg->SV_9DOF_GBY_KALMAN.fRVecPl[1] = vectors.v.euler_pitch;
                sfg->SV_9DOF_GBY_KALMAN.fRVecPl[2] = vectors.v.euler_roll;
                sfg->SV_9DOF_GBY_KALMAN.fAccGl[0]= vectors.v.lia_x/double(GTOMSEC2);  //sensor library retur
                sfg->SV_9DOF_GBY_KALMAN.fAccGl[1]= vectors.v.lia_y/double(GTOMSEC2);  //SI units
                sfg->SV_9DOF_GBY_KALMAN.fAccGl[2]= vectors.v.lia_z/double(GTOMSEC2);
                sfg->SV_9DOF_GBY_KALMAN.fgPl[0]= vectors.v.gravity_x/double(GTOMSEC2);  //sensor library retur
                sfg->SV_9DOF_GBY_KALMAN.fgPl[1]= vectors.v.gravity_y/double(GTOMSEC2);  //SI units
                sfg->SV_9DOF_GBY_KALMAN.fgPl[2]= vectors.v.gravity_z/double(GTOMSEC2);
                sfg->SV_9DOF_GBY_KALMAN.fqPl.q0 = vectors.v.q0;
                sfg->SV_9DOF_GBY_KALMAN.fqPl.q1 = vectors.v.q1;
                sfg->SV_9DOF_GBY_KALMAN.fqPl.q2 = vectors.v.q2;
                sfg->SV_9DOF_GBY_KALMAN.fqPl.q3 = vectors.v.q3;
                
                sfg->SV_9DOF_GBY_KALMAN.fRhoPl = sfg->SV_9DOF_GBY_KALMAN.fRVecPl[0] /* *double(DEG_TO_RAD) */;    //heading by fusion
                sfg->SV_9DOF_GBY_KALMAN.fPhiPl = sfg->SV_9DOF_GBY_KALMAN.fRVecPl[1] /* *double(DEG_TO_RAD) */;   //roll by fusion
                sfg->SV_9DOF_GBY_KALMAN.fThePl = sfg->SV_9DOF_GBY_KALMAN.fRVecPl[2] /* *double(DEG_TO_RAD) */;   //pitch by fusion 
                
                sfg->SV_9DOF_GBY_KALMAN.fOmega[0] = vectors.v.gyro_x /* *double(DEG_TO_RAD) */;   //roll by fusion
                sfg->SV_9DOF_GBY_KALMAN.fOmega[1] = vectors.v.gyro_y /* *double(DEG_TO_RAD) */;   //roll by fusion
                sfg->SV_9DOF_GBY_KALMAN.fOmega[2] = vectors.v.gyro_z /* *double(DEG_TO_RAD) */;   //roll by fusion
                
  #endif
                                                                // copy into sfg the calibration indexes
                                                                // calculated by internal algorithms of
                                                                // smartsensor MCU. A value of 3 means
                                                                // full calibrated
                sfg->IMU_Calib_index = calib->IMU_Calib_index;
                sfg->GyroCal.Gyro_Calib_index = calib->Gyro_Calib_index;
                sfg->AccelCal.Accel_Calib_index = calib->Accel_Calib_index;
                sfg->MagCal.Mag_Calib_index = calib->Mag_Calib_index;
        }
    } else 
        return SENSOR_ERROR_INIT;
    return sts1+sts2+sts3;
  #endif
  #ifndef F_USING_SMARTSENSOR  
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
  #endif
} // end Combo_AMG_Read()

// Combo_AMG_Idle places the entire sensor into STANDBY mode (wakeup time = 1/ODR+1ms)
// This driver is all-on or all-off. It does not support single subsensor only.
// If you want that functionality, you can write your own using the initialization
// function in this file as a starting template.  We've chosen not to cover all
// permutations in the interest of simplicity.
int8_t Combo_AMG_Idle(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg) {
    int32_t     status = SENSOR_ERROR_NONE;
    if(sensor->isInitialized == (F_USING_ACCEL|F_USING_MAG|F_USING_GYRO)) {
#ifdef F_USING_SMARTSENSOR
    i2c_sensorhandle_t* pSensorHandle_ = (i2c_sensorhandle_t*) sensor->deviceInfo.functionParam;
    if (pSensorHandle_->isInitialized) {
        OR_sensor_ *Combo_AMG_sensor = (OR_sensor_ *)pSensorHandle_->deviceInfo.functionParam; // recover reference to OR_sensor_ object
        Combo_AMG_sensor->enterSuspendMode();                                                                          // from initialized sensor handle
    } else return SENSOR_ERROR_INIT;
#endif
#ifndef F_USING_SMARTSENSOR
    status = Sensor_I2C_Write_List(&sensor->deviceInfo, sensor->addr, FXOS8700_FULL_IDLE ); //OLD IMPLEMENTATION depending on FXOS7800
#endif
        sensor->isInitialized = 0;
#if F_USING_ACCEL
        sfg->Accel.isEnabled = false;
#endif
#if F_USING_MAG
        sfg->Mag.isEnabled = false;
#endif
#if F_USING_GYRO
        sfg->Gyro.isEnabled = false;
#endif
    } else {
      return SENSOR_ERROR_INIT;
    }
    return status;
} // end Combo_AMG_Idle()

void Get_NDOF_Calib(SensorFusionGlobals* sfg) {

#ifdef F_USING_SMARTSENSOR
    int16_t offsets_and_radiuses[SIZE_OFFSET_REGISTERS/2];
    PhysicalSensor* pSensor_;
    int8_t i;
    // find the correct sensor handle starting from sfg
    for (pSensor_=sfg->pSensors; pSensor_ != NULL; pSensor_= pSensor_->next) {
      if (pSensor_->isInitialized & F_USING_MAG|F_USING_ACCEL|F_USING_GYRO){
        // found. read values from IMU registers
        Combo_AMG_I2C_GetCalibrationData((i2c_sensorhandle_t *)pSensor_->deviceInfo.functionParam, 
                                         (uint8_t *) &offsets_and_radiuses[0]);
      }  
    }
    for (i=0; i<3; i++) {
      sfg->AccelCal.fV[i] = offsets_and_radiuses[i];
      sfg->MagCal.fV[i] = offsets_and_radiuses[i+3];
      sfg->GyroCal.fV[i] = offsets_and_radiuses[i+6];
    }
    sfg->AccelCal.Radius = offsets_and_radiuses[9];
    sfg->MagCal.Radius = offsets_and_radiuses[10];
#endif
}  // end Get_NDOF_Calib()

void SaveMagneticCalib(SensorFusionGlobals* sfg) {
#ifndef F_USING_SMARTSENSOR
    InjectCommand("SVMC");
#endif
#ifdef F_USING_SMARTSENSOR
    int16_t offsets_and_radiuses[SIZE_OFFSET_REGISTERS/2];
    PhysicalSensor* pSensor_;
    int8_t i;
    for (i=0; i<3; i++) {
      offsets_and_radiuses[i]= sfg->AccelCal.fV[i];
      offsets_and_radiuses[i+3]= sfg->MagCal.fV[i];
      offsets_and_radiuses[i+6]= sfg->GyroCal.fV[i];
    }
    offsets_and_radiuses[9] = sfg->AccelCal.Radius;
    offsets_and_radiuses[10] = sfg->MagCal.Radius;
    // find the correct sensor handle starting from sfg
    for (pSensor_ = sfg->pSensors; pSensor_ != NULL; pSensor_ = pSensor_->next)
    {
        if (pSensor_->isInitialized & F_USING_MAG|F_USING_ACCEL|F_USING_GYRO){
            // found. write values to IMU registers
            i2c_sensorhandle_t* pSensorHandle_ = (i2c_sensorhandle_t*) pSensor_->deviceInfo.functionParam;
            if (pSensorHandle_ != NULL && pSensorHandle_->isInitialized) {
                Combo_AMG_I2C_SetCalibrationData( pSensorHandle_, (uint8_t *) &offsets_and_radiuses[0]);
                return;
            }
            Serial.println("breakpoint ---- SaveMagneticCalib called for NULL or not initialized sensor handle");
            while (true){};
        }
    }
#endif
}  // end SaveMagneticCalib()

void EraseMagneticCalib (SensorFusionGlobals* sfg) {
#ifndef F_USING_SMARTSENSOR
    InjectCommand("ERMC");
#endif
#ifdef F_USING_SMARTSENSOR
    // write  zero values to IMU registers 
    int8_t i;
    for (i=0; i<3; i++) {sfg->AccelCal.fV[i] = sfg->MagCal.fV[i] = sfg->GyroCal.fV[i] = 0;}
    sfg->AccelCal.Radius = sfg->MagCal.Radius = 0;
    SaveMagneticCalib (sfg);
#endif
}  // end EraseMagneticCalib

void SetAxisremap (SensorFusionGlobals* sfg, physical_sensor_position_t pos) {
#ifndef F_USING_SMARTSENSOR
#endif
#ifdef F_USING_SMARTSENSOR
    registerwritelist_t list_; //only "value" member is used to pass pos parameter
    int8_t i = 0;
    list_.value = pos;
    PhysicalSensor* pSensor_ = sfg->pSensors;
    while (!(pSensor_ == NULL) && (pSensor_->isInitialized & F_USING_MAG|F_USING_ACCEL|F_USING_GYRO) 
           && (i < MAX_NUM_SENSORS)){
      // found. write configuration values to IMU registers
      Combo_AMG_I2C_Configure((i2c_sensorhandle_t *)pSensor_->deviceInfo.functionParam, 
                                                    &list_);
      // write values SPIFFS relative paths
      pSensor_= pSensor_->next;
      i++;
    }
#endif
}  // end SetAxisRemap


// IMPORT SNIPPET from   sensor_fusion/sensor_fusion.c
#ifdef __cplusplus
extern "C" {
#endif

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

/// installSensor is used to instantiate a physical sensor driver into the
/// sensor fusion system. It doesn't actually communicate with the sensor.
/// This function is normally invoked via the "sfg." global pointer.
int8_t installSensor(
                     SensorFusionGlobals *sfg,  ///< top level fusion structure
                     PhysicalSensor *pSensor,    ///< pointer to structure describing physical sensor
                     uint16_t addr,             ///< I2C address for sensor (if applicable)
                     uint16_t schedule,         ///< Sensor is read each time loop_count % schedule == 0
                     registerDeviceInfo_t *busInfo, ///< information required for bus power management
                     SensorType            sensor_type, ///< required to manage order of initialization
                     initializeSensor_t *initialize,    ///< pointer to sensor initialization function
                     readSensor_t *read)        ///< pointer to sensor read function
{
    if (sfg && pSensor && initialize && read)
    {
    /* was  pSensor->deviceInfo.deviceInstance = busInfo->deviceInstance;
        pSensor->deviceInfo.functionParam = busInfo->functionParam;
        pSensor->deviceInfo.idleFunction = busInfo->idleFunction;
        but these aren't used. Instead of changing structs everywhere, 
        we just set to zero. 

        **** UPDATE ***** use a 'spare' members of deviceInfo (functionParam) 
        to store a pointer to SensorHandle*/
        
        auto* pSensorHandle = new (i2c_sensorhandle_t);
        pSensorHandle->isInitialized = false;
        pSensorHandle->deviceInfo.idleFunction =NULL;
        pSensorHandle->deviceInfo.functionParam =NULL; //in case of smartsensor will be used to store pointer to
                                                       // smartsensor library object interface 
        pSensorHandle->deviceInfo.deviceInstance =0x00;
        pSensor->deviceInfo.deviceInstance = 0x00;
        pSensor->deviceInfo.functionParam = pSensorHandle;  // Sensor.deviceInfo.functionParam contains pointer to SensorHandle
                                                            // SensorHandle.deviceInfo.functionParam will be initialized
                                                            // to a pointer to the sensor lower level object class offered 
                                                            // by selected library (THIS BEHAVIOR IN CASE OF SMARTSENSOR)
        pSensor->deviceInfo.idleFunction = NULL;            // so far not used in project
        pSensor->sensor_type = sensor_type;
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
    #ifndef EXTERN_BARO_THERM_SENSOR            // if temperature sensor to be used is the one inside other sensors
    PhysicalSensor* pSensor_init_after {NULL};  // it must be initialized as the last one in order to copy pointers 
    #endif                                      // to already initialized structures and objects of the containing sendor
    for (pSensor = sfg->pSensors; pSensor != NULL; pSensor = pSensor->next)
    {
    #ifndef EXTERN_BARO_THERM_SENSOR
        if (pSensor->sensor_type == SensorType::kThermometer )  {
            pSensor_init_after = pSensor;
            continue;
        }  
    #endif
        s = pSensor->initialize(pSensor, sfg);
        if (status == 0) status = s;            // will return 1st error flag, but try all sensors
    } 
    #ifndef EXTERN_BARO_THERM_SENSOR
        if (pSensor_init_after != NULL )  {
        s = pSensor_init_after->initialize(pSensor_init_after, sfg);
        if (status == 0) status = s;            // will return 1st error flag, but try all sensors
        }  
    #endif
    return (status);
} // end initializeSensors()

// process<Sensor>Data routines do post processing for HAL and averaging.  They
// are called from the readSensors() function below.
#if F_USING_ACCEL
void processAccelData(SensorFusionGlobals *sfg) //with smartsensor becomes a NOP
{
#ifndef F_USING_SMARTSENSOR    
    int32_t iSum[3];		        // channel sums
    int16_t i, j;			        // counters
    if (sfg->combo_sensor_is_smart) return; ///NOP if sensor does fusion algo autonomously
    if (sfg->Accel.iFIFOExceeded > 0) {
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
                       &(sfg->Accel) /*,
                       &(sfg->pControlSubsystem->AccelCalPacketOn)*/);
 #endif
    return;
} // end processAccelData()
#endif

#if F_USING_MAG
void processMagData(SensorFusionGlobals *sfg) //with smartsensor becomes a NOP
{
  #ifndef F_USING_SMARTSENSOR
    int32_t iSum[3];		        // channel sums
    int16_t i, j;			        // counters
    if (sfg->combo_sensor_is_smart) return; ///NOP if sensor does fusion algo autonomously
    
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
  #endif
    return;
} // end processMagData()
#endif

#if F_USING_GYRO
void processGyroData(SensorFusionGlobals *sfg) //with smartsensor becomes a NOP
{
    #ifndef F_USING_SMARTSENSOR
    int32_t iSum[3];		        // channel sums
    int16_t i, j;			        // counters
    if (sfg->combo_sensor_is_smart) return; ///NOP if sensor does fusion algo autonomously


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
    #endif
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
    bool            at_least_one_working    = false;

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
                
                if (status == SENSOR_ERROR_NONE) {status = s; at_least_one_working = true;} // will return 1st error flag, but try all sensors
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

void conditionSample(
    int16_t sample[3]                                   ///< 16-bit register value from triaxial sensor read
){
    //placeholder
}
/// conditionSensorReadings() transforms raw software FIFO readings into forms that
/// can be consumed by the sensor fusion engine.  This include sample averaging
/// and (in the case of the gyro) integrations, applying hardware abstraction layers,
/// and calibration functions.
/// This function is normally invoked via the "sfg." global pointer.
void conditionSensorReadings(SensorFusionGlobals *sfg) //with smartsensor becomes a NOP
{
#ifndef F_USING_SMARTSENSOR
#if F_USING_ACCEL
    if (sfg->Accel.isEnabled) processAccelData(sfg);
#endif

#if F_USING_MAG
    if (sfg->Mag.isEnabled) processMagData(sfg);
#endif

#if F_USING_GYRO
    if (sfg->Gyro.isEnabled) processGyroData(sfg);
#endif
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
#ifndef F_USING_SMARTSENSOR //becomes a NOP when using a smartsensor
  // We only clear FIFOs if the sensors are enabled.  This allows us
  // to continue to use these values when we've shut higher power consumption
  // sensors down during periods of no activity.

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
#endif
} // end clearFIFOs()

/// Function to insert a step during test with toolbox at the end of each fusion computation
void ApplyPerturbation(SensorFusionGlobals *sfg) {
#ifndef F_USING_SMARTSENSOR //becomes a NOP when using a smartsensor
  
#endif
} // end ApplyPerturbation()



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
  return Combo_AMG_therm_Init(sensor, sfg);
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
        //Use the same init function for thermometer, magnetometer, accelererometer, gyroscope - it will
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
  return Combo_AMG_Therm_Read(sensor, sfg);
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
{                               // with smartsensor you have only to setMode "Fusion" on sensor
                                // and resister this in a sfg boolean
                                //since fusion is done by the sensor IMU
#ifdef F_USING_SMARTSENSOR

#endif
#ifndef F_USING_SMARTSENSOR                                  
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

    fFuseSensors(pSV_1DOF_P_BASIC, pSV_3DOF_G_BASIC,
                 pSV_3DOF_B_BASIC, pSV_3DOF_Y_BASIC,
                 pSV_6DOF_GB_BASIC, pSV_6DOF_GY_KALMAN,
                 pSV_9DOF_GBY_KALMAN, pAccel, pMag, pGyro,
                 pPressure, pMagCal);
                 
    clearFIFOs (sfg);
#endif    
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
    // initialize the magnetic calibration and magnetometer data buffer
#if F_USING_MAG  //TO BE VERIFIED if this flag covers all use cases of interest for enabling this code
    fInitializeMagCalibration(sfg); //Since smartsensor calculates an store calibration data for Accel,Mag, Gyro
                                    //this function is intended for all calibrating subsensors in smarsensor case.
#endif
    // initialize the sensor fusion algorithms
    fInitializeFusion(sfg);

    // reset the loop counter to zero for first iteration
    sfg->loopcounter = 0;

}  // end initializeFusionEngine()

//END OF IMPORT from sensor_fusion/sensor_fusion.c
#ifdef __cplusplus
}
#endif

/* OLD IMPLEMENTATION FX8700 dependent
int8_t Combo_AMG_sensor_Init(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg) {


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
