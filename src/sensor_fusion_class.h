/**
 * Copyright (c) 2020-2021 Bjarne Hansen
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
/**
 * @file sensor_fusion_class.h
 *
 * Wrapper for the NXP Sensor Fusion v7 functions.
 */

#ifndef SENSOR_FUSION_CLASS_H_
#define SENSOR_FUSION_CLASS_H_

#include <Stream.h>
#include <Arduino.h>
//#include <WifiClient.h>

//#include "board.h"
#include "driver_sensors.h"
#include "driver_sensors_types.h"
//#include "build.h"
#include "sensor_fusion.h"
//#include "sensor_fusion/control.h"
//#include "sensor_fusion/status.h"
#include "combo_amg_sensor_driver.h"
 
/*! \file status.h    --------------IMPORT SNIPPET FROM sensor_fusion/status.h
    \brief Application-specific status subsystem

    Applications may change how they choose to display status information.
    The default implementation here uses LEDs.
    You may swap out implementations as long as the "Required" methods and states
    are retained.
*/

/// StatusSubsystem() provides an object-like interface for communicating status to the user

typedef struct StatusSubsystem {
	// Required internal states
	fusion_status_t		previous;       ///< Previous status state - fusion_status_t is defined in sensor_fusion.h
	fusion_status_t		status;         ///< Current status
	fusion_status_t		next;           ///< Pending status change
	// Required methods
	ssSetStatus_t          *set;	        ///< change status immediately - no delay
    //void                    *set;	        ///< change status immediately - no delay
	ssGetStatus_t           *get;	        ///< return status
	ssSetStatus_t           *queue;         ///< queue status change for next regular interval
	ssUpdateStatus_t        *update;        ///< make pending status active/visible
	ssUpdateStatus_t        *test ;         ///< unit test which simply increments to next state
	// application-specific internal variables
	uint8_t toggle;                         ///< This implementation can change LED color and have either solid/toggle
} StatusSubsystem ;
//end of IMPORT

/*/// initializeStatusSubsystem() should be called once at startup to initialize the
/// data structure and to put hardware into the proper state for communicating status.
void initializeStatusSubsystem (
  StatusSubsystem *pStatus                      ///< pointer to the status subsystem
);
*/
/**
 *  Class that wraps the various mostly-C-style functions of the
 *  sensor fusion code into easier to use methods. Not all the
 *  lower-level functions are exposed however; for more advanced
 *  use it will be necessary to call them directly.
 */
class SensorFusion {
    
 public:
  SensorFusion();
  //bool InitializeInputOutputSubsystem(const Stream *serial_port = NULL, //IO Subsystem not useful using SignalK 
  //                                                                      // as communication interface  
  //                                    const void *tcp_client = NULL);
  //void UpdateWiFiStream(void *tcp_client);                              //IO Subsystem not useful using SignalK 
  //void ProduceToolboxOutput(void);
  //bool SendArbitraryData(const char *buffer, uint16_t data_length);
  //void ProcessCommands(void);
  //void InjectCommand(const char *command);

  bool InstallSensor(uint8_t sensor_i2c_addr, SensorType sensor_type);
  void Begin(int pin_i2c_sda = -1, int pin_i2c_scl = -1);  
  void ReadSensors(void);
  void RunFusion(void);
  
  void SaveMagneticCalibration(void);
  void EraseMagneticCalibration(void);
  bool IsDataValid(void);
  int  GetSystemStatus(void);
  float GetHeadingDegrees(void);
  float GetPitchDegrees(void);
  float GetRollDegrees(void);
  float GetHeadingRadians(void);
  float GetPitchRadians(void);
  float GetRollRadians(void);
  float GetAccelXGees(void);
  float GetAccelYGees(void);
  float GetAccelZGees(void);
  float GetAccelXMPerSS(void);
  float GetAccelYMPerSS(void);
  float GetAccelZMPerSS(void);
  float GetTurnRateDegPerS(void);
  float GetPitchRateDegPerS(void);
  float GetRollRateDegPerS(void);
  float GetTurnRateRadPerS(void);
  float GetPitchRateRadPerS(void);
  float GetRollRateRadPerS(void);
  float GetTemperatureC(void);//to be revised with adding of BMP280 breakout board
  float GetTemperatureK(void);//to be revised with adding of BMP280 breakout board
  float GetPressureHpa(void); //to be revised with adding of BMP280 breakout board
  void  GetOrientationQuaternion(Quaternion *quat);
  #ifndef F_USING_SMARTSENSOR
  float GetMagneticFitError(void);
  float GetMagneticFitErrorTrial(void);
  float GetMagneticBMag(void);
  float GetMagneticBMagTrial(void);
  float GetMagneticInclinationDeg(void);
  float GetMagneticInclinationRad(void);
  float GetMagneticNoiseCovariance(void);
  float GetMagneticCalSolver(void);
  #endif
  #ifdef F_USING_SMARTSENSOR
  
  uint8_t               GetIMUCalibrationIndex(void);           //range 0..3
  uint8_t               GetMagCalibrationIndex(void);           //range 0..3
  uint8_t               GetAccelCalibrationIndex(void);         //range 0..3
  uint8_t               GetGyroCalibrationIndex(void);          //range 0..3
  void                  GetCalibrationData(int16_t* Cal_data);  //copies offset vectors and radiuses out of globals
  void                  SetCalibrationData(int16_t* Cal_data);  //copies offset vectors and radiuses into globalsvoid
  void                  ReadCalibrationData();                  //reads  offset vectors and radiuses from smartsensor registers
                                                                //and copies them into globals
  void                  SetAxisRemap(physical_sensor_position_t //changes the reference coordinates of the IMU chip
                                    pos);                       //according to the physical position it is installed in                         
  #endif
  
  private:
  void InitializeStatusSubsystem(void);
  void InitializeSensorFusionGlobals(void);
  
  public:
  SensorFusionGlobals  sfg;
  StatusSubsystem status_subsystem;
  bool sensor_initialized;
  PhysicalSensor sensors[MAX_NUM_SENSORS];  // this implementation uses up to 5 sensors
                          // (accel/mag/gyro; internal temp or external baro/thermo)
                          // TODO - can use std::vector as per
                          // https://stackoverflow.com/questions/14114686/create-an-array-of-structure-using-new
                          // which will allow adding arbitrary # of sensors.
  private:  
  SensorFusionGlobals *sfg_;  				        ///< Primary sensor fusion data structure
  // ControlSubsystem    *control_subsystem_;       ///< command and data streaming structure
  StatusSubsystem     *status_subsystem_;           ///< visual status indicator structure
  PhysicalSensor      *sensors_;                    ///< linked list of up to 5 sensors
  //PhysicalSensor      *current_;
  uint8_t              num_sensors_installed_;      ///< tracks how many sensors have been added to list

  /**
   * Constants of the form kLoopsPer_____ set the relationship between
   * number of sensor reads and each execution of the fusion algorithm.
   * Normally there is a 1:1 relationship (i.e. read, fuse, read, fuse,...)
   * but other arrangements are possible (e.g. read, read, fuse, read,
   * read,...) The rate at which main loop() executes is set by
   * LOOP_RATE_HZ in build.h
   */
  const uint8_t kLoopsPerMagRead =
      1;  ///< how often a magnetometer read is performed
  const uint8_t kLoopsPerThermRead =
      1;  ///< how often a thermometer read is performed
  const uint8_t kLoopsPerBaroRead =
      1;  ///< how often a barometer read is performed
  const uint8_t kLoopsPerAccelRead =
      1;  ///< how often an accelerometer read is performed
  const uint8_t kLoopsPerGyroRead =
      1;  ///< how often a gyroscope read is performed
  const uint8_t kLoopsPerFusionCalc =
      1;  ///< how often to fuse. Usually the max of previous 3 constants.
  uint8_t loops_per_fuse_counter_ =
      0;  ///< counts how many times through loop have been done

  
};  // end SensorFusion

#endif /* SENSOR_FUSION_CLASS_H_ */