/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Sensor fusion requires a fairly extensive set of data structures, which are
// defined in this file.  The top level structure is shown near the bottom.  The
// size of this structure (SensorFusionGlobals) varies dramatically as a function
// of which fusion variations have been selected in build.h.

/*! \file sensor_fusion.h
    \brief The sensor_fusion.h file implements the top level programming interface
*/

#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <Arduino.h>
// Library dependencies (Adafruit BNO055 in the case) defined here
#include "Adafruit_I2CDevice.h"
#include "Adafruit_I2CRegister.h"
#include "Adafruit_BNO055.h"
#include "Wire.h"

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
#define SIZE_OFFSET_REGISTERS (NUM_BNO055_OFFSET_REGISTERS)

typedef enum { P0,
               P1,
               P2,
               P3,
               P4,
               P5,
               P6,
               P7
} physical_sensor_position_t;
// end of library dependencies
#define MAX_NUM_SENSORS (5)

#ifdef __cplusplus
extern "C" {
#endif
typedef struct offset_registers8_t { int8_t a[SIZE_OFFSET_REGISTERS];} offset_registers8_t;
typedef struct offset_registers16_t { int16_t a[SIZE_OFFSET_REGISTERS/2];} offset_registers16_t;

// Standard includes
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "board.h"						// Hardware-specific details (e.g. particular sensor ICs)
#include "build.h"                      // This is where the build parameters are defined
#include "driver_sensors_types.h"		// Typedefs for the sensor hardware
/**
 *  enum constants used to indicate what type of sensor is being installed
 *  when calling InstallSensor().
 */
typedef enum  SensorType {
  none,
  kMagnetometer,
  kAccelerometer,
  kGyroscope,
  kMagnetometerAccelerometer,
  kMagnetometerAccelerometerGyroscope,
  kBarometer,
  kThermometer,
  KBarometerThermometer //Placeholder for integration of BME280 sensor (on another breakout board - TODO next)
}SensorType;


/// the quaternion type to be transmitted (uselfulness of this type to be evaluated since the tossing 
///										   of IOcontrol subsystem where this info is delivered)
typedef enum quaternion {
    Q3,         ///< Quaternion derived from 3-axis accel (tilt)
    Q3M,        ///< Quaternion derived from 3-axis mag only (auto compass algorithm)
    Q3G,        ///< Quaternion derived from 3-axis gyro only (rotation)
    Q6MA,       ///< Quaternion derived from 3-axis accel + 3 axis mag (eCompass)
    Q6AG,       ///< Quaternion derived from 3-axis accel + 3-axis gyro (gaming)
    Q9          ///< Quaternion derived from full 9-axis sensor fusion
} quaternion_type;

/// quaternion structure definition
typedef struct Quaternion       
{
	float q0;	        ///< scalar component
	float q1;	        ///< x vector component
	float q2;	        ///< y vector component
	float q3;	        ///< z vector component
} Quaternion;

/// @name Vector Components
/// Index values for accessing vector terms
///@{
#define CHX 0   ///< Used to access X-channel entries in various data data structures
#define CHY 1   ///< Used to access Y-channel entries in various data data structures
#define CHZ 2   ///< Used to access Z-channel entries in various data data structures
///@}

// booleans
#ifndef true
#define true 1  ///< Boolean TRUE
#endif
#ifndef false
#define false 0 ///< Boolean FALSE
#endif

/// @name Generic bit-field values
/// Generic bit-field values
///@{					///collisions with system file binary.h - seem to be unused anyway
//#define B0 (1 << 0)
//#define B1 (1 << 1)
//#define B2 (1 << 2)
//#define B3 (1 << 3)
///@}

/// @name Math Constants
/// useful multiplicative conversion constants
///@{
#ifndef PI
	#define PI 3.141592654F				///< pi  (it is also defined in Arduino.h)
#endif
#define PIOVER2 1.570796327F			///< pi / 2
#define FPIOVER180 0.01745329251994F	        ///< degrees to radians conversion = pi / 180
#define F180OVERPI 57.2957795130823F	        ///< radians to degrees conversion = 180 / pi
#define F180OVERPISQ 3282.8063500117F	        ///< square of F180OVERPI
#define ONETHIRD 0.33333333F			///< one third
#define ONESIXTH 0.166666667F			///< one sixth
#define ONESIXTEENTH 0.0625F			///< one sixteenth
#define ONEOVER12 0.083333333F			///< 1 / 12
#define ONEOVER48 0.02083333333F		///< 1 / 48
#define ONEOVER120 0.0083333333F		///< 1 / 120
#define ONEOVER3840 0.0002604166667F	        ///< 1 / 3840
#define ONEOVERSQRT2 0.707106781F		///< 1/sqrt(2)
#define SQRT15OVER4  0.968245837F		///< sqrt(15)/4
#define GTOMSEC2 9.80665			///< standard gravity in m/s2
///@

#ifndef F_USING_SMARTSENSOR
#define MAX_ACCEL_CAL_ORIENTATIONS 12
#define MAGBUFFSIZEX 14				///< x dimension in magnetometer buffer (14x28 equals 392 elements)
#endif
#ifdef F_USING_SMARTSENSOR
#define MAX_ACCEL_CAL_ORIENTATIONS 1
#define MAGBUFFSIZEX 1				///< x dimension in magnetometer buffer (14x28 equals 392 elements)
#endif
#define MAGBUFFSIZEY (2 * MAGBUFFSIZEX)		///< y dimension in magnetometer buffer (14x28 equals 392 elements)

// Placeholder structures (redefined later, but needed now for pointer definitions)
struct SensorFusionGlobals;                     ///< Top level structure has pointers to everything else
struct StatusSubsystem;                         ///< Application-specific status subsystem
struct PhysicalSensor;                          ///< We'll have one of these for each physical sensor (FXOS8700 = 1 physical sensor)
//struct ControlSubsystem;                        ///< Application-specific serial communications system

typedef enum fusion_status_t {                                  ///  These are the state definitions for the status subsystem
	OFF,                                    ///< Application hasn't started
	INITIALIZING,                           ///< Initializing sensors and algorithms
	LOWPOWER,                               ///< Running in reduced power mode
	NORMAL,                                 ///< Operation is Nominal
  //RECEIVING_WIRED,                        ///< Receiving commands over wired interface (momentary)
  //RECEIVING_WIRELESS,                     ///< Receiving commands over wireless interface (momentary)
	HARD_FAULT,                             ///< Non-recoverable FAULT = something went very wrong
  SOFT_FAULT                              	///< Recoverable FAULT = something went wrong, but we can keep going
} fusion_status_t;

typedef struct
{
    uint8_t IMU_Calib_index;
    uint8_t Gyro_Calib_index;
    uint8_t Accel_Calib_index;
    uint8_t Mag_Calib_index;
} calib_index_t;

// declare typedefs for function prototypes that need to be installed
typedef int8_t (initializeSensor_t) (
    struct PhysicalSensor *sensor,
    struct SensorFusionGlobals *sfg
) ;
typedef int8_t (readSensor_t) (
    struct PhysicalSensor *sensor,
    struct SensorFusionGlobals *sfg
) ;
typedef int8_t (readSensors_t) (
    struct SensorFusionGlobals *sfg,
    uint8_t read_loop_counter
) ;
typedef int8_t (installSensor_t) (
    struct SensorFusionGlobals *sfg,    ///< Global data structure pointer
    struct PhysicalSensor *sensor,      ///< SF Structure to store sensor configuration
    uint16_t addr,                      ///< I2C address or SPI_ADDR
    uint16_t schedule,                  ///< Specifies sampling interval
    registerDeviceInfo_t *busInfo,      ///< information required for bus power management
	SensorType	sensor_type,			///< required to manage installation order/dependency
    initializeSensor_t *initialize,     ///< SF Sensor Initialization Function pointer
    readSensor_t *read                  ///< SF Sensor Read Function pointer
);

typedef void   (initializeFusionEngine_t) 	(struct SensorFusionGlobals *sfg, int pin_i2c_sda1, int pin_i2c_scl);
typedef void   (runFusion_t) 			(struct SensorFusionGlobals *sfg);
typedef void   (clearFIFOs_t) 			(struct SensorFusionGlobals *sfg);
typedef void   (conditionSensorReadings_t) 	(struct SensorFusionGlobals *sfg);
typedef void   (applyPerturbation_t) 		(struct SensorFusionGlobals *sfg) ;
typedef void   (setStatus_t) 			(struct SensorFusionGlobals *sfg, fusion_status_t status);
typedef fusion_status_t  (getStatus_t) 			(struct SensorFusionGlobals *sfg);
typedef void   (updateStatus_t) 		(struct SensorFusionGlobals *sfg);
typedef void   (ssSetStatus_t) 			(struct StatusSubsystem *pStatus, fusion_status_t status);
typedef fusion_status_t   (ssGetStatus_t) 			(struct StatusSubsystem *pStatus);
typedef void   (ssUpdateStatus_t) 		(struct StatusSubsystem *pStatus);

/// \brief An instance of PhysicalSensor structure type should be allocated for each physical sensors (combo devices = 1)
///
/// These structures sit 'on-top-of' the pre-7.0 sensor fusion structures and give us the ability to do run
/// time driver installation.
typedef struct PhysicalSensor {
        registerDeviceInfo_t deviceInfo;        ///< I2C device context
        registerDeviceInfo_t *busInfo;          ///< information required for bus power management
		uint16_t addr;  						///< I2C address if applicable
        uint16_t isInitialized;                 ///< Bitfields to indicate sensor is active (use SensorBitFields from build.h)
		struct PhysicalSensor *next;			///< pointer to next sensor in this linked list
        uint8_t schedule;                     	///< Parameter to control sensor sampling rate
		SensorType	sensor_type;				///< required to regulate order of initialization
		initializeSensor_t *initialize;  		///< pointer to function to initialize sensor using the supplied drivers
		readSensor_t *read;						///< pointer to function to read sensor using the supplied drivers
}PhysicalSensor;

// Now start "standard" sensor fusion structure definitions

/// \brief The PressureSensor structure stores raw and processed measurements for an altimeter.
///
/// The PressureSensor structure stores raw and processed measurements, as well as
/// metadata for a pressure sensor/altimeter.
typedef struct PressureSensor
{
	uint8_t iWhoAmI;		///< sensor whoami
  	bool  isEnabled;        ///< true if the device is sampling
	int32_t iH;				///< most recent unaveraged height (counts)
	int32_t iP;				///< most recent unaveraged pressure (counts)
	float fH;				///< most recent unaveraged height (m)
	float fT;				///< most recent unaveraged temperature (C)
	float fmPerCount;		///< meters per count
	float fCPerCount;		///< degrees Celsius per count
	int16_t iT;				///< most recent unaveraged temperature (counts)
}PressureSensor;

/// \brief The AccelSensor structure stores raw and processed measurements for a 3-axis accelerometer.
///
/// The AccelSensor structure stores raw and processed measurements, as well as metadata
/// for a single 3-axis accelerometer.  This structure is
/// normally "fed" by the sensor driver and "consumed" by the fusion routines.
typedef struct AccelSensor
{
	uint8_t iWhoAmI;						///< sensor whoami
	bool  isEnabled;                        ///< true if the device is sampling
	uint8_t iFIFOCount;						///< number of measurements read from FIFO
    uint16_t iFIFOExceeded;                 ///< Number of samples received in excess of software FIFO size
	int16_t iGsFIFO[ACCEL_FIFO_SIZE][3];	///< FIFO measurements (counts)
        // End of common fields which can be referenced via FifoSensor union type
	float fGs[3];			        		///< averaged measurement (g)
	float fGc[3];							///< averaged precision calibrated measurement (g)
	float fgPerCount;						///< g per count
	float fCountsPerg;						///< counts per g
	int16_t iGs[3];							///< averaged measurement (counts)
	int16_t iGc[3];							///< averaged precision calibrated measurement (counts)
	int16_t iCountsPerg;					///< counts per g
}AccelSensor;
/// accelerometer measurement buffer
typedef struct  AccelBuffer
{
	float fGsStored[MAX_ACCEL_CAL_ORIENTATIONS][3];	///< uncalibrated accelerometer measurements (g)
	float fS ;///umGs[3];				///< averaging sum for current storage location
	int16_t iStoreCounter;				///< number of remaining iterations at FUSION_HZ to average measurement
	int16_t iStoreLocation;				///< -1 for none, 0 to 11 for the 12 storage locations
	int16_t iStoreFlags;				///< denotes which measurements are present
}AccelBuffer;

/// precision accelerometer calibration structure
typedef struct AccelCalibration
{
	// start of elements stored in flash memory
	float fV[3];					///< offset vector (g)
	float finvW[3][3];				///< inverse gain matrix
	float fR0[3][3];				///< forward rotation matrix for measurement 0
	// end of elements stored in flash memory
	#ifndef F_USING_SMARTSENSOR
	float fmatA[10][10];			        ///< scratch 10x10 matrix used by calibration algorithms
	float fmatB[10][10];			        ///< scratch 10x10 matrix used by calibration algorithms
	float fvecA[10];				///< scratch 10x1 vector used by calibration algorithms
	float fvecB[4];					///< scratch 4x1 vector used by calibration algorithms
	float fA[3][3];					///< ellipsoid matrix A
	float finvA[3][3];				///< inverse of the ellipsoid matrix A
	#endif
	#ifdef F_USING_SMARTSENSOR
	int16_t Radius;
	uint8_t Accel_Calib_index;
	#endif
} AccelCalibration ;

/// \brief The MagSensor structure stores raw and processed measurements for a 3-axis magnetic sensor.
///
/// The MagSensor structure stores raw and processed measurements, as well as metadata
/// for a single 3-axis magnetometer.  This structure is
/// normally "fed" by the sensor driver and "consumed" by the fusion routines.
typedef struct MagSensor
{
	uint8_t iWhoAmI;			///< sensor whoami
    bool  isEnabled;            ///< true if the device is sampling
	uint8_t iFIFOCount;			///< number of measurements read from FIFO
    uint16_t iFIFOExceeded;     ///< Number of samples received in excess of software FIFO size
		
	int16_t iBsFIFO[MAG_FIFO_SIZE][3];	///< FIFO measurements (counts)
        // End of common fields which can be referenced via FifoSensor union type
	float fBs[3];				///< averaged un-calibrated measurement (uT)
	float fBc[3];				///< averaged calibrated measurement (uT)
	float fuTPerCount;			///< uT per count
	float fCountsPeruT;			///< counts per uT
	int16_t iBs[3];				///< averaged uncalibrated measurement (counts)
	int16_t iBc[3];				///< averaged calibrated measurement (counts)
	int16_t iCountsPeruT;			///< counts per uT
}MagSensor;

//IMPORT SNIPPET from sensor_fusion/magnetc.h
/// Magnetic Calibration Structure
typedef struct MagCalibration
{
	// start of elements stored to flash memory on Save (16 * 4 = 64 bytes)
	float fV[3];					///< current hard iron offset x, y, z, (uT)
	float finvW[3][3];				///< current inverse soft iron matrix
	float fB;						///< current geomagnetic field magnitude (uT)
	float fBSq;						///< square of fB (uT^2)
	float fFitErrorpc;				///< current fit error %
	int32_t iValidMagCal;			///< solver used: 0 (no calibration) or 4, 7, 10 element
	// end of elements stored to flash memory
	#ifndef F_USING_SMARTSENSOR
	// start of working elements not stored to flash memory
	float ftrV[3];							///< trial value of hard iron offset z, y, z (uT)
	float ftrinvW[3][3];			        ///< trial inverse soft iron matrix size
	float ftrB;								///< trial value of geomagnetic field magnitude in uT
	float ftrFitErrorpc;			        ///< trial value of fit error %
	float fA[3][3];							///< ellipsoid matrix A
	float finvA[3][3];						///< inverse of ellipsoid matrix A
	float fmatA[10][10];			        ///< scratch 10x10 float matrix used by calibration algorithms
	float fmatB[10][10];			        ///< scratch 10x10 float matrix used by calibration algorithms
	float fvecA[10];						///< scratch 10x1 vector used by calibration algorithms
	float fvecB[4];							///< scratch 4x1 vector used by calibration algorithms
	float fYTY;								///< Y^T.Y for 4 element calibration = (iB^2)^2
	int32_t iSumBs[3];						///< sum of measurements in buffer (counts)
	int32_t iMeanBs[3];						///< average magnetic measurement (counts)
	int32_t itimeslice;						///< counter for tine slicing magnetic calibration calculations
	int8_t iCalInProgress;			        ///< flag denoting that a calibration is in progress
	int8_t iNewCalibrationAvailable;	    ///< flag denoting that a new calibration has been computed
	int8_t iInitiateMagCal;			        ///< flag to start a new magnetic calibration
	int8_t iMagBufferReadOnly;		        ///< flag to denote that the magnetic measurement buffer is temporarily read only
	int8_t i4ElementSolverTried;		    ///< flag to denote at least one attempt made with 4 element calibration
	int8_t i7ElementSolverTried;		    ///< flag to denote at least one attempt made with 7 element calibration
	int8_t i10ElementSolverTried;		    ///< flag to denote at least one attempt made with 10 element calibration
	#endif
	#ifdef F_USING_SMARTSENSOR
	int16_t Radius ;
	uint8_t Mag_Calib_index;
	#endif
}MagCalibration;
//end of IMPORT from sensor_fusion/magnetic.h

/// The Magnetometer Measurement Buffer holds a 3-dimensional "constellation"
/// of data points.
///
/// The constellation of points are used to compute magnetic hard/soft iron compensation terms.
/// The contents of this buffer are updated on a continuing basis.
typedef struct MagBuffer
{
	int16_t iBs[3][MAGBUFFSIZEX][MAGBUFFSIZEY];		///< uncalibrated magnetometer readings
	int32_t index[MAGBUFFSIZEX][MAGBUFFSIZEY];		///< array of time indices
	int16_t tanarray[MAGBUFFSIZEX - 1];				///< array of tangents of (100 * angle)
	int16_t iMagBufferCount;						///< number of magnetometer readings
}MagBuffer;

/// \brief The TempSensor structure stores raw temperature readings
///
/// This may come from an FXOS8700, for example.
typedef struct TempSensor
{	uint8_t iWhoAmI;			///< sensor whoami
	bool isEnabled = false;
	float temperatureC;			// temperature in Celsius
}TempSensor;

typedef struct GyroCalibration {   //only used with smartsensors or sensors with this type of capability
	float 		fV[3];		/// gyroscope offset vector
	uint8_t 	Gyro_Calib_index;
}GyroCalibration;
/// \brief The GyroSensor structure stores raw and processed measurements for a 3-axis gyroscope.
///
/// The GyroSensor structure stores raw and processed measurements, as well as metadata
/// for a single 3-axis gyroscope.  This structure is
/// normally "fed" by the sensor driver and "consumed" by the fusion routines.
typedef struct GyroSensor
{
	uint8_t iWhoAmI;			///< sensor whoami
    bool  isEnabled;                        ///< true if the device is sampling
	uint8_t iFIFOCount;			///< number of measurements read from FIFO
    uint16_t iFIFOExceeded;                 ///< Number of samples received in excess of software FIFO size
	int16_t iYsFIFO[GYRO_FIFO_SIZE][3];	///< FIFO measurements (counts)
        // End of common fields which can be referenced via FifoSensor union type
	float fYs[3];				///< averaged measurement (deg/s)
	float fDegPerSecPerCount;		///< deg/s per count
	int16_t iCountsPerDegPerSec;		///< counts per deg/s
	int16_t iYs[3];				///< average measurement (counts)
}GyroSensor;

/// \brief The FifoSensor union allows us to use common pointers for Accel, Mag & Gyro logical sensor structures.
///
/// Common elements include: iWhoAmI, isEnabled, iFIFOCount, iFIFOExceeded and the FIFO itself.
typedef union FifoSensor  {
    struct GyroSensor Gyro;
    struct MagSensor  Mag;
    struct AccelSensor Accel;
}FifoSensor;
// HERE Inspire/wrap to include external BaroTemp sensor like BMP280
/// The SV_1DOF_P_BASIC structure contains state information for a pressure sensor/altimeter.
typedef struct SV_1DOF_P_BASIC
{
	float fLPH;				///< low pass filtered height (m)
	float fLPT;				///< low pass filtered temperature (C)
	float fdeltat;				///< fusion time interval (s)
	float flpf;				///< low pass filter coefficient
	int32_t systick;			///< systick timer
	int8_t resetflag;			///< flag to request re-initialization on next pass
}SV_1DOF_P_BASIC;

/// SV_9DOF_GBY_KALMAN is the 9DOF Kalman filter accelerometer, magnetometer and gyroscope state vector structure.
typedef struct SV_9DOF_GBY_KALMAN
{
	// start: elements common to all motion state vectors
	float fPhiPl;				///< roll (deg)
	float fThePl;				///< pitch (deg)
	float fPsiPl;				///< yaw (deg)
	float fRhoPl;				///< compass (deg)
	float fChiPl;				///< tilt from vertical (deg)
	float fRPl[3][3];			///< a posteriori orientation matrix
	Quaternion fqPl;			///< a posteriori orientation quaternion
	float fRVecPl[3];			///< rotation vector
	float fOmega[3];			///< average angular velocity (deg/s)
	int32_t systick;			///< systick timer;
	// end: elements common to all motion state vectors
	#ifndef F_USING_SMARTSENSOR
	float fQw9x9[9][9];			///< covariance matrix Qw
	float fK9x6[9][6];			///< kalman filter gain matrix K
	float fQwCT9x6[9][6];			///< Qw.C^T matrix
	float fZErr[6];				///< measurement error vector
	float fQv6x1[6];			///< measurement noise covariance matrix leading diagonal
	float fDeltaPl;				///< a posteriori inclination angle from Kalman filter (deg)
	float fsinDeltaPl;			///< sin(fDeltaPl)
	float fcosDeltaPl;			///< cos(fDeltaPl)
	float fqgErrPl[3];			///< gravity vector tilt orientation quaternion error (dimensionless)
	float fqmErrPl[3];			///< geomagnetic vector tilt orientation quaternion error (dimensionless)
	float fbPl[3];				///< gyro offset (deg/s)
	float fbErrPl[3];			///< gyro offset error (deg/s)
	#endif
	float fAccGl[3];			///< linear acceleration (g) in global frame
	#ifndef F_USING_SMARTSENSOR
	float fVelGl[3];			///< velocity (m/s) in global frame
	float fDisGl[3];			///< displacement (m) in global frame
	float fdeltat;				///< sensor fusion interval (s)
	float fgdeltat;				///< g (m/s2) * fdeltat
	float fAlphaOver2;			///< PI / 180 * fdeltat / 2
	float fAlphaSqOver4;			///< (PI / 180 * fdeltat)^2 / 4
	float fAlphaSqQvYQwbOver12;		///< (PI / 180 * fdeltat)^2 * (QvY + Qwb) / 12
	float fAlphaQwbOver6;			///< (PI / 180 * fdeltat) * Qwb / 6
	float fQwbOver3;			///< Qwb / 3
	float fMaxGyroOffsetChange;		///< maximum permissible gyro offset change per iteration (deg/s)
	int8_t iFirstAccelMagLock;		///< denotes that 9DOF orientation has locked to 6DOF eCompass
	int8_t resetflag;			///< flag to request re-initialization on next pass
	#endif
	#ifdef F_USING_SMARTSENSOR
	float fgPl[3];				///< gravity vector calculated by IMU chip
	#endif
}SV_9DOF_GBY_KALMAN;

/// Excluding SV_1DOF_P_BASIC, Any of the SV_ fusion structures above could
/// be cast to type SV_COMMON for dereferencing.
typedef struct SV_COMMON {
	float fPhi;				///< roll (deg)
	float fThe;				///< pitch (deg)
	float fPsi;				///< yaw (deg)
	float fRho;				///< compass (deg)
	float fChi;				///< tilt from vertical (deg)
	float fRM[3][3];			///< orientation matrix
	Quaternion fq;			        ///< orientation quaternion
	float fRVec[3];			        ///< rotation vector
	float fOmega[3];			///< average angular velocity (deg/s)
	int32_t systick;			///< systick timer;
}SV_COMMON;

typedef struct SV_COMMON *SV_ptr;

/// \brief The top level fusion structure
///
/// The top level fusion structure grows/shrinks based upon flag definitions
/// contained in build.h.  These same flags will populate the .iFlags field for
/// run-time access.
typedef struct SensorFusionGlobals
{
	// Subsystem Pointers
        ///@{
        /// @name SubsystemPointers
        /// The Status and Control subsystems can be used as-is, or completely
        /// replaced with alternate implementations, as long as those implementations
        /// provide the same interfaces defined in control.h and status.h.
	//struct ControlSubsystem *pControlSubsystem;
	struct StatusSubsystem *pStatusSubsystem;
        ///@}
        ///@{
        /// @name MiscFields
    uint32_t iFlags;           		  	///< a bit-field of sensors and algorithms used
	struct PhysicalSensor *pSensors;	///< a linked list of physical sensors
	volatile uint8_t iPerturbation;     ///< test perturbation to be applied
	// Book-keeping variables
	int32_t loopcounter;			///< counter incrementing each iteration of sensor fusion (typically 25Hz)
	int32_t systick_I2C;			///< systick counter to benchmark I2C reads
	int32_t systick_Spare;			///< systick counter for counts spare waiting for timing interrupt
        ///@}
        ///@{
        /// @name SensorRelatedStructures
        /// These structures provide homes for sensor readings, as well as
        /// various calibration functions.  Only those needed for a specific
        /// build are included.
#ifdef F_USING_SMARTSENSOR
	uint8_t		IMU_Calib_index;
	bool		isFusionStarted;
#endif
#ifdef     F_1DOF_P_BASIC
	struct PressureSensor	Pressure;			   ///< pressure sensor storage
#endif
#ifdef     F_USING_ACCEL
	struct AccelSensor 		Accel;                 ///< accelerometer storage
	struct AccelCalibration AccelCal;              ///< structures for accel calibration
	struct AccelBuffer 		AccelBuffer;           ///< storage for points used for calibration
#endif
#ifdef     F_USING_MAG
	struct MagSensor 		Mag;                   ///< magnetometer storage
	struct MagCalibration 	MagCal;                ///< mag cal storage
	struct MagBuffer 		MagBuffer;             ///< mag cal constellation points
#endif
#ifdef    F_USING_GYRO
	struct GyroSensor 		Gyro;                  ///< gyro storage
#ifdef F_USING_SMARTSENSOR
		   GyroCalibration  GyroCal;               ///< gyro cal storage
#endif
#endif
    struct TempSensor 		Temp;				   ///<temperature storage

        ///@}
        ///@{
        /// @name FusionSpecificStructures
#if       F_1DOF_P_BASIC
	struct SV_1DOF_P_BASIC SV_1DOF_P_BASIC;        ///< Pressure
#endif
#if       F_3DOF_G_BASIC
	struct SV_3DOF_G_BASIC SV_3DOF_G_BASIC;        ///< Gravity
#endif
#if       F_3DOF_B_BASIC
	struct SV_3DOF_B_BASIC SV_3DOF_B_BASIC;        ///< Magnetic
#endif
#if       F_3DOF_Y_BASIC
	struct SV_3DOF_Y_BASIC SV_3DOF_Y_BASIC;        ///< Gyro
#endif
#if       F_6DOF_GB_BASIC            // 6DOF accel and mag eCompass: (accel + mag)
	struct SV_6DOF_GB_BASIC SV_6DOF_GB_BASIC;      ///< eCompass (Gravity + Magnetic)
#endif
#if       F_6DOF_GY_KALMAN
	struct SV_6DOF_GY_KALMAN SV_6DOF_GY_KALMAN;    ///< 6-axis gravity + angular rates Kalman storage
#endif
#if       F_9DOF_GBY_KALMAN 
    struct SV_9DOF_GBY_KALMAN SV_9DOF_GBY_KALMAN;  ///< 9-axis storage
#endif
        ///@}
        ///@{
        /// @name FunctionPointers
        /// Function pointers (the SF library external interface)
	installSensor_t 	      *installSensor;           ///< function for installing a new sensor
	initializeFusionEngine_t  *initializeFusionEngine;  ///< set sensor fusion structures to initial values
	applyPerturbation_t       *applyPerturbation;	    ///< apply step function for testing purposes
	readSensors_t		      *readSensors;		        ///< read all physical sensors
	runFusion_t		          *runFusion;				///< run the fusion routines
    conditionSensorReadings_t *conditionSensorReadings; ///< preprocessing step for sensor fusion
    clearFIFOs_t              *clearFIFOs;              ///< clear sensor FIFOs
	setStatus_t				  *setStatus;				///< change status indicator immediately
	setStatus_t				  *queueStatus;  	        ///< queue status change for next regular interval
	updateStatus_t			  *updateStatus; 			///< status=next status
	updateStatus_t			  *testStatus; 				///< increment to next enumerated status value (test only)
  	getStatus_t 			  *getStatus;				///< fetch the current status from the Status Subsystem

        ///@}
} SensorFusionGlobals;

// The following functions are defined in sensor_fusion.c
void initSensorFusionGlobals(
    SensorFusionGlobals *sfg,                           ///< Global data structure pointer
    struct StatusSubsystem *pStatusSubsystem           ///< Status subsystem pointer
    //struct ControlSubsystem *pControlSubsystem          ///< Control subsystem pointer
);
installSensor_t installSensor;
initializeFusionEngine_t initializeFusionEngine ;
/// conditionSensorReadings() transforms raw software FIFO readings into forms that
/// can be consumed by the sensor fusion engine.  This include sample averaging
/// and (in the case of the gyro) integrations, applying hardware abstraction layers,
/// and calibration functions.
/// This function is normally involved via the "sfg." global pointer.
void conditionSensorReadings(
    SensorFusionGlobals *sfg                            ///< Global data structure pointer
);
void clearFIFOs (
    SensorFusionGlobals *sfg                            ///< Global data structure pointer
);
runFusion_t runFusion;
readSensors_t readSensors;
void zeroArray(
    struct StatusSubsystem *pStatus,                    ///< Status subsystem pointer
    void* data,                                         ///< pointer to array to be zeroed
    uint16_t size,                                      ///< data type size = 8, 16 or 32
    uint16_t numElements,                               ///< number of elements to zero out
    uint8_t check                                       ///< true if you would like to verify writes, false otherwise
);
/// \brief conditionSample ensures that we never encounter the maximum negative two's complement
/// value for a 16-bit variable (-32768).
///
/// conditionSample ensures that we never encounter the maximum negative two's complement
/// value for a 16-bit variable (-32768).  This value cannot be negated, because the maximum
/// positive value is +32767.  We need the ability to negate to gaurantee that subsequent
/// HAL operations can be run successfully.
void conditionSample(
    int16_t sample[3]                                   ///< 16-bit register value from triaxial sensor read
);

/// \brief addToFifo is called from within sensor driver read functions
///
/// addToFifo is called from within sensor driver read functions to transfer new readings into
/// the sensor structure corresponding to accel, gyro or mag.  This function ensures that the software
/// FIFOs are not overrun.
///
/// example usage: if (status==SENSOR_ERROR_NONE) addToFifo((FifoSensor*) &(sfg->Mag), MAG_FIFO_SIZE, sample);
void addToFifo(
    union FifoSensor *sensor,                                 ///< pointer to structure of type AccelSensor, MagSensor or GyroSensor
    uint16_t maxFifoSize,                               ///< the size of the software (not hardware) FIFO
    int16_t sample[3]                                   ///< the sample to add
);

// The following functions are defined in hal_axis_remap.c
// Please note that these are board-dependent - they account for 
//various orientations of sensor ICs on the sensor PCB.

/// \brief Apply the accelerometer Hardware Abstraction Layer
void ApplyAccelHAL(
    struct AccelSensor *Accel                                  ///< pointer to accelerometer logical sensor
);
/// \brief Apply the magnetometer Hardware Abstraction Layer
void ApplyMagHAL(
    struct MagSensor *Mag                                     ///< pointer to magnetometer logical sensor
);
/// \brief Apply the gyroscope Hardware Abstraction Layer
void ApplyGyroHAL(
    struct GyroSensor *Gyro                                    ///< pointer to gyroscope logical sensor
);
/// \brief ApplyPerturbation is a reverse unit-step test function
///
/// The ApplyPerturbation function applies a user-specified step function to
/// prior fusion results which is then "released" in the next fusion cycle.
/// When used in conjuction with the NXP Sensor Fusion Toolbox, this provides
/// a visual indication of the dynamic behavior of the library. ApplyPerturbation()
/// is defined in fusion_testing.c.
applyPerturbation_t ApplyPerturbation;
// end of IMPORT
#ifdef __cplusplus
}
#endif

#endif // SENSOR_FUSION_H
