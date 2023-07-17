/*
 * Copyright (c) 2020-2021, Bjarne Hansen
 * Copyright (c) 2023, Daniele Pazzaglia
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 *  \file sensor_fusion_class.ccp
 *
 *  @brief    A wrapper based on the "easy-to-use interface" originally built for
 *  the NXP Sensor Fusion version 7 library algorithms by Bjarne Hansen, adapted to
 *  work with the Adafruit BNO055 Library for the BOSH Accell,Gyroscope
 *  and magnetometer IC family BNO055 https://github.com/adafruit/Adafruit_BNO055
 *
 *  It is configured to work with the GY-91 breakout boards containing the 
 *  Invensense 925x magnetometer/accelerometer/gyroscope sensor ICs plus
 *  the Bosh BME280 ICs (interface to be added here)but can be modified to work
 *  with other sensors having an I2C interface. With additional modification,
 *  it can also work with SPI interface sensors (using specific driver/libraries).
 * 
 */



#include "sensor_fusion_class.h"

#include <Stream.h>
#include <stdint.h>
#include <Arduino.h>
#undef EXTERN_BARO_THERM_SENSOR


//#include "sensor_fusion/sensor_fusion.h"
//#include "sensor_fusion/control.h"
//#include "sensor_fusion/driver_sensors.h"
//#include "sensor_fusion/status.h"

const float kDegToRads = PI / 180.0;   ///< To convert Degrees to Radians, multiply by this constant.
const float kCelsiusToKelvin = 273.15; ///< To convert degrees C to K, add this constant.
const float kGeesToMPerSS = 9.80665;   ///< To convert acceleration in G to m/s^2, multiply by this constant.

// IMPORT SNIPPET from sensor_fusion/status.c

// bit-field definitions
#define N   0x00                // No color
#define R   0x04                // Red LED
#define G   0x02                // Green LED
#define B   0x01                // Blue LED

// set RGB LED as a function of bit-field values
void ssSetLeds(int8_t RGB)
{
    if (RGB & R) {
        LED_RED_ON();
    }else{
        LED_RED_OFF();
    }
    if (RGB & G) {
        LED_GREEN_ON();
    }else {
        LED_GREEN_OFF();
    }
    if (RGB & B) {
        LED_BLUE_ON();
    }else {
        LED_BLUE_OFF();
    }
}

// Do an immediate status update
void ssSetStatusNow(StatusSubsystem *pStatus, fusion_status_t status)
{
    pStatus->status = status;
    pStatus->next = status;

    uint8_t blink = false;
    uint8_t RGB = N;

    // This is where we actually change the visual indicator
    switch (status)
    {
        case INITIALIZING:      // solid GREEN
            RGB = G;
            break;
        case NORMAL:            // blinking GREEN
            RGB = G;
            blink = true;
            break;
        case LOWPOWER:          // blinking YELLOW
            RGB = R|G;
            blink = true;
            break;
        case SOFT_FAULT:        // solid RED (usually momentary)
        case HARD_FAULT:        // solid RED
            RGB = R;
            break;
        default:                // default = off;
            RGB = N;
    }

    if ((!blink) | (status != pStatus->previous))
    {
        ssSetLeds(RGB);
        pStatus->toggle = true;
    }
    else
    {
        if (pStatus->toggle)
        {
            ssSetLeds(N);
        }
        else
        {
            ssSetLeds(RGB);
        }

        pStatus->toggle = !pStatus->toggle;
    }
    while (status == HARD_FAULT) ; // Never return on hard fault
    // while (status == SOFT_FAULT) ; // DEBUG ONLY Never return on soft fault
}

// Unit test for status sub-system
void ssTest(StatusSubsystem *pStatus)
{
    switch (pStatus->status)
    {
        case OFF:
            ssSetStatusNow(pStatus, INITIALIZING);
            break;
        case INITIALIZING:
            ssSetStatusNow(pStatus, LOWPOWER);
            break;
        case LOWPOWER:
            ssSetStatusNow(pStatus, NORMAL);
            break;
        case NORMAL:
            ssSetStatusNow(pStatus, SOFT_FAULT);
            break;
        /*case RECEIVING_WIRED:
            ssSetStatusNow(pStatus, RECEIVING_WIRELESS);
            break;
        case RECEIVING_WIRELESS:
            ssSetStatusNow(pStatus, SOFT_FAULT);
            break;*/
        case SOFT_FAULT:
            ssSetStatusNow(pStatus, HARD_FAULT);
            break;
        case HARD_FAULT:
            ssSetStatusNow(pStatus, OFF);
            break;
    }
}

// undefine these just in case some other library needs them
#undef N
#undef R
#undef G
#undef B


// queue up a status change (which will take place at the next updateStatus)
void ssQueueStatus(StatusSubsystem *pStatus, fusion_status_t status)
{
    pStatus->next = status;
}

// promote any previously queued status update
void ssUpdateStatus(StatusSubsystem *pStatus)
{
    pStatus->previous = pStatus->status;
    ssSetStatusNow(pStatus, pStatus->next);
}

// make an immediate update to the system status
void ssSetStatus (StatusSubsystem *pStatus, fusion_status_t status)
{
    pStatus->next = status;
    ssUpdateStatus(pStatus);
}

// return the system status
fusion_status_t ssGetStatus(StatusSubsystem *pStatus)
{
    return pStatus->status;
}

/// initializeStatusSubsystem() should be called once at startup to initialize the
/// data structure and to put hardware into the proper state for communicating status.
void initializeStatusSubsystem(StatusSubsystem *pStatus)
{
    pStatus->set = (ssSetStatus_t*) ssSetStatus;
    pStatus->get = (ssGetStatus_t*) ssGetStatus;
    pStatus->queue = (ssSetStatus_t*) ssQueueStatus;
    pStatus->update = ssUpdateStatus;
    pStatus->test = ssTest;
    pStatus->previous = OFF;
    pStatus->set(pStatus, (fusion_status_t) OFF);
    pStatus->queue(pStatus, OFF);
    pStatus->toggle = false;

    /* set initial values */
    LED_RED_INIT(LOGIC_LED_OFF);
    LED_GREEN_INIT(LOGIC_LED_OFF);
    LED_BLUE_INIT(LOGIC_LED_OFF);
}
//END OF IMPORT from sensor_fusion/status.c

/*
 * Constructor creates and initializes a structure of variables used throughout
 * the functions. It initializes the control and status subsystems as well as
 * several subordinate data structures.
 */
// Adaptation to sensors family capable of embedded sensor fusion algorithms ("smart" AHRS sensors)
/* Eliminate or rework of code:
     1 - InitializeInputOutputSubsystem();          because the SignalK is the communication framework
                                                    and the implementation was strictly bound
                                                    to NX Freescale controlling tools
     2 - control_subsystem_ = new ControlSubsystem; because the libraries for "smart" AHRS sensors have
                                                    generally an high level interface hiding busio details
                            
*/     

SensorFusion::SensorFusion() {
  sfg_ = new SensorFusionGlobals();
  //control_subsystem_ = new ControlSubsystem;
  status_subsystem_ = new StatusSubsystem;
  //auto sensor_ = new OR_sensor (-1, BNO055_ADDRESS_B, &Wire );   ---> TO BE DEFINED AT LOWER LEVEL
  bool sensor_initialized = false;
  sensors_ = new PhysicalSensor
      [MAX_NUM_SENSORS];  // this implementation uses up to 5 sensors
                          // (accel/mag/gyro; internal temp or external baro/thermo)
                          // TODO - can use std::vector as per
                          // https://stackoverflow.com/questions/14114686/create-an-array-of-structure-using-new
                          // which will allow adding arbitrary # of sensors.

  //InitializeInputOutputSubsystem();
  InitializeStatusSubsystem();
  InitializeSensorFusionGlobals();

}  // end SensorFusion()

/**
 * @brief Install Sensor in linked list
 * The max length of the list is checked, and if there is room, the 
 * given sensor is inserted at the head of the list.
 * An accelerometer and magnetometer may be combined in one IC - if
 * that is the case then only one call is required to install, 
 * provided the associated *_Init() and *_Read() function reads both
 * the accel & magnetometer data.  The Init() and Read() functions 
 * of each sensor are defined in driver_*.* files.
 * @param sensor_i2c_addr is the I2C bus address of the sensor IC
 * @param sensor_type indicates the type of sensor (e.g. magnetometer)
 * @return True if sensor installed successfully, else False
 */
bool SensorFusion::InstallSensor(uint8_t sensor_i2c_addr,
                                   SensorType sensor_type) {

    if( num_sensors_installed_ >= MAX_NUM_SENSORS ) {
        //already have max number of sensors installed
      return false;
    }
    switch (sensor_type) {
    case SensorType::kAccelerometer:
      sfg_->installSensor(sfg_, &sensors_[num_sensors_installed_],
                          sensor_i2c_addr, kLoopsPerAccelRead, NULL,
                          Accel_Init, Accel_Read);
      ++num_sensors_installed_;
      break;
    case SensorType::kMagnetometer:
      sfg_->installSensor(sfg_, &sensors_[num_sensors_installed_],
                          sensor_i2c_addr, kLoopsPerMagRead, NULL,
                          Mag_Init, Mag_Read);
      ++num_sensors_installed_;
      break;
    /*case SensorType::kMagnetometerAccelerometer:
      sfg_->installSensor(sfg_, &sensors_[num_sensors_installed_],
                          sensor_i2c_addr, kLoopsPerAccelRead, NULL,
                          Combo_Accel_Mag_Init, Combo_Accel_Mag_Read);
      ++num_sensors_installed_;
      break;
    case SensorType::kMagnetometerAccelerometerGyroscope:
      sfg_->installSensor(sfg_, &sensors_[num_sensors_installed_],
                          sensor_i2c_addr, kLoopsPerAccelRead, NULL,
                          Combo_Accel_Mag_Gyro_Init, Combo_Accel_Mag_Gyro_Read);
      ++num_sensors_installed_;
      break;
    */
    case SensorType::kGyroscope:
      sfg_->installSensor(sfg_, &sensors_[num_sensors_installed_],
                          sensor_i2c_addr, kLoopsPerGyroRead, NULL,
                          Gyro_Init, Gyro_Read);
      ++num_sensors_installed_;
      break;
    case SensorType::kThermometer:
      // use the thermometer built into Combo. Not precise nor calibrated,
      // but OK.
      sfg_->installSensor(sfg_, &sensors_[num_sensors_installed_],
                          sensor_i2c_addr, kLoopsPerThermRead, NULL,
                          Therm_Init, Therm_Read);
      ++num_sensors_installed_;
      break;
#ifdef EXTERN_BARO_THERM_SENSOR     
    case SensorType::kBarometer:
      sfg_->installSensor(sfg_, &sensors_[num_sensors_installed_],
                          sensor_i2c_addr, kLoopsPerBaroRead, NULL,
                          Baro_Init, Baro_Read);
      break;
#endif      
    default:
      // unrecognized sensor type
      break;
  }
  return true;
}  // end InstallSensor()

/**
 * Initialize the Control subsystem, which receives external commands and sends
 * data packets.
 * @param serial_port A Stream pointer to which to send output and receive commands. If
 * NULL, then output is not attempted.
 * @param tcp_client A WiFiClient pointer to which to send output and receive commands. If
 * NULL, then output is not attempted.
 * @return True if the Control subsystem is initialized, else False.
 */
/*bool SensorFusion::InitializeInputOutputSubsystem(const Stream *serial_port,
                                                  const void *tcp_client) {
  return initializeIOSubsystem(control_subsystem_, serial_port, tcp_client);
}  // end InitializeInputOutputSubsystem()
*/
/**
 * Initialize the Sensors. Read calibrations. Set status to Normal.
 */
void SensorFusion::Begin(int pin_i2c_sda, int pin_i2c_scl) {
  sfg_->initializeFusionEngine(
      sfg_, pin_i2c_sda, pin_i2c_scl);                      // Initialize sensors and magnetic calibration
  sfg_->setStatus(sfg_, (fusion_status_t) NORMAL);  // Set status state to NORMAL
//TODO - setStatus should check whether initialize worked.

}  // end InitializeFusionEngine()

/**
 * @brief Update the TCP client pointer.
 * Call when a new TCP connection is made, as reported by WiFiServer::available()
 * When tcp_client is NULL, output via WiFi is not attempted.
 * @param tcp_client A WiFiClient pointer used for input/output
 */
/*
void SensorFusion::UpdateWiFiStream(void *tcp_client) {
  UpdateTCPClient(control_subsystem_, tcp_client);

}  // end UpdateTCPClient()
*/
/**
 * @brief Reads all sensors.
 * Applies HAL remapping, removes invalid values, and stores data for 
 * later processing. Depending on whether a sensor has a built-in FIFO
 * buffer, that sensor may not be read each time this method is called.
 * See kLoopsPerMagRead, etc., in sensor_fusion_class.h
 */
void SensorFusion::ReadSensors(void) {
  sfg_->readSensors(
      sfg_,
      loops_per_fuse_counter_);  // Reads sensors, applies HAL, removes -32768

}  // end ReadSensors()

/**
 * @brief Apply fusion algorithm to sensor raw data.
 * Sensor readings contained in global struct are calibrated and processed.
 * Status is updated and displayed.
 * Loop counter used for coordinating sensor reads is reset.
 * 
 */
void SensorFusion::RunFusion(void) {
  // applies fusion algorithm to data accumulated in buffers

  // only run fusion every kLoopsPerFusionCalc'th time through loop
  if (loops_per_fuse_counter_ < kLoopsPerFusionCalc) {
    ++loops_per_fuse_counter_;
    return;
  }

  sfg_->conditionSensorReadings(sfg_);  // Pre-processing; Magnetic calibration.
  sfg_->runFusion(sfg_);                // Run fusion algorithms

  // Serial.printf(" Algo took %ld us\n", sfg.SV_9DOF_GBY_KALMAN.systick);
  sfg_->loopcounter++;  // loop counter is used to "serialize" mag cal
                        // operations and blink LEDs to indicate status
  // LED Blinking is too fast unless status updates are slowed down.
  // Cycle at least four times per status update.
  // TODO - a better way might be to use a timer.
  if (0 == sfg_->loopcounter % 4) {
    sfg_->updateStatus(sfg_);  // make pending status updates visible
  }

  // assume NORMAL status next pass through the loop
  // this resets temporary error conditions (SOFT_FAULT)
  sfg_->queueStatus(sfg_, (fusion_status_t) NORMAL);

  loops_per_fuse_counter_ = 1;  // reset loop counter

}  // end RunFusion()

/**
 * @brief Generate and send out data, formatted for NXP Orientation Sensor Toolbox.
 * It is not mandatory to call this routine, if Toolbox output is not needed.
 */
/*void SensorFusion::ProduceToolboxOutput(void) {
  // Make & send data to Sensor Fusion Toolbox or whatever UART is
  // connected to.
  if (loops_per_fuse_counter_ == 1) {       // only run if fusion has happened
    sfg_->pControlSubsystem->stream(sfg_);  // create output packet
    sfg_->pControlSubsystem->write(sfg_);   // send output packet
  }

}  // end ProduceToolboxOutput()
*/

/**
 * places data from buffer into Control subsystem's output buffer, and sends
 * it out via serial and/or wifi.  Any existing data in the output buffer
 * that hasn't already been sent will be overwritten.
 * Returns true on success, false on problem such as data_length too long
 * for the transmit buffer.
 */
/*bool SensorFusion::SendArbitraryData(const char *buffer, uint16_t data_length) {
  if (data_length > MAX_LEN_SERIAL_OUTPUT_BUF) {
    return false;
  }
  char *out_buf = (char *)(sfg_->pControlSubsystem->serial_out_buf);
  for (uint16_t i = 0; i < data_length; i++) {
    out_buf[i] = buffer[i];
  }
  sfg_->pControlSubsystem->bytes_to_send = data_length;
  sfg_->pControlSubsystem->write(sfg_);  // send output packet
  return true;
}  // end SendArbitraryData()
*/

/**
 * @brief Process any incoming commands.
 * Commands may arrive by serial or WiFi connection, depending on which of
 * these is enabled (if any).
 * It is not mandatory to call this routine, if command responses are not needed.
 */
/*void SensorFusion::ProcessCommands(void) {
  // process any incoming commands
  sfg_->pControlSubsystem->readCommands(sfg_);
}  // end ProcessCommands()
*/

/**
 * @brief Inject and process a single command in the control subsystem.
 *
 * Command provided by this call bypasses the usual input route (serial
 * UART or WiFi) and is instead provided directly in this function call.
 * This provides an alterative way of manipulating the fusion algortithm
 * without needing to use the entire control or input subsystem, For a
 * list of valid commands, see control_input.c in the Orientation library.
 *
 * @param command is a four-character sequence selected from
 * the list of valid commands. Illegal commands are ignored without
 * any action taken.
 */
/*void SensorFusion::InjectCommand(const char *command) {
  sfg_->pControlSubsystem->injectCommand(sfg_, (uint8_t *)command, 4);
}  // end InjectCommand()
*/

/**
 * @brief Save current magnetic calibration to non-volatile memory.
 *
 * Passes to the fusion control subsystem the command to save
 * the current magnetic calibration parameters to EEPROM. This
 * calibration will then be loaded each time the system restarts.
 * 
 * One can pass this command directly using InjectCommand(), but
 * SaveMagneticCalibration() is provided as a convenience.
 * 
 * The fusion system continuously runs a time-sliced calibration
 * routine in the background, and if a superior set of calibration
 * parameters are calculated based on recent magnetic measurements,
 * then these new parameters replace the current ones in RAM.
 * However, it is only via this command that the parameters can be
 * caused to persist beyond the next system reset.
 */
void SensorFusion::SaveMagneticCalibration(void) {
  //InjectCommand("SVMC");
}  // end SaveMagneticCalibration()

void SensorFusion::EraseMagneticCalibration(void) {
  //InjectCommand("ERMC");
}  // end SaveMagneticCalibration()


/**
 * @brief @return Boolean indicating whether orientation data are valid
 */
bool SensorFusion::IsDataValid(void) {
  if( NORMAL == sfg_->pStatusSubsystem->status ) {
    return true;
  } else {
    return false;
  }
}  // end IsDataValid()

/**
 * @brief @return Fusion System status
 *
 * System status is indicated by an integer. See sensor_fusion.h
 * for details. Common values are: 1 (Initializing); 3 (Normal);
 * and 7 (Soft Fault - usually caused by communications fault with
 * sensor IC)
 */
int SensorFusion::GetSystemStatus(void) {
  return (int)sfg_->pStatusSubsystem->status;
}  // end GetSystemStatus()

// The following Get____() methods return orientation values
// calculated by the 9DOF Kalman algorithm (the most advanced).
// They have been mapped to match the conventions used for
// vessels:
//  Compass Heading; 0 at magnetic north, and increasing CW.
//  Pitch; 0 with boat level, increasing with Bow Up.
//  Roll; 0 with boat level, increasing with Starboard roll.
//  Turn Rate; positive with turn to Starboard
//  Pitch Rate; positive with Bow moving Up.
//  Roll Rate; positive with increasing Starboard heel.
//  Acceleration; X to Bow, Y to Port, Z Up.
// This works with the Adafruit NXP FXOS8700/FXAS21002
//  breakout board, mounted with X toward the bow, Y to port,
//  and Z (component side of PCB) facing up.
// If the sensor orienatation is different than assumed,
//  you may need to remap the axes below.  If a different
//  sensor board is used, you may need also to change the
//  axes mapping in the file hal_axis_remap.c  The latter
//  mapping is applied *before* the fusion algorithm,
//  whereas the below mapping is applied *after*.

/**
 * @brief @return Return the Compass Heading in degrees
 */
float SensorFusion::GetHeadingDegrees(void) {
  // TODO - make generic so it's not dependent on algorithm used
  return (sfg_->SV_9DOF_GBY_KALMAN.fRhoPl <= 90)
             ? (sfg_->SV_9DOF_GBY_KALMAN.fRhoPl + 270.0)
             : (sfg_->SV_9DOF_GBY_KALMAN.fRhoPl - 90.0);
}  // end GetHeadingDegrees()

/**
 * @brief @return Return the Compass Heading in radians
 */
float SensorFusion::GetHeadingRadians(void) {
  return GetHeadingDegrees() * kDegToRads;
}  // end GetHeadingRadians()

/**
 * @brief @return Return the Pitch in degrees
 */
float SensorFusion::GetPitchDegrees(void) {
  return sfg_->SV_9DOF_GBY_KALMAN.fPhiPl;
}  // end GetPitchDegrees()

/**
 * @brief @return Return the Pitch in radians
 */
float SensorFusion::GetPitchRadians(void) {
  return GetPitchDegrees() * kDegToRads;
}  // end GetPitchRadians()

/**
 * @brief @return Return the Roll in degrees
 */
float SensorFusion::GetRollDegrees(void) {
  return -(sfg_->SV_9DOF_GBY_KALMAN.fThePl);
}  // end GetRollDegrees()

/**
 * @brief @return Return the Roll in radians
 */
float SensorFusion::GetRollRadians(void) {
  return GetRollDegrees() * kDegToRads;
}  // end GetRollRadians()

/**
 * @brief @return Return the Temperature in degrees C
 * 
 * This is the temperature of the sensor IC die. It is
 * not factory-trimmed, so it very likely has an offset
 * that should be calibrated out for best accuracy.
 */
float SensorFusion::GetTemperatureC(void) {
  return sfg_->Temp.temperatureC;
}  // end GetTemperatureC()

/**
 * @brief @return Return the Temperature in degrees K
 * 
 * This is the temperature of the sensor IC die. It is
 * not factory-trimmed, so it very likely has an offset
 * that should be calibrated out for best accuracy.
 */
float SensorFusion::GetTemperatureK(void) {
  return GetTemperatureC() + kCelsiusToKelvin;
}  // end GetTemperatureK()

float SensorFusion::GetPressureHpa(void) {
  return 0.0F;//sfg_->Pressure.Hpa;
}  // end GetPressureHpa()

/**
 * @brief @return Return the Turn Rate in degrees
 */
float SensorFusion::GetTurnRateDegPerS(void) {
  return sfg_->SV_9DOF_GBY_KALMAN.fOmega[2];
}  // end GetTurnRateDegPerS()

/**
 * @brief @return Return the Turn Rate in rad/s
 */
float SensorFusion::GetTurnRateRadPerS(void) {
  return GetTurnRateDegPerS() * kDegToRads;
}  // end GetTurnRateRadPerS()

/**
 * @brief @return Return the Pitch Rate in degrees/s
 */
float SensorFusion::GetPitchRateDegPerS(void) {
  return sfg_->SV_9DOF_GBY_KALMAN.fOmega[0];
}  // end GetPitchRateDegPerS()

/**
 * @brief @return Return the Pitch Rate in rad/s
 */
float SensorFusion::GetPitchRateRadPerS(void) {
  return GetPitchRateDegPerS() * kDegToRads;
}  // end GetPitchRateRadPerS()

/**
 * @brief @return Return the Roll Rate in degrees/s
 */
float SensorFusion::GetRollRateDegPerS(void) {
  return -(sfg_->SV_9DOF_GBY_KALMAN.fOmega[1]);
}  // end GetRollRateDegPerS()

/**
 * @brief @return Return the Roll Rate in rad/s
 */
float SensorFusion::GetRollRateRadPerS(void) {
  return GetRollRateDegPerS() * kDegToRads;
}  // end GetRollRateRadPerS()

/**
 * @brief @return Return the X-axis Acceleration in gees
 */
float SensorFusion::GetAccelXGees(void) {
  return sfg_->Accel.fGc[1];
}  // end GetAccelXGees()

/**
 * @brief @return Return the X-axis Acceleration in m/s^2
 */
float SensorFusion::GetAccelXMPerSS(void) {
  return GetAccelXGees() * kGeesToMPerSS;
}  // end GetAccelXMPerSS()

/**
 * @brief @return Return the Y-axis Acceleration in gees
 */
float SensorFusion::GetAccelYGees(void) {
  return sfg_->Accel.fGc[0];
}  // end GetAccelYGees()

/**
 * @brief @return Return the Y-axis Acceleration in m/s^2
 */
float SensorFusion::GetAccelYMPerSS(void) {
  return GetAccelYGees() * kGeesToMPerSS;
}  // end GetAccelYMPerSS()

/**
 * @brief @return Return the Z-axis Acceleration in gees
 */
float SensorFusion::GetAccelZGees(void) {
  return sfg_->Accel.fGc[2];
}  // end GetAccelZGees()

/**
 * @brief @return Return the Z-axis Acceleration in m/s^2
 */
float SensorFusion::GetAccelZMPerSS(void) {
  return GetAccelZGees() * kGeesToMPerSS;
}  // end GetAccelZMPerSS()

/**
 * @brief Return the orientation as a quaternion
 * @param quat pointer to quaternion structure, to be filled by this method
 */
void  SensorFusion::GetOrientationQuaternion(Quaternion *quat) {
  quat->q0 = sfg_->SV_9DOF_GBY_KALMAN.fqPl.q0;
  quat->q1 = sfg_->SV_9DOF_GBY_KALMAN.fqPl.q1;
  quat->q2 = sfg_->SV_9DOF_GBY_KALMAN.fqPl.q2;
  quat->q3 = sfg_->SV_9DOF_GBY_KALMAN.fqPl.q3;
}  // end GetOrientationQuaternion()

/**
 * @brief @return Return magnetic fit error of trial calibration
 * 
 * The background calibration task uses recent magnetic readings
 * to periodically calculate a new set of calibration parameters.
 * This function returns the measure of goodness of fit of the
 * new calibration parameter set. Units are in % and a value 
 * less than 3.5% is considered good. A value of 0 is returned
 * briefly during program startup, indicating that insufficient
 * data are available to calculate a fit. 
 */
float SensorFusion::GetMagneticFitErrorTrial(void) {
  return sfg_->MagCal.ftrFitErrorpc;
}  // end GetMagneticFitErrorTrial()

/**
 * @brief @return Return magnetic fit error of current calibration
 * 
 * This function returns the measure of the goodness of fit of the
 * currently-used calibration parameter set. Units are in % and a
 * value less than 3.5% is considered good. If no calibration is
 * available on startup (i.e. none saved in EEPROM) then a value
 * of 0 is returned briefly during program startup, indicating 
 * that insufficient data are available to calculate a fit. 
 */

float SensorFusion::GetMagneticFitError(void) {
  return sfg_->MagCal.fFitErrorpc;
}  // end GetMagneticFitError()

/**
 * @brief @return Return geomagnetic field magnitude in uT of  trial calibration.
 *
 * This value is the current best-estimate of the B field using
 * recent magnetic sensor readings. It updates with each new sensor
 * reading added to the buffer. If the new trial calibration
 * parameters are accepted and replace the current calibration, then
 * this B field becomes associated with that set.
 *
 * This parameter provides an indication of magnetic interference;
 * if it differs widely from the B value of the current calibration
 * then one can conclude that the current calibration is not valid
 * for the present magnetic environment.
 */

float SensorFusion::GetMagneticBMagTrial(void) {
  return sfg_->MagCal.ftrB;
}  // end GetMagneticBMagTrial()

/**
 * @brief @return Return geomagnetic field magnitude in uT of
 * current calibration.
 *
 * This value does not change with each sensor reading - it only
 * updates if a new calibration parameter set is calculated and
 * becomes the currently-used calibration.
 */

float SensorFusion::GetMagneticBMag(void) {
  return sfg_->MagCal.fB;
}  // end GetMagneticBMag()

/**
 * @brief @return Return geomagnetic field inclination from horizontal in
 * degrees.
 * 
 * This is the a posteriori value from Kalman filter output and
 * updates with each sensor reading. It can be used as an
 * indication of magnetic interference; if its value changes
 * by more than, say, 10 degrees from recent previous values
 * then it indicates a disturbance in the magnetic environment.
 * 
 * Magnetic inclination values vary over the earth's surface
 * and are available from sources such as NOAA.
 */
float SensorFusion::GetMagneticInclinationDeg(void) {
  return sfg_->SV_9DOF_GBY_KALMAN.fDeltaPl;
}  // end GetMagneticInclinationDeg()

/**
 * @brief @return Return geomagnetic field inclination from horizontal in
 * radians.
 * 
 * This is the a posteriori value from Kalman filter output and
 * updates with each sensor reading. It can be used as an
 * indication of magnetic interference; if its value changes
 * by more than, say, 10 degrees from recent previous values
 * then it indicates a disturbance in the magnetic environment.
 * 
 * Magnetic inclination values vary over the earth's surface
 * and are available from sources such as NOAA.
 */
float SensorFusion::GetMagneticInclinationRad(void) {
  return GetMagneticInclinationDeg() * kDegToRads;
}  // end GetMagneticInclinationRad()

/**
 * @brief @return Return measurement noise covariance value for magnetic
 * field.
 *
 * This is proportional to how heavily the magnetic readings are weighted in
 * calculating the a posteriori orientation. This is a good measure of magnetic
 * interference; when the noise covariance value rises above about
 * 0.00056 it indicates the current magnetic reading does not lie
 * close to the surface of the calibrated geomagnetic sphere.
 */
float SensorFusion::GetMagneticNoiseCovariance(void) {
  return sfg_->SV_9DOF_GBY_KALMAN.fQv6x1[3];
}  // end GetMagneticNoiseCovariance()

/**
 * @brief @return Return solver used for current calibration [0,4,7,10]
 * 
 * The solver is the number of elements (complexity) of the calibration
 * algorithm. 0 means no calibration; 10 is the most complex.
 */
/*
float SensorFusion::GetMagneticCalSolver(void) {
  return (float)(sfg_->MagCal.iValidMagCal);
}  // end GetMagneticCalSolver()
*/
//================= end of Get____() methods ==================
//================= start of private methods ==================

/**
 * @brief Initialize the Status reporting system.
 * Status is indicated by LEDs, but other means can be added.
 */
void SensorFusion::InitializeStatusSubsystem(void) {
  initializeStatusSubsystem(
      status_subsystem_);  // configure the status sub-system

}  // end InitializeStatusSubsystem()

/**
 * @brief Set the starting values of variables contained in sfg_
 */
void SensorFusion::InitializeSensorFusionGlobals(void) {
  initSensorFusionGlobals(sfg_, status_subsystem_/*, control_subsystem_*/);

}  // end InitializeSensorFusionGlobals()