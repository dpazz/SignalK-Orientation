/** @file orientation_sensor.cpp
 *  @brief Orientation sensor interface to SensESP
 */

#include "orientation_sensor.h"

#include "sensesp.h"

namespace sensesp {
  
/**
 * @brief Constructor sets up the I2C communications to the sensor and
 * initializes the sensor fusion library.
 *
 * @param pin_i2c_sda Pin of SDA line to sensor. Use -1 for Arduino default.
 * @param pin_i2c_scl Pin of SCL line to sensor. Use -1 for Arduino default.
 * @param bus_i2c_addr structure containing i2c addresses of various sensor combo alternatives:
 *  1st element: i2C address of combo Accel/Mag/Gyro or combo Accel/Mag
 *  2nd element: i2c address of gyro sensor (if present alone)
 *  3rd element: i2c address of external combo Baro/Therm sensor (if present)
 * Superseded_@param accel_mag_i2c_addr I2C address of accelerometer/magnetometer IC.
 * Superseded_@param gyro_i2c_addr I2C address of gyroscope IC.
 * Superseded_@param accel_mag_gyro_i2c_addr I2C address of accelerometer/magnetometer/gyro IC.
 * Superseded_@param baro_therm_i2c_addr I2C address of baro/therm IC
 * @param config_path RESTful path by which sensor can be configured.
 */

OrientationSensor::OrientationSensor(uint8_t pin_i2c_sda, uint8_t pin_i2c_scl,
                                     orsensor_bus_addresses bus_i2c_addr, sensor_cap cap)
  {
//bus_i2c_addr convention:
//                          if using single sensors the first three positions are for (in order)
//                                  Accelerometer/Magnetometer/Gyroscope single sensor i2c addresses
//                          If using combo sensors the first position is for Combo Accel/Mag or
//                                  Combo Accel/MAg/Gyro i2c address
//                          the fourth position is reserved for i_2c_addr of Combo Baro/Therm sensor (like BMP280)

  //copy i2c address vector elements to local variables 
  //sensor_cap type is enum but biwise collects the available sensor capabilities for this build
  
  sensor_interface_ = new SensorFusion();  // create our fusion engine instance

  bool success =false;
  // init IO subsystem, passing NULLs since we use Signal-K output instead.
  
      //sensor_interface_->InitializeInputOutputSubsystem(NULL, NULL) && ****controlsubsystem eliminated
      //                                                                     since we use Signal-K output instead.

      // connect to the sensors.  Remember that some sensor can be of combo type (see Orientation_sensor.h).
      switch (cap){
        case AMT:
        case AMT_PLUS_BT://  it is supposed that if sensor is of Combo Accel/Mag type there are also the companion gyro
                         //  in order to obtain the accel/gyro/mag fusion
        success = sensor_interface_->InstallSensor(bus_i2c_addr.addr[0],
                                       SensorType::kMagnetometerAccelerometer) &&
                  sensor_interface_->InstallSensor(bus_i2c_addr.addr[2],
                                       SensorType::kGyroscope);
        break;
        case AMGT:
        case AMGT_PLUS_BT:
        success = sensor_interface_->InstallSensor(bus_i2c_addr.addr[0],
                                       SensorType::kMagnetometerAccelerometerGyroscope);
        break;
        default : //  it is supposed that if sensor is of "single" type there are also the companion singles
                  //  in order obtain the accel/gyro/mag fusion
        success= sensor_interface_->InstallSensor(bus_i2c_addr.addr[0],
                                       SensorType::kMagnetometer) &&
                  sensor_interface_->InstallSensor(bus_i2c_addr.addr[1],
                                       SensorType::kAccelerometer) &&
                  sensor_interface_->InstallSensor(bus_i2c_addr.addr[2], 
                                       SensorType::kGyroscope); 
        break;
      }
                                 
#ifndef EXTERN_BARO_THERM_SENSOR
      // A thermometer (uncalibrated) is available in the
      // accelerometer/magnetometer IC. Install it as if it was a separate sensor
      // only if the thermometer selected bit is on in "cap" mask field
      // in other words the therm sensor is installed as if it was "single"
      // but using the internal  temp  of combo
      if (cap & THERM_ONLY) {
        success &= sensor_interface_->InstallSensor(bus_i2c_addr.addr[0],
                                       SensorType::kThermometer) ;
      }
#endif
#ifdef EXTERN_BARO_THERM_SENSOR
      // if in the (breakout or main board) hardware solution is available 
      // an enviromental sensor IC like BMP280 .
      if ((cap & THERM_ONLY) && (cap & BARO_ONLY)) success &= sensor_interface_->InstallSensor(bus_i2c_addr.addr[3],
                                       SensorType::kThermometer) &&
                                                  sensor_interface_->InstallSensor(bus_i2c_addr.addr[3],
                                       SensorType::kBarometer);
#endif
      
  if (!success) {
    debugE("Trouble installing sensors.");
  } else {
    sensor_interface_->Begin(pin_i2c_sda, pin_i2c_scl);
    debugI("Sensors connected & Fusion ready");

    // The Fusion Library, in build.h, defines how fast the ICs generate new
    // orientation data and how fast the fusion algorithm runs, using FUSION_HZ.
    // Usually this rate should be the same as ReadAndProcessSensors() is
    // called.
    const uint32_t kFusionIntervalMs = 1000.0 / FUSION_HZ;
    // Start periodic reads of sensor and running of fusion algorithm.
    ReactESP::app->onRepeat(kFusionIntervalMs,
                 [this]() { this->ReadAndProcessSensors(); });
  }

}  // end OrientationSensor()

/**
 * @brief Read the Sensors and calculate orientation parameters
 */
void OrientationSensor::ReadAndProcessSensors (void) {
  sensor_interface_->ReadSensors();
  sensor_interface_->RunFusion();

} // end ReadAndProcessSensors()

/**
 * @brief Constructor sets up the frequency of output and the Signal K path.
 *
 * @param orientation_sensor Pointer to the physical sensor's interface
 * @param report_interval_ms Interval between output reports
 * @param config_path RESTful path by which reporting frequency can be
 * configured.
 */
AttitudeValues::AttitudeValues(OrientationSensor* orientation_sensor,
                               uint report_interval_ms, String config_path)
    : Sensor(config_path),
      orientation_sensor_{orientation_sensor},
      report_interval_ms_{report_interval_ms} {
  load_configuration();
  save_mag_cal_ = 0;
}  // end AttitudeValues()

/**
 * @brief Starts periodic output of Attitude parameters.
 *
 * The start() function is inherited from sensesp::Sensor, and is
 * automatically called when the SensESP app starts.
 */
void AttitudeValues::start() {
  ReactESP::app->onRepeat(report_interval_ms_, [this]() { this->Update(); });
}

/**
 * @brief Provides one Attitude reading from the orientation sensor.
 *
 * Readings are obtained using the sensor fusion library's Get_() methods
 * and assigned to the output variable that passes data from Producers
 * to Consumers. Consumers of the attitude data are then informed
 * by the call to notify(). If data are not valid (e.g. sensor not
 * functioning), a struct member is set to false so when the Signal K
 * message contents are assembled by as_signalk(),they can reflect that. 
 */
void AttitudeValues::Update() {
  //check whether magnetic calibration has been requested to be saved or deleted
  if( 1 == save_mag_cal_ ) {
    orientation_sensor_->sensor_interface_-> ReadCalibrationData ();      //in smartsensor calib data are stored in sensor registers
    orientation_sensor_->sensor_interface_-> SaveMagneticCalibration ();  //in smartsensor case calib is saved for all sensors 
  }else if( -1 == save_mag_cal_ ) {
    orientation_sensor_->sensor_interface_-> EraseMagneticCalibration (); //in smartsensor case calib is erased for all sensors
  }
  save_mag_cal_ = 0;  // set flag back to zero so we don't repeat save/delete
  attitude_.is_data_valid =
      orientation_sensor_->sensor_interface_->IsDataValid();
  attitude_.yaw = 
      orientation_sensor_->sensor_interface_->GetHeadingRadians();
  attitude_.roll =
      orientation_sensor_->sensor_interface_->GetRollRadians();
  attitude_.pitch =
      orientation_sensor_->sensor_interface_->GetPitchRadians();

  output = attitude_;
  notify();
}  // end Update()

/**
 * @brief Define the format for the Orientation value producers.
 *
 * This format is common to the single orientation parameter producers
 * (OrientationValues objects) and the attitude parameter producers
 * (AttitudeValues objects).
 */
static const char SCHEMA[] PROGMEM = R"###({
    "type": "object",
    "properties": {
        "report_interval": { 
          "title": "Report Interval", 
          "type": "number", 
          "description": "Milliseconds between outputs of this parameter" 
        },
        "save_mag_cal": { 
          "title": "Save Magnetic (or all sensor types if the sensor is smart) Cal", 
          "type": "number", 
          "description": "Set to 1 to save current calibration" 
        }
    }
  })###";

/**
 * @brief Get the current sensor configuration and place it in a JSON
 * object that can then be stored in non-volatile memory.
 * 
 * @param doc JSON object to contain the configuration parameters
 * to be updated.
 */
void AttitudeValues::get_configuration(JsonObject& doc) {
  doc["report_interval"] = report_interval_ms_;
  doc["save_mag_cal"] = save_mag_cal_;
}  // end get_configuration()

/**
 * @brief Fetch the JSON format used for holding the configuration.
 */
String AttitudeValues::get_config_schema() { return FPSTR(SCHEMA); }

/**
 * @brief Use the values stored in JSON object config to update
 * the appropriate member variables.
 *
 * @param config JSON object containing the configuration parameters
 * to be updated.
 * @return True if successful; False if a parameter could not be found.
 */
bool AttitudeValues::set_configuration(const JsonObject& config) {
  String expected[] = {"report_interval", "save_mag_cal"};
  for (auto str : expected) {
    if (!config.containsKey(str)) {
      return false;
    }
  }
  report_interval_ms_ = config["report_interval"];
  save_mag_cal_ = config["save_mag_cal"];
  return true;
}  // end set_configuration()

/**
 * @brief Constructor sets up the frequency of output and the Signal K path.
 *
 * @param orientation_sensor Pointer to the physical sensor's interface
 * @param report_interval_ms Interval between output reports
 * @param config_path RESTful path by which reporting frequency can be
 * configured. Also for store/modify sensor calibration offsets
 */
MagCalValues::MagCalValues(OrientationSensor* orientation_sensor,
                           uint report_interval_ms = 100,
/*
    #ifdef F_USING_SMARTSENSOR
                           physical_sensor_position_t pos = P1,
    #endif
*/
                           String config_path = "")
    : Sensor(config_path),
      orientation_sensor_{orientation_sensor},
      report_interval_ms_{report_interval_ms} {
  //mag_cal_.pos = pos;    // explicit init because cannot initialize a struct member with curly braces 
  load_configuration();
  #ifdef F_USING_SMARTSENSOR
   // Execute once the SetaxisRemap in order to set the IMU axis remap and sign registers
   // to a configured (stored) physical position before starting to read the sensor with update
   orientation_sensor->sensor_interface_-> SetAxisRemap(mag_cal_.pos);
   // Execute once the SaveMagneticCalibration in order to set the IMU offset registers
   // to a valid (stored) calibration status before starting to read the sensor with update
   orientation_sensor_->sensor_interface_->SaveMagneticCalibration();
  #endif
}  // end MagCalValues()

/**
 * @brief Starts periodic output of MagCal parameters.
 *
 * The start() function is inherited from sensesp::Sensor, and is
 * automatically called when the SensESP app starts.
 */
void MagCalValues::start() {
  ReactESP::app->onRepeat(report_interval_ms_, [this]() { this->Update(); });
}

/**
 * @brief Provides one MagCal reading from the orientation sensor.
 *
 * Readings are obtained using the sensor fusion library's Get_() methods
 * and assigned to the output variable that passes data from Producers
 * to Consumers. Consumers of the magcal data are then informed
 * by the call to notify(). If data are not valid (e.g. sensor not
 * functioning), a struct member is set to false so when the Signal K
 * message contents are assembled by as_signalk(),they can reflect that. 
 */
void MagCalValues::Update() {
  mag_cal_.is_data_valid =
      orientation_sensor_->sensor_interface_->IsDataValid();
  #ifndef F_USING_SMARTSENSOR    
  mag_cal_.cal_fit_error = orientation_sensor_->sensor_interface_->GetMagneticFitError() / 100.0;
  mag_cal_.cal_fit_error_trial = orientation_sensor_->sensor_interface_->GetMagneticFitErrorTrial() / 100.0;
  mag_cal_.mag_field_magnitude = orientation_sensor_->sensor_interface_->GetMagneticBMag();
  mag_cal_.mag_field_magnitude_trial = orientation_sensor_->sensor_interface_->GetMagneticBMagTrial();
  mag_cal_.mag_noise_covariance = orientation_sensor_->sensor_interface_->GetMagneticNoiseCovariance();
  mag_cal_.mag_solver = orientation_sensor_->sensor_interface_->GetMagneticCalSolver();
  mag_cal_.magnetic_inclination = orientation_sensor_->sensor_interface_->GetMagneticInclinationRad();
  #endif
  #ifdef F_USING_SMARTSENSOR
  // calibration indexes are read-only from IMU sensor so no config path is declared for them
  // so no value is stored in SPIFSS
  mag_cal_.IMU_calibration_index = orientation_sensor_->sensor_interface_->GetIMUCalibrationIndex();
  mag_cal_.Gyro_calibration_index = orientation_sensor_->sensor_interface_->GetGyroCalibrationIndex();
  mag_cal_.Acc_calibration_index = orientation_sensor_->sensor_interface_->GetAccelCalibrationIndex();
  mag_cal_.Mag_calibration_index = orientation_sensor_->sensor_interface_->GetMagCalibrationIndex();
  // Calibration data have sensESP config paths so they shoulh have a safe place on SPIFSS
  orientation_sensor_->sensor_interface_->GetCalibrationData(&mag_cal_.cal_data[0]);
  #endif
  output = mag_cal_; //remember that in case of smartsensor mag_cal_ contains calib data for all sensors
  notify();
}  // end Update()

/**
 * @brief Define the format for the MagCal value producer.
 *
 */
#ifndef F_USING_SMARTSENSOR
static const char SCHEMA_MAGCAL[] PROGMEM = R"###({
    "type": "object",
    "properties": {
        "report_interval": { 
          "title": "Report Interval", 
          "type": "number", 
          "description": "Milliseconds between outputs of this parameter"
        }
    }
  })###";
  #endif
  #ifdef F_USING_SMARTSENSOR
static const char SCHEMA_MAGCAL[] PROGMEM = R"###({
    "type": "object",
    "properties": {
        "report_interval": { 
          "title": "Report Interval", 
          "type": "number", 
          "description": "Milliseconds between MagCal outputs"
        },
        "offset_Accel_x": {
          "title": "offset_Accel_x", 
          "type": "number", 
          "description": "X-axis off/calib" 
        },
        "offset_Accel_y": {
          "title": "offset_Accel_y", 
          "type": "number", 
          "description": "Y-axis off/calib" 
        },
        "offset_Accel_z": {
          "title": "offset_Accel_z", 
          "type": "number", 
          "description": "Z-axis off/calib" 
        },
        "offset_Mag_x": {
          "title": "offset_Mag_x", 
          "type": "number", 
          "description": "X-axis off/calib" 
        },
        "offset_Mag_y": {
          "title": "offset_Mag_y", 
          "type": "number", 
          "description": "Y-axis off/calib" 
        },
        "offset_Mag_z": {
          "title": "offset_Mag_z", 
          "type": "number", 
          "description": "Z-axis off/calib" 
        },
        "offset_Gyr_x": {
          "title": "offset_Gyr_x", 
          "type": "number", 
          "description": "X-axis off/calib" 
        },
        "offset_Gyr_y": {
          "title": "offset_Gyr_y", 
          "type": "number", 
          "description": "Y-axis off/calib" 
        },
        "offset_Gyr_z": {
          "title": "offset_Gyr_z", 
          "type": "number", 
          "description": "Z-axis off/calib" 
        },
        "radius_Accel": {
          "title": "radius_Accel", 
          "type": "number", 
          "description": "Radius" 
        },
        "radius_Mag": {
          "title": "radius_Mag", 
          "type": "number", 
          "description": "Radius" 
        },
        "physical_position": {
          "title": "physical_position", 
          "type": "number", 
          "description": "Sensor orientation ref" 
        }
    }
  })###";
  #endif


/**
 * @brief Get the current sensor configuration and place it in a JSON
 * object that can then be stored in non-volatile memory.
 * 
 * @param doc JSON object to contain the configuration parameters
 * to be updated.
 */
void MagCalValues::get_configuration(JsonObject& doc) {
  doc["report_interval"]    = report_interval_ms_;
  #ifdef F_USING_SMARTSENSOR
  doc["physical_position"]  = mag_cal_.pos;
  doc[ "offset_Accel_x"]    = mag_cal_.offset_accel_x;
  doc[ "offset_Accel_y"]    = mag_cal_.offset_accel_y;
  doc[ "offset_Accel_z"]    = mag_cal_.offset_accel_z;
  doc[ "offset_Mag_x"]      = mag_cal_.offset_mag_x;
  doc[ "offset_Mag_y"]      = mag_cal_.offset_mag_y;
  doc[ "offset_Mag_z"]      = mag_cal_.offset_mag_z;
  doc[ "offset_Gyr_x"]      = mag_cal_.offset_gyr_x;
  doc[ "offset_Gyr_y"]      = mag_cal_.offset_gyr_y;
  doc[ "offset_Gyr_z"]      = mag_cal_.offset_gyr_z;
  doc[ "radius_Mag"]        = mag_cal_.mag_radius;
  doc[ "radius_Accel"]      = mag_cal_.accel_radius;
  #endif
}  // end get_configuration()

/**
 * @brief Fetch the JSON format used for holding the configuration.
 */
String MagCalValues::get_config_schema() { return FPSTR(SCHEMA_MAGCAL); }

/**
 * @brief Use the values stored in JSON object config to update
 * the appropriate member variables.
 * 
 * #IF F_USING_SMARTSENSOR:
 * Updates also the corresponding variables in interfaced sensor
 *
 * @param config JSON object containing the configuration parameters
 * to be updated.
 * @return True if successful; False if a parameter could not be found.
 */
bool MagCalValues::set_configuration(const JsonObject& config) {
  #ifdef F_USING_SMARTSENSOR
  orientation_sensor_->sensor_interface_->GetCalibrationData(&mag_cal_.cal_data[0]);
  // don't call SetAxisRemap. It is called once at startup (pos will vary only moving sensor in another place)
  #endif
  int8_t i = 0;
  #ifndef F_USING_SMARTSENSOR
  String expected[] = {"report_interval" };
  #endif
  #ifdef F_USING_SMARTSENSOR
  String expected[] = {   "report_interval"
                        , "offset_Accel_x"
                        , "offset_Accel_y"
                        , "offset_Accel_z"
                        , "offset_Mag_x"
                        , "offset_Mag_y"
                        , "offset_Mag_z"
                        , "offset_Gyr_x"
                        , "offset_Gyr_y"
                        , "offset_Gyr_z"
                        , "radius_Accel"
                        , "radius_Mag" 
                        , "physical_position"
                      };
  #endif
  for (auto str : expected) {
    if (!config.containsKey(str)) {
      return false;
    }
  }
  
  report_interval_ms_       = config["report_interval"];
  #ifdef F_USING_SMARTSENSOR
  mag_cal_.pos              = config["physical_position"];
  mag_cal_.offset_accel_x   = config[ "offset_Accel_x"];
  mag_cal_.offset_accel_y   = config[ "offset_Accel_y"];
  mag_cal_.offset_accel_z   = config[ "offset_Accel_z"];
  mag_cal_.offset_mag_x     = config[ "offset_Mag_x"];
  mag_cal_.offset_mag_y     = config[ "offset_Mag_y"];
  mag_cal_.offset_mag_z     = config[ "offset_Mag_z"];
  mag_cal_.offset_gyr_x     = config[ "offset_Gyr_x"];
  mag_cal_.offset_gyr_y     = config[ "offset_Gyr_y"];
  mag_cal_.offset_gyr_z     = config[ "offset_Gyr_z"];
  mag_cal_.mag_radius       = config[ "radius_Mag"];
  mag_cal_.accel_radius     = config[ "radius_Accel"];
  #endif
  
  #ifdef F_USING_SMARTSENSOR
  orientation_sensor_->sensor_interface_->SetCalibrationData(&mag_cal_.cal_data[0]);
  // don't call SetAxisRemap. It is called once at startup (pos will vary only moving sensor in another place)
  #endif
  return true;
}  // end set_configuration()


/**
 * @brief Constructor sets up the frequency of output and the Signal K path.
 *
 * @param orientation_sensor Pointer to the physical sensor's interface
 * @param val_type The type of orientation parameter to be sent
 * @param report_interval_ms Interval between output reports
 * @param config_path RESTful path by which reporting frequency can be
 * configured.
 */
OrientationValues::OrientationValues(OrientationSensor* orientation_sensor,
                                     OrientationValType val_type,
                                     uint report_interval_ms, String config_path)
    : FloatSensor(config_path),
      orientation_sensor_{orientation_sensor},
      value_type_{val_type},
      report_interval_ms_{report_interval_ms} {
  load_configuration();
  save_mag_cal_ = 0;

}  // end OrientationValues()

/**
 * @brief Starts periodic output of orientation parameter.
 *
 * The start() function is inherited from sensesp::Sensor, and is
 * automatically called when the SensESP app starts.
 */
void OrientationValues::start() {
  ReactESP::app->onRepeat(report_interval_ms_, [this]() { this->Update(); });
}

/**
 * @brief Provides one orientation parameter reading from the sensor.
 *
 * value_type_ determines which particular parameter is output.
 * Readings are obtained using the sensor fusion library's get() methods
 * and assigned to the output variable that passes data from Producers
 * to Consumers. Consumers of the orientation data are then informed
 * by the call to notify()
 */
void OrientationValues::Update() {
  //check whether magnetic calibration has been requested to be saved or deleted
  if( 1 == save_mag_cal_ ) {
    orientation_sensor_->sensor_interface_-> SaveMagneticCalibration ();;
  }else if( -1 == save_mag_cal_ ) {
    orientation_sensor_->sensor_interface_-> EraseMagneticCalibration ();;
  }
  save_mag_cal_ = 0;  // set flag back to zero so we don't repeat save/delete
  //check which type of parameter is requested, and pass it on
  switch (value_type_) {
    case (kCompassHeading):
      output = orientation_sensor_->sensor_interface_->GetHeadingRadians();
      break;
    case (kRoll):
      output = orientation_sensor_->sensor_interface_->GetRollRadians();
      break;
    case (kPitch):
      output = orientation_sensor_->sensor_interface_->GetPitchRadians();
      break;
    case (kAccelerationX):
      output = orientation_sensor_->sensor_interface_->GetAccelXMPerSS();
      break;
    case (kAccelerationY):
      output = orientation_sensor_->sensor_interface_->GetAccelYMPerSS();
      break;
    case (kAccelerationZ):
      output = orientation_sensor_->sensor_interface_->GetAccelZMPerSS();
      break;
    case (kRateOfTurn):
      output = orientation_sensor_->sensor_interface_->GetTurnRateRadPerS();
      break;
    case (kRateOfPitch):
      output = orientation_sensor_->sensor_interface_->GetPitchRateRadPerS();
      break;
    case (kRateOfRoll):
      output = orientation_sensor_->sensor_interface_->GetRollRateRadPerS();
      break;
    case (kTemperature):
      output = orientation_sensor_->sensor_interface_->GetTemperatureK();
      break;
    #ifndef F_USING_SMARTSENSOR
    case (kMagCalFitInUse):
      output = orientation_sensor_->sensor_interface_->GetMagneticFitError();
      break;
    case (kMagCalFitTrial):
      output = orientation_sensor_->sensor_interface_->GetMagneticFitErrorTrial();
      break;
    case (kMagCalAlgorithmSolver):
      output = orientation_sensor_->sensor_interface_->GetMagneticCalSolver();
      break;
    case (kMagInclination):
      output = orientation_sensor_->sensor_interface_->GetMagneticInclinationRad();
      break;
    case (kMagFieldMagnitude):
      //TODO report in T rather than uT, however need widget to be able to display
      output = orientation_sensor_->sensor_interface_->GetMagneticBMag();
      break;
    case (kMagFieldMagnitudeTrial):
      //TODO report in T rather than uT, however need widget to be able to display
      output = orientation_sensor_->sensor_interface_->GetMagneticBMagTrial();
      break;
    case (kMagNoiseCovariance):
      output = orientation_sensor_->sensor_interface_->GetMagneticNoiseCovariance();
      break;
      #endif
      #ifdef F_USING_SMARTSENSOR
    case (kIMUCalibrationIndex):  
      output = orientation_sensor_->sensor_interface_->GetIMUCalibrationIndex();
      break;
    case (kGyroCalibrationIndex):  
      output = orientation_sensor_->sensor_interface_->GetGyroCalibrationIndex();
      break;
    case (kAccelCalibrationIndex):  
      output = orientation_sensor_->sensor_interface_->GetAccelCalibrationIndex();
      break;
    case (kMagCalibrationIndex):  
      output = orientation_sensor_->sensor_interface_->GetMagCalibrationIndex();
      break;
      #endif
    default:
      return; //skip the notify(), due to unrecognized value type
  }
  if (orientation_sensor_->sensor_interface_->IsDataValid()) {
    notify();  // only pass on the data if it is valid
  }
}  // end Update()

/**
 * @brief Get the current sensor configuration and place it in a JSON
 * object that can then be stored in non-volatile memory.
 *
 * @param doc JSON object to contain the configuration parameters
 * to be updated.
 */
void OrientationValues::get_configuration(JsonObject& doc) {
  doc["report_interval"] = report_interval_ms_;
  doc["save_mag_cal"] = save_mag_cal_;
#ifdef F_USING_SMARTSENSOR

#endif
}  // end get_configuration()

/**
 * @brief Fetch the JSON format used for holding the configuration.
 */
String OrientationValues::get_config_schema() { return FPSTR(SCHEMA); }

/**
 * @brief Use the values stored in JSON object config to update
 * the appropriate member variables.
 *
 * @param config JSON object containing the configuration parameters
 * to be updated.
 * @return True if successful; False if a parameter could not be found.
 */
bool OrientationValues::set_configuration(const JsonObject& config) {
  String expected[] = {"report_interval", "save_mag_cal"};
  for (auto str : expected) {
    if (!config.containsKey(str)) {
      return false;
    }
  }
  report_interval_ms_ = config["report_interval"];
  save_mag_cal_ = config["save_mag_cal"];
  return true;
}

} //namespace sensesp