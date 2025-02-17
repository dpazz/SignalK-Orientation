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
  //copy i2c address vector elements to local variables 
  //sensor_cap type is enum but biwise collects the available sensor capabilities for this build
  
  int8_t combo_i2c_addr, gyro_i2c_addr, baro_therm_i2c_addr = 0x0;                                                     
  combo_i2c_addr = gyro_i2c_addr = bus_i2c_addr.addr[0];
  if (cap==AMT || cap==AMT_PLUS_BT) gyro_i2c_addr = bus_i2c_addr.addr[1];
  if (cap == BARO_THERM || cap == AMGT_PLUS_BT || cap == AMT_PLUS_BT) baro_therm_i2c_addr = bus_i2c_addr.addr[2];

  sensor_interface_ = new SensorFusion();  // create our fusion engine instance

  bool success;
  // init IO subsystem, passing NULLs since we use Signal-K output instead.
  success =
      //sensor_interface_->InitializeInputOutputSubsystem(NULL, NULL) && ****controlsubsystem eliminated
      //                                                                     since we use Signal-K output instead.

      // connect to the sensors.  Remember that some sensor can be of combo type (see Orientation_sensor.h).
      sensor_interface_->InstallSensor(combo_i2c_addr,
                                       SensorType::kMagnetometer) &&
      sensor_interface_->InstallSensor(combo_i2c_addr,
                                       SensorType::kAccelerometer) &&
      sensor_interface_->InstallSensor(gyro_i2c_addr, //if cap == AMG or AMG_PLUSBT equals combo_i2c_addr
                                       SensorType::kGyroscope) &&                                      
#ifndef EXTERN_BARO_THERM_SENSOR
       // A thermometer (uncalibrated) is available in the
      // accelerometer/magnetometer IC.
      sensor_interface_->InstallSensor(combo_i2c_addr,
                                       SensorType::kThermometer) ;
#endif
#ifdef EXTERN_BARO_THERM_SENSOR
      // if in the (breakout or main board) hardware solution is available 
      // an enviromental sensor IC like BMP280 .
      sensor_interface_->InstallSensor(baro_therm_i2c_addr_addr,
                                       SensorType::kThermometer) &&
      sensor_interface_->InstallSensor(baro_therm_i2c_addr_addr,
                                       SensorType::kBarometer) && ;
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
    orientation_sensor_->sensor_interface_-> SaveMagneticCalibration ();
  }else if( -1 == save_mag_cal_ ) {
    orientation_sensor_->sensor_interface_-> EraseMagneticCalibration ();
  }
  save_mag_cal_ = 0;  // set flag back to zero so we don't repeat save/delete
  attitude_.is_data_valid =
      orientation_sensor_->sensor_interface_->IsDataValid();
  attitude_.yaw = orientation_sensor_->sensor_interface_->GetHeadingRadians();
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
          "title": "Save Magnetic Cal", 
          "type": "number", 
          "description": "Set to 1 to save current magnetic calibration" 
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
 * configured.
 */
MagCalValues::MagCalValues(OrientationSensor* orientation_sensor,
                               uint report_interval_ms, String config_path)
    : Sensor(config_path),
      orientation_sensor_{orientation_sensor},
      report_interval_ms_{report_interval_ms} {
  load_configuration();
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
  mag_cal_.cal_fit_error = orientation_sensor_->sensor_interface_->GetMagneticFitError() / 100.0;
  mag_cal_.cal_fit_error_trial = orientation_sensor_->sensor_interface_->GetMagneticFitErrorTrial() / 100.0;
  mag_cal_.mag_field_magnitude = orientation_sensor_->sensor_interface_->GetMagneticBMag();
  mag_cal_.mag_field_magnitude_trial = orientation_sensor_->sensor_interface_->GetMagneticBMagTrial();
  mag_cal_.mag_noise_covariance = orientation_sensor_->sensor_interface_->GetMagneticNoiseCovariance();
  mag_cal_.mag_solver = orientation_sensor_->sensor_interface_->GetMagneticCalSolver();
  mag_cal_.magnetic_inclination = orientation_sensor_->sensor_interface_->GetMagneticInclinationRad();

  output = mag_cal_;
  notify();
}  // end Update()

/**
 * @brief Define the format for the MagCal value producer.
 *
 */
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

/**
 * @brief Get the current sensor configuration and place it in a JSON
 * object that can then be stored in non-volatile memory.
 * 
 * @param doc JSON object to contain the configuration parameters
 * to be updated.
 */
void MagCalValues::get_configuration(JsonObject& doc) {
  doc["report_interval"] = report_interval_ms_;
}  // end get_configuration()

/**
 * @brief Fetch the JSON format used for holding the configuration.
 */
String MagCalValues::get_config_schema() { return FPSTR(SCHEMA_MAGCAL); }

/**
 * @brief Use the values stored in JSON object config to update
 * the appropriate member variables.
 *
 * @param config JSON object containing the configuration parameters
 * to be updated.
 * @return True if successful; False if a parameter could not be found.
 */
bool MagCalValues::set_configuration(const JsonObject& config) {
  String expected[] = {"report_interval"};
  for (auto str : expected) {
    if (!config.containsKey(str)) {
      return false;
    }
  }
  report_interval_ms_ = config["report_interval"];
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