/** @file orientation_sensor.h
 *  @brief Orientation sensor interface to SensESP
 * 
 * Provides Orientation from 9DOF sensor combination (magnetometer,
 * accelerometer, gyroscope) consisting of FXOS8700 + FXAS21002
 */

#ifndef orientation_sensor_H_
#define orientation_sensor_H_

#include "sensor_fusion_class.h"  // for OrientationSensorFusion-ESP library

#include "sensesp/sensors/sensor.h"
#include "signalk_orientation.h"

namespace sensesp {
/**
 * @brief OrientationSensor represents a 9-Degrees-of-Freedom sensor
 * (magnetometer, accelerometer, and gyroscope).
 *
 * This class provides the interface to the SensorFusion library which performs
 * the I2C communication with the sensor and runs the sensor fusion algorithm.
 *
 * A compatible sensor is the NXP FXOS8700 + FXAS21002 combination sensor.
 * This combination sensor is found on products such as the
 * Adafruit #3463 breakout board. The OrientationSensorFusion-ESP
 * library is configured to use this NXP sensor by default, though
 * other sensors can be used by adjusting the library's build.h
 * and board.h files. Calling the public SensorFusion:: methods
 * can be done after you instantiate OrientationSensor, for example by:
 * orientation_sensor->sensor_interface_->GetOrientationQuaternion();
 * The OrientationSensorFusion-ESP library has details:
 * @see https://github.com/BjarneBitscrambler/OrientationSensorFusion-ESP.git
  */
typedef enum sensor_cap {
ACC_ONLY              = 0b00000001,  //sigle sensor
MAG_ONLY              = 0b00000010,  //sigle sensor
GYRO_ONLY             = 0b00000100,  //sigle sensor 
THERM_ONLY            = 0b00001000,  //sigle sensor
BARO_ONLY             = 0b00010000,  //sigle sensor 

AMT                   = 0b00101011,  //combo sensor
AMGT                  = 0b00101111,  //combo sensor
BARO_THERM            = 0b00111000,  //combo sensor
AMT_PLUS_BT           = 0b01011011,  //combo sensor
AMGT_PLUS_BT          = 0b01011111   //combo sensor

} sensor_cap;

typedef struct orsensor_bus_addresses {int8_t addr[4];} orsensor_bus_addresses;

class OrientationSensor {
 public:
  OrientationSensor(uint8_t pin_i2c_sda, uint8_t pin_i2c_scl,
                    orsensor_bus_addresses bus_i2c_addr, sensor_cap cap
                   );
  SensorFusion* sensor_interface_;  ///< sensor's Fusion Library interface

 private:
  void ReadAndProcessSensors(void);  ///< reads sensor and runs fusion algorithm
};

/**
 * @brief AttitudeValues reads and outputs attitude (yaw,pitch,roll) parameters.
 *
 * The three parameters are stored in an Attitude struct, and sent together
 * in one Signal K message. The units are radians.
 */
class AttitudeValues : public AttitudeProducer, public sensesp::Sensor {
 public:
  AttitudeValues(OrientationSensor* orientation_sensor,
                 uint report_interval_ms = 100, String config_path = "");
//sensESP v2 changes enable() to start()  void enable() override final;  ///< starts periodic outputs of Attitude
  void start() override final;  ///< starts periodic outputs of Attitude
  OrientationSensor*
      orientation_sensor_;  ///< Pointer to the orientation sensor

 private:
  void Update(void);  ///< fetches current attitude and notifies consumer
  virtual void get_configuration(JsonObject& doc) override;
  virtual bool set_configuration(const JsonObject& config) override;
  virtual String get_config_schema() override;
  Attitude attitude_;  ///< struct storing the current yaw,pitch,roll values
  uint report_interval_ms_;  ///< interval between attitude updates to Signal K
  int8_t save_mag_cal_;      ///< Flag for saving current magnetic calibration

};  // end class AttitudeValues

/**
 * @brief MagCalValues reads and outputs magnetic calibration parameters.
 *
 * The parameters are stored in an MagCal struct, and sent together
 * in one Signal K message. They are useful in determining how well
 * the existing magnetic calibration suits the current magnetic
 * environment.
 */
class MagCalValues : public MagCalProducer, public sensesp::Sensor {
 public:
  MagCalValues(OrientationSensor* orientation_sensor,
                 uint report_interval_ms,
/*
#ifdef F_USING_SMARTSENSOR
                 physical_sensor_position_t pos,
#endif
*/
                 String config_path);
//sensESP v2 changes enable() to start()  void enable() override final;  ///< starts periodic outputs of MagCal values
  void start() override final;  ///< starts periodic outputs of MagCal values
  OrientationSensor*
      orientation_sensor_;  ///< Pointer to the orientation sensor

 private:
  void Update(void);  ///< fetches current attitude and notifies consumer
  virtual void get_configuration(JsonObject& doc) override;
  virtual bool set_configuration(const JsonObject& config) override;
  virtual String get_config_schema() override;
  
  MagCal mag_cal_;  ///< struct storing the current magnetic calibration parameters
  uint report_interval_ms_;  ///< interval between attitude updates to Signal K

};  // end class MagCalValues

/**
 * @brief OrientationValues reads and outputs orientation parameters.
 *
 * One parameter is sent per instance of OrientationValues, selected
 * from the list of OrientationValType. The one exception is the
 * attitude (yaw,pitch,roll) which consists of three parameters and
 * is provided by the AttitudeValues class instead of this one.
 * Create new instances in main.cpp for each parameter desired.
 */
//sensESP v2 replaced NumericSensor with various sub-types  class OrientationValues : public NumericSensor {
class OrientationValues : public FloatSensor {
 public:
  enum OrientationValType {
    kCompassHeading,      ///< compass heading, also called yaw
    kYaw,                 ///< rotation about the vertical axis
    kPitch,               ///< rotation about the transverse axis
    kRoll,                ///< rotation about the longitudinal axis
    kAttitude,            ///< attitude combines heading, pitch, and roll
    kAccelerationX,       ///< acceleration in the stern-to-bow axis
    kAccelerationY,       ///< acceleration in the starboard-to-port axis
    kAccelerationZ,       ///< acceleration in the down-to-up axis
    kRateOfTurn,          ///< rate of change of compass heading
    kRateOfPitch,         ///< rate of change of pitch
    kRateOfRoll,          ///< rate of change of roll
    kTemperature         ///< temperature as reported by sensor IC
    #ifndef F_USING_SMARTSENSOR
    , kMagCalFitInUse,      ///< fit of currently-used calibration. <3.5 is good.
    kMagCalFitTrial,      ///< fit of candidate calibration. <3.5 is good.
    kMagCalAlgorithmSolver,   ///< cal algorithm order used. [0,4,7,10] 10 is best
    kMagInclination,      ///< geomagnetic inclination based on current readings
    kMagFieldMagnitude,   ///< geomagnetic magnitude of current calibration
    kMagFieldMagnitudeTrial,  ///< geomagnetic magnitude based on current readings
    kMagNoiseCovariance   ///< deviation of current reading from calibrated geomag sphere
    #endif
    #ifdef F_USING_SMARTSENSOR
    , kIMUCalibrationIndex, ///< the computed index of calibration for IMU chip [0(uncalibrated)..3(fully calibrated)]
    kGyroCalibrationIndex,  ///< the computed index of calibration for Gyroscope [0(uncalibrated)..3(fully calibrated)]
    kAccelCalibrationIndex, ///< the computed index of calibration for Accelerometer [0(uncalibrated)..3(fully calibrated)]
    kMagCalibrationIndex    ///< the computed index of calibration for Magnetometer [0(uncalibrated)..3(fully calibrated)]
    #endif
  };
  OrientationValues(OrientationSensor* orientation_sensor,
                    OrientationValType value_type = kCompassHeading,
                    uint report_interval_ms = 100, String config_path = "");
//sensESP v2 changes enable() to start()    void enable() override final;  ///< starts periodic outputs of Attitude
  void start() override final;  ///< starts periodic outputs of Attitude
  OrientationSensor*
      orientation_sensor_;  ///< Pointer to the orientation sensor

 private:
  void Update(
      void);  ///< fetches current orientation parameter and notifies consumer
  virtual void get_configuration(JsonObject& doc) override;
  virtual bool set_configuration(const JsonObject& config) override;
  virtual String get_config_schema() override;
  OrientationValType
      value_type_;           ///< Particular type of orientation parameter supplied
  uint report_interval_ms_;  ///< Interval between data outputs via Signal K
  int8_t save_mag_cal_;      ///< Flag for saving current magnetic calibration

};  // end class OrientationValues

} // namespace sensesp

#endif  // ORIENTATION_SENSOR_H_
