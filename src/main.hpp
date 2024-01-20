#define MAGNETOMETER_ADDRESS 0x1F
#define GYROSCOPE_ADDRESS 0x68
#define UPDATE_RATE 10
#define PUBLISH_RATE 10
#define NO_OF_SAMPLES 1000

#ifndef CONFIG_HPP
#define CONFIG_HPP
#define NUMBER_OF_THRUSTERS 7
#define INIT_THRUSTER_PWM 1472
#define THRUSTER_PINS {2, 3, 4, 5, 6, 7, 8}
#define THRUSTER_MIN_PWM 1000
#define THRUSTER_MAX_PWM 2000
#define NORMAL_MODE 0
#define CALIBRATION_MODE 1
#define CALIBRATION_UPDATE_MODE 2
#define MAG_UT_LSB (0.1F)
#define g 9.80665
#endif // CONFIG_HPP


void initializeThrusters();
void initializeImu();
void initializeDepthSensor();
void calculateOffset();
void getSensorData();
void applyImuCalibration();
void updateOrientation();
void eulerToQuaternion();
void applyLowPassFilter();
void updatePublishData();
void applyComplementaryFilter();
void calculateYaw();