#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include <math.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <Wire.h>
#include "MadgwickAHRS.h"
#include "MS5837.h"
#include <Adafruit_FXOS8700.h>
#include <mpu6050.hpp>
#include <geometry_msgs/Quaternion.h>
#include "main.hpp"

float raw_a[3];
float raw_g[3];
float raw_m[3];
float cal_a[3];
float cal_g[3];
float cal_m[3];
float _ax_offset = 0.0, _ay_offset = 0.0, _az_offset = 0.0;
float _gx_offset = 0.0, _gy_offset = 0.0, _gz_offset = 0.0;
float _mx_offset = 57.59, _my_offset = 88.22, _mz_offset = 74.47, yaw_offset=0;
float soft_iron_matrix[3][3]={{0.971,-0.048,-0.024},{-0.048,0.972,-0.011},{-0.024,-0.011,1.063}};
float gyroAngleX = 0, gyroAngleY = 0, gyroAngleZ = 0;
float yaw, pitch, roll, depth,roll_lpf=0,pitch_lpf=0,yaw_lpf=0,roll_cf=0,pitch_cf=0,yaw_cf=0;
float qx,qy,qz,qw;
float headingRadians,headingDegrees,declinationAngle;
int prev_time_update, prev_time_publish, gyro_update_time=0, prev_gyro_update_time = 0;

std_msgs::Float64 depth_data;
geometry_msgs::Vector3 linear_acceleration;
geometry_msgs::Vector3 angular_velocity;
geometry_msgs::Vector3 magnetic_field;
geometry_msgs::Pose pose;
std_msgs::Int32MultiArray pwm_msg;

void throttleCb(const std_msgs::Int32MultiArray& pwm_msg);

ros::NodeHandle nh;
ros::Subscriber <std_msgs::Int32MultiArray> sub("pwm_values", &throttleCb);
ros::Publisher pub1("linear_acceleration", &linear_acceleration);
ros::Publisher pub2("angular_velocity", &angular_velocity);
ros::Publisher pub3("magnetic_field", &magnetic_field);
ros::Publisher pub4("pose", &pose);
ros::Publisher pub5("depth_data", &depth_data);

Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
MPU6050 gyro;
MS5837 Depth_Sensor;
Madgwick Filter;
Servo g_thrusters[NUMBER_OF_THRUSTERS];
const uint8_t g_kPinMap[NUMBER_OF_THRUSTERS] = {PA0, PA1, PA2, PA3, PA6, PB0, PA7};

void setup() 
{
  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  initializeThrusters();
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub1);
  nh.advertise(pub2);
  nh.advertise(pub3);
  nh.advertise(pub4);
  nh.advertise(pub5);

  // initializeThrusters();
  delay(3000);
  initializeImu();
  initializeDepthSensor();
  calculateOffset();

  prev_time_update = millis();
  prev_time_publish = millis();
  prev_gyro_update_time = millis();
}

  
void loop() 
{
  if(millis() - prev_time_update >= UPDATE_RATE)
  {
    getSensorData();
    applyImuCalibration();
    // applyComplementaryFilter();
    // calculateYaw();
    applyLowPassFilter();
    prev_time_update = millis();
  }

    if(millis() - prev_time_publish >= PUBLISH_RATE)
    {
    updatePublishData();
    pub1.publish(&linear_acceleration);
    pub2.publish(&angular_velocity);
    pub3.publish(&magnetic_field);
    pub4.publish(&pose);
    pub5.publish(&depth_data);
    prev_time_publish = millis();
    
    }

    nh.spinOnce();
  
}

void initializeThrusters(){
  for (int thruster_index = 0; thruster_index < NUMBER_OF_THRUSTERS; thruster_index++){
    g_thrusters[thruster_index].attach(g_kPinMap[thruster_index]);
    g_thrusters[thruster_index].writeMicroseconds(INIT_THRUSTER_PWM);
  }
}

void throttleCb(const std_msgs::Int32MultiArray& pwm_msg){
    int pwm_value;
    for (int thruster_index = 0; thruster_index < NUMBER_OF_THRUSTERS; thruster_index ++)
    {
        pwm_value = pwm_msg.data[thruster_index];
        g_thrusters[thruster_index].writeMicroseconds(pwm_value);
    }
}


void initializeImu()
{
  // Magnetometer.init();
  sensor_t accel, mag;
  accelmag.begin();
  gyro.begin();
  gyro.setAccelerometerRange(ACCELERO_METER_RANGE_2);
  gyro.setGyroscopeRange(GYROSCOPE_RANGE_250);
  gyro.setSampleRateDivider(0);
  gyro.disableSleepMode(); 
}

void initializeDepthSensor()
{
  Depth_Sensor.init(&Wire);
  Depth_Sensor.setFluidDensity(997);
}

void calculateOffset()
{
  for (int  sample_no = 0; sample_no < NO_OF_SAMPLES ; sample_no++)
    {
         gyro.getSensorsReadings(raw_a[0], raw_a[1], raw_a[2], raw_g[0], raw_g[1], raw_g[2],false);
        _ax_offset += raw_a[1];
        _ay_offset += -raw_a[0];
        _az_offset += raw_a[2];
        _gx_offset += raw_g[1];
        _gy_offset += -raw_g[0];
        _gz_offset += raw_g[2];
    }
    _ax_offset /= NO_OF_SAMPLES;
    _ay_offset /= NO_OF_SAMPLES;
    _az_offset /= NO_OF_SAMPLES;
    _gx_offset /= NO_OF_SAMPLES;
    _gy_offset /= NO_OF_SAMPLES;
    _gz_offset /= NO_OF_SAMPLES;
    _az_offset -= g;

}

void getSensorData()
{
    Depth_Sensor.read();
    depth = Depth_Sensor.depth();
    // Magnetometer.read();
    sensors_event_t aevent, mevent;
    accelmag.getEvent(&aevent, &mevent);
    raw_m[0] = mevent.magnetic.x;
    raw_m[1] = mevent.magnetic.y;
    raw_m[2] = mevent.magnetic.z;
    gyro.getSensorsReadings(raw_a[0], raw_a[1], raw_a[2], raw_g[0], raw_g[1], raw_g[2]);
    // raw_m[0]*=MAG_UT_LSB;
    // raw_m[1]*=MAG_UT_LSB;
    // raw_m[2]*=MAG_UT_LSB;

}

void applyImuCalibration()
{
    cal_a[1]=raw_a[1]-_ax_offset;
    cal_a[0]=-raw_a[0]-_ay_offset;
    cal_a[2]=raw_a[2]-_az_offset;

    cal_g[1]=raw_g[1]-_gx_offset;
    cal_g[0]=-raw_g[0]-_gy_offset;
    cal_g[2]=raw_g[2]-_gz_offset;

    cal_m[0]=raw_m[0]-_mx_offset;
    cal_m[1]=raw_m[1]-_my_offset;
    cal_m[2]=raw_m[2]-_mz_offset;

    cal_m[0]=cal_m[0]*soft_iron_matrix[0][0]+cal_m[1]*soft_iron_matrix[0][1]+cal_m[2]*soft_iron_matrix[0][2];
    cal_m[1]=cal_m[0]*soft_iron_matrix[1][0]+cal_m[1]*soft_iron_matrix[1][1]+cal_m[2]*soft_iron_matrix[1][2];
    cal_m[2]=cal_m[0]*soft_iron_matrix[2][0]+cal_m[1]*soft_iron_matrix[2][1]+cal_m[2]*soft_iron_matrix[2][2];
}

void updateOrientation()
{
    Filter.update(cal_g[0], cal_g[1], cal_g[2], cal_a[0], cal_a[1], cal_a[2], cal_m[0], cal_m[1], cal_m[2]);
    yaw = Filter.getYaw();
    pitch = Filter.getPitch();
    roll = Filter.getRoll();
}

void eulerToQuaternion()
{
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);
}

void applyLowPassFilter()
{
    roll_lpf = 0.9 * roll_lpf + 0.1 * roll_cf;
    pitch_lpf = 0.9 * pitch_lpf + 0.1 * pitch_cf;
    yaw_lpf = 0.9 * yaw_lpf + 0.1 * yaw;
}

void applyComplementaryFilter()
{
    // Calculating Roll and Pitch from the accelerometer data
    float accAngleX = (atan(cal_a[1] / sqrt(pow(cal_a[0], 2) + pow(cal_a[2], 2))) * 180 / PI); // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
    float accAngleY = (atan(-1 * cal_a[0] / sqrt(pow(cal_a[1], 2) + pow(cal_a[2], 2))) * 180 / PI); // AccErrorY ~(-1.58)
    gyro_update_time = millis();
    gyroAngleX = gyroAngleX + cal_g[0] * 0.001*(gyro_update_time - prev_gyro_update_time);
    gyroAngleY = gyroAngleY + cal_g[1] * 0.001*(gyro_update_time - prev_gyro_update_time);
    prev_gyro_update_time = gyro_update_time;

    roll_cf = 0* gyroAngleX + 1* accAngleX;
    pitch_cf = 0 * gyroAngleY + 1 * accAngleY;
    // roll_cf = 0* cal_g[0] + 1* cal_a[0];
    // pitch_cf = 0 * cal_g[1] + 1* cal_a[1];
    // yaw_cf = 0.96 * cal_g[2] + 0.04 * cal_a[2];
}

void calculateYaw()
{
    headingRadians = atan2(cal_m[1], cal_m[0]) - yaw_offset;
    yaw = headingRadians * 180.0/PI;
}

void updatePublishData()
{
    depth_data.data = depth;

    linear_acceleration.x= cal_a[0]/g;
    linear_acceleration.y= cal_a[1]/g;
    linear_acceleration.z= cal_a[2]/g;

    angular_velocity.x= cal_g[0];
    angular_velocity.y= cal_g[1];
    angular_velocity.z= cal_g[2];

    magnetic_field.x= cal_m[0];
    magnetic_field.y= cal_m[1];
    magnetic_field.z= cal_m[2];

    pose.position.x = roll_lpf;
    pose.position.y = pitch_lpf;
    pose.position.z = yaw_lpf;

    pose.orientation.x=qx;
    pose.orientation.y=qy;
    pose.orientation.z=qz;
    pose.orientation.w=qw;
}