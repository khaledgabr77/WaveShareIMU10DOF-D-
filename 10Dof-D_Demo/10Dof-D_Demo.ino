/*
 * Khaled Gabr
*/
#include <ros.h>
#include <sensor_msgs/Imu.h> // need this for generating imu messages in ros
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry2/tf2/include/tf2/LinearMath/Quaternion.h>
#include "Waveshare_10Dof-D.h"

bool gbSenserConnectState = false;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

tf2::Quaternion quaternion_;

ros::NodeHandle nh;

sensor_msgs::Imu imu_msg;
//sensor_msgs::MagneticField mag_msg;

ros::Publisher imu_data("/imu_data", &imu_msg);
//ros::Publisher mag_data("/mag", &mag_msg);

char frameid[] = "/imu";
//char child[] = "/imu";

void setup() {
  nh.getHardware()->setBaud(115200); // setting the baud rate for rosserial
  nh.initNode();

  // put your setup code here, to run once:
  bool bRet;
  IMU_EN_SENSOR_TYPE enMotionSensorType, enPressureType;
//  Serial.begin(115200);
  nh.advertise(imu_data);
  //  nh.advertise(mag_data);


  broadcaster.init(nh);



  imuInit(&enMotionSensorType, &enPressureType);
  if (IMU_EN_SENSOR_TYPE_ICM20948 == enMotionSensorType)
  {
    //    Serial.println("Motion sersor is ICM-20948");
  }
  else
  {
    //    Serial.println("Motion sersor NULL");
  }
  if (IMU_EN_SENSOR_TYPE_BMP280 == enPressureType)
  {
    //    Serial.println("Pressure sersor is BMP280");
  }
  else
  {
    //    Serial.println("Pressure sersor NULL");
  }
  delay(1000);
}



void loop() {
  // put your main code here, to run repeatedly:
  IMU_ST_ANGLES_DATA stAngles;
  IMU_ST_SENSOR_DATA stGyroRawData;
  IMU_ST_SENSOR_DATA stAccelRawData;
  IMU_ST_SENSOR_DATA stMagnRawData;
  //  int32_t s32PressureVal = 0, s32TemperatureVal = 0, s32AltitudeVal = 0;

  imuDataGet( &stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
  //  pressSensorDataGet(&s32TemperatureVal, &s32PressureVal, &s32AltitudeVal);

  //convert Values From RPY to Quaterian Values
  //  double cosYaw = cos(stAngles.fYaw * 0.5);
  //  double sinYaw = sin(stAngles.fYaw * 0.5);
  //  double cosPitch = cos(stAngles.fPitch * 0.5);
  //  double sinPitch = sin(stAngles.fPitch * 0.5);
  //  double cosRoll = cos(stAngles.fRoll * 0.5);
  //  double sinRoll = sin(stAngles.fRoll * 0.5);
  //  double qx = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
  //  double qy = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
  //  double qz = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
  //  double qw = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
  ////


  //  t.header.frame_id = frameid;
  //  t.child_frame_id = child;
  //  t.transform.translation.x = 1.0;
  //  t.transform.rotation.x = qx;
  //  t.transform.rotation.y = qy;
  //  t.transform.rotation.z = qz;
  //  t.transform.rotation.w = qw;
  //  t.header.stamp = nh.now();
  //  broadcaster.sendTransform(t);
  imu_msg.header.frame_id = "/imu";
  //
//    imu_msg.angular_velocity.x = stGyroRawData.s16X ;
//    imu_msg.angular_velocity.y = stGyroRawData.s16Y ;
//    imu_msg.angular_velocity.z = stGyroRawData.s16Z ;
//    imu_msg.angular_velocity_covariance[0] = 0.02;
//    imu_msg.angular_velocity_covariance[1] = 0;
//    imu_msg.angular_velocity_covariance[2] = 0;
//    imu_msg.angular_velocity_covariance[3] = 0;
//    imu_msg.angular_velocity_covariance[4] = 0.02;
//    imu_msg.angular_velocity_covariance[5] = 0;
//    imu_msg.angular_velocity_covariance[6] = 0;
//    imu_msg.angular_velocity_covariance[7] = 0;
//    imu_msg.angular_velocity_covariance[8] = 0.02;
//    //  //
//    imu_msg.linear_acceleration.x =  stAccelRawData.s16X ;
//    imu_msg.linear_acceleration.y = stAccelRawData.s16Y ;
//    imu_msg.linear_acceleration.z = stAccelRawData.s16Z ;
//    imu_msg.linear_acceleration_covariance[0] = 0.04;
//    imu_msg.linear_acceleration_covariance[1] = 0;
//    imu_msg.linear_acceleration_covariance[2] = 0;
//    imu_msg.linear_acceleration_covariance[3] = 0;
//    imu_msg.linear_acceleration_covariance[4] = 0.04;
//    imu_msg.linear_acceleration_covariance[5] = 0;
//    imu_msg.linear_acceleration_covariance[6] = 0;
//    imu_msg.linear_acceleration_covariance[7] = 0;
//    imu_msg.linear_acceleration_covariance[8] = 0.04;
  //
  //
  quaternion_.setRPY( stAngles.fRoll * 3.14 / 180, stAngles.fPitch * 3.14 / 180, stAngles.fYaw * 3.14 / 180 );
//  quaternion_ = quaternion_.normalize();
  imu_msg.orientation.x = quaternion_.x();
  imu_msg.orientation.y = quaternion_.y();
  imu_msg.orientation.z =  quaternion_.z();
  imu_msg.orientation.w = quaternion_.w();
//    imu_msg.orientation_covariance[0] = 0.0025;
//    imu_msg.orientation_covariance[1] = 0;
//    imu_msg.orientation_covariance[2] = 0;
//    imu_msg.orientation_covariance[3] = 0;
//    imu_msg.orientation_covariance[4] = 0.0025;
//    imu_msg.orientation_covariance[5] = 0;
//    imu_msg.orientation_covariance[6] = 0;
//    imu_msg.orientation_covariance[7] = 0;
//    imu_msg.orientation_covariance[8] = 0.0025;
  //
  //  mag_msg.magnetic_field.x = stMagnRawData.s16X;
  //  mag_msg.magnetic_field.y = stMagnRawData.s16Y;
  //  mag_msg.magnetic_field.z = stMagnRawData.s16Z;
  //
  imu_data.publish(&imu_msg); // publishing the imu data
  //  mag_data.publish(&mag_msg);
  
  nh.spinOnce();

  //  Serial.println();
  //    Serial.println("/-------------------------------------------------------------/");
  //    Serial.print("Roll : "); Serial.print(stAngles.fRoll);
  //    Serial.print("    Pitch : "); Serial.print(stAngles.fPitch);
  //    Serial.print("    Yaw : "); Serial.print(stAngles.fYaw);
  //    Serial.println();
  //    Serial.print("Acceleration: X : "); Serial.print(stAccelRawData.s16X);
  //    Serial.print("    Acceleration: Y : "); Serial.print(stAccelRawData.s16Y);
  //    Serial.print("    Acceleration: Z : "); Serial.print(stAccelRawData.s16Z);
  //    Serial.println();
  //    Serial.print("Gyroscope: X : "); Serial.print(stGyroRawData.s16X);
  //    Serial.print("       Gyroscope: Y : "); Serial.print(stGyroRawData.s16Y);
  //    Serial.print("       Gyroscope: Z : "); Serial.print(stGyroRawData.s16Z);
  //    Serial.println();
  //    Serial.print("Magnetic: X : "); Serial.print(stMagnRawData.s16X);
  //    Serial.print("      Magnetic: Y : "); Serial.print(stMagnRawData.s16Y);
  //    Serial.print("      Magnetic: Z : "); Serial.print(stMagnRawData.s16Z);
  //    Serial.println();
  //    Serial.print("Pressure : "); Serial.print((float)s32PressureVal / 100);
  //    Serial.print("     Altitude : "); Serial.print((float)s32AltitudeVal / 100);
  //    Serial.println();
  //    Serial.print("Temperature : "); Serial.print((float)s32TemperatureVal / 100);
  //  Serial.println();
}
