#include<ros.h>
#include <sensor_msgs/Imu.h> // need this for generating imu messages in ros
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/MagneticField.h>
#include "Waveshare_10Dof-D.h"
bool gbSenserConnectState = false;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

ros::NodeHandle nh;

sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField mag_msg;

ros::Publisher imu_data("/imu_data", &imu_msg);
ros::Publisher mag_data("/mag", &mag_msg);

char frameid[] = "/base_link";
char child[] = "/imu";

void setup() {
  nh.getHardware()->setBaud(115200); // setting the baud rate for rosserial
  nh.initNode();

  // put your setup code here, to run once:
  bool bRet;
  IMU_EN_SENSOR_TYPE enMotionSensorType, enPressureType;
  Serial.begin(115200);
  nh.advertise(imu_data);

  broadcaster.init(nh);



  imuInit(&enMotionSensorType, &enPressureType);
  if (IMU_EN_SENSOR_TYPE_ICM20948 == enMotionSensorType)
  {
    Serial.println("Motion sersor is ICM-20948");
  }
  else
  {
    Serial.println("Motion sersor NULL");
  }
  if (IMU_EN_SENSOR_TYPE_BMP280 == enPressureType)
  {
    Serial.println("Pressure sersor is BMP280");
  }
  else
  {
    Serial.println("Pressure sersor NULL");
  }
  delay(1000);
}



void loop() {
  // put your main code here, to run repeatedly:
  IMU_ST_ANGLES_DATA stAngles;
  IMU_ST_SENSOR_DATA stGyroRawData;
  IMU_ST_SENSOR_DATA stAccelRawData;
  IMU_ST_SENSOR_DATA stMagnRawData;
  int32_t s32PressureVal = 0, s32TemperatureVal = 0, s32AltitudeVal = 0;

  imuDataGet( &stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
  pressSensorDataGet(&s32TemperatureVal, &s32PressureVal, &s32AltitudeVal);

  float cosYaw = cos(stAngles.fYaw / 2.0);
  float sinYaw = sin(stAngles.fYaw / 2.0);
  float cosPitch = cos(stAngles.fPitch / 2.0);
  float sinPitch = sin(stAngles.fPitch / 2.0);
  float cosRoll = cos(stAngles.fRoll / 2.0);
  float sinRoll = sin(stAngles.fRoll / 2.0);
  float qx = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
  float qy = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
  float qz = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
  float qw = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;

  t.header.frame_id = frameid;
  t.child_frame_id = child;
  t.transform.translation.x = 1.0;
  t.transform.rotation.x = qx;
  t.transform.rotation.y = qy;
  t.transform.rotation.z = qz;
  t.transform.rotation.w = qw;
  t.header.stamp = nh.now();
  broadcaster.sendTransform(t);
  //Serial.print(cosYaw);
  //  imu_msg.header.frame_id = "/imu";
  //  imu_msg.angular_velocity.x = stGyroRawData.s16X;
  //  imu_msg.angular_velocity.y = stGyroRawData.s16Y;
  //  imu_msg.angular_velocity.z = stGyroRawData.s16Z;
  //
  //  imu_msg.linear_acceleration.x =  stAccelRawData.s16X;
  //  imu_msg.linear_acceleration.y = stAccelRawData.s16Y;
  //  imu_msg.linear_acceleration.z = stAccelRawData.s16Z;

  
    mag_msg.magnetic_field.x = stMagnRawData.s16X;
    mag_msg.magnetic_field.y = stMagnRawData.s16Y;
    mag_msg.magnetic_field.z = stMagnRawData.s16Z;

//  imu_msg.orientation.x = qx/360;
//  imu_msg.orientation.y = qy/360;
//  imu_msg.orientation.z = qz/360;
//  imu_msg.orientation.w = qw/360;

  imu_data.publish(&mag_msg); // publishing the imu data
  nh.spinOnce();


  Serial.println();
  //  Serial.println("/-------------------------------------------------------------/");
  //  Serial.print("Roll : "); Serial.print(stAngles.fRoll);
  //  Serial.print("    Pitch : "); Serial.print(stAngles.fPitch);
  //  Serial.print("    Yaw : "); Serial.print(stAngles.fYaw);
  //  Serial.println();
  //  Serial.print("Acceleration: X : "); Serial.print(stAccelRawData.s16X);
  //  Serial.print("    Acceleration: Y : "); Serial.print(stAccelRawData.s16Y);
  //  Serial.print("    Acceleration: Z : "); Serial.print(stAccelRawData.s16Z);
  //  Serial.println();
  //  Serial.print("Gyroscope: X : "); Serial.print(stGyroRawData.s16X);
  //  Serial.print("       Gyroscope: Y : "); Serial.print(stGyroRawData.s16Y);
  //  Serial.print("       Gyroscope: Z : "); Serial.print(stGyroRawData.s16Z);
  //  Serial.println();
  //  Serial.print("Magnetic: X : "); Serial.print(stMagnRawData.s16X);
  //  Serial.print("      Magnetic: Y : "); Serial.print(stMagnRawData.s16Y);
  //  Serial.print("      Magnetic: Z : "); Serial.print(stMagnRawData.s16Z);
  //  Serial.println();
  //  Serial.print("Pressure : "); Serial.print((float)s32PressureVal / 100);
  //  Serial.print("     Altitude : "); Serial.print((float)s32AltitudeVal / 100);
  //  Serial.println();
  //  Serial.print("Temperature : "); Serial.print((float)s32TemperatureVal / 100);
  Serial.println();
}
