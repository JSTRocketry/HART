#include "Arduino.h"
#include "MPU9250.h"
#include "SFE_BMP180.h"
#include "math.h"
double T, P, p0, a;
unsigned long timeStamp;

MPU9250 imu;
SFE_BMP180 pressure;
bool initIMU(){
  if(imu.begin(MPU9250_GYRO_RANGE_2000_DPS, MPU9250_ACCEL_RANGE_16_GPS) < 0){
    Serial.println("IMU init Fail!");
    return false;
  }
  Serial.println("IMU init Success!");
  Serial.println("Calibrating Gyroscope!");
  imu.calibrateGyroOffsets();
  imu.zero();
  imu.calibrateAccel();
  imu.setMagnetometerCalibrationOffsets(-36.35, 40.36, -140.07);
  imu.setMagnetometerCalibrationScales(1.0, 1.03, .92);
  imu.setDataFuseMode(MPU9250_DATA_FUSE_GYRO_MAG_AUTO_ACCEL);
  //imu.calibrateMagnetometer();
  return true;
}

bool initBMP(){
  if (pressure.begin())
  {
    Serial.println("BMP 180 INIT SUCCESS!");
  }
  else
  {
    Serial.println("BMP 180 INIT FAILED!");
    while(true);
  }

}

float getAltitude(){
  float ALTITUDE = 124.31;
  char status = pressure.startTemperature();
  if (status != 0)
  {
    delay(status);
    if (status != 0)
    {
      status = pressure.getTemperature(T);
      if (status != 0)
      {
        status = pressure.startPressure(3);
        if (status != 0)
        {
          delay(status);
          status = pressure.getPressure(P,T);
          if (status != 0)
          {
            p0 = pressure.sealevel(P, ALTITUDE);
            a = pressure.calcAlt(P, p0);
          }
          else
          {
            Serial.println("FAILURE!");
          }
        }
        else
        {
          Serial.println("FAILURE!");
        }
      }
      else
      {
        Serial.println("FAILURE!");
      }
    }
    else
  {
    Serial.println("FAILURE");
  }
  }
  else
  {
    Serial.println("FAILURE!");
  }

}

void calcAlt(P, p0){
  double base = (P/p0);
  double exponent = (1/5.255);
  float alt = 44300 * (1- pow(base, exponent));
  return(alt);
}

void setup() {
    // put your setup code here, to run once:
    delay(5000);
    Serial.begin(115200);
    if(!initIMU()){
      Serial.println("IMU INIT FAILED! STOPPING!");
      while(true);
    }
    initBMP();
    timeStamp = millis();

}
//MPU9250_Raw_Data imuData;
MPU9250_Data imuData;

long timeStart = 0;
void loop() {
    timeStart = micros();
    imu.getData(&imuData);
    Serial.println("@{OX:" + String(imuData.orientation.x) + ";OY:" + String(imuData.orientation.y) + ";OZ:" + String(imuData.orientation.z) + ";TS:" + String(timeStamp) "}@");
    Serial.println("@{AX:" + String(imuData.accel.x) + ";AY:" + String(imuData.accel.y) + ";AZ:" + String(imuData.accel.z) + ";TS:" + String(timeStamp) "}@");
    Serial.println("@{GX:" + String(imuData.gyro.x) + ";GY:" + String(imuData.gyro.y) + ";GZ:" + String(imuData.gyro.z) + ";TS:" + String(timeStamp) "}@");
    getAltitude();
    Serial.println("Altitude: " + String(a));

    //Serial.println("Linear Acceleration: " + String(imuData.linearAcceleration));
    /*
    Serial.print("DATA FUSE MODE: ");
    switch(imuData.dataFuseMode){
      case MPU9250_DATA_FUSE_FULL_9_DOF:
        Serial.println("9_DOF");
        break;
      case MPU9250_DATA_FUSE_GYRO_MAG_AUTO_ACCEL:
        Serial.println("GYRO MAG AUTO ACCEL");
        break;
      case MPU9250_DATA_FUSE_GYRO_MAG_AUTO_NO_ACCEL:
        Serial.println("GYRO MAG AUTO NO ACCEL");
        break;
    }
    */
    //Serial.println("Velocity z: " + String(imuData.velocity.z));
    delay(200);
}
