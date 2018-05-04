#include "Arduino.h"
#include "MPU9250.h"
#include "BMP180.h"
#include "math.h"
#include "SD.h"
double T, P, p0, a;
unsigned long timeStamp;


File flightData;
File testSD;
MPU9250 imu;
BMP180 pressure;
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

bool verifySD(){
  testSD = SD.open("testt.txt", FILE_WRITE);
  testSD.println("1");
  testSD.close();
  testSD = SD.open("testt.txt", FILE_READ);
  if(testSD.read() = 1){
    return true;
  }
  else{
    return false;
  }

}

void setup() {
    // put your setup code here, to run once:
    delay(5000);
    Serial.begin(115200);
    if(!initIMU()){
      Serial.println("IMU INIT FAILED! STOPPING!");
      while(true);
    }
    Serial.println("IMU INIT SUCCESS!");
    initBMP();
    //while(!Serial.available());
    if(!SD.begin(PA4)){

      //Serial.println("")
      while(true)Serial.println("SD init failed!");
    }

    flightData = SD.open("DATA.txt",FILE_WRITE);
    flightData.println("Hello World!");
    //flightData.close();
    //while(true);
    timeStamp = millis();

}
//MPU9250_Raw_Data imuData;
MPU9250_Data imuData;
float currentAlt = 0;
long timeStart = 0;
void loop() {
  if(verifySD()){
    if(millis() < 10000){
      timeStamp = millis();
      currentAlt = pressure.altitude(pressure.getPressureAsync());
      imu.getData(&imuData);
      //flightData.println("High");
      flightData.println("@{OX:" + String(imuData.orientation.x) + ";OY:" + String(imuData.orientation.y) + ";OZ:" + String(imuData.orientation.z) + ";TS:" + String(timeStamp) + ";}@");
      flightData.println("@{PA:" + String(currentAlt) + ";TS:" + String(timeStamp) + ";}");
      //flightData.close();
      Serial.println();
      Serial.println("@{AX:" + String(imuData.accel.x) + ";AY:" + String(imuData.accel.y) + ";AZ:" + String(imuData.accel.z) + ";TS:" + String(timeStamp) + ";}@");
      Serial.println("@{GX:" + String(imuData.gyro.x) + ";GY:" + String(imuData.gyro.y) + ";GZ:" + String(imuData.gyro.z) + ";TS:" + String(timeStamp)+ ";}@");
      Serial.println("@{PA:" + String(currentAlt) + ";TS:" + String(timeStamp) + ";}");
    }
    else{
      flightData.close();
      while(true);
    }
  }
    //getAltitude();
    //Serial.println("Altitude: " + String(a));

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
    delay(2);
}
