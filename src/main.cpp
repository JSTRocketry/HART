#include "Arduino.h"
#include "MPU9250.h"
#include "BMP180.h"
#include "math.h"
#include "SD.h"

#define SD_ERROR_LED PC13
#define IMU_ERROR_LED PC14
#define BMP_ERROR_LED PC15

//double T, P, p0, a;
unsigned long timeStamp;

#define SD_BUFFER_SIZE 1024

File flightData;
MPU9250 imu;
BMP180 pressure;

String fileName;

String getFileName(){
  String base = "fdat";
  int counter = 0;
  while(counter < 10){
    if(!SD.exists(base + String(counter) + ".txt")){
      return base + String(counter) + ".txt";
    }
    else counter ++;
  }
  return "";
}

char writeBuff[SD_BUFFER_SIZE] = {};
int buffIndex = 0;

void writeData(String toWrite, bool force){
  if(buffIndex + toWrite.length() + 1 >= SD_BUFFER_SIZE || force){
    flightData.write(writeBuff,buffIndex);
    flightData.flush();
    //Serial.println("Buff Size: " + String(buffIndex));
    for(int i = 0; i < buffIndex; i ++){
      writeBuff[i] = '\0';
    }
    buffIndex = 0;
    //if(!force) writeData("Buffer wrote!\n", false);
    //Serial.println("Writing buff!");
  }
  for(uint i = 0; i < toWrite.length(); i ++){
    writeBuff[buffIndex] = toWrite.charAt(i);
    buffIndex ++;
  }
}

bool initIMU(){
  if(imu.begin(MPU9250_GYRO_RANGE_2000_DPS, MPU9250_ACCEL_RANGE_16_GPS) < 0){
    //Serial.println("IMU init Fail!");
    //digitalWrite(IMUFail, HIGH);
    return false;
  }
  //Serial.println("IMU init Success!");
  //Serial.println("Calibrating Gyroscope!");
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
    return true;
  }
  return false;
}

void setup() {
    // put your setup code here, to run once:
    delay(5000);
    Serial.begin(115200);
    pinMode(IMU_ERROR_LED, OUTPUT);
    pinMode(BMP_ERROR_LED, OUTPUT);
    pinMode(SD_ERROR_LED, OUTPUT);
    digitalWrite(IMU_ERROR_LED, LOW);
    digitalWrite(BMP_ERROR_LED,LOW);
    digitalWrite(SD_ERROR_LED, LOW);
    if(!initIMU()){
      digitalWrite(IMU_ERROR_LED, HIGH);
      while(true){
          Serial.println("IMU INIT FAILED! STOPPING!");
          delay(20);

      }
    }
    Serial.println("IMU INIT SUCCESS!");
    if(!initBMP()){
        digitalWrite(BMP_ERROR_LED,HIGH);
        while(true){
          Serial.println("BMP FAILURE!");
          delay(20);
        }
    }
    if(!SD.begin(SPI_FULL_SPEED,PA4)){
      digitalWrite(SD_ERROR_LED, HIGH);
      while(true){
        Serial.println("SD Begin FAILED!");
        delay(20);
      }
    }
    fileName = getFileName();
    if(fileName.equals("")){
      while(true){
        Serial.println("SD Card FULL!");
        delay(20);
      }
    }
    flightData = SD.open(fileName,O_CREAT | O_WRITE);
    delay(10);
    if(flightData){
      Serial.println("Opened!");
    }
    else{
      while(true){
        Serial.println("Failed to open!" + fileName + "n");
        delay(20);
      }
    }
    timeStamp = millis();
}

//MPU9250_Raw_Data imuData;
MPU9250_Data imuData;
float currentAlt = 0;
long timeStart = 0;
void loop() {
    timeStamp = millis();
    currentAlt = pressure.altitude(pressure.getPressureAsync());
    imu.getData(&imuData);
    //flightData.println("High");
    writeData("@{OX:" + String(imuData.orientation.x) + ";OY:" + String(imuData.orientation.y) + ";OZ:" + String(imuData.orientation.z) + ";TS:" + String(timeStamp) + ";}@\n",false);
    writeData("@{PA:" + String(currentAlt) + ";TS:" + String(timeStamp) + ";}@\n",false);
    writeData("@{AX:" + String(imuData.accel.x) + ";AY:" + String(imuData.accel.y) + ";AZ:" + String(imuData.accel.z) + ";TS:" + String(timeStamp) + ";}@\n",false);
    //flightData.close();
    //Serial.println();
    //Serial.println("@{AX:" + String(imuData.accel.x) + ";AY:" + String(imuData.accel.y) + ";AZ:" + String(imuData.accel.z) + ";TS:" + String(timeStamp) + ";}@\n");
    //Serial.println("@{GX:" + String(imuData.gyro.x) + ";GY:" + String(imuData.gyro.y) + ";GZ:" + String(imuData.gyro.z) + ";TS:" + String(timeStamp)+ ";}@\n");
    //Serial.println("@{PA:" + String(currentAlt) + ";TS:" + String(timeStamp) + ";}")
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
    //delay(2);
}
