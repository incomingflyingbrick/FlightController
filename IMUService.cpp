#include <math.h>
#include "IMUService.h"
#include <Wire.h>
#include <MadgwickAHRS.h>
#include <Arduino.h>

IMUService::IMUService(int rate) // should be called in setup part
{
    refreshRate = rate;
}

void IMUService::prepare()
{
    setupIMU();
    // 加速度计校准,2000次取值,然后去平均值
    for (int cal_int = 0; cal_int < 2000; cal_int++)
    {                                  //Run this code 2000 times
        recordGyroRegistersForSetUp(); //Read the raw acc and gyro data from the MPU-6050
        gyro_x_cal += gyroX;           //Add the gyro x-axis offset to the gyro_x_cal variable
        gyro_y_cal += gyroY;           //Add the gyro y-axis offset to the gyro_y_cal variable
        gyro_z_cal += gyroZ;           //Add the gyro z-axis offset to the gyro_z_cal variable
    }
    gyro_x_cal /= 2000; //Divide the gyro_x_cal variable by 2000 to get the avarage offset
    gyro_y_cal /= 2000; //Divide the gyro_y_cal variable by 2000 to get the avarage offset
    gyro_z_cal /= 2000; //Divide the gyro_z_cal variable by 2000 to get the avarage offset
    filter.begin(refreshRate);
    microsPrevious = micros();
}

void IMUService::loopIMU()
{
    unsigned long microsNow;
    microsNow = micros();
    if (microsNow - microsPrevious >= microsPerReading)
    {
        recordAccelRegisters();
        recordGyroRegisters();
        filter.updateIMU(rotX, rotY, rotZ, gForceX, gForceY, gForceZ);
        roll = filter.getRoll();
        pitch = filter.getPitch();
        heading = filter.getYaw();
        printOritation();
        //increment previous time, so we keep proper pace
        microsPrevious = microsPrevious + microsPerReading;
    }
}

bool IMUService::freeFallDetection(float gForceX, float gForceY, float gForceZ)
{
    float totalAccel = pow(gForceX, 2) + pow(gForceY, 2) + pow(gForceZ, 2);
    if (sqrt(totalAccel) <= IMUService::freeFallThreshold) //free fall threadhold 0.25g
    {
        return true;
    }
    return false;
}

float IMUService::getRoll()
{
    return filter.getRoll();
}
float IMUService::getPitch()
{
    return filter.getPitch();
}
float IMUService::getYaw()
{
    return filter.getYaw();
}

void IMUService::setupIMU()
{
    Wire.begin();
    Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
    Wire.write(0x6B);                  //Accessing the register 6B - Power Management (Sec. 4.28)
    Wire.write(0b00000000);            //Setting SLEEP register to 0. (Required; see Note on p. 9)
    Wire.endTransmission();
    Wire.beginTransmission(0b1101000); //I2C address of the MPU
    Wire.write(0x1B);                  //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4)
    Wire.write(0x00000000);            //Setting the gyro to full scale +/- 250deg./s
    Wire.endTransmission();
    Wire.beginTransmission(0b1101000); //I2C address of the MPU
    Wire.write(0x1C);                  //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5)
    Wire.write(0b00000000);            //Setting the accel to +/- 2g
    Wire.endTransmission();
}

// read gyro for setup caliberation
void IMUService::recordGyroRegistersForSetUp()
{
    Wire.beginTransmission(0b1101000); //I2C address of the MPU
    Wire.write(0x43);                  //Starting register for Gyro Readings
    Wire.endTransmission();
    Wire.requestFrom(0b1101000, 6); //Request Gyro Registers (43 - 48)
    while (Wire.available() < 6)
        ;
    gyroX = Wire.read() << 8 | Wire.read(); //Store first two bytes into accelX
    gyroY = Wire.read() << 8 | Wire.read(); //Store middle two bytes into accelY
    gyroZ = Wire.read() << 8 | Wire.read(); //Store last two bytes into accelZ
}

// get accel data
void IMUService::recordAccelRegisters()
{
    Wire.beginTransmission(0b1101000); //I2C address of the MPU
    Wire.write(0x3B);                  //Starting register for Accel Readings
    Wire.endTransmission();
    Wire.requestFrom(0b1101000, 6); //Request Accel Registers (3B - 40)
    while (Wire.available() < 6)
        ;
    accelX = Wire.read() << 8 | Wire.read(); //Store first two bytes into accelX
    accelY = Wire.read() << 8 | Wire.read(); //Store middle two bytes into accelY
    accelZ = Wire.read() << 8 | Wire.read(); //Store last two bytes into accelZ
    processAccelData();
}

// get gyro data
void IMUService::recordGyroRegisters()
{
    Wire.beginTransmission(0b1101000); //I2C address of the MPU
    Wire.write(0x43);                  //Starting register for Gyro Readings
    Wire.endTransmission();
    Wire.requestFrom(0b1101000, 6); //Request Gyro Registers (43 - 48)
    while (Wire.available() < 6)
        ;
    gyroX = Wire.read() << 8 | Wire.read(); //Store first two bytes into accelX
    gyroY = Wire.read() << 8 | Wire.read(); //Store middle two bytes into accelY
    gyroZ = Wire.read() << 8 | Wire.read(); //Store last two bytes into accelZ
    processGyroData();
}

void IMUService::processAccelData()
{
    gForceX = accelX / 16384.0;
    gForceY = accelY / 16384.0;
    gForceZ = accelZ / 16384.0;
}

// process gyro data
void IMUService::processGyroData()
{
    gyroX -= gyro_x_cal;
    gyroY -= gyro_y_cal;
    gyroZ -= gyro_z_cal;
    rotX = gyroX / 131.0;
    rotY = gyroY / 131.0;
    rotZ = gyroZ / 131.0;
}

void IMUService::printData()
{
    Serial.print("Gyro (deg)");
    Serial.print(" X=");
    Serial.print(rotX);
    Serial.print(" Y=");
    Serial.print(rotY);
    Serial.print(" Z=");
    Serial.print(rotZ);
    Serial.print(" Accel (g)");
    Serial.print(" X=");
    Serial.print(gForceX);
    Serial.print(" Y=");
    Serial.print(gForceY);
    Serial.print(" Z=");
    Serial.println(gForceZ);
}

void IMUService::printOritation()
{
    Serial.print("Roll:");
    Serial.print(roll);
    Serial.print(" Pitch:");
    Serial.print(pitch);
    Serial.print(" Yaw:");
    Serial.println(heading);
}
