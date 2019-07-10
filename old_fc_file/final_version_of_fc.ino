#include <Servo.h>
#include <Wire.h>
#include <MadgwickAHRS.h>
#include <PID_v1.h>
//#include "Temprature.h"
// make filter for orientation
Madgwick filter;

//accel related variable
long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

//gyro related variable
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

//oritation related variable
float roll, pitch, heading;

// count sample rate
unsigned long microsPerReading, microsPrevious;

// PID library setup
double SetpointRoll = 0.0;// roll should be 90 when IMU is standing
double SetpointPitch = 0.0;
double InputRoll, OutputRoll, InputPitch, OutputPitch;

PID rollPID(&InputRoll, &OutputRoll, &SetpointRoll, 1, 0, 0, DIRECT);
PID pitchPID(&InputPitch, &OutputPitch, &SetpointPitch, 1, 0, 0, DIRECT);
// servo setup
Servo servoX;
Servo servoY;

//Temprature temp(21);

void setup()
{
  // turn off LED build in
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  // attach servo X and Y
  servoX.attach(6); // roll control
  servoY.attach(7); // pitch control
  servoX.write(85);
  servoY.write(85);
  Serial.begin(9600);
  Wire.begin();
  setupMPU();

  // turn on PID
  rollPID.SetOutputLimits(-30, 30);
  rollPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(-30, 30);
  pitchPID.SetMode(AUTOMATIC);

  // 加速度计校准
  for (int cal_int = 0; cal_int < 2000; cal_int++)
  {                                //Run this code 2000 times
    recordGyroRegistersForSetUp(); //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyroX;           //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyroY;           //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyroZ;           //Add the gyro z-axis offset to the gyro_z_cal variable
  }
  gyro_x_cal /= 2000; //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000; //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000; //Divide the gyro_z_cal variable by 2000 to get the avarage offset
  //setup filter setup sample rate of IMU data to 25Hz
  filter.begin(25);
  //always at last line 25Hz for the filter
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();
}
// for plane X axis is PITCH, Y is ROll, Z is yaw

void loop()
{
  Serial.println("logging");
  // pitch is Y, roll is X
  unsigned long microsNow;
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading)
  {
    recordAccelRegisters();
    recordGyroRegisters();
    //printData();
    filter.updateIMU(rotX, rotY, rotZ, gForceX, gForceY, gForceZ);
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    InputRoll = roll;
    InputPitch = pitch;
    rollPID.Compute();
    pitchPID.Compute();
    printOritation();
    servoX.write(85 + OutputRoll);
    servoY.write(85 + OutputPitch);
    delay(15);
    microsPrevious = microsPrevious + microsPerReading; // very imporant line, do not remove
  }
}

//print roll pitch yaw and PID
void printOritation()
{
  Serial.print("Roll:");
  Serial.print(roll);
  Serial.print(" Pitch:");
  Serial.print(pitch);
  Serial.print(" Yaw:");
  Serial.println(heading);
}

// call in setup()
void setupMPU()
{
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

// get accel data
void recordAccelRegisters()
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

// process accel data
void processAccelData()
{
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0;
  gForceZ = accelZ / 16384.0;
}

// read gyro for setup caliberation
void recordGyroRegistersForSetUp()
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

// get gyro data
void recordGyroRegisters()
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

// process gyro data
void processGyroData()
{
  gyroX -= gyro_x_cal;
  gyroY -= gyro_y_cal;
  gyroZ -= gyro_z_cal;
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0;
  rotZ = gyroZ / 131.0;
}

void printData()
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
