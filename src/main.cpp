
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <Servo.h>
Servo servoX, servoY, servoZ;
MPU6050 mpu6050(Wire);
//...

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  
  servoX.attach(5);
  servoY.attach(6);
  servoZ.attach(7);
  
  servoX.write(0);
  servoY.write(0);
  servoZ.write(0);
}

void loop() {
  mpu6050.update();

  float valorSensorX = mpu6050.getAngleX();
  float valorSensorY = mpu6050.getAngleY();
  float valorSensorZ = mpu6050.getAngleZ();

  int valorServoX = map(valorSensorX,  -90, 90, 180, 0);
  int valorServoY = map(valorSensorY,  -90, 90, 180, 0);
  int valorServoZ = map(valorSensorZ,  -90, 90, 180, 0);

  servoX.write(valorServoX);
  servoY.write(valorServoY);
  servoZ.write(valorServoZ);
/*
  Serial.print("angulo X: ");
  Serial.print(valorSensorX);
  Serial.print("  \tangulo Y: ");
  Serial.print(valorSensorY);
  Serial.print("  \tangulo Z: ");
  Serial.println(valorSensorZ);
*/ 
///*
  Serial.print("Servo X: ");
  Serial.print(valorServoX);
  Serial.print("  \tServo Y: ");
  Serial.print(valorServoY);
  Serial.print("  \tServo Z: ");
  Serial.println(valorServoZ); 
//*/
  
}