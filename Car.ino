#include <SoftwareSerial.h>

SoftwareSerial bluetoothSerial(2, 3); // RX, TX

// define motor action,

#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define TURNLEFT 3
#define TURNRIGHT 4
#define CHANGESPEED 5


#define DEFAULTVOLTAGE 120
#define MINVOLTAGE 0
#define MAXVOLTAGE 255

#define LOWSPEED 0
#define HIGHSPEED 1
// declare motor driver module ctrl pin:
int leftMotorPin1 = 4;
int leftMotorPin2 = 5;
int rightMotorPin1 = 6;
int rightMotorPin2 = 7;
int leftPWM = 9;
int rightPWM = 10;
int motorAction = STOP;
int motorVoltage = MINVOLTAGE;
int motorSpeedLevel = LOWSPEED;

/*
  motor ctrl matrix
  1. stop
  2. forward
  3. backward
  4. turn left
  5. turn right
*/
int ctrlMatrix[5][4] = {
  {LOW, LOW, LOW, LOW},
  {LOW, HIGH, LOW, HIGH},
  {HIGH, LOW, HIGH, LOW},
  {HIGH, LOW, LOW, HIGH},
  {LOW, HIGH, HIGH, LOW}
};

void motorRun(int cmd)
{
  Serial.print("recv cmd: ");
  Serial.println(cmd);
  digitalWrite(leftMotorPin1, ctrlMatrix[cmd][0]);
  digitalWrite(leftMotorPin2, ctrlMatrix[cmd][1]);
  digitalWrite(rightMotorPin1, ctrlMatrix[cmd][2]);
  digitalWrite(rightMotorPin2, ctrlMatrix[cmd][3]);
}

void changeSpeed(int voltage) {
  Serial.print("recv voltage: ");
  Serial.println(voltage);
  analogWrite(leftPWM, voltage);
  analogWrite(rightPWM, voltage);
}
void changeSpeedLevel(int level) {
  Serial.print("recv voltage level: ");
  Serial.println(level);
  if (level == HIGHSPEED) {
    analogWrite(leftPWM, MAXVOLTAGE);
    analogWrite(rightPWM, MAXVOLTAGE);
  } else {
    analogWrite(leftPWM, DEFAULTVOLTAGE);
    analogWrite(rightPWM, DEFAULTVOLTAGE);
  }

}
void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  bluetoothSerial.begin(115200);
  bluetoothSerial.print("AT");
  Serial.println("AT");
  delay(1000);
  bluetoothSerial.print("AT+VERSION");
  delay(1000);
  bluetoothSerial.print("AT+BAUD8");
  delay(1000);
  bluetoothSerial.print("AT+NAMEarduino_car");
  delay(1000);
  // set pin mode for motor
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(leftPWM, OUTPUT);
  pinMode(rightPWM, OUTPUT);
  Serial.println("setup has been down.");
}

void loop()
{
  if (bluetoothSerial.available()) {
    int bluetoothCmd = (int)bluetoothSerial.read();
    Serial.print("bluttooth Command: ");
    Serial.println(bluetoothCmd);
    if (bluetoothCmd >= STOP && bluetoothCmd <= TURNLEFT) {
      motorAction = bluetoothCmd;
      motorRun(motorAction);
    } else if (bluetoothCmd > DEFAULTVOLTAGE && bluetoothCmd <= MAXVOLTAGE) {
      motorVoltage = bluetoothCmd;
      changeSpeed(bluetoothCmd);
    } else if (bluetoothCmd == CHANGESPEED) {
      if (motorSpeedLevel == LOWSPEED) {
        motorSpeedLevel = HIGHSPEED;
      } else {
        motorSpeedLevel = LOWSPEED;
      }
      changeSpeedLevel(motorSpeedLevel);
    }

  }
  if (Serial.available()) {
    Serial.print("Serial available: ");
    Serial.println(Serial.read());
    bluetoothSerial.write(Serial.read());
  }
}
