#include <SoftwareSerial.h>
#include <TaskScheduler.h>

#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define TURNLEFT 3
#define TURNRIGHT 4
#define CHANGESPEED 5


#define DEFAULTVOLTAGE 150

#define MINVOLTAGE 0

#define MAXVOLTAGE 255

#define LOWSPEED 0
#define HIGHSPEED 1

SoftwareSerial bluetoothSerial(11, 12); // RX, TX

 Scheduler runner;

 void ultrasonicTaskCallback();
 void bluetoothTaskCallback();

Task bluetoothTask(150,TASK_FOREVER,&bluetoothTaskCallback, &runner, true);
Task ultrasonicTask(500,TASK_FOREVER,&ultrasonicTaskCallback, &runner,true);


// declare motor driver module ctrl pin:
int leftMotorPin1 = 16;
int leftMotorPin2 = 17;
int rightMotorPin1 = 18;
int rightMotorPin2 = 19;

int leftPWM = 9;
int rightPWM = 10;

int ultrasonicInputPin = 7;
int ultrasonicOutputPin = 8;

int motorAction = STOP;
int motorVoltage = MINVOLTAGE;
int motorSpeedLevel = HIGHSPEED;

int distance = 0;
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


void motorRun(int cmd) {
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
void bluetoothTaskCallback() {
  if (bluetoothSerial.available()) {
    int bluetoothCmd = (int)bluetoothSerial.read();
    Serial.print("bluttooth Command: ");
    Serial.println(bluetoothCmd);
    motorCtrl(bluetoothCmd);
  }
  if (Serial.available()) {
    Serial.print("Serial available: ");
    Serial.println(Serial.read());
    bluetoothSerial.write(Serial.read());
  }
}

void ultrasonicTaskCallback() {
  int distance = getDistance();
  if (distance <= 30) {
    motorRun(STOP);
    Serial.print("distance is dangerous: ");
    Serial.println(distance);
    if (bluetoothSerial.available()) {
      bluetoothSerial.print("distance is dangerous: ");
      bluetoothSerial.println(distance);
    }
    changeSpeedLevel(LOWSPEED);
    motorRun(STOP);
  } else if (distance >= 100) {
    if (bluetoothSerial.available()) {
      bluetoothSerial.print("distance is fine,speed up: ");
      bluetoothSerial.println(distance);
    }
    motorRun(motorAction);
    changeSpeedLevel(HIGHSPEED);
  } else {
    if (bluetoothSerial.available()) {
      bluetoothSerial.print("distance is just ok: ");
      bluetoothSerial.println(distance);
    }
    motorRun(motorAction);
  }

}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  bluetoothSerial.begin(115200);
   bluetoothSerial.print("AT");
  delay(1000);
  bluetoothSerial.println("AT+VERSION");
  delay(1000);
  bluetoothSerial.println("AT+BAUD8");
  delay(1000);
  bluetoothSerial.println("AT+NAMEarduino_car");
  delay(1000);
  // set pin mode for motor
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(leftPWM, OUTPUT);
  pinMode(rightPWM, OUTPUT);

  pinMode(ultrasonicInputPin, INPUT);
  pinMode(ultrasonicOutputPin, OUTPUT);
  changeSpeedLevel(motorSpeedLevel);
  runner.startNow();
  Serial.println("setup has been down.");
}
void motorCtrl(int cmd) {
  if (cmd >= STOP && cmd <= TURNRIGHT) {
    motorAction = cmd;
    motorRun(motorAction);
  } else if (cmd > DEFAULTVOLTAGE && cmd <= MAXVOLTAGE) {
    motorVoltage = cmd;
    changeSpeed(cmd);
  } else if (cmd == CHANGESPEED) {
    if (motorSpeedLevel == LOWSPEED) {
      motorSpeedLevel = HIGHSPEED;
    } else {
      motorSpeedLevel = LOWSPEED;
    }
    changeSpeedLevel(motorSpeedLevel);
  }

}

int getDistance () {
  digitalWrite(ultrasonicOutputPin, LOW);
  delayMicroseconds(2);
  digitalWrite(ultrasonicOutputPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultrasonicOutputPin, LOW);

  int distance = pulseIn(ultrasonicInputPin, HIGH);
  distance = distance / 58;
  return distance > 0 ? distance: 0;
}

void loop() {
  runner.execute();
}
