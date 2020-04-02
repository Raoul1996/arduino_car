# Arduino 蓝牙遥控+超声避障小车

## 成品预览

![](https://imgkr.cn-bj.ufileos.com/2388d1f7-bd3b-4575-8b8b-a7b863cd17fc.jpg)
不好意思，拿错了。

这张才是我做的（超声波模块被我拔了）：

![](https://imgkr.cn-bj.ufileos.com/aae30482-c966-4316-80a2-5c3069efe13e.JPG)


## 材料

1. Arduino UNO 一块(和毕设老师要的)
2. L298N 电机驱动一个(淘宝6块钱)
3. 三轮小车底座一个（淘宝15块钱左右）
4. 铜柱、螺丝、螺母、杜邦线若干
5. 电池四节、电池盒一个
6. HC-06 蓝牙模块一个
7. 蓝牙串口助手 app
8. HC_SR04 超声波模块一个
9. 差速电机两个，一般买底盘会带，为了防止烧坏建议多备用一个
10. Arduino 开发工具

## Arduino UNO 片上资源

![](https://imgkr.cn-bj.ufileos.com/459e063a-1fd7-4220-b025-2e0154a1594f.png)

其中，左下角部分支持模拟量（Analog）的PIN（A0到A5）的编号是14到19，同样支持输出PWM，之后我们会用到。

## L298N 模块 PWM 调速

模块接法、PWM 调速原理

## HC_SR04 超声波模块测距

距离计算，接法

## HC-06 蓝牙模块遥控

接法、蓝牙遥控指令

## 任务调度

调速、测距任务与遥控任务

## 完整代码

完整代码访问 [GitHub](https://github.com/Raoul1996/arduino_car) 获取，或者公众号对话框回复“arduino小车”也可

```c++
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
```
