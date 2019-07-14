# Arduino Toy Car

> 只是个小玩具

## Introduction

### Hardware

1. arduino UNO 1 块
2. HC-06 bluetooth(别名 JY-MCU) 1 个
3. 数字电压表 1 块
4. 差速电机 2 个
5. 小车底座 1 个

### Software

1. VSCode
2. Arduino 官方编辑器

### PIN Name confusion

| Arduino | L298N | HC-06 Bluetooth |
| ------- | ----- | --------------- |
| Vin     | +12v  |                 |
| GND     | GND   | GND             |
| +3.3V   |       | VCC             |
| 2       |       | TXD             |
| 3       |       | RXD             |
| 4       | IN4   |                 |
| 5       | IN3   |                 |
| 6       | IN2   |                 |
| 7       | IN1   |                 |
| 9       | ENA   |                 |
| 10      | ENB   |                 |

## Voltage

6V - 12V


