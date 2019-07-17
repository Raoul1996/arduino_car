# Arduino Toy Car

> Just a toy car

## Introduction

### Hardware

1. arduino UNO 1 块
2. HC-06 bluetooth(别名 JY-MCU) 模块 1 个
3. 数字电压表 1 块
4. 差速电机 2 个
5. 小车底座 1 个
6. 1.5V 干电池 4 节
7. 干电池盒 1 个
8. 杜邦线、铜柱、螺丝、螺母若干

### Tools

1. 热熔胶
2. 烙铁焊锡
3. 螺丝刀
4. 热缩管
5. 剥线钳（俩门牙）

### Software

2. Arduino offical IDE

### Pin && Connection

| Arduino | L298N | HC-06 Bluetooth | HYSR04 | Voltage Source |
| ------- | ----- | --------------- | ------ | -------------- |
| Vin     | +12v  |                 |        | VCC            |
| GND     | GND   | GND             | GND    | GND            |
| +3.3V   |       | VCC             |        |                |
| +5V     |       |                 | VCC    |                |
| 11      |       | TXD             |        |                |
| 12      |       | RXD             |        |                |
| 16      | IN1   |                 |        |                |
| 17      | IN2   |                 |        |                |
| 18      | IN3   |                 |        |                |
| 19      | IN4   |                 |        |                |
| 9       | ENA   |                 |        |                |
| 10      | ENB   |                 |        |                |
| 11      |       |                 | Trig   |                |
| 12      |       |                 | Echo   |                |

## Feature


- [x] 电机差速调速，使用 L298 驱动
    - ST公司生产的 L298 芯片包含两个 H 桥，驱动两轮车实在是有点大材小用，同时因为体积较大，一般用来驱动四轮小车。5块钱一个的高昂价格也确实让我有些不堪重负，所以可以买一些比较便宜的电机驱动（1块钱左右）。不过便宜电机驱动选购时需要注意其适用电压，有可能4节干电池（5-6V）驱动不起来。

- [x] 蓝牙控制车的运行状态
    - 蓝牙使用 HC-06 蓝牙模块。注意使用 AT 指令修改其名称。同时为了方便调试和开发，蓝牙软串口和板子自带串口设置相同波特率

- [x] 接入HYSR04模块测距
    > 使用 HYSR04 测定距离。公式推导如下
    > 声音在干燥 20 摄氏度的空气中的传播速度大约为 34300 cm/s, 走过 1cm 需要的时间是 29.15 微秒。由于超声波的工作原理，所以实际的距离是路程的一半。所以实际距离（单位：cm）需要是发射与接收时间差（单位：微秒）除以 29.15 * 2 = 58.3，约为 58

- [x] 蓝牙控制车前进的同时，超声波防止车撞上障碍物
    - 在嵌入式开发中，可以使用任务调度器来实现对多个任务的同时运行。不能使用 delay 函数实现延时，因为 delay会阻塞 CPU。也可以自己使用 mills 函数实现任务调度，比较困难。选用了[TaskScheduler](https://github.com/arkhipenko/TaskScheduler)作为任务调度器。

- [] 舵机 + 超声波，自动避障 
    - 舵机还没到买，计划使用 9g 舵机，性能应该足够

- [] 18650 锂电池
    - 同没买，电池盒已经有了。一节电压 3.7V - 4.2V，两节就够

- [] 循迹模块
    - 有两个，传感器性能堪忧，至少需要四路吧

- [] NodeMCU + Arduino 实现 Wifi 控制
    - 使用串口通信
