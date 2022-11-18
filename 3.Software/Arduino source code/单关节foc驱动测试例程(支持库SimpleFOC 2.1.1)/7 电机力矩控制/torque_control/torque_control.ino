// Deng's FOC 开环位置控制例程 测试库：SimpleFOC 2.1.1 测试硬件：灯哥开源FOC V1.0
// 串口中输入"T+数字"设定两个电机的位置，如设置电机转到到180度，输入 "T3.14"（弧度制的180度）
// 在使用自己的电机时，请一定记得修改默认极对数，即 BLDCMotor(7) 中的值，设置为自己的极对数数字
// 程序默认设置的供电电压为 7.4V,用其他电压供电请记得修改 voltage_power_supply , voltage_limit 变量中的值
//               FOC_0.2  Baize_Foc
// IN1     pwm1     9        17
// IN2     pwm2     6        18
// IN3     pwm3     5        19
// INH1   enable1   8        21
// INH2   enable2   7        22
// INH3   enable3   4        23
// 电机极对数:11            减速比:11
//in-line current sense - phase 1/A 35
//in-line current sense - phase 1/C 34
// ESP32  iic接口  SCL:25   SDA:26
// AS5600 iic地址: 0x36
//I2Cone.begin(sda, scl, frequency); 

/**
Deng's FOC 电压力矩控制例程 测试库：SimpleFOC 2.1.1 测试硬件：灯哥开源FOC V1.0
上电后，两个电机的力矩值已经分别设置为3和-3，会各自反方向转动
在使用自己的电机时，请一定记得修改默认极对数，即 BLDCMotor(7) 中的值，设置为自己的极对数数字
程序默认设置的供电电压为 7.4V,用其他电压供电请记得修改 voltage_power_supply , voltage_limit 变量中的值
 */
#include <SimpleFOC.h>

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);

// Motor instance
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(17, 18, 19, 21, 22, 23);

//定义 TROT 步态变量
void setup() {
  I2Cone.begin(26, 25, 400000); 
  sensor.init(&I2Cone);
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12.0;
  driver.init();

  // link the motor and the driver
  motor.linkDriver(&driver);
  
  // set motion control loop to be used
  motor.controller = MotionControlType::torque;

  // contoller configuration 
  // default parameters in defaults.h


  // maximal voltage to be set to the motor
  motor.voltage_limit = 12.0;
  // use monitoring with serial 
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);
  
  //初始化电机
  motor.init();
  motor.initFOC();


  Serial.println("Motor ready.");
  _delay(1000);
  
}

void loop() {
  motor.loopFOC();
  motor.move(-10.0);
}
