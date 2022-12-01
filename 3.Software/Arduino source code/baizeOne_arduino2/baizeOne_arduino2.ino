#include "WiFi.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <rosserial_arduino/Adc.h>
#include <robot_msg/quadrupedrobot_jointstate.h>
#include <Wire.h>
#include <SimpleFOC.h>


int rec[18] = {
  320,320,327,
  327,345,310,
  350,327,310,
  350,327,337,
  327,327,350,
  350,340,320
};
int direct[18] = {1,1,1,
1,1,1,
1,1,1,
1,1,1,
1,1,1,
1,1,1
};

BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(17, 18, 19, 21, 22, 23);


//目标变量
float target_velocity = 0;

//设置AP信息(SSID和PASSWORD)
const char* ssid = "S725";
const char* password = "s725s725";

IPAddress server(192, 168, 1, 106); // ip of your ROS server
IPAddress ip_address;
int status = WL_IDLE_STATUS;

WiFiClient client;
 
class WiFiHardware {
 
  public:
  WiFiHardware() {};
 
  void init() {
    // do your initialization here. this probably includes TCP server/client setup
    client.connect(server, 11411);
  }
 
  // read a byte from the serial port. -1 = failure
  int read() {
    // implement this method so that it reads a byte from the TCP connection and returns it
    //  you may return -1 is there is an error; for example if the TCP connection is not open
    return client.read();         //will return -1 when it will works
  }
 
  // write data to the connection to ROS
  void write(uint8_t* data, int length) {
    // implement this so that it takes the arguments and writes or prints them to the TCP connection
    for(int i=0; i<length; i++)
      client.write(data[i]);
  }
 
  // returns milliseconds since start of program
  unsigned long time() {
     return millis(); // easy; did this one for you
  }
};
 
int i;

void JointStateCallback(const robot_msg::quadrupedrobot_jointstate& jointstate) {
  
  motor.move(jointstate.position[1]);

  // We can now plot text on screen using the "print" class
  delay(1);
}
 
 
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
ros::Subscriber<robot_msg::quadrupedrobot_jointstate> subjoint("/quadruped_joint", JointStateCallback);

ros::NodeHandle_<WiFiHardware> nh;
char hello[20] = "ESP32 wifi alive!";
 
 
void setupWiFi()
{
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to "); Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if(i == 21){
    Serial.print("Could not connect to"); Serial.println(ssid);
    while(1) delay(500);
  }
  Serial.print("Ready! Use ");
  Serial.print(WiFi.localIP());
  Serial.println(" to access client");
}

void setup() {
  Serial.begin(115200);
  setupWiFi();
  delay(2000);
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(subjoint);

  delay(100);

  driver.voltage_power_supply = 12.0;
  driver.init();
  motor.linkDriver(&driver);
  motor.voltage_limit = 12.0;   // [V]
  motor.velocity_limit = 15; // [rad/s]
  
  //开环控制模式设置
  motor.controller = MotionControlType::angle_openloop;

  //初始化硬件
  motor.init();


//  Serial.println("Motor ready!");
//  Serial.println("Set target velocity [rad/s]");
  _delay(1000);
}
 
void loop() {
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(5);
}
