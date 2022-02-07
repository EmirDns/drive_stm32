#include <Arduino.h>
#include "HardwareSerial.h"
#include "VescUart.h"
#include "ros.h"
#include "std_msgs/Int64.h"

std_msgs::Int64 rpm_data;

ros::NodeHandle nh;
ros::Publisher rpm_pub("rpm_topic",&rpm_data);

HardwareSerial mySerial3(PB11, PB10);
HardwareSerial mySerial2(PC5, PC4);

VescUart motor; 

void setup() {
  mySerial3.begin(115200);
  mySerial2.begin(9600);

  while(!mySerial3) {;}

  motor.setSerialPort(&mySerial3);
  nh.initNode();
  nh.advertise(rpm_pub);
}

void loop() {
  motor.setRPM(-1000);
  
  if(motor.getVescValues()){
    mySerial2.print("RPM:");
    mySerial2.println(motor.data.rpm);
  }
  else{
    mySerial2.println("Failed to get the RPM data.");
  }
  
  
  nh.spinOnce();
  if(motor.getVescValues()){
    rpm_data.data=motor.data.rpm;
    rpm_pub.publish(&rpm_data);
  }
  else{
    nh.logerror("Failed to publish rpm data.");
  }
  delay(50);
}
