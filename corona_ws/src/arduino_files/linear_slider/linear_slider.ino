#include <ros.h>
#include <ArduinoHardware.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
//
//Pin definitions for linear slider
#define EN 6
#define DIR 7
#define PULSE 8
#define LED 9

int slideDelay = 200;
long slideCount = 5000;
ros::NodeHandle slideInterface;
std_msgs::Int32 slideOutput;
//sliderDelay sets speed. Higher delay = lower speed. Options for microsteps are 200,400,800,1600,3200,6400
//sliderInput sets the number of counts of microsteps and initiates movement. Negative is towards the motor, Positive is towards the end of the stroke length.


ros::Publisher slidePub("slide_output", &slideOutput);
void slideDelay_callback(const std_msgs::Int32& slide_delay){
  slideDelay = slide_delay.data;
}

void slideSub_callback(const std_msgs::Int64& slider_input){
  slideCount = slider_input.data;
  if(slideCount >= 0){
    for (int i=0; i<abs(slideCount); i++)    //Forward 5000 steps
    {
      digitalWrite(DIR,HIGH);
      digitalWrite(EN,HIGH);
      digitalWrite(PULSE,HIGH);
      delayMicroseconds(slideDelay);
      digitalWrite(PULSE,LOW);
      delayMicroseconds(slideDelay);
      slideInterface.spinOnce();
    }
  }
  else{
    for (int i=0; i<abs(slideCount); i++)    //Forward 5000 steps
    {
      digitalWrite(DIR,LOW);
      digitalWrite(EN,HIGH);
      digitalWrite(PULSE,HIGH);
      delayMicroseconds(slideDelay);
      digitalWrite(PULSE,LOW);
      delayMicroseconds(slideDelay);
      slideInterface.spinOnce();
    }
  }
  slideOutput.data = 1;
  slidePub.publish(&slideOutput);
}

void slideLED_callback(const std_msgs::Int32& LED_input){
  if(LED_input.data == 1){
    digitalWrite(LED, HIGH);
  }
  else if(LED_input.data == 0){
    digitalWrite(LED, LOW);
  }
  else{
    delay(1);
  }
}
ros::Subscriber<std_msgs::Int64> slideInputSub("slider_count", &slideSub_callback);
ros::Subscriber<std_msgs::Int32> slideDelaySub("slider_delay", &slideDelay_callback);
ros::Subscriber<std_msgs::Int32> slideLEDSub("slider_led", &slideLED_callback);

void setup() {
  // put your setup code here, to run once:
  pinMode(PULSE, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(EN, OUTPUT);
  pinMode(LED, OUTPUT);
  slideInterface.initNode();
  slideInterface.subscribe(slideInputSub);
  slideInterface.subscribe(slideDelaySub);
  slideInterface.subscribe(slideLEDSub);
  slideInterface.advertise(slidePub);
}

void loop() {
  
  ArduinoInterface.spinOnce();
  delay(1);
}

