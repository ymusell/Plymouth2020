
#include <Adafruit_PWMServoDriver.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>

#ifndef CONFIG_H
#define CONFIG_H


/*******************************Suscriber*************************************/
void rudderCallBack(const std_msgs::Float32& cmd);
ros::Subscriber<std_msgs::Float32> rudderSub("mode_send_u_rudder", &rudderCallBack );


void sailCallBack(const std_msgs::Float32& cmd);
ros::Subscriber<std_msgs::Float32> sailSub("mode_send_u_sail", &sailCallBack );




/*******************************Actuator**************************************/
#define NB_ACTUATOR 2

#define ACTUATOR_RUDDER    0
#define ACTUATOR_SAIL    1

/****************Configuration*******************/   
#define MIN_PULSE_WIDTH       160 //650
#define MAX_PULSE_WIDTH       425 //2350
//#define DEFAULT_PULSE_WIDTH   1500
#define FREQUENCY             50

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMINRUDDER  160 
#define SERVOMAXRUDDER  425

float rudderAngle = 293;
//float maxRudderAngle = 120;
//float minRudderAngle = 30;

float sailAngle = 22;
float maxSailAngle = 55;
float minSailAngle = 0;

/************** PIN******************************/

#define RUDDER_PIN 12
#define SAIL_PIN   1

//// rc module 
//
//
//#define MAX_CHRUDDER 1980
//#define MIN_CHRUDDER 990
//
//#define MAX_CHSAIL 1950
//#define MIN_CHSAIL 980
//
//unsigned long timerRc;
//
///* pin */ 
//const int chPinRudder = 4; // channel 1 sur le pin 4
//const int chPinSail = 5; // channel 2 sur le pin 5
//
//float chRudder;
//float chSail;
//
//unsigned long duration = 10000;
//geometry_msgs::Vector3 rcMsg;
//ros::Publisher pubRc("ardu_send_RC",&rcMsg);

#endif
