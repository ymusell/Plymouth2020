#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// For the rudder
#define SERVOMINRUDDER  160 // This is the 'minimum' pulse length count (out of 4096), after some tries the value for SERVOMIN for an angle of 0 degree is 160  
#define SERVOMAXRUDDER  425 // This is the 'maximum' pulse length count (out of 4096),and the maw value for SERVOMAX is 470


#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

uint8_t servonum = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Test for the first channel servo!");

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
}

void loop() {
/*  // First Step to find the value of SERVOMIN and SERVOMAX or to test the other servo channel
  // Drive each servo one at a time using setPWM()
  Serial.print("servonum = ");
  Serial.println(servonum);
  Serial.println("enter in the Servomin");
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    pwm.setPWM(servonum, 0, pulselen);
    Serial.print("pulselen = ");
    Serial.println(pulselen);
  }

  delay(2000);  //Wait for 2 s between each way
  Serial.println("enter in the Servomax");
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(servonum, 0, pulselen);
    Serial.print("pulselen = ");
    Serial.println(pulselen);
  }

  delay(2000);  //Wait for 2 s

//  servonum++; // If you want to test the first 8 servo channels
//  if (servonum > 7) servonum = 0; 
*/

  // Validation of the values found for the rudder
  Serial.println("O deg");
  pwm.setPWM(servonum, 0, pulseWidthRudder(-PI/4));
  delay(2000);
  Serial.println("9O deg");
  pwm.setPWM(servonum, 0, pulseWidthRudder(0));
  delay(2000);
  Serial.println("18O deg");
  pwm.setPWM(servonum, 0, pulseWidthRudder(PI/4));
  delay(2000);

}

double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int pulseWidthRudder(double angle) //The angle is in rad
{
  int pulse_wide;
  Serial.println(angle);
  pulse_wide   = mapf(angle,-PI/2,PI/2,SERVOMINRUDDER,SERVOMAXRUDDER);
  Serial.println(pulse_wide);
  return pulse_wide;
}
