#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  160 // This is the 'minimum' pulse length count (out of 4096), after some tries the value for SERVOMIN for an angle of 0 degree is 160  
#define SERVOMAX  470 // This is the 'maximum' pulse length count (out of 4096),and th emaw value for SERVOMAX is 470
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

uint8_t servonum = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Test for the first channe servo!");

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
  // Validation of the values found
  Serial.println("O deg");
  pwm.setPWM(servonum, 0, pulseWidth(0));
  delay(2000);
  Serial.println("9O deg");
  pwm.setPWM(servonum, 0, pulseWidth(90));
  delay(2000);
  Serial.println("18O deg");
  pwm.setPWM(servonum, 0, pulseWidth(180));
  delay(2000);

}


int pulseWidth(int angle)
{
  int pulse_wide, analog_value;
  pulse_wide   = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  return pulse_wide;
}
