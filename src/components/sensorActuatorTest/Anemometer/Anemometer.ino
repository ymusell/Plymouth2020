#include "TimerOne.h"
#include <math.h>

#define WindSensor_Pin (2) // digital pin for wind speed sensor
#define WindVane_Pin (A14) // analog pin for wind direction sensor
#define VaneOffset 0 // define the offset for caclulating wind direction // TODO, lien avec le nord

volatile bool isSampleRequired; // this is set every 2.5sec to generate wind speed
volatile unsigned int timerCount; // used to count ticks for 2.5sec timer count
volatile unsigned long rotations; // cup rotation counter for wind speed calcs
volatile unsigned long contactBounceTime; // timer to avoid contact bounce in wind speed sensor
volatile float windSpeed;

int vaneValue; // raw analog value from wind vane
int vaneDirection; // translated 0 - 360 wind direction
int calDirection; // calibrated direction after offset applied
int lastDirValue; // last recorded direction value

int val = 0;

void setup() {

  // setup anemometer values
  lastDirValue = 0;
  rotations = 0;
  isSampleRequired = false;

  // setup timer values
  timerCount = 0;
  
  Serial.begin(9600);
  Serial.println("Davis Sensor Test");
  Serial.println("Speed Direction");

  pinMode(WindSensor_Pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(WindSensor_Pin), isr_rotation, FALLING);

  // setup the timer for 0.5 second
  Timer1.initialize(500000);
  Timer1.attachInterrupt(isr_timer);

  sei();// Enable Interrupts
}

  void loop() {
    val = digitalRead(WindSensor_Pin);
    Serial.println(val);
//  if(isSampleRequired) {
//  
//    getWindDirection();
//
//    Serial.print(windSpeed); Serial.print(" mph\t");
//    Serial.print(calDirection); Serial.println("*");
//    
//    isSampleRequired = false;
//  }
}

// Interrupt handler routine for timer interrupt
  void isr_timer() {
  
  timerCount++;
  
  if(timerCount == 5) {
    windSpeed = rotations * 0.9;
    rotations = 0;
    isSampleRequired = true;
    timerCount = 0;
  }
}

// Interrupt handler routine to increment the rotation count for wind speed
void isr_rotation() {
  
  if((millis() - contactBounceTime) > 15 ) { // debounce the switch contact
    rotations++;
    contactBounceTime = millis();
  }
}

// Get Wind Direction
void getWindDirection() {

  vaneValue = analogRead(WindVane_Pin);
  vaneDirection = map(vaneValue, 0, 1023, 0, 359);
  calDirection = vaneDirection + VaneOffset;
  
  if(calDirection > 360)
  calDirection = calDirection - 360;
  
  if(calDirection > 360)
  calDirection = calDirection - 360;
} 
