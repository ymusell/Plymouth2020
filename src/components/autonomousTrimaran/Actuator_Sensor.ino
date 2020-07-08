

void setupActuator(){
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  setupRudder();
  setupSail();
}



void Actuator(){
  controlRudder();
  controlSail();
}


/*************************************************************************************/

void setupSensor(){
  setupImu();
  //setupRC();
  setupWind();
 }


void updateSensor(){
  updateImu();
  //updateRC();
  updateWind();
  
}
