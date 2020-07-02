

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
  //setupImuCmps();
  //setupRC();
  setupWind();
 }


void updateSensor(){
  updateImu();
  //updateImuCmps();
  //updateRC();
  updateWind();
  
}
