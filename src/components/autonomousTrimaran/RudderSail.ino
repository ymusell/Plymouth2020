
void setupRudder(){
  nh.subscribe(rudderSub);
  
}


void controlRudder(){
  pwm.setPWM(ACTUATOR_RUDDER,0,pulseWidthRudder(rudderAngle));
  //delay(10);
}

// commande between -pi/4 et pi/4
void rudderCallBack(const std_msgs::Float32& cmd){
  if (watchRc == 0){
    //if rc is off
    rudderAngle = min(PI/4,cmd.data);
    rudderAngle = max(-PI/4,rudderAngle);
//    char text[] = "la valeur est";
//    char result[8];
//    dtostrf(rudderAngle, 6, 2, result); 
//    nh.loginfo(text);
//    nh.loginfo(result);
    }
}
/*--------------------------------------------------------------*/


void setupSail(){
  nh.subscribe(sailSub);
}


void controlSail(){
  pwm.setPWM(SAIL_PIN,0,pulseWidth(sailAngle));
  //delay(10);
}

//commande between 0 et pi/2
void sailCallBack(const std_msgs::Float32& cmd){
  if (watchRc == 0){
  //if rc is off
    sailAngle = max(0,cmd.data);
    sailAngle = min(PI/2,sailAngle);
    sailAngle = (sailAngle-0)*(maxSailAngle-minSailAngle)/(PI/2 - 0)+minSailAngle;
    
    
    //nh.loginfo("envoie angle sail...");
    }
}


/*--------------------- Functions ----------------------------*/

int pulseWidth(int angle)
{
  int analog_value;
  analog_value   = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  return analog_value;
}

// New function map for float and double
double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int pulseWidthRudder(double angle) //The angle is in rad
{
  int pulse_wide;
  pulse_wide = mapf(angle,-PI/2,PI/2,SERVOMINRUDDER,SERVOMAXRUDDER);
  return pulse_wide;
}
