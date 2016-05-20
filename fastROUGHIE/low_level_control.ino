float rotPID(float rot) {//Roll PID controller, needs to be tuned significantly
  float error = -(rot - imu.roll);
  rollI = rollI + error/100.0;
  rotOutput = param.rollkp*error + param.rollki+rollI + param.rollkd*imu.rollD + rotOutput;
  rotOutput = constrain(rotOutput,-45.0,45.0);
  return rotOutput;  
}

int linPIDRate(int lin) {//Linear PID controller
  float P = -(lin - imu.pitch);
  linI = linI + P/100.0;
  return param.linkp*P + param.linki*linI + param.linkd*imu.pitchD;
}

int linFuzzyRate(int lin) {//Linear fuzzy controller
  float error_act = abs(lin - imu.pitch);
  fuzzy->setInput(1,error_act);
  fuzzy->fuzzify();
  int rate = constrain(abs(fuzzy->defuzzify(1)),0,255);
  if (error_act < 0) {
    rate = -rate;
  }
  return rate;  
}
