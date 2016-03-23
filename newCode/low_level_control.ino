float rotPID(float rot) {//Roll PID controller, needs to be tuned significantly
  float error = -(rot - imu.roll);
  rollI = rollI + error/100.0;
  rotOutput = param.rollkp*error + param.rollki+rollI + rotOutput;
  rotOutput = constrain(rotOutput,-45.0,45.0);
  return rotOutput;  
}

static float error_prev, error_act;
int linPIDRate(int lin) {//Linear PID controller
  float P,D;
  error_prev = error_act;
  error_act = lin - imu.pitch;
  P = error_act;
  linI = linI + error_act;

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
