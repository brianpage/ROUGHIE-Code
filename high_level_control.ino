int sawtooth(int &lin,int &rot,int &pump,int &linMode) {
  int pumpDone;
  if(currentState == DOWNGLIDE) {
    pump = param.tankBackLimit;
    pumpDone = checkPump(pump);//Returns true if done pumping
    //Linear Calculations
    if(linPID || linFuzzy) {
      linMode = PWM;
      lin = param.linNoseDownTarget;
    }
    if(feedforward && (!pumpDone || feedforward) && (!linPID && !linFuzzy)) {
      linMode = POSITION;
      lin = param.downFeedforward;
    }

    //Roll Calculations
    if(circle || (dubin && (millis() - t0 < param.dubinTime))) {
      rot = param.rollover;
    } else {
      rot = 0.0;
    }

    if(millis() - t0 > param.desTime) {
      currentState = NEUTRAL;
      nextState = UPGLIDE;
      t0 = millis();
    }
  }

  if(currentState == NEUTRAL) {
    pump = param.tankMid;
    pumpDone = checkPump(pump);//Returns true if done pumping

    if(millis() - t0 > param.neutralTime) {
      currentState = nextState;
      t0 = millis();
    }
  }

  if(currentState == UPGLIDE) {
    pump = param.tankFrontLimit;    
    pumpDone = checkPump(pump);//Returns true if done pumping
    //Linear Calculations
    if(linPID || linFuzzy) {
      linMode = PWM;
      lin = param.linNoseUpTarget;
    }
    if(feedforward && (!pumpDone || feedforward) && (!linPID && !linFuzzy)) {
      linMode = POSITION;
      lin = param.upFeedforward;
    }

    //Roll Calculations
    if(circle) {
      rot = -param.rollover;
    } else if(dubin && (millis() - t0 < param.dubinTime)) {
      rot = param.rollover;
    } else {
      rot = 0.0;
    }

        
    if(millis() - t0 > param.riseTime) {
      currentState = NEUTRAL;
      nextState = DOWNGLIDE;
      t0 = millis();
    }
  }

  if(linMode == PWM) {
    float error_prev, error_act;
    if(linPID) {
      lin = linPIDRate(lin);
    }
    if(linFuzzy) {
      lin = linFuzzyRate(lin);
    }
  }

  if(turnFeedback) {
    rot = rotPID(rot);
  }
}
