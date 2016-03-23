int sawtooth(int &lin,int &rot,int &pump,int &linMode) {
  //sawtooth calculates the required glide targets based on system parameters and timing
  
  int pumpDone;
  int glideCompleted = 0;//returns 0 unless it just completed a glide
  if(currentState == DOWNGLIDE) {//DOWNGLIDE tries to send the ROUGHIE into a downward glide
    pump = param.tankBackLimit;//pump to the backlimit
    pumpDone = checkPump(pump);//Returns true if done pumping
    //Linear Calculations
    if((feedforward && !pumpDone) || (!linPID && !linFuzzy)) {//Feedforward/feedback logic to determine the required output
      linMode = POSITION;
      lin = param.downFeedforward;
    } else {
      linMode = PWM;
      lin = param.linNoseDownTarget;
    }

    //Roll Calculations
    if(circle || (dubin && (millis() - t0 < param.dubinTime))) {
      rot = param.rollover;
    } else {
      rot = 0.0;
    }
    
    if(millis() - t0 > param.desTime) {//Once its been gliding for desTime go to the next state
      currentState = NEUTRAL;
      nextState = UPGLIDE;
      t0 = millis();
      //delay(50);
      Serial.println(F("DOWN GLIDE DONE"));
      
    }
  }

  if(currentState == NEUTRAL) {
    pump = param.tankMid;//Send the pump to the center
    if(millis() - t0 > param.neutralTime) {//Once neutral time is complete go to the next state
      currentState = nextState;
      t0 = millis();
      delay(50);
      Serial.println(F("Neutral Done"));
    }
  }

  if(currentState == UPGLIDE) {
    pump = param.tankFrontLimit;  //Empty the ballast tank  
    pumpDone = checkPump(pump);//Returns true if done pumping
    //Linear Calculations
    if((feedforward && !pumpDone) || (!linPID && !linFuzzy)) {//feedforward/feedback calculations
      linMode = POSITION;
      lin = param.upFeedforward;
    } else {
      linMode = PWM;
      lin = param.linNoseUpTarget;
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
      glideCompleted = 1;
      t0 = millis();
      Serial.println(F("Glide Completed"));
    }
  }

  if(linMode == PWM) {//If its PWM mode either do fuzzy or PID
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
  return glideCompleted;
  
}
