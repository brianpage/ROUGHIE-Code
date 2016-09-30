int sawtooth(int &lin,float &rot,int &pump,int &linMode) {
  //sawtooth calculates the required glide targets based on system parameters and timing
  
  int pumpDone;
  int glideCompleted = 0;//returns 0 unless it just completed a glide
  if(currentState == DOWNGLIDE) {//DOWNGLIDE tries to send the ROUGHIE into a downward glide
    
    pump = param.tankBackLimit;//pump to the backlimit
    //pumpDone = checkPump(pump);//Returns true if done pumping
    checkFF(pump,param.linNoseDownTarget,flag);
    
    //Linear Calculations
    if((feedforward && !flag) || (!linPID && !linFuzzy)) {//Feedforward/feedback logic to determine the required output
      linMode = POSITION;
      lin = param.downFeedforward;
    } else {
      linMode = PWM;
      lin = param.linNoseDownTarget;
    }

    //Roll Calculations
//    if(circle || (dubin && (millis() - t0 < param.dubinTime))) {
//      rot = param.rollover;
//    } else {
//      rot = 0.0;
//    }

    if(!delayRoll) {
      if(circle || (dubin && (millis() - t0 < param.dubinTime))) {
        rot = param.rollover;
      } else {
        rot = 0.0;
      }
    } else {
      if((flag) && (circle || (dubin && (millis() - t0 > param.dubinTime)))) {//Feedforward/feedback logic to determine the required output
        rot = param.rollover;
      } else {
        rot = 0.0;
      }
    }

    if(headingControl){
      rot = headingFeedback(currentState,rotStorage);
    }

    if(millis() - t0 > param.desTime - 1000) {
      downLoops = downLoops + 1;
      lastDownAngle = lastDownAngle + imu.pitch;
      Serial.println(lastDownAngle/downLoops);
    }
    
    if(millis() - t0 > param.desTime) {//Once its been gliding for desTime go to the next state
      currentState = NEUTRAL;
      nextState = UPGLIDE;
      rotStorage = rot;
      t0 = millis();
      //delay(50);
      Serial.println(F("DOWN GLIDE DONE"));
      
    }
  }

  if(currentState == NEUTRAL) {
    pump = param.tankMid;//Send the pump to the center
    if(feedforward && millis() - t0 > pumpTime) {
      //Serial.println(nextState);
      if(nextState == UPGLIDE) {
        lin = param.upFeedforward;
      }
      if(nextState == DOWNGLIDE) {
        lin = param.downFeedforward;
      }
      mode = POSITION;
    } else {
      lin = 0;
      mode = PWM;
    }
    rot = rotStorage;

    
    if(millis() - t0 > param.neutralTime) {//Once neutral time is complete go to the next state
      currentState = nextState;
      t0 = millis();
      Serial.println(F("Neutral Done"));
    }
    flag = false;
  }

  if(currentState == UPGLIDE) {
    
    if(millis() - t0 > param.riseTime - 1000) {
      upLoops = upLoops + 1;
      lastUpAngle = lastUpAngle + imu.pitch;
      Serial.println(lastUpAngle/upLoops);
    }
    
    pump = param.tankFrontLimit;  //Empty the ballast tank  
    //pumpDone = checkPump(pump);//Returns true if done pumping
    checkFF(pump,param.linNoseUpTarget,flag);
    
    
    //Linear Calculations
    if((feedforward && !flag) || (!linPID && !linFuzzy)) {//feedforward/feedback calculations
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

    if(!delayRoll) {
      if(circle) {
        rot = -param.rollover;
      } else if(dubin && (millis() - t0 < param.dubinTime)) {
        rot = param.rollover;
      } else {
        rot = 0.0;
      }
    } else {
      if((flag)) {//Feedforward/feedback logic to determine the required output
        if(circle) {
          rot = -param.rollover;
        } else if(dubin && (millis() - t0 < param.dubinTime)) {
          rot = param.rollover;
        } else {
          rot = 0.0;
        }
      } else {
        rot = 0.0;
      }
    }

    
    if(headingControl){
      rot = headingFeedback(currentState,rotStorage);
    }

    


    if(millis() - t0 > param.riseTime) {
      currentState = NEUTRAL;
      nextState = DOWNGLIDE;
      glideCompleted = 1;
      rotStorage = rot;
      t0 = millis();
      Serial.println(F("Glide Completed"));
    }
  }
  rotStorage = rot;
  if(linMode == PWM) {//If its PWM mode either do fuzzy or PID
    float error_prev, error_act;
    if(linPID) {
      lin = linPIDRate(lin);
    }
    if(linFuzzy) {
      lin = linFuzzyRate(lin);
    }
  } else {
    linI = 0.0;
  }

  if(turnFeedback) {
    rot = rotPID(rot);
  }
  if(glideCompleted && !(completedGlides+1 >= param.number_of_glides)) {
    lastUpAngle = 0.0;
    lastDownAngle = 0.0;
    upLoops = 0;
    downLoops = 0;
  }
  
  return glideCompleted;
  
}

float headingFeedback(int glideState, float currentRot) {
  float error = compass.heading - desHeading;
  float headingKp = param.headingKp;
  if(glideState == DOWNGLIDE) {
    headingKp = -param.headingKp;
  }
  Serial.print(error);
  Serial.print("\t");
  Serial.print(headingKp);
  Serial.print("\t");
  float output = constrain(headingKp * error,-2,2);
  Serial.print(output);
  output = constrain(output + currentRot,-45,45);
  Serial.print("\t");
  Serial.println(output);
  return output;
}

