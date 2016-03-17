void actuate(float lin, float rot, int tank, int mode) {
  //Bang Bang Control
  if (mode == POSITION) {

    //Pitch Mass Control
    int currentLinPos = getFiltAnalog(linPos);
    if (lin < param.linFrontLimit) {
      Serial.println(F("Too far forward, going to limit"));
      lin = param.linFrontLimit;
    }
    if (lin > param.linBackLimit) {
      Serial.println(F("Too far backward, going to limit"));
      lin = param.linBackLimit;
    }
    if(abs(currentLinPos - lin) < 20) {
      Serial.println(F("Lin Done"));
      analogWrite(motAPWM, 0);
      digitalWrite(motStdby, LOW);
    } else {
      if(currentLinPos > lin) {
        digitalWrite(motAConf1, LOW);
        digitalWrite(motAConf2, HIGH);
      } else {
        digitalWrite(motAConf1, HIGH);
        digitalWrite(motAConf2, LOW);
      }
      digitalWrite(motStdby, HIGH);
      analogWrite(motAPWM, param.linRate);
    }
  }

  if (mode == PWM) {
    //Pitch Mass Control
    int currentLinPos = getFiltAnalog(linPos);
    if(lin > 0){//Rate direction switching
      digitalWrite(motAConf1, LOW);
      digitalWrite(motAConf2, HIGH);
      if(currentLinPos >= param.linBackLimit) {  //If it is trying to drive out of bounds turn off motor
        digitalWrite(motStdby, LOW);
      } else {
        digitalWrite(motStdby, HIGH);
      }
      
    } else{
      digitalWrite(motAConf1, HIGH);
      digitalWrite(motAConf2, LOW);
      if(currentLinPos <= param.linFrontLimit) {  //If it is trying to drive out of bounds turn off motor
        digitalWrite(motStdby, LOW);
      } else {
        digitalWrite(motStdby, HIGH);
      }
    }
    if(abs(lin) < 10) {
      lin = 0;
    }
    analogWrite(motAPWM,constrain(abs(lin),0,255));
    
  }

  //Roll Control
  rotServo.write(constrain(map(param.rotMid + rot,-45,45,rotPWMmin,rotPWMmax),rotPWMmin,rotPWMmax));
  
  //Pump Control
  int currentTankPos = getFiltAnalog(tankLevel);
  if(tank < param.tankBackLimit) {
    Serial.println(F("Too far back, going to limit"));
    tank = param.tankBackLimit;
  }
  if(tank > param.tankFrontLimit) {
    Serial.println(F("Too far forward, going to limit"));
    tank = param.tankFrontLimit;
  }
  if(abs(currentTankPos - tank) < 10) {
    Serial.println(F("Tank Done"));
    digitalWrite(pumpOn, LOW);
  } else {
    if(currentTankPos > tank) {
      digitalWrite(pumpDir, HIGH);
    } else {
      digitalWrite(pumpDir, LOW);
    }
    digitalWrite(pumpOn, HIGH);
  }
}
