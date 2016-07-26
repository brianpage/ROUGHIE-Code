void actuate(int lin, float rot, int tank, int mode) { //lin is either PWM or position depending on mode.  rot is servo angle, tank is ballast tank reading
  //Low level motion control, physically actuates things
  


  
  if (mode == POSITION) {  //Bang Bang Control
    analogWrite(motAPWM, param.linRate);
    //Pitch Mass Control

    if (lin < param.linFrontLimit) {//Double check to make sure we aren't going to run out of bounds
      Serial.println(F("Too far forward, going to limit"));
      lin = param.linFrontLimit;
    }
    if (lin > param.linBackLimit) {
      Serial.println(F("Too far backward, going to limit"));
      lin = param.linBackLimit;
    }
    if(abs(glider.linPos - lin) < 10) {//If we are close turn off
      digitalWrite(motStdby, LOW);
    } else {
      if(glider.linPos > lin) {//Set motor direction
        digitalWrite(motAConf1, LOW);
        digitalWrite(motAConf2, HIGH);
      } else {
        digitalWrite(motAConf1, HIGH);
        digitalWrite(motAConf2, LOW);
      }
      digitalWrite(motStdby, HIGH);//Turn on the motor
      
    }
  }

  if (mode == PWM) {//PWM mode is used for all controllers except feedforward
    lin = constrain(lin,-255,255);//Set it in the PWM output range
    //Pitch Mass Control

    if(lin > 0 && glider.linPos <= param.linFrontLimit){//If it is at a limit and trying to go the wrong way, stop it
      lin = 0;
    }

    if(lin < 0 && glider.linPos >= param.linBackLimit) {
      lin = 0;
    }
    if(abs(lin) < 10) {//If it is super slow just stop
      lin = 0;
      digitalWrite(motStdby,LOW);
    } else {
      digitalWrite(motStdby, HIGH);//otherwise turn on the motor
    }
    if(lin > 0) {//Set direction
      digitalWrite(motAConf1, LOW);
      digitalWrite(motAConf2, HIGH);      
    } else{
      digitalWrite(motAConf1, HIGH);
      digitalWrite(motAConf2, LOW);
    }
    //Serial.println(lin);
    analogWrite(motAPWM,abs(lin));//Set output speed based on PWM rate
    
  }

  //Roll Control
  rotStor = rot;
  rollOutput = mapfloat(constrain(param.rotMid + rot,param.rotLowLimit,param.rotHighLimit),-90.0,90.0,rotPWMmin,rotPWMmax);

  rotServo.write(rollOutput);//Roll the servo to 'rot' which is a roll angle +/- 45 from centerline
  
  //Pump Control
  
  if(tank < param.tankBackLimit) {
    Serial.println(F("Too far back, going to limit"));
    tank = param.tankBackLimit;
  }
  if(tank > param.tankFrontLimit) {
    Serial.println(F("Too far forward, going to limit"));
    tank = param.tankFrontLimit;
  }
  
  if(abs(glider.tankPos - tank) < 10) {//If it is close turn off the pump
    //Serial.println(F("Tank Done"));
    digitalWrite(pumpOn, LOW);
    return;
  }
  if(glider.tankPos > tank) {//Set pump direction
    digitalWrite(pumpDir, HIGH);
  } else {
    digitalWrite(pumpDir, LOW);
  }
  digitalWrite(pumpOn, HIGH);
  return;
}
