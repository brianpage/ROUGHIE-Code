int getFiltAnalog(int APIN)//Averaging filter to get rid of some noise, may or may not be needed
{
  int val = 0;
  for(int a=0; a<10; a++) {
    val = val + analogRead(APIN);
  }
  val = val/1;
  return val;
}

bool checkPump(int tank) {//boolean check to see if the pump is at the target
  int currentTankPos = getFiltAnalog(tankLevel);
  if(abs(currentTankPos - tank) < 10) {
    //Serial.println("Pump Done2");
    return true;
  } else {
    return false;
  }
}

void checkFF(int tank, int angle, bool &flag) {
  int currentTankPos = getFiltAnalog(tankLevel);
  if(abs(currentTankPos - tank) < 10 && abs(imu.pitch - angle) < param.FFerror) {
    //Serial.println("FF Complete");
    flag = true;
  }
  
  if(millis() - t0 > param.FFtime) {
    //Serial.println("FF TIMEOUT");
    flag = true;
  }  
}

bool checkMass(int mass) {//boolean check to see if the mass is at the target
  int currentMassPos = getFiltAnalog(linPos);
  if(abs(currentMassPos - mass) < 10) {
    //Serial.println("Mass done2");
    return true;
  } else {
    return false;
  }
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void turnOff() {// turn off pump and motor
  digitalWrite(motAPWM, 0);//Turn off linear motor
  digitalWrite(motStdby, LOW);//Turn off motor controller
  digitalWrite(pumpOn, LOW);//Turn off pump
  //Serial.println(F("Turning Off"));
}

void updateIMU(void) {//Update the IMU structure
  // Read IMU
  boolean inChar = Serial3.find('$');
  String input = Serial3.readStringUntil('$');
  float temp = input.substring(39,47).toFloat() + imu.pitchOffset;
  imu.pitchD  = imu.pitch - temp;
  imu.pitch = temp;
  temp = input.substring(30,38).toFloat();
  imu.yawD = imu.yaw - temp;
  imu.yaw = temp;
  temp = input.substring(48,56).toFloat() + imu.rollOffset;
  imu.rollD = imu.roll - temp;
  imu.roll = temp;
}
