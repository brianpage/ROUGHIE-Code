int getFiltAnalog(int APIN)
{
  int val = 0;
  for(int a=0; a<10; a++) {
    val = val + analogRead(APIN);
  }
  val = val/10;
  return val;
}

bool checkPump(int tank) {
  int currentTankPos = getFiltAnalog(tankLevel);
  if(abs(currentTankPos - tank) < 10) {
    return true;
  } else {
    return false;
  }
}

void turnOff() {
  digitalWrite(motAPWM, 0);//Turn off linear motor
  digitalWrite(motStdby, LOW);//Turn off motor controller
  digitalWrite(pumpOn, LOW);//Turn off pump
}

void updateIMU(void) {
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
