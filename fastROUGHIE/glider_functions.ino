int getFiltAnalog(int APIN)//Averaging filter to get rid of some noise, may or may not be needed
{
  int val = 0;
  for(int a=0; a<10; a++) {
    val = val + analogRead(APIN);
  }
  val = val/10;
  return val;
}

void updateGlider() {
  glider.linPos = getFiltAnalog(linPos);
  glider.tankPos = getFiltAnalog(tankLevel);
  glider.pressure = getFiltAnalog(pressureSensorPin);
  glider.runTime = millis() - tstart;
  //Serial.println(glider.runTime);
  
}

bool checkPump(int tank) {//boolean check to see if the pump is at the target
  if(abs(glider.tankPos - tank) < 10) {
    //Serial.println("Pump Done2");
    return true;
  } else {
    return false;
  }
}

void checkFF(int tank, int angle, bool &flag) {
  if(abs(glider.tankPos - tank) < 10 && abs(imu.pitch - angle) < param.FFerror) {
    //Serial.println("FF Complete");
    flag = true;
  }
  
  if(millis() - t0 > param.FFtime) {
    //Serial.println("FF TIMEOUT");
    flag = true;
  }  
}

bool checkMass(int mass) {//boolean check to see if the mass is at the target
  if(abs(glider.linPos - mass) < 10) {
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

void updateCompass(void) {//Update compass measurements. Works with HMC5883L chip from Sparkfun
  Wire.beginTransmission(address);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.requestFrom(address,6);
  if(6<=Wire.available()) {
    compass.rawX = Wire.read()<<8;
    compass.rawX |= Wire.read(); //X lsb
    compass.rawZ = Wire.read()<<8; //Z msb
    compass.rawZ |= Wire.read(); //Z lsb
    compass.rawY = Wire.read()<<8; //Y msb
    compass.rawY |= Wire.read(); //Y lsb
  }
  //Perform tilt compensation based on IMU measurements, works reasonably well up to about 45 degrees.
  //float compX = compass.rawX*cos(0.017444*imu.pitch)+compass.rawY*sin(0.017444*imu.roll)*sin(0.017444*imu.pitch)+compass.rawZ*cos(0.017444*imu.roll)*sin(0.017444*imu.pitch);
  //float compY = compass.rawY*cos(0.017444*imu.roll)-compass.rawZ*sin(0.017444*imu.roll);
//  Serial.print(compass.rawX);
//  Serial.print("\t");
//  Serial.print(compass.rawY);
//  Serial.print("\t");
//  Serial.print(compass.rawZ);
//  Serial.print("\t");
//  Serial.print(rotStor);
  //float compX = compass.rawX*cos(0.01744*imu.pitch)+compass.rawZ*sin(0.01744*imu.pitch);
  //float compY = compass.rawX*sin(0.01744*imu.roll)*sin(0.01744*imu.pitch)+compass.rawY*cos(0.01744*imu.roll)-compass.rawZ*sin(0.01744*imu.roll)*cos(0.01744*imu.pitch);
  float cosAngle = cos((imu.roll+rotStor)*0.01744);
  float sinAngle = sin((imu.roll+rotStor)*0.01744);
  float Bfy = compass.rawY*cosAngle-compass.rawZ*sinAngle;
  float Bz = compass.rawY*sinAngle+compass.rawZ*cosAngle;
  cosAngle = cos(imu.pitch*0.01744);
  sinAngle = sin(imu.pitch*0.01744);
  float Bfx = compass.rawX*cosAngle+Bz*sinAngle;
  float Bfz = -compass.rawX*sinAngle+Bz*cosAngle;
  compass.heading = 180*atan2(-Bfy,Bfx)/3.14;
  //compass.heading = 180*atan2(-compY,compX)/3.14;//-4-4/60;//heading adjusted for declination in Houghton
  //Serial.println(compass.heading);
//  Serial.print("\t");
//  Serial.println(compass.heading);
}

