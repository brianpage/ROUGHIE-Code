void paramInit(void) {//Initiate the param structure to default values;
  rotStor = 0.0;
  param.linRate = linRate_default;
  param.linFrontLimit = linfrontlimit;
  param.linBackLimit = linbacklimit;
  param.tankBackLimit = tankbacklimit;
  param.tankFrontLimit = tankfrontlimit;
  param.rotLowLimit = rotlowlimit;
  param.rotHighLimit = rothighlimit;
  param.desTime = desTime_default;
  param.riseTime = riseTime_default;
  param.tankMid = tankmid;
  param.linMid = linmid;
  param.rotMid = rotmid;
  param.allowedWorkTime = allowedWorkTime_default;
  param.linNoseDownTarget = linNoseDownTarget_default;
  param.linNoseUpTarget = linNoseUpTarget_default;
  param.linkp = linkp_default;
  param.linki = linki_default;
  param.linkd = linkd_default;
  param.rollkp = rollkp_default;
  param.rollki = rollki_default;
  param.rollkd = rollkd_default;
  param.number_of_glides = number_of_glides_default;
  param.glide_cycle_bottom = glide_cycle_bottom_default;
  param.glide_cycle_top = glide_cycle_top_default;
  param.desiredRotaryPosition = desiredRotaryPosition_default;
  param.rollover = rollover_default;
  param.downFeedforward = downFeedforward_default;
  param.upFeedforward = upFeedforward_default;
  param.neutralTime = neutralTime_default;
  param.dubinTime = dubinTime_default;
  param.FFtime = FFtime_default;
  param.FFerror = FFerror_default;
}

void makeFuzzy(void) {//Create fuzzy
  // VARIABLES FOR LINEAR FUZZY CONTROLLER
  float fuzz_size = 5; //center fuzzy triangle size
  float fuzz_growth = 2; //scaling factor for fuzzy triangles away from center
  
  // START FUZZY SYSTEM
  //Fuzzy* fuzzy = new Fuzzy();
  
  // INITIALIZE FUZZY SETS
  FuzzySet* AZ_ang = new FuzzySet(-fuzz_size,0,0,fuzz_size);
  FuzzySet* PS_ang = new FuzzySet(0,fuzz_size,fuzz_size,fuzz_growth*fuzz_size);
  FuzzySet* PM_ang = new FuzzySet(fuzz_size,fuzz_growth*fuzz_size,fuzz_growth*fuzz_size,fuzz_growth*fuzz_growth*fuzz_size);
  FuzzySet* PL_ang = new FuzzySet(fuzz_size*fuzz_growth,fuzz_growth*fuzz_growth*fuzz_size,300,300);
  
  FuzzyInput* angleErr = new FuzzyInput(1);
  
  angleErr->addFuzzySet(AZ_ang);
  angleErr->addFuzzySet(PS_ang);
  angleErr->addFuzzySet(PM_ang);
  angleErr->addFuzzySet(PL_ang);
  
  fuzzy->addFuzzyInput(angleErr);
  
  FuzzyOutput* motorV = new FuzzyOutput(1);
  int motor_rate = 100;
  FuzzySet* AZ_V = new FuzzySet(-motor_rate,0,0,motor_rate);
  motorV->addFuzzySet(AZ_V);
  FuzzySet* PS_V = new FuzzySet(0,motor_rate,motor_rate,2*motor_rate);
  motorV->addFuzzySet(PS_V);
  FuzzySet* PM_V = new FuzzySet(motor_rate,2*motor_rate,2*motor_rate,3*motor_rate);
  motorV->addFuzzySet(PM_V);
  FuzzySet* PL_V = new FuzzySet(motor_rate*2,3*motor_rate,300,500);
  motorV->addFuzzySet(PL_V);
  
  fuzzy->addFuzzyOutput(motorV);
  
  // Build Fuzzy Rules
  FuzzyRuleAntecedent* angleAZ = new FuzzyRuleAntecedent();
  angleAZ->joinSingle(AZ_ang);
  FuzzyRuleAntecedent* anglePS = new FuzzyRuleAntecedent();
  anglePS->joinSingle(PS_ang);
  FuzzyRuleAntecedent* anglePM = new FuzzyRuleAntecedent();
  anglePM->joinSingle(PM_ang);
  FuzzyRuleAntecedent* anglePL = new FuzzyRuleAntecedent();
  anglePL->joinSingle(PL_ang);
  
  FuzzyRuleConsequent* thenMotorV_AZ = new FuzzyRuleConsequent();
  thenMotorV_AZ->addOutput(AZ_V);
  FuzzyRuleConsequent* thenMotorV_PS = new FuzzyRuleConsequent();
  thenMotorV_PS->addOutput(PS_V);
  FuzzyRuleConsequent* thenMotorV_PM = new FuzzyRuleConsequent();
  thenMotorV_PM->addOutput(PM_V);
  FuzzyRuleConsequent* thenMotorV_PL = new FuzzyRuleConsequent();
  thenMotorV_PL->addOutput(PL_V);
  
  FuzzyRule* fuzzyRule1 = new FuzzyRule(1, angleAZ, thenMotorV_AZ);
  fuzzy->addFuzzyRule(fuzzyRule1);
  FuzzyRule* fuzzyRule2 = new FuzzyRule(2, anglePS, thenMotorV_PS);
  fuzzy->addFuzzyRule(fuzzyRule2);
  FuzzyRule* fuzzyRule3 = new FuzzyRule(3, anglePM, thenMotorV_PM);
  fuzzy->addFuzzyRule(fuzzyRule3);
  FuzzyRule* fuzzyRule4 = new FuzzyRule(4, anglePL, thenMotorV_PL);
  fuzzy->addFuzzyRule(fuzzyRule4);
}

void createSDfile(char* name_of_file) {//Create the SD file

  if(!sd.begin(10)) {
    sd.errorPrint(F("SD isn't working, try again?"));
    return;
  }
  char filename[] = "LOGGER00.csv";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (!sd.exists(filename)) {
      Serial.print(F("Logging to: "));
      Serial.println(filename);
      strcpy(name_of_file, filename);
//      sd.clearerr();
      break;
    }
//    sd.clearerr();

  }
  if(!file.open(filename,O_CREAT | O_WRITE | O_EXCL)) {
    Serial.println(F("open fail, try again (or maybe the SD card is not working right) "));
//    sd.clearerr();
    return;
  }
  file.println(F("m,pitch,roll,yaw,linPos,tankPos,pressure,compassX,compassY,compassZ,heading,servo,latitude,longitude"));
}

void logData() {
  file.print(glider.runTime);
  file.print(F(","));
  file.print(imu.pitch);
  file.print(F(","));
  file.print(imu.roll);
  file.print(F(","));
  file.print(imu.yaw);
  file.print(F(","));
  file.print(glider.linPos);
  file.print(F(","));
  file.print(glider.tankPos);
  file.print(F(","));
  file.print(glider.pressure);
  file.print(F(","));
  file.print(compass.rawX);
  file.print(F(","));
  file.print(compass.rawY);
  file.print(F(","));
  file.print(compass.rawZ);
  file.print(F(","));
  file.print(compass.heading);
  file.print(F(","));
  file.print(rotStor);
  file.print(F(","));
  file.print(imu.latitude,6);
  file.print(F(","));
  file.println(imu.longitude,6);
//  stdioFile.printField(rotStor,',');
//  stdioFile.println(rotStor);
  //stdioFile.fflush();
}

void closeFile() {
  file.println();

  file.println(F(""));
  file.println(F("Params"));
  file.print(F("Lin Limits (front/mid/back), "));
  file.print(param.linFrontLimit);
  file.print(F(", "));
  file.print(param.linMid);
  file.print(F(", "));
  file.println(param.linBackLimit);
  file.print(F("Tank Limits (front/middle/back), "));
  file.print(param.tankFrontLimit);
  file.print(F(", "));
  file.print(param.tankMid);
  file.print(F(", "));
  file.println(param.tankBackLimit);
  file.print(F("Glide times (des/rise/neutral/dubin), "));
  file.print(param.desTime);
  file.print(F(", "));
  file.print(param.riseTime);
  file.print(F(", "));
  file.print(param.neutralTime);
  file.print(F(", "));
  file.println(param.dubinTime);
  file.print(F("Linear PID, "));
  file.print(param.linkp);
  file.print(F(", "));
  file.print(param.linki);
  file.print(F(", "));
  file.println(param.linkd);
  file.print(F("Roll PI, "));
  file.print(param.rollkp);
  file.print(F(", "));
  file.println(param.rollki);
  file.print(F("Feedforward Positions (down/up), "));
  file.print(param.downFeedforward);
  file.print(F(", "));
  file.println(param.upFeedforward);
  file.print(F("Target angles (down/up), "));
  file.print(param.linNoseDownTarget);
  file.print(F(", "));
  file.println(param.linNoseUpTarget);
  file.print(F("Rollover angle, "));
  file.println(param.rollover);
  file.println(F(""));
  file.println(F("Controller Status"));
  file.print(F("Feedforward, "));
  file.println(feedforward);
  file.print(F("Linear PID, "));
  file.println(linPID);
  file.print(F("Linear Fuzzy, "));
  file.println(linFuzzy);
  file.print(F("Circle, "));
  file.println(circle);
  file.print(F("Dubin, "));
  file.println(dubin);
  file.print(F("Turn Feedback, "));
  file.println(turnFeedback);
  file.print(F("Feedforward timeout, "));
  file.println(param.FFtime);
  file.print(F("Feedforward error bound, "));
  file.println(param.FFerror);
  file.print(F("Heading Control, "));
  file.println(headingControl);
  file.print(F("Desired Heading, "));
  file.println(desHeading);
  //sd.fflush();
  file.close();
}
//void logData(uint32_t m) {//Update the SD file with new values
//  // OPEN FILE
//  logfile = SD.open(name_of_file, FILE_WRITE);
//  
//  //millis
//  logfile.print(m);           // milliseconds since start
//  logfile.print(", "); 
//  
//  // logging data 
//  //pressure   
//  logfile.print(getFiltAnalog(pressureSensorPin));
//  logfile.print(", ");    
//  //pitch
//  logfile.print(imu.pitch);// change the name if neccessary
//  logfile.print(", ");
//  //roll
//  logfile.print(imu.roll);
//  logfile.print(", ");
//  //water tank position    
//  logfile.print(getFiltAnalog(tankLevel));
//  logfile.print(", ");
//  //linear mass position
//  logfile.print(getFiltAnalog(linPos));
//  logfile.print(", ");
//  //Servo Angle
//  logfile.print(rollOutput);
//  logfile.print(", ");
//  //yaw
//  logfile.print(imu.yaw);
//  logfile.println(", ");
//  
//  
////  //estimated depth
////  logfile.print(pressure_m * (getFiltAnalog(pressureSensorPin) - pressure_b));
////  logfile.print(", ");
////  //Vin
////  logfile.print( getFiltAnalog(Vin_readback) );
////  logfile.print(", ");
////  //Iin
////  logfile.print( getFiltAnalog(Iin_readback) );
////  logfile.println("");
////  
//  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
//  // which uses a bunch of power and takes time
////  if ((millis() - syncTime) > SYNC_INTERVAL) {
////    syncTime = millis();
////  
////    // updating FAT!
////    logfile.flush();
////  }
////  if((logfile.size()-syncSize)<SYNC_SIZE){
////    syncSize=logfile.size();
////    logfile.flush();
////  }
//  logfile.close();
//}

//void logData(uint32_t m) {
  // OPEN FILE
//  logfile = SD.open(name_of_file, FILE_WRITE);
//  
//  //millis
//  logfile.print(m);           // milliseconds since start
//  logfile.print(", "); 
//  
//  // logging data 
//  //pressure   
//  logfile.print(getFiltAnalog(pressureSensorPin));
//  logfile.print(", ");    
//  //pitch
//  logfile.print(imu.pitch);// change the name if neccessary
//  logfile.print(", ");
//  //roll
//  logfile.print(imu.roll);
//  logfile.print(", ");
//  //water tank position    
//  logfile.print(getFiltAnalog(tankLevel));
//  logfile.print(", ");
//  //linear mass position
//  logfile.print(getFiltAnalog(linPos));
//  logfile.print(", ");
//  //Servo Angle
//  logfile.print(rotStor);
//  logfile.print(", ");
//  //yaw
//  logfile.print(imu.yaw);
//  logfile.println(", ");
//  
  
//  //estimated depth
//  logfile.print(pressure_m * (getFiltAnalog(pressureSensorPin) - pressure_b));
//  logfile.print(", ");
//  //Vin
//  logfile.print( getFiltAnalog(Vin_readback) );
//  logfile.print(", ");
//  //Iin
//  logfile.print( getFiltAnalog(Iin_readback) );
//  logfile.println("");
//  
  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
//  if ((millis() - syncTime) > SYNC_INTERVAL) {
//    syncTime = millis();
//  
//    // updating FAT!
//    logfile.flush();
//  }
//  if((logfile.size()-syncSize)<SYNC_SIZE){
//    syncSize=logfile.size();
//    logfile.flush();
//  }
//  logfile.close();
//}

