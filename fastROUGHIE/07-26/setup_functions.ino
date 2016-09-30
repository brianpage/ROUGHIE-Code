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
      Serial.print(F("Logging4 to: "));
      Serial.println(filename);
      strcpy(name_of_file, filename);
      stdioFile.clearerr();
      break;
    }
    stdioFile.clearerr();

  }
  if(!stdioFile.fopen(filename,"w+")) {
    Serial.println(F("fopen fail, try again (or maybe the SD card is not working right) "));
    stdioFile.clearerr();
    return;
  }
  stdioFile.println(F("m,pitch,roll,yaw,linPos,tankPos,pressure,compassX,compassY,compassZ,heading,servo,latitude,longitude"));
}

void logData() {
  stdioFile.printField(glider.runTime,',');
  stdioFile.printField(imu.pitch,',');
  stdioFile.printField(imu.roll,',');
  stdioFile.printField(imu.yaw,',');
  stdioFile.printField(glider.linPos,',');
  stdioFile.printField(glider.tankPos,',');
  stdioFile.printField(glider.pressure,',');
  stdioFile.printField(compass.rawX,',');
  stdioFile.printField(compass.rawY,',');
  stdioFile.printField(compass.rawZ,',');
  stdioFile.printField(compass.heading,',');
  stdioFile.printField(rotStor,',');
  stdioFile.printField(imu.latitude,',',6);
  stdioFile.println(imu.longitude,6);
//  stdioFile.printField(rotStor,',');
//  stdioFile.println(rotStor);
  //stdioFile.fflush();
}

void closeFile() {
  stdioFile.println();

  stdioFile.println(F(""));
  stdioFile.println(F("Params"));
  stdioFile.print(F("Lin Limits (front/mid/back), "));
  stdioFile.print(param.linFrontLimit);
  stdioFile.print(F(", "));
  stdioFile.print(param.linMid);
  stdioFile.print(F(", "));
  stdioFile.println(param.linBackLimit);
  stdioFile.print(F("Tank Limits (front/middle/back), "));
  stdioFile.print(param.tankFrontLimit);
  stdioFile.print(F(", "));
  stdioFile.print(param.tankMid);
  stdioFile.print(F(", "));
  stdioFile.println(param.tankBackLimit);
  stdioFile.print(F("Glide times (des/rise/neutral/dubin), "));
  stdioFile.print(param.desTime);
  stdioFile.print(F(", "));
  stdioFile.print(param.riseTime);
  stdioFile.print(F(", "));
  stdioFile.print(param.neutralTime);
  stdioFile.print(F(", "));
  stdioFile.println(param.dubinTime);
  stdioFile.print(F("Linear PID, "));
  stdioFile.print(param.linkp);
  stdioFile.print(F(", "));
  stdioFile.print(param.linki);
  stdioFile.print(F(", "));
  stdioFile.println(param.linkd);
  stdioFile.print(F("Roll PI, "));
  stdioFile.print(param.rollkp);
  stdioFile.print(F(", "));
  stdioFile.println(param.rollki);
  stdioFile.print(F("Feedforward Positions (down/up), "));
  stdioFile.print(param.downFeedforward);
  stdioFile.print(F(", "));
  stdioFile.println(param.upFeedforward);
  stdioFile.print(F("Target angles (down/up), "));
  stdioFile.print(param.linNoseDownTarget);
  stdioFile.print(F(", "));
  stdioFile.println(param.linNoseUpTarget);
  stdioFile.print(F("Rollover angle, "));
  stdioFile.println(param.rollover);
  stdioFile.println(F(""));
  stdioFile.println(F("Controller Status"));
  stdioFile.print(F("Feedforward, "));
  stdioFile.println(feedforward);
  stdioFile.print(F("Linear PID, "));
  stdioFile.println(linPID);
  stdioFile.print(F("Linear Fuzzy, "));
  stdioFile.println(linFuzzy);
  stdioFile.print(F("Circle, "));
  stdioFile.println(circle);
  stdioFile.print(F("Dubin, "));
  stdioFile.println(dubin);
  stdioFile.print(F("Turn Feedback, "));
  stdioFile.println(turnFeedback);
  stdioFile.print(F("Feedforward timeout, "));
  stdioFile.println(param.FFtime);
  stdioFile.print(F("Feedforward error bound, "));
  stdioFile.println(param.FFerror);
  stdioFile.print(F("Heading Control, "));
  stdioFile.println(headingControl);
  stdioFile.print(F("Desired Heading, "));
  stdioFile.println(desHeading);
  stdioFile.fflush();
  stdioFile.fclose();
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

