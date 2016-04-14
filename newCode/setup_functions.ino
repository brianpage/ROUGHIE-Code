void paramInit(void) {//Initiate the param structure to default values;
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
  param.number_of_glides = number_of_glides_default;
  param.glide_cycle_bottom = glide_cycle_bottom_default;
  param.glide_cycle_top = glide_cycle_top_default;
  param.desiredRotaryPosition = desiredRotaryPosition_default;
  param.rollover = rollover_default;
  param.downFeedforward = downFeedforward_default;
  param.upFeedforward = upFeedforward_default;
  param.neutralTime = neutralTime_default;
  param.dubinTime = dubinTime_default;
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
  char filename[] = "LOGGER00.csv";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      File logfile = SD.open(filename, FILE_WRITE);
      
      if (! logfile) {
        Serial.println(F("couldnt create file"));
      }

      Serial.print(F("Logging to: "));
      Serial.println(filename);
  
      // connect to RTC
      //Wire.begin();  
      //if (!rtc.begin()) {
      //  logfile.println(F("RTC failed"));
     // }
  
      //logfile.println("millis,stamp,datetime,Pressure,Pitch,Roll,BallastTank,LinMassPos,tp1,tp2,yaw,rolld,pitchd,yawd,north,east,up,estdepth,ECOPUCK");    
      logfile.println(F("ms,Pressure,Pitch,Roll,BallastTank,LinMassPos,ServoAngle,yaw,estdepth,Vin,Iin"));
      logfile.close();
      //SDgo = 0;
      break;  // leave the loop!
    }
  }
  strcpy(name_of_file, filename);
  //return filename;
}

void logData(uint32_t m) {//Update the SD file with new values
  // OPEN FILE
  logfile = SD.open(name_of_file, FILE_WRITE);
  
  //millis
  logfile.print(m);           // milliseconds since start
  logfile.print(", "); 
  
  // logging data 
  //pressure   
  logfile.print(getFiltAnalog(pressureSensorPin));
  logfile.print(", ");    
  //pitch
  logfile.print(imu.pitch);// change the name if neccessary
  logfile.print(", ");
  //roll
  logfile.print(imu.roll);
  logfile.print(", ");
  //water tank position    
  logfile.print(getFiltAnalog(tankLevel));
  logfile.print(", ");
  //linear mass position
  logfile.print(getFiltAnalog(linPos));
  logfile.print(", ");
  //Servo Angle
  logfile.print(rotOutput);
  logfile.print(", ");
  //yaw
  logfile.print(imu.yaw);
  logfile.print(", ");
  
  
  //estimated depth
  logfile.print(pressure_m * (getFiltAnalog(pressureSensorPin) - pressure_b));
  logfile.print(", ");
  //Vin
  logfile.print( getFiltAnalog(Vin_readback) );
  logfile.print(", ");
  //Iin
  logfile.print( getFiltAnalog(Iin_readback) );
  logfile.println("");
  
  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
  if ((millis() - syncTime) > SYNC_INTERVAL) {
    syncTime = millis();
  
    // updating FAT!
    logfile.flush();
  }
  
  logfile.close();
}
