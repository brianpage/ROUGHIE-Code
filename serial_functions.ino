int readSerial() {
  const int BUFF_LEN = 80;
  char buff[BUFF_LEN];
  DateTime now;
  int command = NULL;
  // PROCESS SERIAL INPUT FROM USER (IF AVAILABLE)
  if(Serial.available()) // if command was read
  {
    char arg[3][80];
    int len = Serial.readBytesUntil('\r', buff, BUFF_LEN);
    buff[len] = '\0';

    Serial.print("@");
    Serial.print(millis());
    Serial.print(" > ");
    Serial.println(buff); // echo back with timestamp

    sscanf(buff, "%s %s %s", arg[0], arg[1], arg[2]); // parsing

    if(strcmp(arg[0], "reset") == 0) {
      Serial.println("GO!");
      command = RESET;
    }
    
    else if(strcmp(arg[0], "start") == 0) {
      pressure_b = pressure_m * getFiltAnalog(pressureSensorPin);
      Serial.println(F("Pressure Caanklibrated!"));
      Serial.println(F("Starting"));
      command = START;
    }
    
    else if(strcmp(arg[0], "stop") == 0) {
      command = STOP;
      Serial.println(F("Stopping"));
    }
    
    else if(strcmp(arg[0], "sdstart") == 0) {
      // CREATE FILE ON SD CARD  
      createSDfile(name_of_file);
      SDgo = 1;
      Serial.println(F("SD Start"));
    }
    
    else if(strcmp(arg[0], "sdstop") == 0) {
      
      // OPEN FILE
      logfile = SD.open(name_of_file, FILE_WRITE);
      logfile.println("");
      logfile.println("");
      logfile.print(F("Notes,"));
      logfile.println(buff);
      SDgo = 0;
      logfile.close();
      Serial.println(F("Logging stopped and log file was closed"));
    }
    
    else if(strcmp(arg[0], "turnto") == 0) {
      Serial.println(F("Moving to desired turn angle..."));
      param.desiredRotaryPosition = atoi(arg[1]);
      if (param.desiredRotaryPosition < -44) {
        param.desiredRotaryPosition = -44;
      }
      else if (param.desiredRotaryPosition > 44) {
        param.desiredRotaryPosition = 44;
      }
      rotServo.write( map( param.desiredRotaryPosition, -45, 45, rotPWMmin, rotPWMmax ) );
    }
    
    else if(strcmp(arg[0], "servoparameters") == 0) {
      Serial.print(F("Desired rotation angle: "));
      Serial.println(param.desiredRotaryPosition);
    }
        
    else if(strcmp(arg[0], "circle") == 0) {
      if(circle) {
        circle = 0;
      }
      else {
        circle = 1;
        dubin = 0;
      }
      printController();
    }

    else if(strcmp(arg[0], "feedforward") == 0) {
      feedforward = !feedforward;
      printController();      
    }
    else if(strcmp(arg[0], "dubin") == 0) {
      if(dubin) {
        dubin = 0;
      }
      else {
        dubin = 1;
        circle = 0;
      }
      printController();   
    }

    else if(strcmp(arg[0], "linear") == 0) {
      if(linPID) {
        linPID = 0;
      }
      else {
        linPID = 1;
        linFuzzy = 0;
      }
      printController();
    }
    
    else if(strcmp(arg[0], "fuzzy") ==0) {
      if(linFuzzy) {
        linFuzzy = 0;
      }
      else {
        linPID = 0;
        linFuzzy = 1;
      }
      printController();
    }
    else if(strcmp(arg[0], "turnFeedback") ==0) {
      turnFeedback = !turnFeedback;
      printController();
    }
    
    else if(strcmp(arg[0], "pressurecontrol") == 0) {
      pressureControl = !pressureControl;
      printController();  
    }
    
    else if(strcmp(arg[0], "pressurecal") == 0) {
      // CALIBRATE PRESSURE SENSOR @ 0 DEPTH THIS HAS NOT BEEN VERIFIED!
      pressure_b = pressure_m * getFiltAnalog(pressureSensorPin);
    }
    
    else if(strcmp(arg[0], "gimme") == 0) {
      
      if(strcmp(arg[1], "power") == 0) {
        double voltage = getFiltAnalog(Vin_readback) * 30 / 1023;
        double current = getFiltAnalog(Iin_readback) * 5 / 1.023 / 0.3; //In mA off by 3x!
        Serial.print(F("Battery voltage: "));
        Serial.print(voltage);
        Serial.print(F(" Volts -- raw reading is: "));
        Serial.println(getFiltAnalog(Vin_readback));
        
        Serial.print(F("Current draw: "));
        Serial.print(current);
        Serial.print(F(" mA -- raw reading is: "));
        Serial.println(getFiltAnalog(Iin_readback));
        
        Serial.print(F("Instantaneous power: "));
        Serial.print(voltage * current);
        Serial.println(F(" mW"));
      }
      else if(strcmp(arg[1], "attitude") == 0) {
        if(millis()%1423>100){
          Serial.print(F("Roll: "));
          Serial.println(imu.roll);
          Serial.print(F("Pitch: "));
          Serial.println(imu.pitch);
          Serial.print(F("Yaw: "));
          Serial.println(imu.yaw);
          if(millis()%132<10){
            Serial.println(F("You want attitude?"));
          }
          if(millis()%132>125){
            Serial.println(F("Deal with it"));
          }
        } else {
          Serial.println(F("No"));
        }
      }
      else if(strcmp(arg[1], "roll") == 0) {
        Serial.print(F("Roll: "));
        Serial.println(imu.roll);
      }
      
      else if(strcmp(arg[1], "pitch") == 0) {
        Serial.print(F("Pitch: "));
        Serial.println(imu.pitch);
      }
            
      else if(strcmp(arg[1], "yaw") == 0) {
        Serial.print(F("Yaw: "));
        Serial.println(imu.yaw);
      }
      
      else if(strcmp(arg[1], "tank") == 0) {
        Serial.print(F("Tank level: "));
        Serial.println(getFiltAnalog(tankLevel));
      }
      
      else if(strcmp(arg[1], "linear") == 0) {
        Serial.print(F("Linear mass position: "));
        Serial.println(getFiltAnalog(linPos));
      }
      
      
      else if(strcmp(arg[1], "ecopuck") == 0) {
        Serial.print(F("Ecopuck voltage: "));
        Serial.println(getFiltAnalog(ecopuck_pin));
      }
      
      else if(strcmp(arg[1], "pressure") == 0) {
        Serial.print(F("Pressure voltage: "));
        Serial.println(getFiltAnalog(pressureSensorPin));
        Serial.print(F("Estimated depth (ft): "));
        Serial.println(pressure_m * (getFiltAnalog(pressureSensorPin) - pressure_b));
      }

      else {
        printHelp();
      }
      
    }


    
    else if(strcmp(arg[0], "update") == 0) {  // update parameter
      if(strcmp(arg[1], "-rollover") == 0) {
        param.rollover = atoi(arg[2]);
        Serial.println(F("Updated Rollover"));
      }

      else if(strcmp(arg[1], "-dubinTime") == 0) {
        param.dubinTime = atoi(arg[2]);
        Serial.println(F("Updated Dubin Time"));
      }
      else if(strcmp(arg[1], "-upFeedforward") == 0) {
        param.upFeedforward = atoi(arg[2]);
        Serial.print(F("Up Feedforward updated to: "));
        Serial.println(param.upFeedforward);
      }

      else if(strcmp(arg[1], "-downFeedforward") == 0) {
        param.downFeedforward = atoi(arg[2]);
        Serial.print(F("Down Feedforward updated to: "));
        Serial.println(param.upFeedforward);
      }
      
      else if(strcmp(arg[1], "-glidebottom") == 0) {
        param.glide_cycle_bottom = atoi(arg[2]);
        Serial.println(F("Updated Glide Bottom"));
      }
      
      else if(strcmp(arg[1], "-glidetop") == 0) {
        param.glide_cycle_top = atoi(arg[2]);
        Serial.println(F("Updated Glide Top"));
      }
      
      else if(strcmp(arg[1], "-linrate") == 0) {
        param.linRate = atoi(arg[2]);
        Serial.println(F("Updated Lin Rate"));
      }
      
      else if(strcmp(arg[1], "-destime") == 0) {
        param.desTime = atoi(arg[2]);
        Serial.println(F("Updated Descent Time"));
      }
      
      else if(strcmp(arg[1], "-neutralTime") == 0) {
        param.neutralTime = atoi(arg[2]);
        Serial.println(F("Updated Neutral Time"));
      }
      else if(strcmp(arg[1], "-risetime") == 0) {
        param.riseTime = atoi(arg[2]);
        Serial.println(F("Updated Rise Time"));
      }
      
      else if(strcmp(arg[1], "-linmid") == 0) {
        param.linMid = atoi(arg[2]);
        Serial.println(F("Updated Linear Mid"));
      }
      
      else if(strcmp(arg[1], "-rotmid") == 0) {
        param.rotMid = atoi(arg[2]);
        Serial.println(F("Updated Rotary Mid"));
      }
      
      else if(strcmp(arg[1], "-tankmid") == 0) {
        param.tankMid = atoi(arg[2]);
        Serial.println(F("Updated Tank Mid"));
      }
      
      else if(strcmp(arg[1], "-linfrontlimit") == 0) {
        param.linFrontLimit = atoi(arg[2]);
        Serial.println(F("Updated Lin Front Limit"));
      }
      
      else if(strcmp(arg[1], "-linbacklimit") == 0) {
        param.linBackLimit = atoi(arg[2]);
        Serial.println(F("Updated Lin Back Limit"));
      }
      
      else if(strcmp(arg[1], "-tankfrontlimit") == 0) {
        param.tankFrontLimit = atoi(arg[2]);
        Serial.println(F("Updated Tank Front Limit"));
      }
      
      else if(strcmp(arg[1], "-tankbacklimit") == 0) {
        param.tankBackLimit = atoi(arg[2]);
        Serial.println(F("Updated Tank Back Limit"));
      }
      
      else if(strcmp(arg[1], "-rotlowlimit") == 0) {
        param.rotLowLimit = atoi(arg[2]);
        Serial.println(F("Updated Rotary Low Limit"));
      }
      
      else if(strcmp(arg[1], "-rothighlimit") == 0) {
        param.rotHighLimit = atoi(arg[2]);
        Serial.println(F("Updated Rotary High Limit"));
      }
      
      else if(strcmp(arg[1], "-allowedWorkTime") == 0) {
        param.allowedWorkTime = atoi(arg[2]);
        Serial.println(F("Updated Allowed Work Time"));
      }
      
      else if(strcmp(arg[1], "-linkp") == 0) {
        param.linkp = atoi(arg[2]);
        Serial.print(F("Linear Kp updated to: "));
        Serial.println(param.linkp);
      }
      
      else if(strcmp(arg[1], "-linki") == 0) {
        param.linki = atoi(arg[2]);        
        Serial.print(F("Linear Ki updated to: "));
        Serial.println(param.linki);
      }  
      
      else if(strcmp(arg[1], "-linkd") == 0) {
        param.linkd = atoi(arg[2]);        
        Serial.print(F("Linear Kd updated to: "));
        Serial.println(param.linkd);
      }
      else if(strcmp(arg[1], "-rollkp") == 0) {
        param.rollkp = atof(arg[2]);        
        Serial.print(F("Roll Kp updated to: "));
        Serial.println(param.rollkp);
      }
      else if(strcmp(arg[1], "-rollki") == 0) {
        param.rollki = atof(arg[2]);        
        Serial.print(F("Roll Ki updated to: "));
        Serial.println(param.rollki);
      }
      
      else if(strcmp(arg[1], "-linNoseUpTarget") == 0) {
        param.linNoseUpTarget = atoi(arg[2]);
        Serial.print(F("PID Nose up target set to: "));
        Serial.println(param.linNoseUpTarget);
      }
      
      else if(strcmp(arg[1], "-linNoseDownTarget") == 0) {
        param.linNoseDownTarget = atoi(arg[2]);
        Serial.print(F("PID Nose down target set to: "));
        Serial.println(param.linNoseDownTarget);
      }
      
      else if(strcmp(arg[1], "-number_of_glides") == 0) {
        param.number_of_glides = atoi(arg[2]);
        Serial.print(F("Number of glides updated to: "));
        Serial.println(param.number_of_glides);
      }
      else {
        printHelp();
      }
      
    }
    
    else if(strcmp(arg[0], "currentpos") == 0) { // Current positions
      Serial.print(F("Linear Mass: "));
      Serial.println(getFiltAnalog(linPos));
      Serial.print(F("Water tank: "));
      Serial.println(getFiltAnalog(tankLevel));
    }
    
    else if(strcmp(arg[0], "imucal") == 0) {
      imu.rollOffset = -imu.roll;
      imu.pitchOffset = -imu.pitch;
      Serial.println(F("IMU Calibrated"));
      Serial.print(F("Rolloffset: "));
      Serial.println(imu.rollOffset);
      Serial.print(F("Pitch: "));
      Serial.println(imu.pitchOffset);
    }
    else if(strcmp(arg[0], "imureset") == 0) {
      imu.rollOffset = 0;
      imu.pitchOffset = 0;
      Serial.println(F("IMU calibration forgotten"));
    }
    
    else if(strcmp(arg[0], "params") == 0) { // Current parameters
      Serial.print(F("Desired Rotational Mass Position: "));
      Serial.print(param.desiredRotaryPosition);
      Serial.print(F("  Acceptable Range:  "));
      Serial.print(param.rotLowLimit);
      Serial.print(F(" to "));
      Serial.println(param.rotHighLimit);

      Serial.print(F("  Linear Mass Acceptable Range:  "));
      Serial.print(param.linFrontLimit);
      Serial.print(F(" to "));
      Serial.println(param.linBackLimit);

      Serial.print(F("  Ballast Tank Acceptable Range:  "));
      Serial.print(param.tankBackLimit);
      Serial.print(F(" to "));
      Serial.println(param.tankFrontLimit);
      
      Serial.print(F("Descent Time: "));
      Serial.println(param.desTime);
      Serial.print(F("Rise Time: "));
      Serial.println(param.riseTime);
      Serial.print(F("Neutral Time: "));
      Serial.println(param.neutralTime);
      Serial.print(F("Dubin Time: "));
      Serial.println(param.dubinTime);
      
      Serial.println(F("MIDDLE SETTINGS"));
      Serial.print(F("Tank: "));
      Serial.print(param.tankMid);
      Serial.print(F("  Default: "));
      Serial.println(tankmid);
      Serial.print(F("Linear Mass: "));
      Serial.print(param.linMid);
      Serial.print(F("  Default: "));
      Serial.println(linmid);
      Serial.print(F("Rotational Mass: "));
      Serial.print(param.rotMid);
      Serial.print(F("  Default: "));
      Serial.println(rotmid);

      Serial.print(F("upFeedforward position: "));
      Serial.println(param.upFeedforward);
      Serial.print(F("downFeedforward position: "));
      Serial.println(param.downFeedforward);

      Serial.print(F("Down Glide Angle: "));
      Serial.println(param.linNoseDownTarget);
      Serial.print(F("Up Glide Angle: "));
      Serial.println(param.linNoseUpTarget);
      
      Serial.print(F("Maximum allowed work time: "));
      Serial.print(param.allowedWorkTime);
      Serial.println(F("...THIS DOES NOT APPLY TO THE RESET FUNCTION!"));
      
      Serial.println(F("PID Settings"));
      Serial.print(F("kp: "));
      Serial.println(param.linkp);
      Serial.print(F("ki: "));
      Serial.println(param.linki);
      Serial.print(F("kd: "));
      Serial.println(param.linkd);

      Serial.print(F("Roll Kp: "));
      Serial.println(param.rollkp);
      Serial.print(F("Roll Ki: "));
      Serial.println(param.rollki);
      
      Serial.print(F("Glide cyble bottom: "));
      Serial.println(param.glide_cycle_bottom);
      Serial.print(F("Glide cyble top: "));
      Serial.println(param.glide_cycle_top);
      
      Serial.print(F("Rollover: "));
      Serial.println(param.rollover);
      
    }
      
    else {
      printHelp();
    }
  }

  return command;
}

void printController(void) {
    Serial.print(F("PID Status: "));
    Serial.println(linPID);
    Serial.print(F("Fuzzy Status: "));
    Serial.println(linFuzzy);
    Serial.print(F("Feedforward Status: "));
    Serial.println(feedforward);
    Serial.print(F("Circle Status: "));
    Serial.println(circle);
    Serial.print(F("Dubin Status: "));
    Serial.println(dubin);
    Serial.print(F("Turn Feedback: "));
    Serial.println(turnFeedback);
}

void printHelp(void) {
  for (int i = 0; i < 63; i++)
  {
    strcpy_P(buffer, (char*)pgm_read_word(&(helpTable[i]))); // Necessary casts and dereferencing, just copy.
    Serial.println(buffer);
  }
}