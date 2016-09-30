
//New ROUGHIE code that should hopefully be easier to work with

//Import libraries, a bunch of them are for fuzzy.
#include <Fuzzy.h>
#include <FuzzyComposition.h>
#include <FuzzyInput.h>
#include <FuzzyIO.h>
#include <FuzzyOutput.h>
#include <FuzzyRule.h>
#include <FuzzyRuleAntecedent.h>
#include <FuzzyRuleConsequent.h>
#include <FuzzySet.h>
//#include <SoftwareSerial.h>
#include <SdFat.h>
#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
//#include <TinyGPS.h>
#include <avr/pgmspace.h>
#include "init_param.h"//Sets up all global variables, help menu, structures, and objects


//Define command states
#define GLIDE 0
#define RESET 1
#define START 2
#define STOP 3
#define ROLLTEST 4
#define ROLLSTART 5
#define FLOAT 6


//Define linear control modes
#define POSITION 1
#define PWM 2


//Define glide states
#define DOWNGLIDE 1
#define NEUTRAL 2
#define UPGLIDE 3

//define the compass
#define address 0x1E    

void setup() {
  //Setup things  
  Serial3.begin(115200);//IMU Serial
  Serial.begin(9600);//Comm serial
  Wire.begin();
  Wire.beginTransmission(address);
  Wire.write(0x02);
  Wire.write(0x00);
  Wire.endTransmission();  
  Serial.setTimeout(10);
  Serial.println(F("Version 6.13.16 Switchable turning?"));
  printHelp();
  Serial.println("printed help");
  paramInit();//Set params to default values
  Serial.println("set INIT");
  makeFuzzy();//Create fuzzy object
  
  // CENTER THE ROTARY SERVO
  Serial.println(F("Centering servo..."));
  rotServo.attach(rotServo_pin, 1100, 1900);  // THE HI TEC SERVO HAS A MIN & MAX PULSE WIDTH OF 1100us & 1900us, respectively.
  rotServo.write( map( param.rotMid, -45, 45, rotPWMmin, rotPWMmax ) );
  Serial.println(F("Setup done!"));

  //Set pin modes
  pinMode(motAPWM, OUTPUT);
  pinMode(motAConf1, OUTPUT);
  pinMode(motAConf2, OUTPUT);
  pinMode(motStdby, OUTPUT);
  pinMode(pumpOn, OUTPUT);
  pinMode(pumpDir, OUTPUT);

  //Turn things off in case they aren't already
  command = STOP;
  
}

void loop() {
  updateIMU();
  updateCompass();
  //Serial.println(compass.heading);
  updateGlider();
  if(SDgo) {//If we are supposed to record, log data
    logData();
  }
  //Serial.println(glider.runTime);
  int newCommand = readSerial();//Check serial for commands or param updates
  if (newCommand != GLIDE) {//GLIDE means no command read, so just continue with old one
    command = newCommand;
  }
  //Serial.println(glider.linPos);

  if (command == START) {//Start a glide cycle with timing based controller
    if(checkPump(param.tankMid) && checkMass(param.linMid)) {    
      completedGlides = 0;
      t0 = millis();
      tstart = t0;
      rollI = 0.0;
      rotOutput = 0.0;
      lastUpAngle = 0.0;
      lastDownAngle = 0.0;
      downLoops = 0;
      upLoops = 0;    
      currentState = DOWNGLIDE;
      flag = false;
      command = GLIDE;
    } else {
      actuate(param.linMid,param.rotMid,param.tankMid,POSITION);//reset everything before it starts the glide cycle
    }
  }
  if (command == STOP) {
    turnOff();
  }
  if (command == RESET) {//Reset to neutrally bouyant, centered CG, zero servo roll
    actuate(param.linMid,param.rotMid,param.tankMid,POSITION);
    if(checkPump(param.tankMid) && checkMass(param.linMid)){//If both pump and mass are done, STOP
      command = STOP;
      Serial.println(F("Glider RESET done!"));
    }
  }
  if (command == FLOAT) {//Reset to neutrally bouyant, centered CG, zero servo roll
    actuate(param.linMid,param.rotMid+180,param.tankFrontLimit,POSITION);
    Serial.print(F("Hope you guys didn't lose me. GPS:\t"));
    Serial.print(imu.latitude);
    Serial.print(F("\t"));
    Serial.println(imu.longitude);
//    if(checkPump(param.tankFrontLimit) && checkMass(param.linMid)){//If both pump and mass are done, STOP
//      command = STOP;
//      Serial.println(F("Glider FLOAT done!"));
//    }
  }
  if (command == GLIDE) {//Runs whenever while gliding
    //First do high level control
//    uint32_t m = millis();  // UPDATE m, THE # OF MILLISECONDS SINCE STARTING
//    if(SDgo) {//If we are supposed to record, log data
//      logData(m);
//    }
    completedGlides = completedGlides + sawtooth(lin,rot,pump,mode);//sawtooth function does timing based high level control and returns a 1 if it completed a glide cycle, otherwise zero.
    actuate(lin,rot,pump,mode);//Low level control of motors and such.
    if(completedGlides >= param.number_of_glides) {//Once it completes its command go to RESET
      Serial.println(F("Done!"));
      command = FLOAT;
    }
  }
  if (command == ROLLSTART) {
    tstart = millis();
    rollI = 0.0;
    rotStor = 0.0;
    rollOutput = 0.0;
    glider.runTime = 0;
    command = ROLLTEST;
  }
  if (command == ROLLTEST) {
    float rollAngle;
//    uint32_t m = millis();  // UPDATE m, THE # OF MILLISECONDS SINCE STARTING
//    if(SDgo) {//If we are supposed to record, log data
//      logData(m);
//    }
    //Serial.println(diff);
    if(glider.runTime < 30000) {
      rollAngle = 0;
    } else if(glider.runTime < 45000) {
      rollAngle = param.rollover;
    } else if (glider.runTime < 60000) {
      rollAngle = -param.rollover;
    } else if (glider.runTime < 75000) {
      rollAngle = param.rollover;
    } else if (glider.runTime < 90000) {
      rollAngle = -param.rollover;
    } else if (glider.runTime < 105000) {
      rollAngle = param.rollover;
    } else if (glider.runTime < 120000) {
      rollAngle = -param.rollover;
    } else if (glider.runTime < 135000) {
      rollAngle = param.rollover;
    } else if (glider.runTime < 150000) {
      rollAngle = -param.rollover;
    } else {
      command = RESET;
    }
//    Serial.print(rollAngle);
//    Serial.print(F("\t"));
    if(turnFeedback) {
      rollAngle = rotPID(rollAngle);
    }
//    Serial.print(rollAngle);
//    Serial.print(F("\t"));
//    Serial.println(imu.roll);
    actuate(param.linMid,rollAngle,param.tankMid,POSITION);    

  }
  //Serial.println(millis()-m);
}
