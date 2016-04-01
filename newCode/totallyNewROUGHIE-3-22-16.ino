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
#include <SoftwareSerial.h>
#include <SD.h>
#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <TinyGPS.h>
#include <avr/pgmspace.h>
#include "init_param.h"//Sets up all global variables, help menu, structures, and objects


//Define command states
#define GLIDE 0
#define RESET 1
#define START 2
#define STOP 3


//Define linear control modes
#define POSITION 1
#define PWM 2


//Define glide states
#define DOWNGLIDE 1
#define NEUTRAL 2
#define UPGLIDE 3
    

void setup() {
  //Setup things  
  Serial3.begin(115200);//IMU Serial
  Serial.begin(9600);//Comm serial
  Serial.setTimeout(10);
  Serial.println(F("Version 3.22.16 Totally new code"));
  printHelp();
  paramInit();//Set params to default values
  makeFuzzy();//Create fuzzy object
  // INITIALIZE SD CARD
  Serial.println(F("Initializing SD card..."));
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  // see if the card is present and can be initialized:
  if (!SD.begin(10,11,12,13)){;//10)) {
    Serial.println(F("Card failed, or not present"));
  }
  else {
    Serial.println(F("SD card initialized!"));
  }
  
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
  uint32_t m = millis();  // UPDATE m, THE # OF MILLISECONDS SINCE STARTING
  if(SDgo) {//If we are supposed to record, log data
    logData(m);
  }

  int newCommand = readSerial();//Check serial for commands or param updates
  if (newCommand != GLIDE) {//GLIDE means no command read, so just continue with old one
    command = newCommand;
  }

  if (command == START) {//Start a glide cycle with timing based controller
    completedGlides = 0;
    t0 = millis();
    rollI = 0.0;
    rotOutput = 0.0;
    lastUpAngle = 0.0;
    lastDownAngle = 0.0;
    downLoops = 0;
    upLoops = 0;    
    currentState = DOWNGLIDE;
    command = GLIDE;
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
  if (command == GLIDE) {//Runs whenever while gliding
    //First do high level control
    uint32_t m = millis();  // UPDATE m, THE # OF MILLISECONDS SINCE STARTING
    if(SDgo) {//If we are supposed to record, log data
      logData(m);
    }
    completedGlides = completedGlides + sawtooth(lin,rot,pump,mode);//sawtooth function does timing based high level control and returns a 1 if it completed a glide cycle, otherwise zero.
    actuate(lin,rot,pump,mode);//Low level control of motors and such.
    if(completedGlides >= param.number_of_glides) {//Once it completes its command go to RESET
      Serial.println(F("Done!"));
      command = RESET;
    }
  }

}
