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
#include "RTClib.h"
#include <SD.h>
#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <TinyGPS.h>

#include <avr/pgmspace.h>

/*
  SPI header of UM7 goes to Arduino (Vin, gnd, tx, rx)
  Secondary header of UM7 goes to GPS (rx2, tx2)
*/

// DEFINE STATE MACHINE COMMANDS AND STATES AS INTEGERS
#define GC_NULL  0
#define GC_RESET 1
#define GC_STOP  2
#define GC_BEGIN 3
#define GC_START 4
#define GC_ROLL 5
#define GC_LINEAR 6
#define GC_ROTARY 7
#define GC_PRESSURE_CONTROL 8
#define GC_FLOAT 9
#define GC_CIRCLE 10
#define GC_FUZZY 11
#define GC_FEEDFORWARD 12

#define ME_NOSE_DOWN 0
#define ME_GLIDE_DOWN 1
#define ME_NOSE_UP 2
#define ME_GLIDE_UP 3
#define ME_PAUSE 5
#define ME_FLOAT 6

// CREATE OBJECTS
Servo rotServo;   // SERVO OBJECT FOR ROTARY SERVO (HI-TEC, PWM CONTROLLED)
RTC_DS1307 rtc;   // RTC OBJECT, FOR SD DATA LOGGING. (THE RTC COMPONENT WON'T WORK BECAUSE WE DON'T HAVE I2C PLUGGED IN)
File logfile;     // FILE OBJECT FOR SD DATA LOGGING.

// DEFINE THE INTERVAL AT WHICH TO WRITE TO THE SD CARD AND INITIALIZE THE syncTime VARIABLE (USED FOR SD CARD)
#define SYNC_INTERVAL 500
uint32_t syncTime = 0;
char* name_of_file = "LOGGER00.csv";

// INITIALIZE BOLLEAN FOR ENABLING SD LOGGING
bool SDgo = 0;

// CREATE param STRUCT THAT CONTAINS A BUNCH OF PARAMETERS FOR THE GLIDER
struct param_t {
  int linRate;                      // SPEED LINEAR MASS SHOULD MOVE (INTEGER BETWEEN 0 & 255)
  int linFrontLimit;                // FRONT LIMIT FOR THE LINEAR MASS (BATTERY)
  int linBackLimit;                 // BACK LIMIT FOR THE LINEAR MASS (BATTERY)
  int tankBackLimit;                // BACK LIMIT FOR THE BALLAST TANK
  int tankFrontLimit;               // FRONT LIMIT FOR THE BALLAST TANK
  int rotLowLimit;                  // LOW LIMIT FOR ROTARY SERVO
  int rotHighLimit;                 // HIGH LIMIT FOR ROTARY SERVO
  unsigned int desTime;             // DESCENT TIME (IN MILLISECONDS)
  unsigned int riseTime;            // RISE TIME (IN MILLISECONDS)
  int tankMid;                      // MIDDLE SETTING FOR BALLAST TANK
  int linMid;                       // MIDDLE SETTING FOR LINEAR MASS (BATTERY)
  int rotMid;                       // MIDDLE SETTING FOR ROTARY MASS (SERVO)
  unsigned int allowedWorkTime;     // AMOUNT OF TIME (IN MILLISECONDS) TO GIVE PROCESSES BEFORE TIMING OUT
  float linkp;                      // PROPORTIONAL GAIN FOR LINEAR MASS PID CONTROLLER
  float linki;                      // INTEGRAL GAIN FOR LINEAR MASS PID CONTROLLER
  float linkd;                      // DERIVATIVE GAIN FOR LINEAR MASS PID CONTROLLER
  int linNoseDownTarget;            // GLIDER ANGLE TARGET FOR DESCENT
  int linNoseUpTarget;              // GLIDER ANGLE TARGET FOR ASCENT
  int number_of_glides;             // NUMBER OF GLIDE CYCLES TO COMPLETE BEFORE RETURNING TO SURFACE
  float glide_cycle_bottom;         // DESIRED BOTTOM OF GLIDE CYCLE (FT), FOR USE WITH PRESSURE SENSOR CONTROL
  float glide_cycle_top;            // DESIRED TOP OF GLIDE CYCLE (FT), FOR USE WITH PRESSURE SENSOR CONTROL
  int desiredRotaryPosition;         // GLIDER ROLL ANGLE TARGET
  int rollover;                     // AMOUNT FOR ROTARY SERVO TO TURN (INTEGER BETWEEN 0 & 179)
  int downFeedforward;
  int upFeedforward;
}
param;

struct imu_t {
  float pitch;
  float roll;
  float yaw;
}
imu;

// DEFINE PIN NUMBERS
const int motAPWM = 2;      // PWM PIN FOR LINEAR MASS MOTOR
const int motAConf1 = 28;   // CONFIGURATION PIN 1 FOR LINEAR MASS MOTOR
const int motAConf2 = 26;   // CONFIGURATION PIN 2 FOR LINEAR MASS MOTOR
const int motStdby = 30;    // MOTOR STANDBY PIN FOR LINEAR MASS MOTOR
const int motBPWM = 3;      // PWM PIN FOR 2ND MOTOR (UNUSED)
const int motBConf1 = 32;   // CONFIGURATION PIN 1 FOR 2ND MOTOR (UNUSED)
const int motBConf2 = 34;   // CONFIGURATION PIN 2 FOR 2ND MOTOR (UNUSED)

const int pumpOn = 22;      // PUMP ON/OFF PIN
const int pumpDir = 24;     // PUMP DIRECTION PIN

int tankLevel = A8;         // DRAW WIRE SENSOR FOR BALLAST TANK POSITION FEEDBACK
int linPos = A9;            // DRAW WIRE SENSOR FOR LINEAR MASS POSITION FEEDBACK
int pressureSensorPin = A11; // PRESSURE SENSOR FOR DEPTH FEEDBACK
int rotServo_pin = 7;      // ROTARY SERVO PIN
int ecopuck_pin = A15;      // ECOPUCK INPUT

int Iin_readback = A7;      // Current sensor
int Vin_readback = A13;     // Battery voltage

// DEFINE LIMITS FOR THE 3 MOVING PIECES
// LINEAR MASS LIMITS
const int linmid = (340+430)/2;        // MIDDLE POSITION FOR LINEAR MASS
const int linfrontlimit = 250; // FRONT LIMIT FOR LINEAR MASS
const int linbacklimit = 600;  // BACK LIMIT FOR LINEAR MASS
const int downFeedforward_default = 340;
const int upFeedforward_default = 430;

// WATER TANK LIMITS
const int tankmid = 270;        // MIDDLE POSITION FOR BALLAST TANK
const int tankbacklimit = 80;   // BACK LIMIT FOR BALLAST TANK
const int tankfrontlimit = 500; // FRONT LIMIT FOR BALLAST TANK

// ROTARY MASS LIMITS
const int rotmid = 0;                 // MIDDLE POSITION FOR ROTARY MASS
const int rotlowlimit = rotmid - 44;  // LOW LIMIT FOR ROTARY MASS
const int rothighlimit = rotmid + 44; // HIGH LIMIT FOR ROTARY MASS
const int rotPWMmin = 65;             // ROTARY LOW PWM LIMIT (FOUND VIA BENCHTEST)
const int rotPWMmax = 170;            // ROTARY HIGH PWM LIMIT (FOUND VIA BENCHTEST)


// DEFAULT PARAMETER VALUES
const int linRate_default = 255;       // DEFAULT RATE FOR LINEAR MASS MOTOR
const unsigned int desTime_default = 28000;       // DEFAULT DESCENT TIME (MILLISECONDS)
const unsigned int riseTime_default = 28000;      // DEFAULT RISE TIME (MILLISECONDS)
const unsigned int allowedWorkTime_default = 60000;      // DEFAULT ALLOWED WORK TIME
const int linNoseDownTarget_default = -30;
const int linNoseUpTarget_default = 30;
const float linkp_default = 20;
const float linki_default = 0;
const float linkd_default = 0;
const int number_of_glides_default = 3;
const float glide_cycle_bottom_default = 6;
const float glide_cycle_top_default = 2;
const int desiredRotaryPosition_default = 90;
const int rollover_default = 45;


// VARIABLES FOR LINEAR MASS PID CONTROLLER
float I = 0;
float Ir = 0;
float error_act = 0;
float error_act_r = 0;
float error_prev = 0;
float error_prev_r = 0;

// PRESSURE SENSOR PARAMETERS
double pressure_m = 0.0423062;
double pressure_b = 102.3;

Fuzzy *fuzzy = new Fuzzy();//Setup fuzzy object

//
//// DEFINE HELP MENU
char *help[] = {
                "-----------ROUGHIE HELP MENU-----------",
                "params - shows the current parameters and acceptable ranges",
                "reset - centers the linear mass and water tank (for trimming)",
                "currentpos - shows the current position of the actuators",
                "start - starts the glide cycle",
                "stop - stops the glide cycle",
                "linear - toggles the PID controller on/off for the linear mass",
                "feedforward - toggles the feedfoward controller on/off for feedfoward/feedback",
                "fuzzy - toggles fuzzy controller on/off for the linear mass",
                "sdstart - opens a new file and starts datalogging",
                "sdstop <notes> - stops datalogging, adds the text in <notes> to end of file and closes file",
                "pressurecontrol - toggles pressure control on/off",
                "turnto <position> - rotates the rotary servo to <position> (90 is center)",
                "float - commands glider to float with linear mass in front position",
                "pressurecal - calibrates pressure sensor (glider must be on or above surface of water)",
                "update - <parameter> <newValue> - updates <parameter> to <newValue> (don't forget the -)",
                "Parameters available for updating are:",
                "\trollover - If a circle path is enabled, this is the amount the rotary servo will turn in each direction",
                "\tglidebottom - If using pressure control, this is the depth in feet of the glide top",
                "\tglidetop - If using pressure control, this is the depth in feet of the glide bottom",
                "\tnumber_of_glides - Number of glides to complete before floating",
                "\trothighlimit - Rotary servo high limit",
                "\trotlowlimit - Rotary servo low limit",
                "\tlinfrontlimit - Linear mass front limit",
                "\tlinbacklimit - Linear mass back limit",
                "\tupFeedforward - Feedforward position used on up glides",
                "\tdownFeedforward - Feedforward position used on down glides",
                "\ttankhighlimit - Ballast tank's high (empty) limit",
                "\ttanklowlimit - Ballast tank's low (full) limit",
                "\tlinrate - How quickly to move the linear mass (when PID isn't on)--this is an integer between 0 & 255",
                "\tdestime - The descent time (in milliseconds)",
                "\trisetime - The risetime (in milliseconds)",
                "\ttankmid - The definition of the middle position for the ballast tank",
                "\tlinmid - The definition of the middle position for the linear mass",
                "\trotmid - The definition of the middle position for the rotary servo",
                "\tallowedWorkTime - The amount of time to give the system before timing out",
                "\tlinNoseUpTarget - Target angle of the glider when coming up (postive angle)",
                "\tlinNoseDownTarget - Target angle of the glider when going down (negative angle)",
                "\tlinkp - Linear mass PID proportional gain",
                "\tlinki - Linear mass PID integral gain",
                "\tlinkd - Linear mass PID derivative gain",
                "gimme <something> - prints the current reading of <something>",
                "Things to look at are:",
                "\tattitude - New function that prints roll, pitch, yaw",
                "\troll",
                "\tpitch",
                "\ttank",
                "\tlinear",
                "\tgps",
                "\tecopuck",
                "\tpressure",
                "\tpower",
                "printFile - Prints last recorded glide cycle to Serial for a quick check in Excel",
                "\tnot tested yet, may print out the entire file and crash everything. I don't know",
                "---------------------------------------", 0
                };



void setup() {

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

  
  // SETUP IMU ON SERIAL CHANNEL 3
  Serial3.begin(115200);
  // INITIALIZE SERIAL COMM
  Serial.begin(9600);
  Serial.setTimeout(10);
  
  // SHOW THE VERSION OF THE CODE
  Serial.println(F("Version 2.23.2016 Print Serial"));
  
  // PRINT THE HELP MENU
  printHelp();
  
  // INITIALIZE PARAMETERS
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
  param.number_of_glides = number_of_glides_default;
  param.glide_cycle_bottom = glide_cycle_bottom_default;
  param.glide_cycle_top = glide_cycle_top_default;
  param.desiredRotaryPosition = desiredRotaryPosition_default;
  param.rollover = rollover_default;
  param.downFeedforward = downFeedforward_default;
  param.upFeedforward = upFeedforward_default;
  
  // INITIALIZE ALL IMPORTANT PINS VIA GC_BEGIN, THEN TELL GLIDER TO IDLE VIA GC_STOP
  gliderStateMachine(GC_BEGIN);
  gliderStateMachine(GC_STOP);

  // INITIALIZE SD CARD
  Serial.println(F("Initializing SD card..."));
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  // see if the card is present and can be initialized:
  if (!SD.begin(10, 11, 12, 13)) {
    error("Card failed, or not present");
  }
  else {
    Serial.println(F("SD card initialized!"));
  }
  
  // CENTER THE ROTARY SERVO
  Serial.println(F("Centering servo..."));
  rotServo.attach(rotServo_pin, 1100, 1900);  // THE HI TEC SERVO HAS A MIN & MAX PULSE WIDTH OF 1100us & 1900us, respectively.
  rotServo.write( map( param.rotMid, -45, 45, rotPWMmin, rotPWMmax ) );
  Serial.println(F("Setup done!"));
  Serial.println(F("Lets see if fuzzy works or sets the ROUGHIE on fire"));
} // END OF SETUP

void loop() {
  const int BUFF_LEN = 80;
  char buff[BUFF_LEN];
  DateTime now;

  // Read IMU
  boolean inChar = Serial3.find('$');
  String input = Serial3.readStringUntil('$');
  imu.pitch  = input.substring(39,47).toFloat();
  imu.yaw = input.substring(30,38).toFloat();
  imu.roll = input.substring(48,56).toFloat();
  gliderStateMachine(GC_NULL);
  
  
  
  uint32_t m = millis();  // UPDATE m, THE # OF MILLISECONDS SINCE STARTING
  
  // IF DATALOGGING IS ENABLED, LOG DATA
  if(SDgo) {
    
    // OPEN FILE
    logfile = SD.open(name_of_file, FILE_WRITE);
  
    //millis
    logfile.print(m);           // milliseconds since start
    logfile.print(", "); 
    /*
    now = rtc.now();
    // stamp
    // log time
    logfile.print(now.unixtime()); // seconds since 1/1/1970
    logfile.print(", ");
    logfile.print('"');
    //datetime
    logfile.print(now.year(), DEC);
    logfile.print("/");
    logfile.print(now.month(), DEC);
    logfile.print("/");
    logfile.print(now.day(), DEC);
    logfile.print(" ");
    logfile.print(now.hour(), DEC);
    logfile.print(":");
    logfile.print(now.minute(), DEC);
    logfile.print(":");
    logfile.print(now.second(), DEC);
    logfile.print('"');
    */
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
      gliderStateMachine(GC_RESET);
    }
    
    else if(strcmp(arg[0], "start") == 0) {
      pressure_b = pressure_m * getFiltAnalog(pressureSensorPin);
      Serial.println(F("Pressure Calibrated!"));
      Serial.println(F("Starting"));
      gliderStateMachine(GC_START);
    }
    
    else if(strcmp(arg[0], "stop") == 0) {
      gliderStateMachine(GC_STOP);
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
    
    else if(strcmp(arg[0], "float") == 0) {
      Serial.println("FLOAT!!");
      gliderStateMachine(GC_FLOAT);
    }
    
    else if(strcmp(arg[0], "circle") == 0) {
      gliderStateMachine(GC_CIRCLE);
      //Serial.println(F("Circle"));
    }

    else if(strcmp(arg[0], "feedforward") == 0) {
      gliderStateMachine(GC_FEEDFORWARD);
      
    }
    
    else if(strcmp(arg[0], "pressurecontrol") == 0) {
      gliderStateMachine(GC_PRESSURE_CONTROL);
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
      if(strcmp(arg[1], "attitude") == 0) {
        Serial.print(F("Roll: "));
        Serial.println(imu.roll);
        Serial.print(F("Pitch: "));
        Serial.println(imu.pitch);
        Serial.print(F("Yaw: "));
        Serial.println(imu.yaw);
      }
      if(strcmp(arg[1], "roll") == 0) {
        Serial.print(F("Roll: "));
        Serial.println(imu.roll);
      }
      
      if(strcmp(arg[1], "pitch") == 0) {
        Serial.print(F("Pitch: "));
        Serial.println(imu.pitch);
      }
            
      if(strcmp(arg[1], "yaw") == 0) {
        Serial.print(F("Yaw: "));
        Serial.println(imu.yaw);
      }
      
      if(strcmp(arg[1], "tank") == 0) {
        Serial.print(F("Tank level: "));
        Serial.println(getFiltAnalog(tankLevel));
      }
      
      if(strcmp(arg[1], "linear") == 0) {
        Serial.print(F("Linear mass position: "));
        Serial.println(getFiltAnalog(linPos));
      }
      
      
      if(strcmp(arg[1], "ecopuck") == 0) {
        Serial.print(F("Ecopuck voltage: "));
        Serial.println(getFiltAnalog(ecopuck_pin));
      }
      
      if(strcmp(arg[1], "pressure") == 0) {
        Serial.print(F("Pressure voltage: "));
        Serial.println(getFiltAnalog(pressureSensorPin));
        Serial.print(F("Estimated depth (ft): "));
        Serial.println(pressure_m * (getFiltAnalog(pressureSensorPin) - pressure_b));
      }
      
    }

    else if(strcmp(arg[0], "printFile") == 0) {  // print pitch
      printFile();
    }
    
    else if(strcmp(arg[0], "update") == 0) {  // update parameter
      if(strcmp(arg[1], "-rollover") == 0) {
        param.rollover = atoi(arg[2]);
        Serial.println(F("Updated Rollover"));
      }

      if(strcmp(arg[1], "-upFeedforward") == 0) {
        param.upFeedforward = atoi(arg[2]);
        Serial.print(F("Up Feedforward updated to: "));
        Serial.println(param.upFeedforward);
      }

      if(strcmp(arg[1], "-downFeedforward") == 0) {
        param.downFeedforward = atoi(arg[2]);
        Serial.print(F("Down Feedforward updated to: "));
        Serial.println(param.upFeedforward);
      }
      
      if(strcmp(arg[1], "-glidebottom") == 0) {
        param.glide_cycle_bottom = atoi(arg[2]);
        Serial.println(F("Updated Glide Bottom"));
      }
      
      if(strcmp(arg[1], "-glidetop") == 0) {
        param.glide_cycle_top = atoi(arg[2]);
        Serial.println(F("Updated Glide Top"));
      }
      
      if(strcmp(arg[1], "-linrate") == 0) {
        param.linRate = atoi(arg[2]);
        Serial.println(F("Updated Lin Rate"));
      }
      
      if(strcmp(arg[1], "-destime") == 0) {
        param.desTime = atoi(arg[2]);
        Serial.println(F("Updated Descent Time"));
      }
      
      if(strcmp(arg[1], "-risetime") == 0) {
        param.riseTime = atoi(arg[2]);
        Serial.println(F("Updated Rise Time"));
      }
      
      if(strcmp(arg[1], "-linmid") == 0) {
        param.linMid = atoi(arg[2]);
        Serial.println(F("Updated Linear Mid"));
      }
      
      if(strcmp(arg[1], "-rotmid") == 0) {
        param.rotMid = atoi(arg[2]);
        Serial.println(F("Updated Rotary Mid"));
      }
      
      if(strcmp(arg[1], "-tankmid") == 0) {
        param.tankMid = atoi(arg[2]);
        Serial.println(F("Updated Tank Mid"));
      }
      
      if(strcmp(arg[1], "-linfrontlimit") == 0) {
        param.linFrontLimit = atoi(arg[2]);
        Serial.println(F("Updated Lin Front Limit"));
      }
      
      if(strcmp(arg[1], "-linbacklimit") == 0) {
        param.linBackLimit = atoi(arg[2]);
        Serial.println(F("Updated Lin Back Limit"));
      }
      
      if(strcmp(arg[1], "-tankfrontlimit") == 0) {
        param.tankFrontLimit = atoi(arg[2]);
        Serial.println(F("Updated Tank Front Limit"));
      }
      
      if(strcmp(arg[1], "-tankbacklimit") == 0) {
        param.tankBackLimit = atoi(arg[2]);
        Serial.println(F("Updated Tank Back Limit"));
      }
      
      if(strcmp(arg[1], "-rotlowlimit") == 0) {
        param.rotLowLimit = atoi(arg[2]);
        Serial.println(F("Updated Rotary Low Limit"));
      }
      
      if(strcmp(arg[1], "-rothighlimit") == 0) {
        param.rotHighLimit = atoi(arg[2]);
        Serial.println(F("Updated Rotary High Limit"));
      }
      
      if(strcmp(arg[1], "-allowedWorkTime") == 0) {
        param.allowedWorkTime = atoi(arg[2]);
        Serial.println(F("Updated Allowed Work Time"));
      }
      
      if(strcmp(arg[1], "-linkp") == 0) {
        param.linkp = atoi(arg[2]);
        Serial.print(F("Linear Kp updated to: "));
        Serial.println(param.linkp);
      }
      
      if(strcmp(arg[1], "-linki") == 0) {
        param.linki = atoi(arg[2]);        
        Serial.print(F("Linear Ki updated to: "));
        Serial.println(param.linki);
      }  
      
      if(strcmp(arg[1], "-linkd") == 0) {
        param.linkd = atoi(arg[2]);        
        Serial.print(F("Linear Kd updated to: "));
        Serial.println(param.linkd);
      }
      
      if(strcmp(arg[1], "-linNoseUpTarget") == 0) {
        param.linNoseUpTarget = atoi(arg[2]);
        Serial.print(F("PID Nose up target set to: "));
        Serial.println(param.linNoseUpTarget);
      }
      
      if(strcmp(arg[1], "-linNoseDownTarget") == 0) {
        param.linNoseDownTarget = atoi(arg[2]);
        Serial.print(F("PID Nose down target set to: "));
        Serial.println(param.linNoseDownTarget);
      }
      
      if(strcmp(arg[1], "-number_of_glides") == 0) {
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
    
    else if(strcmp(arg[0], "linear") == 0) {
      gliderStateMachine(GC_LINEAR);
    }
    
    else if(strcmp(arg[0], "fuzzy") ==0) {
      Serial.println(F("You typed in fuzzy"));
      gliderStateMachine(GC_FUZZY);
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


} // END OF LOOP

void printHelp() {
  Serial.println();
  Serial.println();
  for (int i = 0; help[i] ; i++){
   Serial.println(help[i]);
   delay(10);
  }
  Serial.println();
  Serial.println();

//  for (int i = 0; i < 48; i++)
//  {
//    strcpy_P(buffer, (char*)pgm_read_word(&(helpTable[i]))); // Necessary casts and dereferencing, just copy.
//    Serial.println(buffer);
//  }
}

void gliderStateMachine(int cmd) {

  static int state = 0;        // machine state
  static bool enGlider = 0;    // enable cycle
  static bool entry;           // if it's first time executing the current state
  static bool circle = 0;
  static bool feedforward = 0; // toggles feedforward
  static bool pumpDone;        // If the pump is done pumping
  static bool linDone;         // If the linear mass is done moving
  static bool DoLinPID = 0;
  static bool DoFuzzy = 0;
  static bool pressureControlOn = 0;
  static unsigned long int t0; // initial time of current state
  static int glide_cycles_completed = 0;
  static float est_depth = 0;
  int motRate;

  if(cmd == GC_START) {
    enGlider = 1;
    state = ME_NOSE_DOWN;
    entry = 1;
  }
  if(cmd == GC_STOP) {
    Serial.println(F("STOP"));
    enGlider = 0;
    pumpOff();
    digitalWrite(motAPWM, 0);
    digitalWrite(motStdby, LOW);
    glide_cycles_completed = 0;
  }
  
  if(cmd == GC_CIRCLE) {
    if(circle) {
      circle = 0;
      Serial.println(F("Circle off!"));
    }
    else {
      circle = 1;
      Serial.println(F("Circle on!"));
    }
  }

  if(cmd == GC_FEEDFORWARD) {
    if(feedforward) {
      feedforward = 0;
    }
    else{
      feedforward = 1;
    }
    Serial.print(F("PID Status: "));
    Serial.println(DoLinPID);
    Serial.print(F("Fuzzy Status: "));
    Serial.println(DoFuzzy);
    Serial.print(F("Feedforward Status: "));
    Serial.println(feedforward);
  }


  if(cmd == GC_BEGIN) { // execute once at beginning of test
    Serial.println(F("This should be first"));
    pinMode(motAPWM, OUTPUT);
    pinMode(motAConf1, OUTPUT);
    pinMode(motAConf2, OUTPUT);
    pinMode(motStdby, OUTPUT);

    pinMode(pumpOn, OUTPUT);
    pinMode(pumpDir, OUTPUT);

    digitalWrite(pumpOn, LOW);
    delay(10);
    digitalWrite(pumpDir, LOW);
    delay(10);
    digitalWrite(motAConf1, LOW);
    delay(10);
    digitalWrite(motAConf2, LOW);
    delay(10);
    digitalWrite(motStdby, LOW);
    delay(10);
    analogWrite(motAPWM, 0);
    delay(10);
  }
  
  if(cmd == GC_PRESSURE_CONTROL) {
    if(pressureControlOn) {
      pressureControlOn = 0;
      Serial.println(F("Pressure Control OFF"));
    }
    else {
      pressureControlOn = 1;
      Serial.println(F("Pressure Control ON"));
    }
    
  }
  
  if(cmd == GC_FLOAT) {
    enGlider = 1;
    state = ME_FLOAT;
    entry = 1;
  }
  
  if(cmd == GC_LINEAR) {
    if(DoLinPID) {
      DoLinPID = 0;
      //Serial.println(F("Linear PID off"));
      error_act = 0;
      error_prev = 0;
      I = 0;
      param.linRate = 200;
    }
    else {
      DoLinPID = 1;
      DoFuzzy = 0;
      //Serial.println(F("Linear PID on"));
    }
    Serial.print(F("PID Status: "));
    Serial.println(DoLinPID);
    Serial.print(F("Fuzzy Status: "));
    Serial.println(DoFuzzy);
    Serial.print(F("Feedforward Status: "));
    Serial.println(feedforward);
  }

  if(cmd == GC_FUZZY){
    if(DoFuzzy) {
      DoFuzzy = 0;


    }
    else{
      DoFuzzy = 1;
      DoLinPID = 0;
      
    }
    Serial.print(F("PID Status: "));
    Serial.println(DoLinPID);
    Serial.print(F("Fuzzy Status: "));
    Serial.println(DoFuzzy);
    Serial.print(F("Feedforward Status: "));
    Serial.println(feedforward);
  }
  
  if(cmd == GC_RESET) { // GC_RESET to trimming position    
    moveWater(param.tankMid);
    //turn off pump when it's time
    while(abs(getFiltAnalog(tankLevel)-param.tankMid) > 20) {
    }
    digitalWrite(pumpOn, LOW);
    Serial.println(F("Tank Reset"));

    moveLinMass(param.linMid, param.linRate);
    // Turn linear mass off when it's time
      while(abs(getFiltAnalog(linPos)-param.linMid) > 20) {
      }
      digitalWrite(motAPWM, 0);
      digitalWrite(motStdby, LOW);
      Serial.println(F("Linear Mass Reset"));
    enGlider = 0;
  }
  
  if(cmd == GC_NULL) { // continue normal state machine run
    if(!enGlider) return;
    switch(state) { // select current state
    
      case ME_NOSE_DOWN:
        if(entry) {
          Serial.println(F("ME_NOSE_DOWN"));
          t0 = millis();
          moveWater(param.tankBackLimit); //sets pump direction then turns pump on
          
          if(circle) {
            Serial.println(F("Circling down"));
            rotServo.write( map( param.rotMid + param.rollover, -45, 45, rotPWMmin, rotPWMmax ) );
            
          }
          if(feedforward) {
            moveLinMass(param.downFeedforward, param.linRate);
          }
          else {
            if(DoLinPID) {
              motRate = linMassRatePID(param.linNoseDownTarget);
              analogWrite(motAPWM, motRate);
            }
            else if(DoFuzzy) {
              fuzzyControl(param.linNoseDownTarget,fuzzy);
              if((getFiltAnalog(linPos) >= param.linBackLimit) && (imu.pitch < param.linNoseUpTarget)) {
                analogWrite(motAPWM, 0);
                digitalWrite(motStdby, LOW);
              }
            }
            else
            {
              analogWrite(motAPWM, 0);
              digitalWrite(motStdby, LOW);
            }
          }
          
          
          entry = 0;
          pumpDone = 0;
          linDone = 0;
        }
        
        // Turn pump off when it's time
        if(abs(getFiltAnalog(tankLevel)-param.tankBackLimit) < 50) {
          digitalWrite(pumpOn, LOW);
          pumpDone = 1;
        }
        
        if(pumpDone || !feedforward) {
          if(DoLinPID) {
            if((getFiltAnalog(linPos) <= param.linFrontLimit) && (imu.pitch > param.linNoseDownTarget)) {
              analogWrite(motAPWM, 0);
              digitalWrite(motStdby, LOW);
              linDone = 1;
            }
            else if((getFiltAnalog(linPos) >= param.linBackLimit) && (imu.pitch < param.linNoseDownTarget)) {
              analogWrite(motAPWM, 0);
              digitalWrite(motStdby, LOW);
              linDone = 1;
            }
            else {
              I = 0;
              motRate = linMassRatePID(param.linNoseDownTarget);
              analogWrite(motAPWM, motRate);
            }
          }
          else if(DoFuzzy) {
            
            if((getFiltAnalog(linPos) <= param.linFrontLimit) && (imu.pitch > param.linNoseDownTarget)) {
              analogWrite(motAPWM, 0);
              digitalWrite(motStdby, LOW);
              linDone = 1;
            }
            else if((getFiltAnalog(linPos) >= param.linBackLimit) && (imu.pitch < param.linNoseDownTarget)) {
              analogWrite(motAPWM, 0);
              digitalWrite(motStdby, LOW);
              linDone = 1;
            }
            else {
              motRate = fuzzyControl(param.linNoseDownTarget,fuzzy);
              analogWrite(motAPWM, motRate);
            }
          }
          else if(!linDone) {
            moveLinMass(param.downFeedforward, param.linRate);//sets direction then turns on
          }
          
          // Turn linear mass off when it's time
          if(DoLinPID) {
            
            if((getFiltAnalog(linPos) <= param.linFrontLimit) && (imu.pitch > param.linNoseDownTarget)) {
              analogWrite(motAPWM, 0);
              digitalWrite(motStdby, LOW);
              linDone = 1;
            }
            else if((getFiltAnalog(linPos) >= param.linBackLimit) && (imu.pitch < param.linNoseDownTarget)) {
              analogWrite(motAPWM, 0);
              digitalWrite(motStdby, LOW);
              linDone = 1;
            }
            else {
              motRate = linMassRatePID(param.linNoseDownTarget);
              analogWrite(motAPWM, motRate);
            }
          }
          else {
            if((abs(getFiltAnalog(linPos) - param.downFeedforward)) < 20) {
              analogWrite(motAPWM, 0);
              digitalWrite(motStdby, LOW);
              linDone = 1;
            }
          }
        }
        else {
          if(abs(getFiltAnalog(linPos)-param.linMid) < 20) {
            digitalWrite(motAPWM, 0);
            digitalWrite(motStdby, LOW);
          }
        }
        
        if(pumpDone) {
          if(DoLinPID) {
            entry = 1;
            state = ME_GLIDE_DOWN;
            Serial.println(F("Nose is down."));
          }
          if(DoFuzzy) {
            entry = 1;
            state = ME_GLIDE_DOWN;
            Serial.println(F("Nose if fuzzily down."));
          }
          else {
            if(linDone) {
              entry = 1;
              state = ME_GLIDE_DOWN;
              Serial.println(F("Nose is down."));
            }
          }
        }
        
        if(pressureControlOn) {
            est_depth = pressure_m * (getFiltAnalog(pressureSensorPin) - pressure_b);
            if(est_depth >= param.glide_cycle_bottom) {
              digitalWrite(pumpOn, LOW); // turn pump off
              state = ME_NOSE_UP;
              entry = 1;
            }
          }
        
        if(millis() - t0 > param.desTime) { //exit condition
            digitalWrite(pumpOn, LOW); // turn pump off
            state = ME_NOSE_UP;
            entry = 1;
          }
        
        if(millis() - t0 > param.allowedWorkTime) { //Taking too long
          state = ME_PAUSE;
          entry = 1;
          Serial.println(F("Something was taking too long!"));
        }
        
          break;
        
      case ME_GLIDE_DOWN:
          if(entry) {
            Serial.println(F("ME_GLIDE_DOWN"));
            pumpOff();
            //t0 = millis();
            entry = 0;
          }
         
          if(DoLinPID) {
            
            if((getFiltAnalog(linPos) <= param.linFrontLimit) && (imu.pitch > param.linNoseDownTarget)) {
              analogWrite(motAPWM, 0);
              digitalWrite(motStdby, LOW);
              linDone = 1;
            }
            else if((getFiltAnalog(linPos) >= param.linBackLimit) && (imu.pitch < param.linNoseDownTarget)) {
              analogWrite(motAPWM, 0);
              digitalWrite(motStdby, LOW);
              linDone = 1;
            }
            else {
              motRate = linMassRatePID(param.linNoseDownTarget);
              analogWrite(motAPWM, motRate);
            }
          }
          else if(DoFuzzy) {
            
            if((getFiltAnalog(linPos) <= param.linFrontLimit) && (imu.pitch > param.linNoseDownTarget)) {
              analogWrite(motAPWM, 0);
              digitalWrite(motStdby, LOW);
              linDone = 1;
            }
            else if((getFiltAnalog(linPos) >= param.linBackLimit) && (imu.pitch < param.linNoseDownTarget)) {
              analogWrite(motAPWM, 0);
              digitalWrite(motStdby, LOW);
              linDone = 1;
            }
            else {
              motRate = fuzzyControl(param.linNoseDownTarget,fuzzy);
              analogWrite(motAPWM, motRate);
            }
          }
          else {
            if((abs(getFiltAnalog(linPos) - param.downFeedforward)) < 20) {
              analogWrite(motAPWM, 0);
              digitalWrite(motStdby, LOW);
              linDone = 1;
              }
            }
            
          if(pressureControlOn) {
            est_depth = pressure_m * (getFiltAnalog(pressureSensorPin) - pressure_b);
            if(est_depth >= param.glide_cycle_bottom) {
              state = ME_NOSE_UP;
              entry = 1;
            }
          }
              
          if(millis() - t0 > param.desTime) { //exit condition
            state = ME_NOSE_UP;
            entry = 1;
          }
          break;
        
      case ME_NOSE_UP:
        if(entry) {
          Serial.println(F("ME_NOSE_UP"));
          t0 = millis();
          I = 0;

          moveWater(param.tankFrontLimit);//set pump direction and turn on
          if(feedforward){
            moveLinMass(param.upFeedforward, param.linRate);
          }
          else {
            if(DoLinPID) {
            motRate = linMassRatePID(param.linNoseUpTarget);
            analogWrite(motAPWM, motRate);
            }
            else if(DoFuzzy) {
              fuzzyControl(param.linNoseUpTarget,fuzzy);
              if((getFiltAnalog(linPos) >= param.linBackLimit) && (imu.pitch < param.linNoseUpTarget)) {
                analogWrite(motAPWM, 0);
                digitalWrite(motStdby, LOW);
              }
            }
            else
            {
            analogWrite(motAPWM, 0);
            digitalWrite(motStdby, LOW);
            }
          }
          
          
          if (circle) {
            Serial.println(F("Circling down"));
            rotServo.write( map( param.rotMid - param.rollover, -45, 45, rotPWMmin, rotPWMmax ) );
          }
          
          entry = 0;
          pumpDone = 0;
          linDone = 0;
        }
        
        //turn off pump when it's time
        if(abs(getFiltAnalog(tankLevel)-param.tankFrontLimit) < 20) {
          digitalWrite(pumpOn, LOW);
          pumpDone = 1;
        }
        
        if(pumpDone || !feedforward) {
          if(DoLinPID) {
            motRate = linMassRatePID(param.linNoseUpTarget);
            analogWrite(motAPWM, motRate);
          }
          else if(DoFuzzy) {
            if((getFiltAnalog(linPos) >= param.linBackLimit) && (imu.pitch < param.linNoseUpTarget)) {
              analogWrite(motAPWM, 0);
              digitalWrite(motStdby, LOW);
              linDone = 1;
            }
            else if((getFiltAnalog(linPos) <= param.linFrontLimit) && (imu.pitch > param.linNoseUpTarget)) {
              analogWrite(motAPWM, 0);
              digitalWrite(motStdby, LOW);
              linDone = 1;
            }
            else {
              motRate = fuzzyControl(param.linNoseUpTarget,fuzzy);
              analogWrite(motAPWM, motRate);
            }
          }
          else if(!linDone){
           
            moveLinMass(param.upFeedforward, param.linRate);//set linmass direction and turn on
           
          }
        
        if(DoLinPID) {
         
          if((getFiltAnalog(linPos) >= param.linBackLimit) && (imu.pitch < param.linNoseUpTarget)) {
            analogWrite(motAPWM, 0);
            digitalWrite(motStdby, LOW);
            linDone = 1;
          }
          else if((getFiltAnalog(linPos) <= param.linFrontLimit) && (imu.pitch > param.linNoseUpTarget)) {
            analogWrite(motAPWM, 0);
            digitalWrite(motStdby, LOW);
            linDone = 1;
          }
          else {
            motRate = linMassRatePID(param.linNoseUpTarget);
            analogWrite(motAPWM, motRate);
          }
        }
        else {
          if(abs(getFiltAnalog(linPos) - param.upFeedforward) < 20) {
            analogWrite(motAPWM, 0);
            digitalWrite(motStdby, LOW);
            linDone = 1;
          }
        }
        }
        else {
          if(abs(getFiltAnalog(linPos)-param.linMid) < 20) {
            digitalWrite(motAPWM, 0);
            digitalWrite(motStdby, LOW);
          }
        }
        
        if(pumpDone) {
          if(DoLinPID) {
            entry = 1;
            state = ME_GLIDE_UP;
            Serial.println(F("Nose is up."));
          }
          if(DoFuzzy) {
            entry = 1;
            state = ME_GLIDE_UP;
            Serial.println(F("Nose is fuzzy up."));
          }
          else {
            if(linDone) {
              entry = 1;
              state = ME_GLIDE_UP;
              Serial.println(F("Nose is up."));
            }
          }
        }
        
        if(pressureControlOn) {
          est_depth = pressure_m * (getFiltAnalog(pressureSensorPin) - pressure_b);
          if(est_depth <= param.glide_cycle_top) {
            glide_cycles_completed += 1;
            Serial.print(glide_cycles_completed);
            Serial.print(F(" of "));
            Serial.print(param.number_of_glides);
            Serial.println(F(" glide cycles completed."));
            if( glide_cycles_completed < param.number_of_glides ) {
              digitalWrite(pumpOn, LOW);
              state = ME_NOSE_DOWN;
              entry = 1;
            }
            else {
              //stop
              digitalWrite(pumpOn, LOW);
              state = ME_PAUSE;
              glide_cycles_completed = 0;
            }
            }
          }
        
        if(millis() - t0 > param.riseTime) { //exit condition
          glide_cycles_completed += 1;
          Serial.print(glide_cycles_completed);
          Serial.print(F(" of "));
          Serial.print(param.number_of_glides);
          Serial.println(F(" glide cycles completed."));
          if( glide_cycles_completed < param.number_of_glides ) {
            digitalWrite(pumpOn, LOW);
            state = ME_NOSE_DOWN;
            entry = 1;
          }
          else {
            //stop
            state = ME_FLOAT;
            glide_cycles_completed = 0;
          }
          }
        
        if(millis() - t0 > param.allowedWorkTime) { //Taking too long
          state = ME_PAUSE;
          entry = 1;
          Serial.println(F("Something was taking too long!"));
        }

        break;
        
      case ME_GLIDE_UP:
        if(entry) {
          Serial.println(F("ME_GLIDE_UP"));
          //t0 = millis();
          pumpOff();
          entry = 0;
        }
        
        if(DoLinPID) {

          if((getFiltAnalog(linPos) >= param.linBackLimit) && (imu.pitch < param.linNoseUpTarget)) {
            analogWrite(motAPWM, 0);
            digitalWrite(motStdby, LOW);
            linDone = 1;
          }
          else if((getFiltAnalog(linPos) <= param.linFrontLimit) && (imu.pitch > param.linNoseUpTarget)) {
            analogWrite(motAPWM, 0);
            digitalWrite(motStdby, LOW);
            linDone = 1;
          }
          else {
            motRate = linMassRatePID(param.linNoseUpTarget);
            analogWrite(motAPWM, motRate);
          }
        }
        else if(DoFuzzy) {
          
            
            if((getFiltAnalog(linPos) >= param.linBackLimit) && (imu.pitch < param.linNoseUpTarget)) {
              analogWrite(motAPWM, 0);
              digitalWrite(motStdby, LOW);
              linDone = 1;
            }
            else if((getFiltAnalog(linPos) <= param.linFrontLimit) && (imu.pitch > param.linNoseUpTarget)) {
              analogWrite(motAPWM, 0);
              digitalWrite(motStdby, LOW);
              linDone = 1;
            }
            else {
              motRate = fuzzyControl(param.linNoseUpTarget,fuzzy);
              analogWrite(motAPWM, motRate);
            }
          }
        else {
          if(abs(getFiltAnalog(linPos) - param.upFeedforward) < 20) {
            analogWrite(motAPWM, 0);
            digitalWrite(motStdby, LOW);
            linDone = 1;
          }
        }
        
        if(pressureControlOn) {
          est_depth = pressure_m * (getFiltAnalog(pressureSensorPin) - pressure_b);
          if(est_depth <= param.glide_cycle_top) {
            glide_cycles_completed += 1;
            Serial.print(glide_cycles_completed);
            Serial.print(F(" of "));
            Serial.print(param.number_of_glides);
            Serial.println(F(" glide cycles completed."));
            
            if( glide_cycles_completed < param.number_of_glides ) {
              state = ME_NOSE_DOWN;
              entry = 1;
            }
            else {
              //stop
              state = ME_FLOAT;
              entry = 1;
              glide_cycles_completed = 0;
            }
            }
          }
        
        if(millis() - t0 > param.riseTime) {
          glide_cycles_completed += 1;
          Serial.print(glide_cycles_completed);
          Serial.print(F(" of "));
          Serial.print(param.number_of_glides);
          Serial.println(F(" glide cycles completed."));
          
          if( glide_cycles_completed < param.number_of_glides ) {
            state = ME_NOSE_DOWN;
            entry = 1;
          }
          else {
            //stop
            state = ME_FLOAT;
            entry = 1;
            glide_cycles_completed = 0;
          }
        }
        break;
        
      case ME_PAUSE:
      //Serial.println("ME_PAUSE");
        digitalWrite(pumpOn, LOW);
        analogWrite(motAPWM, 0);
        analogWrite(motBPWM, 0);
        digitalWrite(motStdby, LOW);
      break;

      case ME_FLOAT:
      if(entry) {
        Serial.println(F("ME_FLOAT"));
        moveWater(param.tankFrontLimit);//set pump direction and turn on
        moveLinMass(param.upFeedforward, param.linRate); //moves linmass to upFeedforward position
        entry = 0;
        linDone = 0;
      }

      if(abs(getFiltAnalog(linPos) - param.linFrontLimit) < 20) {
            analogWrite(motAPWM, 0);
            digitalWrite(motStdby, LOW);
            linDone = 1;
      }

      if(abs(getFiltAnalog(tankLevel)-param.tankFrontLimit) < 20) {
          digitalWrite(pumpOn, LOW);
          pumpDone = 1;
        }

      if(linDone && pumpDone) {
        enGlider = 0;
      }
      
      break;
    }
  }
  
  
}

void pumpOff() {
  digitalWrite(pumpOn, LOW);
}
 
void error(char *str)
{
  Serial.print(F("error: "));
  Serial.println(str);
}

int getFiltAnalog(int APIN)
{
  int val = 0;
  for(int a=0; a<10; a++) {
    val = val + analogRead(APIN);
  }
  val = val/10;
  return val;
}

void moveWater(int dest) {
  delay(100);
  int currentPos = getFiltAnalog(tankLevel);
  if(currentPos == 0) {
    Serial.println(F("Draw wire (water tank) reads zero!"));
    Serial.println(F("...and also Eric sucks."));
    return;
  }
  Serial.print(F("Water Tank Start: "));
  Serial.println(currentPos);
  if(dest < tankbacklimit) {
    Serial.println(F("Cannot go that far back"));
    Serial.print(F("Setting destination to "));
    Serial.println(tankbacklimit);
    param.tankBackLimit = tankbacklimit;
    dest = param.tankBackLimit;
  }
  if(dest > tankfrontlimit) {
    Serial.println(F("Cannot go that far forward"));
    Serial.print(F("Setting destination to "));
    Serial.println(tankfrontlimit);
    param.tankFrontLimit = tankfrontlimit;
    dest = param.tankFrontLimit;
  }
  if(abs(currentPos - dest) < 10) {
    Serial.println("Already there");
    return; // Already there
  }

  if(currentPos > dest) {
    digitalWrite(pumpDir, HIGH); //WAS HIGH
  }
  else {
    digitalWrite(pumpDir, LOW); //WAS LOW
  }
  digitalWrite(pumpOn, HIGH);
  return;
}

void moveLinMass(int dest, int rate) {
  int currentPos = getFiltAnalog(linPos);
  if(currentPos == 0) {
    Serial.println(F("Draw wire (linmass) reads zero!"));
    Serial.println(F("...and also Eric sucks."));
    return;
  }
  //Serial.print("Linear Mass Start: ");
  //Serial.println(currentPos);
  if(dest < linfrontlimit) {
    Serial.println(F("Cannot go that far forward"));
    Serial.print(F("Setting destination to "));
    Serial.println(linfrontlimit);
    param.linFrontLimit = linfrontlimit;
    dest = param.linFrontLimit;
  }
  if(dest > linbacklimit) {
    Serial.println(F("Cannot go that far backward"));
    Serial.print(F("Setting destination to "));
    Serial.println(linbacklimit);
    param.linBackLimit = linbacklimit;
    dest = param.linBackLimit;
  }
  if(abs(currentPos - dest) < 20) {
    Serial.println(F("Already there"));
    return; // Already there
  }

  if(currentPos > dest) {
    digitalWrite(motAConf1, LOW);
    digitalWrite(motAConf2, HIGH);
  }
  else {
    digitalWrite(motAConf1, HIGH);
    digitalWrite(motAConf2, LOW);
  }
  digitalWrite(motStdby, HIGH);
  analogWrite(motAPWM, rate);

  return;
}

float linMassRatePID(int dest) {
  float kp, kd, ki;//kp = 10, ki = 0, kd = 0 currently
  kp=param.linkp;//Values set in parameters or by calling update -linkp [number]
  ki=param.linki;//
  kd=param.linkd;//
  float P, D, rate;
  int currentPos = getFiltAnalog(linPos);//This is giving some funky positions. Maybe need to filter a bit harder.
  int linpitchlimit = 45;
  if(dest < -linpitchlimit) {//check bounds
    Serial.println(F("Cannot go that far forward"));
    Serial.print(F("Setting destination to "));
    Serial.println(-linpitchlimit);
    dest = -linpitchlimit;
  }
  if(dest > linpitchlimit) {//check bounds
    Serial.println(F("Cannot go that far backward"));
    Serial.print(F("Setting destination to "));
    Serial.println(linpitchlimit);
    dest = linpitchlimit;
  }
  
  error_prev = error_act;
//  error_act = dest - currentPitch;
  error_act = dest - imu.pitch;
  P = error_act;
  I = I + error_prev;
  D = 0;
  rate = (P*kp + I*ki + D*kd);
//  rate = P*kp/param.rateScale;
  rate = abs(rate);//make it positive rate so things don't get too weird.
  rate = constrain(rate, 0, 255);
  if(rate < 15) {
    rate = 0;
  }
  if(imu.pitch > dest) {//if it has to go forward, set pins to do that
    digitalWrite(motAConf1, LOW);
    digitalWrite(motAConf2, HIGH);
  }
  else {
    digitalWrite(motAConf1, HIGH);//backward reverse pins
    digitalWrite(motAConf2, LOW);
  }
  digitalWrite(motStdby, HIGH);//turn on
  return rate;
}

float fuzzyControl(int dest,Fuzzy *fuzzy) {
  
  
  float rate;
  int currentPos = getFiltAnalog(linPos);//This is giving some funky positions. Maybe need to filter a bit harder.
  int linpitchlimit = 45;
  if(dest < -linpitchlimit) {//check bounds
    Serial.println(F("Cannot go that far forward"));
    Serial.print(F("Setting destination to "));
    Serial.println(-linpitchlimit);
    dest = -linpitchlimit;
  }
  if(dest > linpitchlimit) {//check bounds
    Serial.println(F("Cannot go that far backward"));
    Serial.print(F("Setting destination to "));
    Serial.println(linpitchlimit);
    dest = linpitchlimit;
  }
  
//  error_prev = error_act;
//  error_act = dest - currentPitch;
  error_act = dest - imu.pitch;
  Serial.print("AngleErr: ");
  Serial.print(error_act);
  error_act = abs(error_act);
  fuzzy->setInput(1,error_act);
  fuzzy->fuzzify();
  rate = fuzzy->defuzzify(1);
  rate = abs(rate);//make it positive rate so things don't get too weird.
  rate = constrain(rate, 0, 255);
  Serial.print(" Motor Rate: ");
  Serial.println(rate);
  if(rate < 15) {
    rate = 0;
  }
  //rate = 0; //Make it so it doesn't actually do it
  if(imu.pitch > dest) {//if it has to go forward, set pins to do that
    digitalWrite(motAConf1, LOW);
    digitalWrite(motAConf2, HIGH);
  }
  else {
    digitalWrite(motAConf1, HIGH);//backward reverse pins
    digitalWrite(motAConf2, LOW);
  }
  digitalWrite(motStdby, HIGH);//turn on

  
  return rate;
}

void createSDfile(char* name_of_file) {
  char filename[] = "LOGGER00.csv";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      File logfile = SD.open(filename, FILE_WRITE);
      
      if (! logfile) {
        error("couldnt create file");
      }

      Serial.print(F("Logging to: "));
      Serial.println(filename);
  
      // connect to RTC
      Wire.begin();  
      if (!rtc.begin()) {
        logfile.println(F("RTC failed"));
      }
  
      //logfile.println("millis,stamp,datetime,Pressure,Pitch,Roll,BallastTank,LinMassPos,tp1,tp2,yaw,rolld,pitchd,yawd,north,east,up,estdepth,ECOPUCK");    
      logfile.println(F("ms,Pressure,Pitch,Roll,BallastTank,LinMassPos,yaw,estdepth,Vin,Iin"));
      logfile.close();
      //SDgo = 0;
      break;  // leave the loop!
    }
  }
  strcpy(name_of_file, filename);
  //return filename;
}

void printFile(void) {
  File dataFile = SD.open(name_of_file);

  // if the file is available, write to it:
  if (dataFile) {
    while (dataFile.available()) {
      Serial.write(dataFile.read());
    }
    dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.print("error opening ");
    Serial.println(name_of_file);
  }
}

