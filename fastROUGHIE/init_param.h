//Record last bits of glide angles
float lastUpAngle;
float lastDownAngle;
int downLoops;
int upLoops;

//Create objects
Servo rotServo;
//File logfile; //SD logging stuff

//Set up SD card stuff
//#define SYNC_INTERVAL 1000
//uint32_t syncTime = 0;
////#define SYNC_SIZE 512
////uint32_t syncSize = 0;
char* name_of_file = "LOGGER00.csv";

//Setup fuzzy object
Fuzzy *fuzzy = new Fuzzy();

//Set up different boolean variables
bool SDgo = 0;
bool circle = 0;
bool dubin = 0;
bool feedforward = 0;
bool linPID = 0;
bool linFuzzy = 0;
bool pressureControl = 0;
bool turnFeedback = 0;
int command;//Tracks the current command from serial
bool flag = false;


//Set up timing parameters
unsigned long t0;
unsigned long tstart;
int completedGlides;
int currentState;
int nextState;

//Set up main glider parameters structure, things in here don't get changed except during config
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
  float rollkp;
  float rollki;
  float rollkd;
  int linNoseDownTarget;            // GLIDER ANGLE TARGET FOR DESCENT
  int linNoseUpTarget;              // GLIDER ANGLE TARGET FOR ASCENT
  int number_of_glides;             // NUMBER OF GLIDE CYCLES TO COMPLETE BEFORE RETURNING TO SURFACE
  float glide_cycle_bottom;         // DESIRED BOTTOM OF GLIDE CYCLE (FT), FOR USE WITH PRESSURE SENSOR CONTROL
  float glide_cycle_top;            // DESIRED TOP OF GLIDE CYCLE (FT), FOR USE WITH PRESSURE SENSOR CONTROL
  int desiredRotaryPosition;        // GLIDER ROLL ANGLE TARGET
  int rollover;                     // AMOUNT FOR ROTARY SERVO TO TURN (INTEGER BETWEEN 0 & 179)
  int downFeedforward;              // down feedforward location
  int upFeedforward;                // up feedforward location
  unsigned int neutralTime;         // neutral wait time
  unsigned int dubinTime;           // dubin turn time
  unsigned int FFtime;              // feedforward timeout
  int FFerror;                      // feedforward error bounds
}
param;

//IMU structure, gets updated every loop
struct imu_t {
  float pitch;
  float pitchD;
  float roll;
  float rollD;
  float yaw;
  float yawD;
  float pitchOffset;
  float rollOffset;
}
imu;

struct compass_t {
  int rawX;
  int rawY;
  int rawZ;
  float heading;
}
compass;

//Glider structure, gets updated every loop
struct glider_t {
  int linPos;
  int tankPos;
  int pressure;
  unsigned long runTime;
}
glider;

//Setup pin numbers
const int motAPWM = 3;//Actual Pin: 2;      // PWM PIN FOR LINEAR MASS MOTOR
const int motAConf1 = 32;//Actual Pin: 28;   // CONFIGURATION PIN 1 FOR LINEAR MASS MOTOR
const int motAConf2 = 34;//Actual Pin: 26;   // CONFIGURATION PIN 2 FOR LINEAR MASS MOTOR
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

//Define the limits for things
//Linear limits
const int linmid = (340+430)/2;        // MIDDLE POSITION FOR LINEAR MASS
const int linfrontlimit = 250; // FRONT LIMIT FOR LINEAR MASS
const int linbacklimit = 600;  // BACK LIMIT FOR LINEAR MASS
const int downFeedforward_default = 393;
const int upFeedforward_default = (340+430)/2;

// WATER TANK LIMITS
const int tankmid = 270;        // MIDDLE POSITION FOR BALLAST TANK
const int tankbacklimit = 80;   // BACK LIMIT FOR BALLAST TANK
const int tankfrontlimit = 400; // FRONT LIMIT FOR BALLAST TANK

// ROTARY MASS LIMITS
const int rotmid = 0;                 // MIDDLE POSITION FOR ROTARY MASS
const int rotlowlimit = rotmid - 44;  // LOW LIMIT FOR ROTARY MASS
const int rothighlimit = rotmid + 44; // HIGH LIMIT FOR ROTARY MASS
const float rotPWMmin = 65.0;             // ROTARY LOW PWM LIMIT (FOUND VIA BENCHTEST)
const float rotPWMmax = 170.0;            // ROTARY HIGH PWM LIMIT (FOUND VIA BENCHTEST)


// DEFAULT PARAMETER VALUES
const int linRate_default = 255;       // DEFAULT RATE FOR LINEAR MASS MOTOR
const unsigned int desTime_default = 28000;       // DEFAULT DESCENT TIME (MILLISECONDS)
const unsigned int riseTime_default = 28000;      // DEFAULT RISE TIME (MILLISECONDS)
const unsigned int allowedWorkTime_default = 60000;      // DEFAULT ALLOWED WORK TIME
const int linNoseDownTarget_default = -30;    //Default glide angles
const int linNoseUpTarget_default = 30;
const float linkp_default = 10.0;
const float linki_default = 0.0;
const float linkd_default = 0.0;
const float rollkp_default = 0.2;
const float rollki_default = 0.0;
const float rollkd_default = 0.0;
const int number_of_glides_default = 3;
const float glide_cycle_bottom_default = 6;   //Depth target in feet (experimental)
const float glide_cycle_top_default = 2;
const int desiredRotaryPosition_default = 90;
const int rollover_default = 20;          //Default roll angle for turn and dubin
const unsigned int neutralTime_default = 10000;  //Neutral wait time at glide inflection
const unsigned int dubinTime_default = 10000;   //Turning time for dubins path
const unsigned int FFtime_default = 20000;
const int FFerror_default = 5;
const unsigned int pumpTime = 5000;

//Initiate PID stuff
float linI = 0.0;
float rollI = 0.0;
float rotOutput = 0.0;
float rollOutput;
float rotStor = 0.0;

//Set up pressure sensor calibration
double pressure_m = 0.0423062;
double pressure_b = 102.3;


//Setup Help
const char help_1[] PROGMEM = "-----------ROUGHIE HELP MENU-----------";
const char help_2[] PROGMEM = "params - shows the current parameters and acceptable ranges";
const char help_3[] PROGMEM = "reset - centers the linear mass and water tank (for trimming)";
const char help_4[] PROGMEM = "currentpos - shows the current position of the actuators";
const char help_5[] PROGMEM = "start - starts the glide cycle";
const char help_6[] PROGMEM = "stop - stops the glide cycle";
const char help_7[] PROGMEM = "linear - toggles the PID controller on/off for the linear mass";
const char help_8[] PROGMEM = "feedforward - toggles feedforward controller on/off for feedforward";
const char help_9[] PROGMEM = "fuzzy - toggles fuzzy controller on/off";
const char help_10[] PROGMEM = "circle - toggles spiral mode";
const char help_11[] PROGMEM = "dubin - toggles dubin mode";
const char help_12[] PROGMEM = "turnFeedback - toggles turning feedback";
const char help_13[] PROGMEM = "sdstart - opens a new file and starts datalogging";
const char help_14[] PROGMEM = "sdstop <notes> - stops datalogging, adds the text in <notes> to end of file and closes file";
const char help_15[] PROGMEM = "pressurecontrol - toggles pressure control on/off";
const char help_16[] PROGMEM = "turnto <position> - rotates the rotary servo to <position> (90 is center)";
const char help_17[] PROGMEM = "float - commands glider to float with linear mass in front position";
const char help_18[] PROGMEM = "pressurecal - calibrates pressure sensor (glider must be on or above surface of water)";
const char help_19[] PROGMEM = "imucal - calibrate IMU to current offsets";
const char help_20[] PROGMEM = "imureset - reset IMU calibration";
const char help_21[] PROGMEM = "update - <parameter> <newValue> - updates <parameter> to <newValue> (don't forget the -)";
const char help_22[] PROGMEM = "Parameters available for updating are:";
const char help_23[] PROGMEM = "\trollover - If a circle path is enabled, this is the amount the rotary servo will turn in each direction";
const char help_24[] PROGMEM = "\tglidebottom - If using pressure control, this is the depth in feet of the glide top";
const char help_25[] PROGMEM = "\tglidetop - If using pressure control, this is the depth in feet of the glide bottom";
const char help_26[] PROGMEM = "\tnumber_of_glides - Number of glides to complete before floating";
const char help_27[] PROGMEM = "\trothighlimit - Rotary servo high limit";
const char help_28[] PROGMEM = "\trotlowlimit - Rotary servo low limit";
const char help_29[] PROGMEM = "\tlinfrontlimit - Linear mass front limit";
const char help_30[] PROGMEM = "\tlinbacklimit - Linear mass back limit";
const char help_31[] PROGMEM = "\tupFeedforward - feedforward position used on up glides";
const char help_32[] PROGMEM = "\tdownFeedforward - feedforward position used on down glides";
const char help_33[] PROGMEM = "\ttankfrontlimit - Ballast tank's high (empty) limit";
const char help_34[] PROGMEM = "\ttankbacklimit - Ballast tank's low (full) limit";
const char help_35[] PROGMEM = "\tlinrate - How quickly to move the linear mass (when PID isn't on)--this is an integer between 0 & 255";
const char help_36[] PROGMEM = "\tdestime - The descent time (in milliseconds)";
const char help_37[] PROGMEM = "\trisetime - The risetime (in milliseconds)";
const char help_38[] PROGMEM = "\tneutraltime - The neutral time (in millis)";
const char help_39[] PROGMEM = "\tdubintime - The time to turn for dubin's path";
const char help_40[] PROGMEM = "\ttankmid - The definition of the middle position for the ballast tank";
const char help_41[] PROGMEM = "\tlinmid - The definition of the middle position for the linear mass";
const char help_42[] PROGMEM = "\trotmid - The definition of the middle position for the rotary servo";
const char help_43[] PROGMEM = "\tallowedWorkTime - The amount of time to give the system before timing out";
const char help_44[] PROGMEM = "\tlinNoseUpTarget - Target angle of the glider when coming up (postive angle)";
const char help_45[] PROGMEM = "\tlinNoseDownTarget - Target angle of the glider when going down (negative angle)";
const char help_46[] PROGMEM = "\tlinkp - Linear mass PID proportional gain";
const char help_47[] PROGMEM = "\tlinki - Linear mass PID integral gain";
const char help_48[] PROGMEM = "\tlinkd - Linear mass PID derivative gain";
const char help_49[] PROGMEM = "\trollkp - Roll PID proportional gain";
const char help_50[] PROGMEM = "\trollki - Roll PID integral gain";
const char help_51[] PROGMEM = "\trollkd - Roll PID derivative gain";
const char help_52[] PROGMEM = "\tFFtime - feedforward time";
const char help_53[] PROGMEM = "\tFFerror - feedforward error bounds";
const char help_54[] PROGMEM = "gimme <something> - prints the current reading of <something>";
const char help_55[] PROGMEM = "Things to look at are:";
const char help_56[] PROGMEM = "\troll";
const char help_57[] PROGMEM = "\tpitch";
const char help_58[] PROGMEM = "\tattitude";
const char help_59[] PROGMEM = "\ttank";
const char help_60[] PROGMEM = "\tlinear";
const char help_61[] PROGMEM = "\tgps";
const char help_62[] PROGMEM = "\tecopuck";
const char help_63[] PROGMEM = "\tpressure";
const char help_64[] PROGMEM = "\tpower";
const char help_65[] PROGMEM = "\tglideAngles";
const char help_66[] PROGMEM = "rollTest - tests roll system step response";
const char help_67[] PROGMEM = "toDefault - resets parameters to default values";
const char help_68[] PROGMEM = "---------------------------------------";

const char* const helpTable[] PROGMEM = {help_1,help_2,help_3,help_4,help_5,help_6,help_7,help_8,help_9,help_10,help_11,help_12,help_13,help_14,help_15,help_16,help_17,help_18,help_19,help_20,help_21,help_22,help_23,help_24,help_25,help_26,help_27,help_28,help_29,help_30,help_31,help_32,help_33,help_34,help_35,help_36,help_37,help_38,help_39,help_40,help_41,help_42,help_43,help_44,help_45,help_46,help_47,help_48,help_49,help_50,help_51,help_52,help_53,help_54,help_55,help_56,help_57,help_58,help_59,help_60,help_61,help_62,help_63,help_64,help_65,help_66,help_67,help_68};
char buffer[80];

int lin, pump, mode;
float rot,rotStorage;

///SD LOGGING SETUP
SdFat sd;
StdioStream stdioFile;

