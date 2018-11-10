//  22,23,24,25,26,27,28,29,30,31,32, 34, 36   
//  analog : 0,1,2,3,4,5


//------------------------------------------------------------------------------------------------------------------------------------------------
///////////////////////---Motors---//////////////////////////////////

//motor
#define leftMotorForward 16  //motor directions
#define leftMotorBack 17
#define rightMotorForward 15
#define rightMotorBack 14

#define rightPWM 9 //PWM controls
#define leftPWM 8

#define forward 1
#define backward 0

int  leftSpeed  = 80 ; 
int rightSpeed  = 80 ;
int  backSpeed   = 80; 
int CM3speed =  80 ;
int CM6speed  = 80; 

int adjGoAndTurn = 800;

//------------------------------------------------------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------------------------------------------------------
///////////////////////---hole detection---//////////////////////////////////
#define TRIGGER_PIND  A4  // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PIND     A5  // Arduino pin tied to echo pin on ping sensor.
#define MAX_DISTANCED 10
NewPing sonarDown(TRIGGER_PIND, ECHO_PIND, MAX_DISTANCED);
float DownSensor , downold ;  
//--------------------------------------------------------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------------------------------------------------------
///////////////////////---wall following---//////////////////////////////////
byte maxSpeed;
byte minSpeed;
byte baseSpeed;

#define TRIGGER_PINL  A0  // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PINL     A1  // Arduino pin tied to echo pin on ping sensor.

#define TRIGGER_PINR  A2  // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PINR     A3  // Arduino pin tied to echo pin on ping sensor.

#define MAX_DISTANCE 50 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonarLeft(TRIGGER_PINL, ECHO_PINL, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarRight(TRIGGER_PINR, ECHO_PINR, MAX_DISTANCE);

//unsigned int pingSpeed = 30; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
//unsigned long pingTimer;     // Holds the next ping time.
float oldLeftSensor, oldRightSensor, leftSensor, rightSensor, frontSensor, oldFrontSensor, lSensor, rSensor ;

int wall_threshold = 300 ;
bool leftwall,rightwall;

//--------------------------------------------------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------------------------------------------------------
///////////////////////---coloum counting---//////////////////////////////////

#define MAX_DISTANCE_Co  30 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define TRIGGER_PIN_Co A5// A6 // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PIN_Co     A4//A7  // Arduino pin tied to echo pin on ping sensor.

NewPing sonarCount(TRIGGER_PIN_Co, ECHO_PIN_Co, MAX_DISTANCE_Co); // NewPing setup of pins and maximum distance.
Servo Countservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int countPosition = 0;    // variable to store the servo position
int lpole, mpole, rpole, numOfPole = 0;

//unsigned int pingSpeed = 30; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
//unsigned long pingTimer;     // Holds the next ping time.

float CountSensor , CSensor , oldCountSensor;
//----------------------------------------------------------------------------------------------------------------------------------------------



//----------------------------------------------------------------------------------------------------------------------------------------------
///////////////////////---Line Following---//////////////////////////////////

#define NUM_SENSORS  8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2

float Kp =22;//2 ;      //0.2//1//1.6 experiment to determine this, start by something small that just makes your bot follow the line at a slow speed  0.2
float Kd = 24 ; //29.5;//12.5;   //7.5//10 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
float Ki  = 0.01;//0.01 ; //0.001//0.01//
#define rightMaxSpeed 120 // max speed of the robot
#define leftMaxSpeed 120 // max speed of the robot
#define rightBaseSpeed 100 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 100  // this is the speed at which the motors should spin when the robot is perfectly on the line

#define iniMotorPower 150
#define black 0
#define white 1
#define margin 1000

// sensors 0 through 7 are connected to digital pins 24 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {36 , 34 , 32 , 30 , 28 , 26 , 24, 22}, NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
int motorSpeed;
int lastError = 0;
int integral = 0;
int rightMotorSpeed;
int leftMotorSpeed;

// --------------------junction------------------------------
char junction = ' ' ;
int whiteBLACK = 200;


//===========Mesh Solve==============================
bool a = 0 ;
int mode = 0;
# define STOPPED 0
# define FOLLOWING_LINE 1
# define NO_LINE 2
# define CONT_LINE 3
# define POS_LINE 4
# define RIGHT_TURN 5
# define LEFT_TURN 6
float  L1 , R1  , M;
char path[100] = "";
unsigned char pathLength = 0; // the length of the path
int pathIndex = 0;
unsigned int status = 0; // solving = 0; reach end = 1
unsigned int sensors[8];

//----------------------encoder-----------------------------
volatile long leftcount = 0;
volatile long rightcount = 0;

#define rightONEround   380
#define leftONEround    380

#define forward 1
#define backward 0

//int leftSpeed 120;
//int rightSpeed 120;
//int backSpeed  120 ;
//int CM3speed  120 ;
//int CM6speed  120 ;



//----------------------------------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------------------------------
///////////////////////---Color Detection---//////////////////////////////////
#define S0 23
#define S1 25
#define S2 27
#define S3 29
#define sensorOut 31
int r , g , b = 0;
String color = " ";
int frequency = 0;
//----------------------------------------------------------------------------------------------------------------------------------------------
int count = 0;
int n = 0;
