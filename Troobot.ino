/******************************************************************
* Troobot.ino
* 
* version: 0
* 
* Like move_to_xy, 
* but with separate robot levels (e.g. Interface, Organizer, etc.),
* and without Kangaroo.
******************************************************************/
#include <SoftwareSerial.h>
#include <math.h>

//#include <Kangaroo.h>
//#include "useful_defines.h"

//=====Constants=====
const float piApprox = 3.14159;
const byte numInputs = 2; // Number of user-defined variables
const float rads = 57.2958; // radians to deg conversion

//=====for tracking time=====
unsigned long currentMillis; // [ms]
const byte stopDuration = 1; // [s]

//=====for tracking position=====
float* targets[2];
float introPoint[2] = {0.0, 100.0};
float outroPoint[2] = {1000.0, 100.0};

//=====variables given by user=====
boolean regularTour = true;
boolean useDefaultSettings = true; // The program will use the default settings. Set true because true intuitively relates to default.
int xCoordinate = 0; // [cm]
int yCoordinate = 100; // [cm]
int orientation = 0; // [deg]
int moveSpeed = 20; // [cm / s]
int numPositions = 1; // number of positions the robot moves to in succession
boolean ifTurnClockwise = true; // The robot will move clockwise.
boolean ifMoveForward = true; // The robot will move forward.
int input = 0; // variable that temporarily holds user input for manipulation before reassigning

//=====for user question=====
const char questionXCoordinate[] = "What should be the x coordinate? (units: centimeters [cm])";
const char questionYCoordinate[] = "What should be the y coordinate? (units: centimeters [cm])";

boolean waitingForResponse = false;
char* questions[numInputs + 1]; // an array with # of elements equal to # of inputs from user + 1 for 0 (remember to add 1 element when no longer square)
char* question;

//=====for user response=====
const byte buffSize = 47;
char userResponseXCoordinate[buffSize];
char userResponseYCoordinate[buffSize];

const char endMarker = '\r';
byte bytesRecvd = 0;
boolean ackResponse = false; // There is a response that needs acknowledgment.

//=====for LEDs=====
const byte ledGPin = 6;
const byte ledRPin = 7;
const unsigned long ledOnMillis = 300; // [ms]
const unsigned long ledGBaseInterval = 500; // [ms]
unsigned long ledGInterval = ledGBaseInterval;
byte ledGState = HIGH;
byte ledRState = HIGH;
unsigned long prevLedGMillis; // [ms]

//=====for pushbutton switch=====
const byte buttonPin = 5;
byte buttonState;

//=====for Motion Controller=====
const byte txPin = 11;
const byte rxPin = 10;

SoftwareSerial serialPort (rxPin, txPin);
KangarooSerial K (serialPort);
KangarooChannel Drive (K, 'D');
KangarooChannel Turn (K, 'T');

long speedDivisor = 4;
long speedLimit;

/* ---------------------------------------------------------*/
/* getCurrentPosition() maintains these global accumulator variables: */
float theta = 0.0; // RJ's heading
float xPos = 0.0; // RJ's x position in cm
float yPos = 0.0; // RJ's y position in cm
float totalCm = 0.0; // total cm traveled
/* ---------------------------------------------------------*/
/* robot dimensions: */
// For RoboJay, d=317.5mm wheels, and encoders of x=1024 pulses/revolution (680 RPM motor model)
// For Nexus, d=143mm wheels, and encoders of x=12 counts(AKA pulses?)/revolution (120 RPM motor model)
// For DFRobot Turtle, ...
float pulsesPerCm;

const float wheelDiam = 14.3; // [cm]
const float wheelBase = 28.50; // For Nexus: 28.50 // cm
/* ---------------------------------------------------------*/
/* locateTarget() uses these global variables: */
float xTarget; // x lateral target position
float yTarget; // y lateral target position

float targetBearing; // bearing in rad(?) from current position
float targetDistance; // distance in cm from position
float headingError; // heading error in degrees
/* ---------------------------------------------------------*/
//======Encoders======
byte m1 = 2,
  m2 = 3;

volatile long[] coders = { 0, 0 };

int[] lastSpeed = { 0, 0 };

int pulsesPerRev = 20;

void setup()
{
  Serial.begin(9600);
  
  while(!Serial);
  
  Serial.println("Starting Troobot.ino");
  
  initEncoders();
  
  pinMode(ledGPin, OUTPUT);
  pinMode(ledRPin, OUTPUT);
  
  digitalWrite(ledGPin, HIGH);
  digitalWrite(ledRPin, HIGH);

  pinMode(buttonPin, INPUT_PULLUP);
  
  questions[0] = questionXCoordinate;
  questions[1] = questionYCoordinate;
  
  pulsesPerCm = (float) pulsesPerRev / (piApprox * wheelDiam); // [pulses/cm] = (x [pulses/rev])/(\pi*d [cm])
  
  speedLimit = pulsesPerRev / speedDivisor;
  
  delay(2000);
  
  flipLedG(); // Turns LEDg off, leaving LEDr on
}

void loop()
{
  // RJ is in sleep mode waiting for the button on its shoulder to be pressed
  Serial.println("\nPress the button to start.");
  
  waitForButtonPress(); // Organizer: allow user to talk to RJ

  askGetAck(); // Organizer: Ask for, get, and acknowledge all pre-trial user settings
  
  flipLedR(); // Organizer: Turns LEDr off, leaving LEDg off

  Serial.println("Press the button when you're ready to go.");
  
  waitForButtonPress(); // allow user to begin tour when ready
  
  flipLedG(); // Turns LEDg on to indicate moving, leaving LEDr off
  
  moveToCoordinates(); // Classifier
  
  flipLedG(); // Turns LEDg off to indicate stopping, leaving LEDr off
  
  flipLedR(); // Turns LEDr on, leaving LEDg off
  
  delay(2000);
}

void moveToCoordinates()
{
  getCurrentPosition();  // Organizer
  locateTarget(); // Organizer
  
  turnToHeading(); // Classifier
  driveToCoordinates(); // Classifier
  stopMoving(); // Classifier

  resetCoordinates(); // Organizer
}

/* ---------------------------------------------------------*/
/* getCurrentPosition() runs __ times per second.
 * It uses the Drive.getP() and Turn.getP() values as inputs
 * to maintain a set of global position and heading variables:
 * xPos, yPos, and theta.
*/
void getCurrentPosition()
{
  float driveCm;
  float turnCm;

  long drivePulses, turnPulses;
  long lastDrive, lastTurn;
  long driveSample, turnSample;

  // sample the drive and turn ticks as close together in time as possible
  driveSample = getDrivePosition(); // CHANGE: convert the returned speed to position of wheel rotation
  turnSample = getTurnPosition();

  // determine how many pulses since our last sampling
  drivePulses = driveSample - lastDrive;
  turnPulses = turnSample - lastTurn;

  // and update the last sampling for next time
  lastDrive = driveSample;
  lastTurn = turnSample;

  // convert ticks to cm
  driveCm = (float) drivePulses / pulsesPerCm; // total distance we have traveled since last sampling
  turnCm = (float) turnPulses / pulsesPerCm;
  
  // accumulate total cm traveled
  totalCm += driveCm;
  
  // accumulate total rotation around our center
  theta = turnCm / (piApprox * wheelBase); // ratio of turning circle

  // and clip the rotation to +/-360 deg
  theta *= 360.0; 
  theta = (float)((int)theta % 360); // [deg]
  Serial.print("theta: ");
  Serial.println(theta);

  // now calculate and accumulate our position in cm
  yPos += (driveCm * (cos(theta / rads)));
  Serial.print("yPos: ");
  Serial.println(yPos);
  
  xPos += (driveCm * (sin(theta / rads)));
  Serial.print("xPos: ");
  Serial.println(xPos);

  delay(1000);
}

long getDrivePosition()
{
  long dPos = 0;

  return dPos;
}

long getTurnPosition()
{
  long tPos = 0;

  return tPos;
}

/* ---------------------------------------------------------*/
/* calculate distance and bearing to target
 *  inputs are: xTarget, xPos, and yTarget, yPos
 *  outputs are: targetDistance, headingError
 */
void locateTarget()
{
  float x, y;

  x = xTarget - xPos;
  y = yTarget - yPos;

  targetDistance = sqrt((x * x) + (y * y)); // [cm]

  /* no divide-by-zero allowed! */
  if (x > 0.00001)
    targetBearing = 90.0 - (atan(y / x) * rads);
    
  else if (x < -0.00001)
    targetBearing = -90.0 - (atan(y / x) * rads);
    
  else
    targetBearing = 0.0;
  
  Serial.print("targetBearing: ");
  Serial.println(targetBearing);

  headingError = targetBearing - theta; // [ deg]
  
  if (headingError > 180.0)
    headingError -= 360.0;
    
  else if (headingError < -180.0)
    headingError += 360.0;
  
  Serial.print("headingError: ");
  Serial.println(headingError);

  delay(1000);
}

void turnToHeading()
{
  Serial.println("turning to coordinates");
  
  long he = convertDegToTicks(headingError);

  // ADD: turn commands
  
  //Turn.p(he, tSpeedLimit).wait();
  
  delay(1000);
}

void driveToCoordinates()
{
  Serial.println("driving to coordinates");
  
  long dist = convertCmToPulses(targetDistance);

  // ADD: drive commands
  
  //Drive.p(dist, speedLimit).wait();
  
  delay(1000);
}

void giveIntro()
{
  Serial.println("I am RoboJay.");
  Serial.println("You are about to see JHU's Homewood campus.");
  delay(1000);
}

void giveOutro()
{
  Serial.println("I'll be back home when you're done with the tour, so come talk to me!");
  Serial.println("Goodbye.");
  delay(1000);
}

void stopMoving() 
{
  Serial.println("stopping");
  
  currentMillis = millis();

  int v = 255;
  
  while (millis() - currentMillis <= (stopDuration * 1000)) 
  {
    analogWrite(E1, v);
    analogWrite(E2, v);

    v -= 10;
  }

  digitalWrite(E1, LOW);
  digitalWrite(E2, LOW);
}

long convertCmToPulses(float cm) 
{
  long maxP = pulsesPerRev;
  long minP = 0;
  
  // distance traveled after 360 deg rotation = wheel circumference (assuming no slip)
  float wheelCircumf = piApprox * wheelDiam; //centimeters
  
  long pRange = abs(maxP) + abs(minP); // wheel position range [pulses]
  
  float ratioPulsesPerCm = pRange / wheelCircumf; // [pulses / cm]
  
  float pulses = cm * ratioPulsesPerCm;
  
  return (long) pulses;
}

long convertDegToPulses(float deg) 
{
  long maxP = pulsesPerRev;
  long minP = 0;
  
  long pRange = abs(maxP) + abs(minP); // wheel position range [pulses]
  
  long ratioPulsesPerDeg = pRange / 180; // [pulses / deg]
  
  long pulses = (long) deg * ratioPulsesPerDeg;

  return pulses;
}

void initEncoders() 
{
  attachInterrupt(m1, wheelSpeed1, CHANGE);
  
  attachInterrupt(m2, wheelSpeed2, CHANGE);
  
  //Serial.println("Done attaching interrupts to encoders");
}

void wheelSpeed1()
{
  coder[0]++; // count wheel 1 encoder interrupts
}

void wheelSpeed2()
{
  coder[1]++; // count wheel 2 encoder interrupts
}
