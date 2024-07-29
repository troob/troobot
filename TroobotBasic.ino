/*
 * Troobot Basic:
 * Initial State—[Robot Off]
 * -power pressed->[Robot Resting]
 * -program started->[Robot Moving to Predefined Coordinates]
 * -predefined coordinates reached->[Robot Returning to Initial Coordinates]
 * -initial coordinates reached->[Robot Resting]
 * -power pressed->[Robot Off]
 */

#include <math.h>

//======Encoders======
const byte es1Pin = 2, // encoder signal 1 pin
  es2Pin = 3; // encoder signal 2 pin

int ppr = 20, // pulses per revolution
  dppr = 0, // drive pulses per revolution
  tppr = 0, // turn pulses per revolution
  dVelLimit = 0, // drive velocity limit
  tVelLimit = 0; // turn velocity limit

volatile long pulseCount[] = {0,0}; 
volatile long totPulses[] = {0,0};  

long currentCount[] = {0,0},
  lastCount[] = {0,0},
  countDiff[] = {0,0}; 

static unsigned long lastTime = 0;

long currentTime,
  timeDiff;

double dppcm = 0.0, // drive pulses per cm
  tppcm = 0.0; // turn pulses per cm

double countsPerSec[] = {0.0,0.0},
  err[] = {0.0,0.0},
  rpms[] = {0.0,0.0};

//======Motor Driver======
const byte m1SigPin = 4,
  m1EnablePin = 5,
  m2EnablePin = 6,
  m2SigPin = 7;

//======Mobile Platform======
double wheelDiam = 6.35,
  wheelBase = 18.0;

//======Circle======
double piApprox = 3.14159,
  rads = 57.2958; // radians to deg conversion

//======Robot Pose======
double xPos = 0.0, // x position in cm
  yPos = 0.0, // y position in cm
  theta = 0.0, // heading
  totCm = 0.0, // total cm traveled
  
  dSamplePosition,
  tSamplePosition,

  dLastPosition,
  tLastPosition,

  dPulses,
  tPulses,

  dCm, 
  tCm;

//======Target Location======
double xTarget, // [cm], x lateral target position
  yTarget, // [cm], y lateral target position

  targetBearing, // [deg], bearing from current position
  targetDistance, // [cm], distance from position
  headingError; // [deg], heading error

const int numTourTargets = 2;
long tour[numTourTargets][2] = 
{
  { 0, 1 },
  { 0, 1 }
};

//======Controller======
double desRotVel = 0.1; // [rev/s], desired rotational velocity

//======Stats======
unsigned long loopCount;

//======Run Once======
void setup() 
{
  Serial.begin(9600);
  
  while(!Serial);

  displayMessage("Starting TroobotBasic.ino");

  initMotionControl();

  computePulsesPerRev();

  computePulsesPerCm();

  //computeVelocityLimit();
}

void displayMessage(String msg)
{
  Serial.print("Message: '");
  Serial.print(msg);
  Serial.println("'\n");
}

// Construct a motion controller
void initMotionControl()
{
  startEncoderChannels();
  
  startMotorChannels();

  stopMoving();

  displayMessage("Motion control initialized.");
}

void startEncoderChannels()
{
  attachInterrupt(digitalPinToInterrupt(es1Pin), updateEs1Vel, CHANGE);
  attachInterrupt(digitalPinToInterrupt(es2Pin), updateEs2Vel, CHANGE);

  displayMessage("Encoder channels started.");
}

void startMotorChannels()
{
  /* Start Motor Channel 1 */
  pinMode(m1EnablePin, OUTPUT);
  pinMode(m1SigPin, OUTPUT);

  /* Start Motor Channel 2 */
  pinMode(m2EnablePin, OUTPUT);
  pinMode(m2SigPin, OUTPUT);

  displayMessage("Motor channels started.");
}

void stopMoving()
{
  digitalWrite(m1EnablePin, LOW);
  digitalWrite(m2EnablePin, LOW);

  displayMessage("Stopped moving.");
}

void computePulsesPerRev()
{
  dppr = getMaxDriveValue() - getMinDriveValue();
  tppr = getMaxTurnValue() - getMinTurnValue();

  Serial.print("dppr: ");
  Serial.println(dppr);
  Serial.print("tppr: ");
  Serial.println(tppr);
  Serial.println();
}

int getMaxDriveValue()
{
  return ppr / 2;
}

int getMinDriveValue()
{
  return - ppr / 2;
}

int getMaxTurnValue()
{
  return ppr;
}

int getMinTurnValue()
{
  return - ppr;
}

void computePulsesPerCm()
{
  dppcm = (double) dppr / (piApprox * wheelDiam); // [pulses/cm] = (x [pulses/rev])/(\pi*d [cm])
  tppcm = (double) tppr / (piApprox * wheelDiam); // [pulses/cm] = (x [pulses/rev])/(\pi*d [cm])

  Serial.print("dppcm: ");
  Serial.println(dppcm);
  Serial.print("tppcm: ");
  Serial.println(tppcm);
  Serial.println();
}

void computeVelocityLimit()
{
  int dVelDivisor = 4,
    tVelDivisor = 10;
    
  dVelLimit = dppr / dVelDivisor;
  tVelLimit = tppr / tVelDivisor;
}

//======Run Until Unable to Continue======
void loop() 
{
  Serial.print("======Loop ");
  Serial.print(loopCount);
  Serial.println("======");
  
  if(loopCount < 1)
  {
    setTargetLocation(tour[loopCount][0], tour[loopCount][1]);
    
    moveToCoordinates();

    displayMessage("TB reached given coordinates.");

    //moveToCoordinates(0, 0, 0);

    //displayMessage("TB reached initial coordinates.");
  }

  delay(3000);

  loopCount++;
}

void setTargetLocation(int x, int y) // [ft]
{
  xTarget = convertFtToCm(x);
  yTarget = convertFtToCm(y);

  Serial.print("xTarget (cm): ");
  Serial.println(xTarget);
  Serial.print("yTarget (cm): ");
  Serial.println(yTarget);
  Serial.println();
}

double convertFtToCm(int ft)
{
  return ft * 12 * 2.54;
}

void moveToCoordinates()
{
  computeCurrentPose();
  locateTarget();

  turnToPosition();
  driveToPosition();

  if(digitalRead(m1EnablePin) == HIGH || digitalRead(m2EnablePin) == HIGH)
    stopMoving();

  resetCoordinates();
}

void computeCurrentPose()
{
  // sample the drive and turn positions, measured in pulses, as close together in time as possible
  dSamplePosition = getDrivePosition();
  tSamplePosition = getTurnPosition();

  // determine how many pulses since our last sampling
  dPulses = dSamplePosition - dLastPosition;
  tPulses = tSamplePosition - tLastPosition;
  
  // and update the last sampling for next time
  dLastPosition = dSamplePosition;
  tLastPosition = tSamplePosition;

  // convert pulses to cm
  dCm = dPulses / dppcm; // total distance we have traveled since last sampling
  tCm = tPulses / tppcm;

  // accumulate total cm traveled
  totCm += dCm;

  // accumulate total rotation around our center
  theta = tCm / (piApprox * wheelBase); // ratio of turning circle

  // and clip the rotation to +/-360 deg
  theta *= 360.0;
  theta = (double) ((int) theta % 360); // [deg]

  // now calculate and accumulate our position in cm
  yPos += dCm * cos(theta / rads);
  xPos += dCm * sin(theta / rads);

  displayPosition();
}

double getDrivePosition()
{
  return ( totPulses[0] + totPulses[1] ) / 2.0;
}

double getTurnPosition()
{
  return (double) totPulses[0] - totPulses[1];
}

void displayPosition()
{
  Serial.print("dSamplePosition (p): ");
  Serial.println(dSamplePosition);
  Serial.print("tSamplePosition (p): ");
  Serial.println(tSamplePosition);
  Serial.println();
  Serial.print("dPulses (p): ");
  Serial.println(dPulses);
  Serial.print("tPulses (p): ");
  Serial.println(tPulses);
  Serial.println();
  Serial.print("dLastPosition (p): ");
  Serial.println(dLastPosition);
  Serial.print("tLastPosition (p): ");
  Serial.println(tLastPosition);
  Serial.println();
  Serial.print("dCm (cm): ");
  Serial.println(dCm);
  Serial.print("tCm (cm): ");
  Serial.println(tCm);
  Serial.println();
  Serial.print("totCm (cm): ");
  Serial.println(totCm);
  Serial.println();
  Serial.println("===Current Pose===");
  Serial.print("xPos (cm): ");
  Serial.println(xPos);
  Serial.print("yPos (cm): ");
  Serial.println(yPos);
  Serial.print("theta (deg): ");
  Serial.println(theta);
  Serial.println();
}

void locateTarget()
{
  double x, y;

  x = xTarget - xPos;
  y = yTarget - yPos;

  targetDistance = sqrt( ( x * x ) + ( y * y ) ); // [cm]

  /* no divide-by-zero allowed! */
  if (x > 0.00001)
    targetBearing = 90.0 - (atan(y / x) * rads);
    
  else if (x < -0.00001)
    targetBearing = -90.0 - (atan(y / x) * rads);
    
  else
    targetBearing = 0.0;

  headingError = targetBearing - theta; // [ deg]
  
  if (headingError > 180.0)
    headingError -= 360.0;
    
  else if (headingError < -180.0)
    headingError += 360.0;

  displayTarget(x, y);
}

void displayTarget(double x, double y)
{
  Serial.println("===Target===");
  Serial.print("x (cm): ");
  Serial.println(x);
  Serial.print("y (cm): ");
  Serial.println(y);
  Serial.print("targetDistance (cm): ");
  Serial.println(targetDistance);
  Serial.print("targetBearing (deg): ");
  Serial.println(targetBearing);
  Serial.print("headingError (deg): ");
  Serial.println(headingError);
  Serial.println();
}

void turnToPosition()
{
  long mPulses;
  
  double tPulses,
    desiredRPM;
    
  if(abs(headingError) > 0.0)
  {
    displayMessage("Turning to position.");
    
    // clear encoder data buffer
    pulseCount[0] = 0; 
    pulseCount[1] = 0;
  
    tPulses = convertDegToPulses(headingError);
    
    mPulses = (long) ( tPulses / 2.0 ); // assumes wheels should spin at same speed

    // CHANGE: turn direction based on which way headingError indicates is faster
    // set wheels to spin in different directions
    digitalWrite(m1SigPin, LOW); // left, forward
    digitalWrite(m2SigPin, LOW); // right, backward

    desiredRPM = convertMPSToRPM(0.1);
    
    while(pulseCount[0] < mPulses || pulseCount[1] < mPulses)
      holdRPM(desiredRPM);
  
    stopMoving();
  }
}

double convertDegToPulses(double deg)
{
  int maxP,
    minP,
    pRange,
    ratioPulsesPerDeg,
    pulses;
    
  maxP = getMaxTurnValue();
  minP = getMinTurnValue();

  pRange = abs(maxP) + abs(minP);

  ratioPulsesPerDeg = pRange / 180;

  pulses = (int) (deg * ratioPulsesPerDeg);

  return pulses;
}

void driveToPosition()
{
  double dPulses,
    desiredRPM;

  long mPulses;
  
  if(abs(targetDistance) > 0.0)
  {
    displayMessage("Driving to position.");
    
    // clear encoder data buffer
    pulseCount[0] = 0;
    pulseCount[1] = 0;
  
    dPulses = convertCmToPulses(targetDistance);
    
    mPulses = (long) dPulses; // assumes wheels should spin at same speed

    Serial.print("Motor Pulses: ");
    Serial.println(mPulses);
    
    // set wheels to spin in same direction
    digitalWrite(m1SigPin, LOW); 
    digitalWrite(m2SigPin, HIGH);

    Serial.print("pulseCounts before while loop: ");
    Serial.print(pulseCount[0]);
    Serial.print(", ");
    Serial.println(pulseCount[1]);

    desiredRPM = convertMPSToRPM(0.1);
    
    while(pulseCount[0] < mPulses || pulseCount[1] < mPulses)
    {
      Serial.print("pulseCounts in while loop: ");
      Serial.print(pulseCount[0]);
      Serial.print(", ");
      Serial.println(pulseCount[1]);
      
      holdRPM(desiredRPM);
    }

    Serial.print("pulseCounts after while loop: ");
    Serial.print(pulseCount[0]);
    Serial.print(", ");
    Serial.println(pulseCount[1]);
    
    stopMoving();
  }
}

double convertCmToPulses(double cm)
{
  int maxP,
    minP,
    pRange;
    
  double wheelCircumf,
    ratioPulsesPerCm,
    pulses;

  maxP = getMaxDriveValue();
  minP = getMinDriveValue();

  pRange = abs(maxP) + abs(minP);

  wheelCircumf = piApprox * wheelDiam;

  ratioPulsesPerCm = pRange / wheelCircumf;

  pulses = cm * ratioPulsesPerCm;

  return pulses;
}

double convertMPSToRPM(double mps)
{
  return mps * 600 / piApprox * wheelDiam; //mps * ( 60 / ( piApprox * ( wheelDiam / 10 ) ) );
}

void holdRPM(double rpm)
{
  Serial.print("Desired RPM: ");
  Serial.println(rpm);
  
  currentCount[0] = pulseCount[0];
  currentCount[1] = pulseCount[1];

  currentTime = millis();

  timeDiff = currentTime - lastTime;

  Serial.print("timeDiff (ms): ");
  Serial.println(timeDiff);

  countDiff[0] = currentCount[0] - lastCount[0];
  countDiff[1] = currentCount[1] - lastCount[1];

  Serial.print("countDiffs: ");
  Serial.print(countDiff[0]);
  Serial.print(", ");
  Serial.println(countDiff[1]);

  countsPerSec[0] = (double) countDiff[0] / timeDiff * 1000.0;
  countsPerSec[1] = (double) countDiff[1] / timeDiff * 1000.0;

  Serial.print("countsPerSec: ");
  Serial.print(countsPerSec[0]);
  Serial.print(", ");
  Serial.println(countsPerSec[1]);

  rpms[0] = countsPerSec[0] / ppr * 60.0;
  rpms[1] = countsPerSec[1] / ppr * 60.0;

  Serial.print("RPMs: ");
  Serial.print(rpms[0]);
  Serial.print(", ");
  Serial.println(rpms[1]);

  err[0] = rpm - rpms[0];
  err[1] = rpm - rpms[1];

  Serial.print("Errors: ");
  Serial.print(err[0]);
  Serial.print(", ");
  Serial.println(err[1]);

  analogWrite(m1EnablePin, constrain(err[0], 0, 255));
  analogWrite(m2EnablePin, constrain(err[1], 0, 255));

  lastTime = currentTime;

  lastCount[0] = currentCount[0];
  lastCount[1] = currentCount[1];
}

void resetCoordinates()
{
  xPos = 0.0;
  yPos = 0.0;
  theta = 0.0;
}

//======Interrupt Service Routines======

void updateEs1Vel()
{
  pulseCount[0]++;

  totPulses[0]++;
}

void updateEs2Vel()
{
  pulseCount[1]++;

  totPulses[1]++;
}
