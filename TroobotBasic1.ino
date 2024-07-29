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

//======Advisor======
enum state { moving, resting };

state curState; // track current state

boolean motorStart = false;

//======Interface======
String inputString = "";

boolean stringComplete = false;  // whether the string is complete

//======Encoders======
const byte esPins[] = 
{
  2, // encoder signal 1 pin
  3 // encoder signal 2 pin
};
  
int pulsesPerRev = 20, // pulses per revolution
  dppr = 0, // drive pulses per revolution
  tppr = 0, // turn pulses per revolution
  dVelLimit = 0, // drive velocity limit
  tVelLimit = 0; // turn velocity limit
  
const int numEncoders = 2;

volatile int velPulseCounts[numEncoders],
  rotPos[numEncoders],
  prevRotPos[numEncoders],
  rotVels[numEncoders],
  prevRotVels[numEncoders];

volatile long pulseCounts[numEncoders],
  prevPulseCounts[numEncoders];
  
volatile long pulseCount[] = {0,0}; 
volatile long totPulses[] = {0,0};  

long currentCount[] = {0,0},
  lastCount[] = {0,0},
  countDiff[] = {0,0}; 

static unsigned long lastTime = 0;

long currentTime,
  timeDiff;

double dppcm = 0.0, // drive pulses per cm
  tppcm = 0.0, // turn pulses per cm
  minAngularRes; // [deg/pulse]

double countsPerSec[] = {0.0,0.0},
  err[] = {0.0,0.0},
  rpms[] = {0.0,0.0};

//======Motor Driver======
const byte mSigPins[] = { 4, 7 };
const byte mEnablePins[] = { 5, 6 };

//======Mobile Platform======
double wheelDiam = 6.35,
  wheelBase = 18.0;

//======Circle======
double piApprox = 3.14159,
  degsPerRad = 57.2958; // radians to deg conversion

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
int setRotVel, // [deg/s]
  maxOutVal,
  pubVelRate, // [Hz]
  pubVelTmrCtr,
  topVel, // global, top velocity set by Learner
  botVel; // global, current requested robot velocity

const int numMtrs = 2;

double setTanVel, // [m/s]
  kp, ki, kd;

volatile boolean forward;

volatile int pubMtrCmds[numMtrs],
  signs[numMtrs],
  motorOutAccums[numMtrs],
  P[numMtrs], I[numMtrs], D[numMtrs],
  setVels[numMtrs];
  
//======Stats======
unsigned long loopCount;

//======Run Once======
void setup() 
{
  initNode("TroobotBasic1");

  startProcess(motorDriver()); // control motor(s) speed
  startProcess(cruise()); // accelerate straight ahead
  startProcess(arbitrate()); // send highest priority to motors

  initVars();

  setParams();

  initSubscribers();

  initPublishers();

  /* "spin()" */
  curState = resting;

  prevPulseCounts[0] = pulseCounts[0];
  prevPulseCounts[1] = pulseCounts[1];
  
  prevRotPos[0] = rotPos[0];
  prevRotPos[1] = rotPos[1];

  prevRotVels[0] = rotVels[0]; // [pulses/(1/pubVelRate)s]
  prevRotVels[1] = rotVels[1];
}

void initNode(String id)
{
  Serial.begin(9600);

  while(!Serial);
  
  Serial.print("Starting ");
  Serial.print(id);
  Serial.println(".ino\n");
}

void initVars()
{
  setVelocities("tan", 0.0);

  setTargetLocation(0, 0);

  forward = true;
  
  pubMtrCmds[0] = 0;
  pubMtrCmds[1] = 0;

  signs[0] = 1;
  signs[1] = 1;

  pulseCounts[0] = 0;
  pulseCounts[1] = 0;

  rotPos[0] = 0;
  rotPos[1] = 0;

  rotVels[0] = 0; // [pulses/(1/pubVelRate)s]
  rotVels[1] = 0;
}

void setParams()
{
  setPIDGains( 1.0, 0.0, 0.0 );

  maxOutVal = 25600; // 100 * 256; // max. output value in fixed point integer

  pubVelRate = 10; // [Hz]

  minAngularRes = 360.0 / pulsesPerRev;
  Serial.print("Min. Ang. Res. (deg): ");
  Serial.println(minAngularRes);
  Serial.println();

  computePulsesPerCm();
}

void setPIDGains(double pg, double ig, double dg)
{
  if(pg < 0 || ig < 0 || dg < 0) return;
  
  kp = pg;

  ki = ig;

  kd = dg;

  Serial.print("Controller got kp:");
  Serial.print(kp, 3);
  Serial.print(", ki:");
  Serial.print(ki, 3);
  Serial.print(", kd:");
  Serial.println(kd, 3);
  Serial.println();
}

void initSubscribers()
{
  // pulse count
  attachInterrupt(digitalPinToInterrupt(esPins[0]), encoder1Callback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(esPins[1]), encoder2Callback, CHANGE);
}

void initPublishers()
{
  /* Start Motor Channel 1 */
  pinMode(mEnablePins[0], OUTPUT);
  pinMode(mSigPins[0], OUTPUT);

  /* Start Motor Channel 2 */
  pinMode(mEnablePins[1], OUTPUT);
  pinMode(mSigPins[1], OUTPUT);

  initPubVelTimer(); // ADD: publish rotational (and later translational) rotVel
}

void initPubVelTimer()
{
  noInterrupts();           // disable all interrupts
  
  TCCR1A = 0;
  TCCR1B = 0;
  pubVelTmrCtr = 59286;   // preload timer 65536-16MHz/256/2Hz (34286 for 0.5sec) (59286 for 0.1sec)
  
  TCNT1 = pubVelTmrCtr;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  
  interrupts();             // enable all interrupts
}

void stopMoving()
{
  motorStart = false;
  
  digitalWrite(mEnablePins[0], LOW);
  digitalWrite(mEnablePins[1], LOW);
}

void computePulsesPerCm()
{
  computePulsesPerRev();
  
  dppcm = (double) dppr / (piApprox * wheelDiam); // [pulses/cm] = (x [pulses/rev])/(\pi*d [cm])
  tppcm = (double) tppr / (piApprox * wheelDiam); // [pulses/cm] = (x [pulses/rev])/(\pi*d [cm])

  Serial.print("dppcm: ");
  Serial.println(dppcm);
  Serial.print("tppcm: ");
  Serial.println(tppcm);
  Serial.println();
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
  return pulsesPerRev / 2;
}

int getMinDriveValue()
{
  return -pulsesPerRev / 2;
}

int getMaxTurnValue()
{
  return pulsesPerRev;
}

int getMinTurnValue()
{
  return -pulsesPerRev;
}

//======Run Until Unable to Continue======
void loop() 
{
  checkUserInput();

  // State transitions
  switch(curState)
  {
    case resting: // hold position
      if(motorStart)
        curState = moving;
      break;

    case moving: // turn CW
      if(!motorStart)
        curState = resting;
      break;
  }

  // State outputs
  switch(curState)
  {
    case resting: // hold position
      stopMoving(); // CHANGE: hold position, so return to position where rest started if moved
      break;

    case moving: // move to position
      moveToPosition();
      break;
  }

  if(Serial.available())
    serialEvent();
}

void checkUserInput()
{
  if(stringComplete)
  {
    Serial.print("inputString: ");

    // receive command from user
    if(inputString.substring(0,5) == "start")
    {
      Serial.println("start");

      setTargetLocation(0, 1);
        
      forward = true;

      setVelocities("tan", 0.0);
      
      motorStart = true;
    }
    else if(inputString.substring(0,4) == "stop")
    {
      Serial.println("stop");
      
      stopMoving();
  
      motorStart = false;
    }
    else if(inputString.substring(0,10) == "settanvel ")
      setVelocities("tan", inputString.substring(10, inputString.length()).toFloat()); // get string after 'settanvel '
    else if(inputString.substring(0,10) == "setrotvel ")
      setVelocities("rot", inputString.substring(10, inputString.length()).toFloat()); // get string after 'setrotvel '
    else if(inputString.substring(0,3) == "kp ")
      kp = inputString.substring(3, inputString.length()).toFloat(); // get string after 'kp '
    else if(inputString.substring(0,3) == "ki ")
      ki = inputString.substring(3, inputString.length()).toFloat(); // get string after 'ki '
    else if(inputString.substring(0,3) == "kd ")
      kd = inputString.substring(3, inputString.length()).toFloat(); // get string after 'kd '
  
    Serial.print("motorStart: ");
    Serial.println(motorStart);

    // clear string:
    inputString = ""; //note: in code below, inputString will not become blank, inputString is blank until '\n' is received

    stringComplete = false;
  }
}

void setVelocities(String mode, float vel)
{
  if(mode == "tan")
  {
    setTanVel = vel;
    setRotVel = ( 200 * vel / wheelDiam ) * degsPerRad; // ( setTanVel * degsPerRad ) / ( ( wheelDiam / 100 ) / 2 )
    Serial.print("setRotVel (deg/s): ");
    Serial.println(setRotVel);
  }
  else if(mode == "rot")
  {
    setRotVel = vel;
    setTanVel = degsPerRad * ( wheelDiam * vel / 200 ); // [m/s]
    Serial.print("setTanVel (m/s): ");
    Serial.println(setTanVel);
  }
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

void moveToPosition()
{
  computeCurrentPose();
  locateTarget();

  //turnToPosition();
  driveToPosition();

  // Ensure robot is not moving before resetting coordinates(?)
  if(digitalRead(mEnablePins[0]) == HIGH 
    || digitalRead(mEnablePins[1]) == HIGH)
    stopMoving();

  resetCoordinates();

  cruiseOutputFlag = true; // CHANGE: incorporate stop behavior
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
  yPos += dCm * cos(theta / degsPerRad);
  xPos += dCm * sin(theta / degsPerRad);

  displayPosition();
}

double getDrivePosition()
{
  return ( pulseCounts[0] + pulseCounts[1] ) / 2.0;
}

double getTurnPosition()
{
  return (double) pulseCounts[0] - pulseCounts[1];
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
    targetBearing = 90.0 - (atan(y / x) * degsPerRad);
    
  else if (x < -0.00001)
    targetBearing = -90.0 - (atan(y / x) * degsPerRad);
    
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

//void turnToPosition()
//{
//  long mPulses;
//  
//  double tPulses,
//    desiredRPM;
//    
//  if(abs(headingError) > 0.0)
//  {
//    displayMessage("Turning to position.");
//    
//    // clear encoder data buffer
//    pulseCount[0] = 0; 
//    pulseCount[1] = 0;
//  
//    tPulses = convertDegToPulses(headingError);
//    
//    mPulses = (long) ( tPulses / 2.0 ); // assumes wheels should spin at same speed
//
//    // CHANGE: turn direction based on which way headingError indicates is faster
//    // set wheels to spin in different directions
//    digitalWrite(m1SigPin, LOW); // left, forward
//    digitalWrite(m2SigPin, LOW); // right, backward
//
//    desiredRPM = convertMPSToRPM(0.1);
//    
//    while(pulseCount[0] < mPulses || pulseCount[1] < mPulses)
//      holdRPM(desiredRPM);
//  
//    stopMoving();
//  }
//}
//
//double convertDegToPulses(double deg)
//{
//  int maxP,
//    minP,
//    pRange,
//    ratioPulsesPerDeg,
//    pulses;
//    
//  maxP = getMaxTurnValue();
//  minP = getMinTurnValue();
//
//  pRange = abs(maxP) + abs(minP);
//
//  ratioPulsesPerDeg = pRange / 180;
//
//  pulses = (int) (deg * ratioPulsesPerDeg);
//
//  return pulses;
//}

void driveToPosition()
{
  int dPulses;
  long mPulses[2];
  
  if(abs(targetDistance) > 0.0)
  { 
    dPulses = convertCmToPulses(targetDistance);
    
    mPulses[0] = pulseCounts[0] + dPulses; // assumes wheels should spin at same speed
    mPulses[1] = pulseCounts[1] + dPulses; // assumes wheels should spin at same speed

    Serial.print("Motor Pulses: ");
    Serial.print(mPulses[0]);
    Serial.print(", ");
    Serial.println(mPulses[1]);

    Serial.print("pulseCounts before while loop: ");
    Serial.print(pulseCounts[0]);
    Serial.print(", ");
    Serial.println(pulseCounts[1]);

    setVelocities("tan", 0.2);
    
    while(pulseCounts[0] < mPulses[0] 
      || pulseCounts[1] < mPulses[1]);
  }

  stopMoving();
}

int convertCmToPulses(double cm)
{
  int maxP,
    minP,
    pRange,
    pulses;
    
  double wheelCircumf,
    ratioPulsesPerCm;

  maxP = getMaxDriveValue();
  minP = getMinDriveValue();

  pRange = abs(maxP) + abs(minP);

  wheelCircumf = piApprox * wheelDiam;

  ratioPulsesPerCm = pRange / wheelCircumf;

  pulses = (int) round( cm * ratioPulsesPerCm );

  return pulses;
}

double convertMPSToRPM(double mps)
{
  return mps * 600 / piApprox * wheelDiam; //mps * ( 60 / ( piApprox * ( wheelDiam / 10 ) ) );
}

void resetCoordinates()
{
  xPos = 0.0;
  yPos = 0.0;
  theta = 0.0;
}

void serialEvent()
{
  while (Serial.available()) 
  {
    // get the new byte:
    char inChar = (char) Serial.read();
    
    // add it to the inputString:
    inputString += inChar;
    
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n')
      stringComplete = true;
  }
}

//======Interrupt Service Routines======
void encoder1Callback()
{
  determinePulseCount(1); // increment if forward, decrement if backward // OLD: pulseCount++;
    
  rotPos[0] = (int) round( (double) pulseCounts[0] * minAngularRes ); // [deg]
    
  prevPulseCounts[0] = pulseCounts[0];
}

void encoder2Callback()
{
  determinePulseCount(2); // increment if forward, decrement if backward // OLD: pulseCount++;
    
  rotPos[1] = (int) round( (double) pulseCounts[1] * minAngularRes ); // [deg]
    
  prevPulseCounts[1] = pulseCounts[1];
}

/* The variable velPulseCount is read and zeroed by the
 * "ISR(TIMER1_OVF_vect)" routine, 
 * running 10 (change to maybe 20) times a second, which is
 * the sensor loop and update rate for this robot.
 * This routine copies the accumulated counts from
 * velPulseCount to the variable "rotVel" and
 * then resets velPulseCount to zero.
 */
void determinePulseCount(int eid)
{
  if(signs[eid] == 1)
  {
    pulseCounts[eid - 1]++;
    velPulseCounts[eid - 1]++;
  }
  else
  {
    pulseCounts[eid - 1]--;
    velPulseCounts[eid - 1]--;
  }

  Serial.print("\n\nM");
  Serial.print(eid);
  Serial.print(": Pulse Count = ");
  Serial.print(pulseCounts[eid - 1]);
  Serial.print(", Vel. Pulse Count = ");
  Serial.print(velPulseCounts[eid - 1]);
  Serial.println("\n");
}

/* Run at x (maybe 10-20) Hz in sensor loop, 
 * interrupt service routine - tick every 0.1 s (10 Hz)
 * Read and zero velPulseCount.
 * Copy and accumulate counts from velPulseCount
 * to the variable "rotVel" and
 * then reset velPulseCount to zero.
 */
ISR(TIMER1_OVF_vect) // sensors or cruise or speedometer or other(?)
{
  int outputs[numMtrs];
  
  TCNT1 = pubVelTmrCtr; // set timer

  // Read analog input (i.e. calc rot vel):
  rotVels[0] = velPulseCounts[0] * signs[0]; // copy and accumulate counts from velPulseCount to rotVel
  velPulseCounts[0] = 0; // reset velPulseCount to zero
  
  rotVels[1] = velPulseCounts[1] * signs[1]; // copy and accumulate counts from velPulseCount to rotVel
  velPulseCounts[1] = 0; // reset velPulseCount to zero

  // Compute control signals:
  computeControlSignals(setVels); 

  // Set analog outputs:
  for(int i=0; i < 2; i++)
    outputs[i] = (int) round( motorOutAccums[i] / 256 );
  
  modulatePulseWidths(outputs); // Note: divide by 256 and earlier multiply by 256 b/c earlier operation in fixed point integer

  // Update controller variables (happens async?)
}

void calcMtrCmd() // (?)
{
  botVel = setVel;
  
  // Compute control signals:
  setVels[0] = botVel + rotation; // // left motor = velocity + rotation, (int) round( 1000 * setRotVel / ( minAngularRes * pubVelRate ) ); // convert [deg/s] to [pulses/(1/pubVelRate)s]
  setVels[0] = clip(setVels[0], 100, -100); // don't overflow +/- 100% full speed
  
  setVels[1] = botVel - rotation; // right motor = velocity - rotation
  setVels[1] = clip(setVels[1], 100, -100); // don't overflow +/- 100% full speed
}

/* Basic behavior: generate error signal 
 * based on difference b/t measured rotVel and
 * requested rotVel for the wheel.
 * Use of error signal: Increase or decrease speed of motor
 * to force measured to equal requested rotVel
 * Input to PID controller: Requested rotVel "vel,"
 * which is input rotVel expressed as encoder pulses
 * per 1/pubVelRate second.
 */
void computeControlSignals(int setPoints[])
{
  int errs[2];
  
  int b = 1; // set point weight
  
  Serial.print("Controller got kp:");
  Serial.print(kp);
  Serial.print(", ki:");
  Serial.print(ki);
  Serial.print(", kd:");
  Serial.println(kd, 3);
  Serial.println();

  if(motorStart)
  {
    for(int i=0; i < 2; i++)
    {
      Serial.print("setPoints[");
      Serial.print(i);
      Serial.print("]: ");
      Serial.print(setPoints[i]);
      Serial.print(", rotVels[");
      Serial.print(i);
      Serial.print("]: ");
      Serial.print(rotVels[i]);
    
      errs[i] = (int) round( ( b * setPoints[i] / 1000.0 - rotVels[i] ) * 256 ); // [pulses/(1/pubVelRate)s]*256, generate error signal based on difference b/t measured rotVel and requested rotVel for the wheel. Note: multiply by 256 and later divide by 256 b/c operation in fixed point integer
      Serial.print(", errs[");
      Serial.print(i);
      Serial.print("] (pulses per 25.6 sec): ");
      Serial.println(errs[i]);
      
      P[i] = (int) round( errs[i] / kp ); // P(t_k) = K(by_{sp}(t_k) — y(t_k))
      Serial.print("P[");
      Serial.print(i);
      Serial.print("]: ");
      Serial.println(P[i]);
  
      D[i] = (int) round( ( ( rotVels[i] - prevRotVels[i] ) * 256 ) / kd ); // large when amount of change requested by PID controller is large, and small as signal approaches zero
      Serial.print("D[");
      Serial.print(i);
      Serial.print("]: ");
      Serial.println(D[i]);
  
      Serial.print("motorOutAccum before change: ");
      Serial.println(motorOutAccums[i]);
      motorOutAccums[i] += P[i] + D[i]; // Increase or decrease speed of motor to force measured to equal requested rotVel
  
      prevRotVels[i] = rotVels[i]; // maintain history of previous measured rotVel
  
      motorOutAccums[i] = clip(motorOutAccums[i], maxOutVal, -maxOutVal); // accumulator
    }
  }
  else
  {
    for(int i=0; i < 2; i++)
    {
      errs[i] = 0;

      motorOutAccums[i]= 0;
  
      prevRotVels[i] = 0; // maintain history of previous measured rotVel
    }
  }

  if(setRotVel == 0.0)
  {
    for(int i=0; i < 2; i++)
      motorOutAccums[i] = 0;  
  }
}

/* The accumulator motorOutAccum must be clipped 
 * at some positive and negative value
 * to keep from overflowing the fixed point arithmetic.
 */
int clip(int a, int maximum, int minimum)
{
  Serial.print("Computed val: ");
  Serial.print(a);
    
  if(a > maximum) 
    a = maximum;
  else if(a < minimum) 
    a = minimum;

  Serial.print(", Clipped val: ");
  Serial.println(a);

  return a;
}

/* The PWM code drives the hardware H-bridge, 
 * which actually control the motor.
 * This routine takes a signed value, 
 * -100 < signedVal < 100,
 * sets the sign variable used by the speedometer code,
 * sets the forward/backward (i.e. direct/reverse) bits 
 * on the H-bridge, and
 * uses abs(signedVal) as an index into a 100 entry table 
 * of linear PWM values.
 * This function uses a timer interrupt to generate 
 * a x Hz (maybe 120 Hz) variable pulse-width output.
 */
void modulatePulseWidths(int signedVals[]) // take signed value, b/t -100 and 100
{
  int i;
  
  for(i=0; i < 2; i++)
  {
    setSpeedometerSign(i, signedVals[i]); // set sign variable used by speedometer code

    setHBridgeDirectionBit(i, signedVals[i]);
  
    setPWMValueFromEntryTable(i, abs(signedVals[i])); // use abs(signedVal) as an index into a 100 entry table of linear PWM values
  }

  arbitrate();
  
  for(i=0; i < 2; i++)
    analogWrite(mEnablePins[i], motorInputs[i]); // generate variable pulse-width output
}

/* "winnerId" feedback line 
 * from Arbitrate back to tasks:
 * Essentially global var containing ID of task 
 * that "won" this round of arbitration.
 * It can be used by the individual tasks
 * to determine if they have been subsumed.
 */
void arbitrate()
{
  if(cruiseOutputFlag == 1)
  {
    motorInputs = pubMtrCmds;

    winnerId = cruiseId; // use by cruise task to determine if it has been subsumed
  }
}
/* The sign variable represents the direction of rotation
 * of the motor (1 for forward and -1 for backward).
 * With more expensive quadrature encoders this info is
 * read directly from the encoders.
 * In this implementation I have only simple encoders so
 * the direction of rotation is taken from the sign of the
 * most recent command issued by the PID to the PWM code.
 */
void setSpeedometerSign(int mid, int signedVal) // signedVal should be most recent cmd issued by PID to PWM code
{
  if(signedVal < 0) // {motor direction of rotation} = backward
    signs[mid] = -1;
  else if(signedVal >= 0)
    signs[mid] = 1; // {motor direction of rotation} = {forward | resting}
  else
    Serial.println("Invalid command issued by PID to PWM code!\n");
}

void setHBridgeDirectionBit(int mid, int signedVal)
{
  if(signedVal < 0) // {motor direction of rotation} = backward
  { 
    if(mid == 0) digitalWrite(mSigPins[mid], HIGH);
    else if(mid == 1) digitalWrite(mSigPins[mid], LOW);
  }
  else if(signedVal >= 0) // {motor direction of rotation} = {forward | resting}
  { 
    if(mid == 0) digitalWrite(mSigPins[mid], LOW);
    else if(mid == 1) digitalWrite(mSigPins[mid], HIGH);
  }
  else
    Serial.println("Invalid command issued by PID to PWM code!\n");
}

// use magnitude as an index into a 100 entry table of linear PWM values
void setPWMValueFromEntryTable(int mid, int magnitude)
{
  pubMtrCmds[mid] = (int) round( magnitude * 255 / 100 ); // cruise outputs
}
