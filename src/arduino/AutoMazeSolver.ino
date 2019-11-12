/********************************************************************************************************
*  Author:      Dinca Marius Catalin                                                                    *
*  Board:       Arduino UNO R3                                                                          *
*  Platform:    Windows 10                                                                              *
*  Year:        2019                                                                                    *
*  Project:     Licenta - Proiectarea unui robot autonom cu posibilități de recunoaștere a traseului    *
*  Setup:                                                                                               * 
*    # Driver motoare pinout:                                                                           * 
*   - input1(A) --> pin 11 (galben(in) verde(out))                                                      *
*   - input2(A) --> pin 13 (galben(in) verde(out))                                                      *  
*   - Enable Dreapta(A)--> pin 12 (verde)                                                               * 
*   - input3(B) --> pin 10 (verde (in) albastru(out))                                                   *
*   - input4(B) --> pin 12 (verde (in) albastru(out))                                                   *
*   - Enable Stanga(B)  --> pin 13 (albastru)                                                           *
*   # Senzor array 1,2,3,4,5,6,7,8 pinout                                                               *
*   - pini arduino 9,8,7,6,5,4,3,2                                                                      *
*  Descriere:                                                                                           *
*   Prin codul de mai jos un sistem informatic poate parcuge un traseu, delimitat printr-o linie        *
* neagra, de două ori. Prima parcurgere o reprezintă ”învățarea traseului”, iar cea de-a doua           *
* va fi parcurgerea optimă a aceluiași traseu.                                                          *
********************************************************************************************************/


#include <QTRSensors.h> //Pololu QTR Sensor Library. First you must download and install QTRSensors library
#include <Arduino.h>

int enable_left = A0;  //enable A on analog pin A0  (Left motor - roata stnga)
int enable_right = A1;  // enable B on analog pin A1  (Right motor - roata dreapta)

int pwm_leftFwd = 10;  //viteza motor stanga  (Left motor - mers inainte)
int pwm_leftBack = 12;  //viteza motor stanga  (Left motor - mers inapoi)

int pwm_rightFwd = 11;  //viteza motor dreapta (Right motor - mers inainte)
int pwm_rightBack = 13;  //viteza motor dreapta  (Right motor - mers inapoi)

#define minim_speed 0  //minimum speed of the motors
#define maxim_speed 255 //max. speed of the motors

#define MIDDLE_SENSOR 5       //number of middle sensor used
#define NUM_SENSORS 8         //number of sensors used
#define TIMEOUT 2500          //waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN QTR_NO_EMITTER_PIN         //emitterPin is the Arduino digital pin that controls whether the IR LEDs are on or off. Emitter is controlled by digital pin 2
#define DEBUG 0

QTRSensorsRC qtrrc((unsigned char[]) {
  9, 8, 7, 6, 5, 4, 3, 2
} , NUM_SENSORS, TIMEOUT, EMITTER_PIN);

enum ESensorResult
{
  Found_Line          = 0,
  Found_Straight      = 1,
  Found_TurnLeft      = 2,
  Found_TurnRight     = 3,
  Found_DeadEnd       = 4,
  Found_Intersection  = 5,
  Found_Finish        = 6,
};

enum ERobotAction
{
  Action_FollowLine   = 0,
  Action_TurnLeft     = 1,
  Action_TurnRight    = 2,
  Action_TurnAround   = 3,
  Action_GoStraight   = 4, // This means to pass intersection that has only a right turn
  Action_FoundFinish  = 5,
};

static const char s_actionFollowLine = 'x'; 
static const char s_actionStraight   = 'S'; 
static const char s_actionTurnLeft   = 'L'; 
static const char s_actionTurnRight  = 'R'; 
static const char s_actionTurnAround = 'B'; 
static const char s_actionFinish     = 'F'; 

static const int s_minSpeed = 0;
static const int s_maxSpeed = 200;
static const int s_followLineSpeed = 130;
static const int s_turnSpeed = 200;
static const int s_finishCheckTimeMs = 1000;

const int g_maxRobotPath = 128;
char g_robotPath[g_maxRobotPath] = {0};
int g_pathLength = 0;

static bool g_hasFoundFinishWhileTurning = false;

void setup()
{
  Serial.begin(9600);
  Serial.println ("setup() start calibration...");
  delay(1000);
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  manual_calibration();
  set_motors(0, 0);
  Serial.println ("setup() finished calibration!");
  delay(5000);
}

//calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead
void manual_calibration() 
{
  for (int i = 0; i < 250; i++)
  {
    qtrrc.calibrate();
    delay(20);
  }

  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
}

void set_motors(int left_motorSpeed, int right_motorSpeed)
{
  //set left motor speed
  if (left_motorSpeed > 0)
  {
    if (left_motorSpeed > maxim_speed ) left_motorSpeed = maxim_speed;
    if (left_motorSpeed < minim_speed ) left_motorSpeed = minim_speed;
    digitalWrite(enable_left, HIGH);
    analogWrite(pwm_leftFwd, left_motorSpeed);
    analogWrite(pwm_leftBack, 0);
  }
  else if (left_motorSpeed < 0) {
    if (left_motorSpeed < -(maxim_speed) ) left_motorSpeed = -(maxim_speed);
    digitalWrite(enable_left, HIGH);
    analogWrite(pwm_leftBack, 0);
    analogWrite(pwm_leftFwd, minim_speed);
  }
  if (left_motorSpeed == 0 )
  {
    digitalWrite(enable_left, LOW);
    analogWrite(pwm_leftFwd, 0);
    analogWrite(pwm_leftBack, 0);
  }

  //set right motor speed
  if (right_motorSpeed > 0)
  {
    if (right_motorSpeed > maxim_speed ) right_motorSpeed = maxim_speed;
    if (right_motorSpeed < minim_speed ) right_motorSpeed = minim_speed;
    digitalWrite(enable_right, HIGH);
    analogWrite(pwm_rightFwd, right_motorSpeed);
    analogWrite(pwm_rightBack, 0);

  }
  else if (right_motorSpeed < 0) {
    if (right_motorSpeed < -(maxim_speed) ) right_motorSpeed = -(maxim_speed);
    digitalWrite(enable_right, HIGH);
    analogWrite(pwm_rightBack, 0);
    analogWrite(pwm_rightFwd, minim_speed);
  }
  if (right_motorSpeed == 0 )
  {
    digitalWrite(enable_right, LOW);
    analogWrite(pwm_rightFwd, 0);
    analogWrite(pwm_rightBack, 0);
  }
}

char convertRobotActionToChar(ERobotAction action)
{
  switch(action)
  {
    case Action_FollowLine:
      return s_actionFollowLine;
    case Action_GoStraight:
      return s_actionStraight;
    case Action_TurnLeft:
      return s_actionTurnLeft;
    case Action_TurnRight:
      return s_actionTurnRight;
    case Action_TurnAround:
      return s_actionTurnAround;
  }
  return s_actionFinish;
}

void clampValue(int& value, int minValue, int maxValue)
{
  if (value > maxValue)
  {
    value = maxValue;
  }
  else if (value < minValue)
  {
    value = minValue;
  }
}

bool isSensorOnBlack(unsigned int value)
{
  return (value > 150);
}

bool isSensorOnWhite(unsigned int value)
{
  return (value <= 150);
}

bool areAllSensorsWhite(unsigned int s[NUM_SENSORS])
{
  bool allWhite = true;
  for(int i = 0; i < NUM_SENSORS; ++i)
  {
    if(isSensorOnBlack(s[i]))
    {
      allWhite = false;
      break;
    }
  }
  return allWhite;
}

bool areAllSensorsBlack(unsigned int s[NUM_SENSORS])
{
  bool allBlack = true;
  for(int i = 0; i < NUM_SENSORS; ++i)
  {
    if(isSensorOnWhite(s[i]))
    {
      allBlack = false;
      break;
    }
  }
  return allBlack;
}

bool hasTimedOut(unsigned long startTimeMs, unsigned timeoutMs)
{
  unsigned long currentTimeMs = millis();
  if((startTimeMs + timeoutMs) < currentTimeMs)
  {
    return true;
  }
  return false;
}

bool hasReachedFinishLine(unsigned long& allBlackStartTimeMs, 
  unsigned int sensorValues[NUM_SENSORS])
{
  if(g_hasFoundFinishWhileTurning)
  {
    return true;
  }

  bool isAtFinishLine = false;
  if(areAllSensorsBlack(sensorValues))
  {
    if(allBlackStartTimeMs == 0)
    {
      allBlackStartTimeMs = millis();
    }

    if(hasTimedOut(allBlackStartTimeMs, s_finishCheckTimeMs))
    {
      Serial.println("hasReachedFinishLine() found finish line!");
      g_hasFoundFinishWhileTurning = true;
      isAtFinishLine = true;
    }
  }
  else
  {
    allBlackStartTimeMs = 0;
  }
  return isAtFinishLine;
}


void turnAround()
{
  unsigned int sensorValues[8] = {0};
  int linePos = qtrrc.readLine(sensorValues);
  
  digitalWrite(enable_left, HIGH);
  digitalWrite(enable_right, HIGH);
  
  analogWrite(pwm_leftFwd, 0);
  analogWrite(pwm_rightBack, 0);
  
  // find center
  while (linePos < 1000 || linePos > 6000 || sensorValues[3] < 150)  // tune - wait for line position to find near center
  {      
    analogWrite(pwm_leftBack, 150);
    analogWrite(pwm_rightFwd, 250);
    delay(40);
    analogWrite(pwm_leftBack, 0);
    analogWrite(pwm_rightFwd, 0);
    delay(100);
    linePos = qtrrc.readLine(sensorValues);
  }
  
  analogWrite(pwm_leftFwd, 0);
  analogWrite(pwm_leftBack, 0);
  analogWrite(pwm_rightFwd, 0);
  analogWrite(pwm_rightBack, 0);
  Serial.println("turnAround() finished turning around");
}

void turnLeft()
{
  Serial.println("turnLeft()");

  // Stop the engines and wait  for the inertia to settle
  digitalWrite(enable_right, HIGH); 
  analogWrite(pwm_rightFwd, 0);
  analogWrite(pwm_rightBack, 0);
  digitalWrite(enable_left, HIGH);
  analogWrite(pwm_leftFwd, 0);
  analogWrite(pwm_leftBack, 0);
  delay(200);
  
  unsigned long allBlackStartTimeMs = 0;

  unsigned int sensorValues[NUM_SENSORS] = {0};
  int linePos = qtrrc.readLine(sensorValues);
  
  // Turn until we leave the current line
  while(true)
  {
    analogWrite(pwm_rightFwd, s_turnSpeed);
    delay(50);
    analogWrite(pwm_rightFwd, 0);
    delay(50);
  
    linePos = qtrrc.readLine(sensorValues);

    // Turn until we leave linePos center
    if((linePos <= 2000) || (linePos >= 7000))
    {
      Serial.print("turnLeft() left line, linePos ");
      Serial.println(linePos);
      break;
    }

    // Or turn until all s[i] are white
    bool areAllWhite = true;
    for(int i = 0; i < NUM_SENSORS; ++i)
    {
      if(isSensorOnBlack(sensorValues[i]))
      {
        areAllWhite = false;
        break;
      }
    }
    if(areAllWhite)
    {
      Serial.println("turnLeft() left line, all white");
      break;
    }

    if(hasReachedFinishLine(allBlackStartTimeMs, sensorValues))
    {
      Serial.println("turnLeft() abort, we are at finish");
      return;
    }
  }
  
  // Keep turning until we find the new line center
  Serial.println("turnLeft() waiting for new line...");
  while (isSensorOnBlack(sensorValues[0]) ||
      (isSensorOnWhite(sensorValues[3]) && isSensorOnWhite(sensorValues[4])))
  {
    analogWrite(pwm_rightFwd, s_turnSpeed);
    delay(50);
    analogWrite(pwm_rightFwd, 0);
    delay(50);
   
    linePos = qtrrc.readLine(sensorValues);
    /*Serial.print("turnLeft() linePos: ");
    Serial.print(linePos);
    Serial.print(", sensorValues[3]: ");
    Serial.print(sensorValues[3]);
    Serial.print(", sensorValues[4]: ");
    Serial.println(sensorValues[4]); */
  }
  Serial.println("turnLeft() finished!");
}

void turnRight()
{
  Serial.println("turnRight()");

  // Stop the engines and wait  for the inertia to settle
  digitalWrite(enable_right, HIGH); 
  analogWrite(pwm_rightFwd, 0);
  analogWrite(pwm_rightBack, 0);
  digitalWrite(enable_left, HIGH);
  analogWrite(pwm_leftFwd, 0);
  analogWrite(pwm_leftBack, 0);
  delay(200);
  
  unsigned long allBlackStartTimeMs = 0;

  unsigned int sensorValues[NUM_SENSORS] = {0};
  int linePos = qtrrc.readLine(sensorValues);
  
  // Turn until we leave the current line
  while(true)
  {
    analogWrite(pwm_leftFwd, s_turnSpeed);
    delay(50);
    analogWrite(pwm_leftFwd, 0);
    delay(50);
  
    linePos = qtrrc.readLine(sensorValues);

    // Turn until we leave linePos center
    if((linePos <= 0) || (linePos >= 5000))
    {
      Serial.print("turnRight() left line, linePos ");
      Serial.println(linePos);
      break;
    }

    // Or turn until all s[i] are white
    bool areAllWhite = true;
    for(int i = 0; i < NUM_SENSORS; ++i)
    {
      if(isSensorOnBlack(sensorValues[i]))
      {
        areAllWhite = false;
        break;
      }
    }
    if(areAllWhite)
    {
      Serial.println("turnRight() left line, all white");
      break;
    }

    if(hasReachedFinishLine(allBlackStartTimeMs, sensorValues))
    {
      Serial.println("turnRight() abort, we are at finish");
      return;
    }
  }
  
  // Keep turning until we find the new line center
  Serial.println("turnRight() waiting for new line...");
  while (isSensorOnBlack(sensorValues[7]) ||
      (isSensorOnWhite(sensorValues[3]) && isSensorOnWhite(sensorValues[4])))
  {
    analogWrite(pwm_leftFwd, s_turnSpeed);
    delay(50);
    analogWrite(pwm_leftFwd, 0);
    delay(50);
   
    linePos = qtrrc.readLine(sensorValues);
    /*Serial.print("turnLeft() linePos: ");
    Serial.print(linePos);
    Serial.print(", sensorValues[3]: ");
    Serial.print(sensorValues[3]);
    Serial.print(", sensorValues[4]: ");
    Serial.println(sensorValues[4]); */
  }
  Serial.println("turnRight() finished!");
}

ESensorResult checkForNextTurn()
{
  unsigned int sensorValues[NUM_SENSORS] = {0};
  int linePos = qtrrc.readLine(sensorValues);
  ESensorResult sensorResult = checkSensorsForTurns(linePos, sensorValues);
  if(sensorResult == Found_Intersection)
  {
    if(checkFinish())
    {
      sensorResult = Found_Finish;
    }
    else
    {
      sensorResult = Found_TurnLeft;
    }
  }
  else if(sensorResult == Found_TurnRight)
  {
    if(checkStraightIfFoundRight())
    {
      if(g_hasFoundFinishWhileTurning)
      {
        sensorResult = Found_Finish;
        g_hasFoundFinishWhileTurning = false;
      }
      else
      {
        sensorResult = Found_Straight;
      }
    }
    else
    {
      sensorResult = Found_TurnRight;
    }
  }
  return sensorResult;
}

ESensorResult checkSensorsForTurns(int linePos, unsigned int sensorValues[8])
{
  bool areAllSensorsOnWhite = true;
  for(int i = 0; i < NUM_SENSORS; ++i)
  {
    if(isSensorOnBlack(sensorValues[i]))
    {
      areAllSensorsOnWhite = false;
      break;
    }
  }
  if(areAllSensorsOnWhite)
  {
    Serial.println("checkSensorsForTurns() found dead end");
    return Found_DeadEnd;
  }

  bool isSensorOnBlackLeft = isSensorOnBlack(sensorValues[0]);
  bool isSensorOnBlackRight = isSensorOnBlack(sensorValues[7]);
  if(isSensorOnBlackLeft && isSensorOnBlackRight)
  {
    Serial.println("checkSensorsForTurns() found intersection");
    return Found_Intersection;
  }
  else if(isSensorOnBlackLeft)
  {
    Serial.println("checkSensorsForTurns() found turn left");
    return Found_TurnLeft;
  }
  else if(isSensorOnBlackRight)
  {
    Serial.println("checkSensorsForTurns() found turn right");
    return Found_TurnRight;
  }
  return Found_Line;
}

bool checkFinish()
{
  Serial.println("checkFinish()");

  digitalWrite(enable_left, HIGH);
  digitalWrite(enable_right, HIGH);
  analogWrite(pwm_leftFwd, 0);
  analogWrite(pwm_leftBack, 0);
  analogWrite(pwm_rightFwd, 0);
  analogWrite(pwm_rightBack, 0);
  delay(200);
  
  unsigned int sensorValues[NUM_SENSORS] = {0};
  int linePos = qtrrc.readLine(sensorValues);
  bool areAllBlack = areAllSensorsBlack(sensorValues);
  unsigned long checkStartTimeMs = millis();
  while(areAllBlack && !hasTimedOut(checkStartTimeMs, s_finishCheckTimeMs))
  {
    analogWrite(pwm_leftFwd, s_followLineSpeed);
    analogWrite(pwm_rightFwd, s_followLineSpeed);
    delay(50);
    analogWrite(pwm_leftFwd, 0);
    analogWrite(pwm_rightFwd, 0);
    delay(50);

    linePos = qtrrc.readLine(sensorValues);
    areAllBlack = areAllSensorsBlack(sensorValues);
  }

  bool hasFinished = areAllBlack;
  Serial.print("checkFinish() hasFinished: ");
  Serial.println(hasFinished);
  
  if(!hasFinished)
  {
    // Drive back until we find the intersection again
    while((linePos <= 1000) && (linePos >= 6000))
    {
      analogWrite(pwm_leftBack, s_followLineSpeed);
      analogWrite(pwm_rightBack, s_followLineSpeed);
      delay(50);
      analogWrite(pwm_leftBack, 0);
      analogWrite(pwm_rightBack, 0);
      delay(50);

      linePos = qtrrc.readLine(sensorValues);
    }
  }
  
  return areAllBlack;
}

bool checkStraightIfFoundRight()
{
  bool foundStraight = false;

  digitalWrite(enable_right, HIGH);
  digitalWrite(enable_left, HIGH);
  analogWrite(pwm_rightBack, 0);
  analogWrite(pwm_leftBack, 0);

  unsigned long allBlackStartTimeMs = millis();

  unsigned int sensorValues[NUM_SENSORS] = {0};
  int linePos = qtrrc.readLine(sensorValues);

  while(isSensorOnBlack(sensorValues[7]))
  {
    analogWrite(pwm_rightFwd, s_followLineSpeed);
    analogWrite(pwm_leftFwd, s_followLineSpeed);
    delay(50);
    analogWrite(pwm_rightFwd, 0);
    analogWrite(pwm_leftFwd, 0);
    delay(50);

    linePos = qtrrc.readLine(sensorValues);
    if(hasReachedFinishLine(allBlackStartTimeMs, sensorValues))
    {
      Serial.println("checkStraightIfFoundRight() abort, we are at finish");
      return false;
    }
  }

  foundStraight = ((linePos >= 1000) && (linePos <= 6000));
  if(!foundStraight)
  {
    // Back up until we find the line again
    while(areAllSensorsWhite(sensorValues))
    {
      analogWrite(pwm_rightBack, s_followLineSpeed);
      analogWrite(pwm_leftBack, s_followLineSpeed);
      delay(50);
      analogWrite(pwm_rightBack, 0);
      analogWrite(pwm_leftBack, 0);
      delay(50);

      linePos = qtrrc.readLine(sensorValues);
    }
  }
  
  Serial.print("checkStraightIfFoundRight() foundStraight: ");
  Serial.println(foundStraight);

  return foundStraight;
}

ESensorResult followPath(bool continueUntilFoundLine = false)
{
  static float s_previousError = 0.0f;
  static float s_previousSpeedFwdLeft = 0.0f;
  static float s_previousSpeedFwdRight = 0.0f;

  static unsigned long s_previousTimeMs = millis();
  const unsigned long timeIntervalMs = 10;

  unsigned long currentTimeMs = millis();
  if (currentTimeMs <= (s_previousTimeMs + timeIntervalMs))
  {
    delay(10);
  }

  s_previousTimeMs = currentTimeMs;
  
  unsigned int sensorValues[NUM_SENSORS] = {0};
  int linePos = qtrrc.readLine(sensorValues);
  ESensorResult sensorResult = checkSensorsForTurns(linePos, sensorValues);
  if(!continueUntilFoundLine && (sensorResult != Found_Line))
  {
    // Reset the previous error
    s_previousError = 0.0f;

    // Stop the engines
    {
      Serial.print("followPath() stopping");
      Serial.print(" s_previousSpeedFwdLeft: ");
      Serial.print(s_previousSpeedFwdLeft);
      Serial.print(" s_previousSpeedFwdRight: ");
      Serial.println(s_previousSpeedFwdRight);

      int stopSpeed = (s_previousSpeedFwdLeft < s_previousSpeedFwdRight) 
        ? s_previousSpeedFwdLeft 
        : s_previousSpeedFwdRight;

      analogWrite(pwm_leftFwd, 0);
      analogWrite(pwm_rightFwd, 0);
      analogWrite(pwm_leftBack, stopSpeed);
      analogWrite(pwm_rightBack, stopSpeed);
      delay(50);
      analogWrite(pwm_leftBack, 0);
      analogWrite(pwm_rightBack, 0);
      delay(200);

      s_previousSpeedFwdLeft = 0;
      s_previousSpeedFwdRight = 0;
    }
    
    // Check what type of turn we have
    if(sensorResult == Found_Intersection)
    {
      if(checkFinish())
      {
        sensorResult = Found_Finish;
      }
      else
      {
        sensorResult = Found_TurnLeft;
      }
    }
    else if(sensorResult == Found_TurnRight)
    {
      if(checkStraightIfFoundRight())
      {
        if(g_hasFoundFinishWhileTurning)
        {
          sensorResult = Found_Finish;
          g_hasFoundFinishWhileTurning = false;
        }
        else
        {
          sensorResult = Found_Straight;
        }
      }
      else
      {
        sensorResult = Found_TurnRight;
      }
    }
	
    return sensorResult;
  }

  linePos = qtrrc.readLine(sensorValues);
  float error = (linePos - 3500) / 100.0f;

  // set the motor speed based on proportional and derivative PID terms
  // kp is the a floating-point proportional constant (maybe start with a value around 0.5)
  // kd is the floating-point derivative constant (maybe start with a value around 1)
  // note that when doing PID, it's very important you get your signs right, or else the
  // control loop will be unstable
  float kp = 1.0f;
  float ki = 2.0f;
  float kd = 3.0f;

  float totalError = (error + s_previousError);
  float errorRate = (error - s_previousError);
  float pv = kp * error + ki * totalError + kd * errorRate;
  int speedChange = static_cast<int>(pv);

  clampValue(speedChange, -s_followLineSpeed, s_followLineSpeed);

  int speedLeft = s_followLineSpeed + speedChange;
  int speedRight = s_followLineSpeed - speedChange;

  clampValue(speedLeft, s_minSpeed, s_maxSpeed);
  clampValue(speedRight, s_minSpeed, s_maxSpeed);

  //set motor speeds
  digitalWrite(enable_left, HIGH);
  digitalWrite(enable_right, HIGH);

  if (speedChange == -s_followLineSpeed)
  {
    analogWrite(pwm_leftFwd, 0);
    analogWrite(pwm_leftBack, speedRight);
    s_previousSpeedFwdLeft = 0.0f;
  }
  else
  {
    analogWrite(pwm_leftFwd, speedLeft);
    analogWrite(pwm_leftBack, 0);
    s_previousSpeedFwdLeft = speedLeft;
  }

  if (speedChange == s_followLineSpeed)
  {
    analogWrite(pwm_rightFwd, 0);
    analogWrite(pwm_rightBack, speedLeft);
    s_previousSpeedFwdRight = 0;
  }
  else
  {
    analogWrite(pwm_rightFwd, speedRight);
    analogWrite(pwm_rightBack, 0);
    s_previousSpeedFwdRight = speedRight;
  }

  s_previousError = error;

  return sensorResult;
}

void driveUntilFoundLine()
{
  Serial.println("driveUntilFoundLine()");
  while(true)
  {
    ESensorResult result = followPath(/*continueUntilFoundLine*/true);
    if(result == Found_Line)
    {
      Serial.println("driveUntilFoundLine() found line, stopping");
      break;
    }
  }
}

ERobotAction doRobotAction(ESensorResult sensorResult)
{
  ERobotAction robotAction = Action_FollowLine;
  if(sensorResult == Found_Finish)
  {
    robotAction = Action_FoundFinish;
  }
  else if(sensorResult == Found_TurnLeft)
  {
    robotAction = Action_TurnLeft;
    turnLeft();
  }
  else if(sensorResult == Found_TurnRight)
  {
    robotAction = Action_TurnRight;
    turnRight();
  }
  else if(sensorResult == Found_DeadEnd)
  {
    turnAround();
    robotAction = Action_TurnAround;
  }
  else if(sensorResult == Found_Straight)
  {
    driveUntilFoundLine();
    robotAction = Action_GoStraight;
  }

  if(g_hasFoundFinishWhileTurning)
  {
    robotAction = Action_FoundFinish;
    g_hasFoundFinishWhileTurning = false;
  }
  return robotAction;
}

void addRobotAction(char action)
{
  if(g_pathLength >= g_maxRobotPath)
  {
    return;
  }
  
  if(g_pathLength == 0)
  {
    g_robotPath[g_pathLength] = action;
    ++g_pathLength;
  }
  else
  {
    char previousAction = g_robotPath[g_pathLength-1];
    if(previousAction != action)
    {
      if(action == s_actionFinish)
      {
        // Remove fake turns at the finish line.
        int lastIndex = g_pathLength - 1;
        for(int i = lastIndex; i >= 0; --i)
        {
          if(g_robotPath[i] == s_actionFollowLine)
          {
            // Stop when we found the last follow-line action
            break;
          }
          else
          {
            g_robotPath[i] = 0;
            --g_pathLength;
          }
        }
      }
      
      // Add the action to the path
      g_robotPath[g_pathLength] = action;
      ++g_pathLength;

      Serial.print("addRobotAction() ");
      Serial.println(&g_robotPath[0]); 
    }
  }
}

bool trySimplifyRobotPath()
{
  bool wasPathSimplified = false;
  if(g_pathLength >= 5)
  {
    int actionIndexLast   = g_pathLength - 1;
    int actionIndexMiddle = g_pathLength - 3;
    int actionIndexFirst  = g_pathLength - 5;
    
    const char& actionFirst = g_robotPath[actionIndexFirst];
    const char& actionLast = g_robotPath[actionIndexLast];
    if(g_robotPath[actionIndexMiddle] == s_actionTurnAround)
    {
      char simpleAction = 0;
      if(actionFirst == s_actionTurnLeft)
      {
        if(actionLast == s_actionTurnLeft)
        {
          // LxBxL -> S 
          simpleAction = s_actionStraight;
        }
        else if(actionLast == s_actionTurnRight)
        {
          // LxBxR -> B 
          simpleAction = s_actionTurnAround;
        }
        else if(actionLast == s_actionStraight)
        {
          // LxBxS -> R
          simpleAction = s_actionTurnRight;
        }
      }
      else if(actionFirst == s_actionTurnRight)
      {
        if(actionLast == s_actionTurnLeft)
        {
          // RxBxL -> B
          simpleAction = s_actionTurnAround;
        }
      }
      else if(actionFirst == s_actionStraight)
      {
        if(actionLast == s_actionTurnLeft)
        {
          // SxBxL -> R
          simpleAction = s_actionTurnRight;
        }
        else if(actionLast == s_actionStraight)
        {
          // SxBxS -> B
          simpleAction = s_actionTurnAround;
        }
      }

      if(simpleAction != 0)
      {
        Serial.print("simplifyRobotPath() oldPath: ");
        Serial.println(&g_robotPath[0]);

        // Clear the actions that can be simplified
        g_robotPath[g_pathLength - 1] = 0;
        g_robotPath[g_pathLength - 2] = 0;
        g_robotPath[g_pathLength - 3] = 0;
        g_robotPath[g_pathLength - 4] = 0;
        g_robotPath[g_pathLength - 5] = 0;
        g_pathLength -= 5;
        
        if(g_robotPath[g_pathLength-1] != s_actionFollowLine)
        {
          addRobotAction(s_actionFollowLine);
        }
        addRobotAction(simpleAction);
        
        wasPathSimplified = true;

        Serial.print("simplifyRobotPath() newPath: ");
        Serial.println(&g_robotPath[0]);
      }
    }
  }

  return wasPathSimplified;
}

void simplifyRobotPath()
{
  // Try to simplify the last robot action
  while(trySimplifyRobotPath())
  {
    // If the path was simplified, then
    // keep going and simplify recursively.
  }
}

void resetRobotPath()
{
  for(int i = 0; i < g_maxRobotPath; ++i)
  {
    g_robotPath[i] = 0;
  }
  g_pathLength = 0;
}

void solveMaze()
{
  Serial.println("solveMaze()");
  while (true)
  {
    ESensorResult sensorResult = followPath();
    ERobotAction action = doRobotAction(sensorResult);
    char actionChar = convertRobotActionToChar(action);
    addRobotAction(actionChar);
    simplifyRobotPath();
    
    if(action == Action_FoundFinish)
    {
      Serial.print("solveMaze() found finish: ");
      Serial.println(&g_robotPath[0]);
      delay(10000);
      break;
    }
  }
}

void playbackMazeSolution()
{
  //const char* mazeSolution = "xSxRxRxF";
  //const char* currentActionIt = &mazeSolution[0];

  bool foundFinish = false;

  const char* currentActionIt = &g_robotPath[0];
  for(; currentActionIt != 0; ++currentActionIt)
  {
    if(foundFinish)
    {
      break;
    }

    while(*currentActionIt == s_actionFollowLine)
    {
      ++currentActionIt;
    }
    if(*currentActionIt == 0)
    {
      break;
    }

    
    char targetAction = *currentActionIt;
    Serial.print("playbackMazeSolution() targetAction: ");
    Serial.println(targetAction);
    
    bool hasDoneTargetAction = false;
    while(!hasDoneTargetAction)
    {
      ESensorResult result = followPath();
      if((result == Found_TurnLeft) || 
         (result == Found_TurnRight) || 
         (result == Found_Straight) ||
         (result == Found_DeadEnd) || 
         (result == Found_Intersection))
      {
        if(targetAction == s_actionStraight)
        {
          driveUntilFoundLine();
          hasDoneTargetAction = true;
        }
        else if(targetAction == s_actionTurnLeft)
        {
          turnLeft();
          driveUntilFoundLine();
          hasDoneTargetAction = true;
        }
        else if(targetAction == s_actionTurnRight)
        {
          turnRight();
          driveUntilFoundLine();
          hasDoneTargetAction = true;
        }
      }
      else if(result == Found_Finish)
      {
        hasDoneTargetAction = true;
      }

      if(g_hasFoundFinishWhileTurning || 
        (result == Found_Finish))
      {
        g_hasFoundFinishWhileTurning = false;
        Serial.println("playbackMazeSolution() found finish!");
        delay(10000);
        hasDoneTargetAction = true;
        foundFinish = true;
      }
    }
  }
  Serial.println("playbackMazeSolution() finished");
}

void loop()
{
  Serial.println("loop()");
  
  solveMaze();
  playbackMazeSolution();
  resetRobotPath();

  //Reset global variables
  g_hasFoundFinishWhileTurning = false;
}
