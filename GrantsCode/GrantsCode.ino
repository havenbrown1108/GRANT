/* 
Lab4

*/
#include <Arduino_FreeRTOS.h>
#include "RingoHardware.h"


void TaskDriveForwardController(void *pvParameters);
void TaskTurnController(void *pvParameters);
void TaskSensing(void *pvParameters);


// Globals
TaskHandle_t xControllerHandle;
// TaskHandle_t xDoSquaresHandle;
// TaskHandle_t xPlanningAndGuidanceHandle;
TaskHandle_t xSensingHandle;
// TaskHandle_t xAvoidObstacleHandle;
TaskHandle_t xNavigateMazeHandle;

float guidancePeriod = 200;
float controllerPeriod = 150;
int edgeDetectorPeriod = 100;

int intendedHeading;
int error = 0;



int baseSpeed = 30;
int maxSpeed = 100;
int motorBias = 5; //Our left motor is stronger than our right motor so this should account for that
int turnAdjustment = 0;

int rightTurnAngle = 71;
int leftTurnAngle = -67;
bool turnComplete = false;
// bool turningRight = true;
// int turnAngle = rightTurnAngle;

int numTurns;

float Kp = 1;
float Ki = 1;
float Kd = 1;
// bool newManueverDetected;

char edge;

enum Manuever { DriveStraight, Backup, TurningRight, TurningLeft};
Manuever manuever;


void setup(){
  HardwareBegin();        //initialize Ringo's brain to work with his circuitry
  SetAllPixelsRGB(0, 0, 0);
  SwitchButtonToPixels(); //set shared line to control NeoPixel lights
  SwitchMotorsToSerial(); //Call "SwitchMotorsToSerial()" before using Serial.print functions as motors & serial share a line
  // Set up for IR
  RxIRRestart(4);
  IsIRDone();
  GetIRButton();

  // Setup Navigation
  delay(1000);
  NavigationBegin();

  ResetLookAtEdge();

  xTaskCreate(
  TaskNavigateMaze
  ,  (const portCHAR *)"maze guidance task"
  ,  128 
  ,  NULL
  ,  3 
  ,  &xNavigateMazeHandle );

  xTaskCreate(
  TaskController
  ,  (const portCHAR *)"Drive in direction of intended heading"
  ,  128 
  ,  NULL
  ,  2 
  ,  &xControllerHandle );

  xTaskCreate(
  TaskSensing
  ,  (const portCHAR *)"Check for edges"
  ,  128 
  ,  NULL
  ,  3 
  ,  &xSensingHandle );

  edge = 0x0;
  Serial.println("setup");

  intendedHeading = PresentHeading();
  manuever = DriveStraight;
}

void loop(){}

void TaskNavigateMaze(void *pvParameters) {
  bool drivingStraight = true;
  unsigned long startTime = millis();
  // int drivingStraightTime = 5000;
  Manuever currentManuever = manuever;
  int backingupTimeLimit = 1000;
  unsigned long time = millis() - startTime;
  Serial.println("NavigateMaze");
  char lastEdgeSeen = 0x0;

  for(;;) {
    // Serial.println("start" + time);
    // Serial.print("manuever is ");
    // Serial.println(manuever);

    time = millis() - startTime;
    SetPixelRGB(0, 0, 0 ,0);
    // vTaskDelay(10)
    // Serial.println("error at start of for loop in guidance = " + error);


    // Logic for when to change state
    switch (manuever)
    {
      case DriveStraight:
        SetPixelRGB(0, 0, 0, 100);
        // Serial.println("driving straight part one....");
        if(FrontEdgeDetected(edge)) {
          manuever = Backup;
          lastEdgeSeen = edge;
          startTime = millis();
          // newManueverDetected = true;
        }
        break;
      case Backup:
        SetPixelRGB(0, 0, 100, 0);
        // Serial.println("backing up part one....");
        // Serial.println(time);
        bool timeLimitReached = (time >= backingupTimeLimit) ? true : false;
        if(timeLimitReached) {
          // Serial.println("time limit reached");
          Serial.println(lastEdgeSeen, HEX);
        }
        if(timeLimitReached && RightFrontEdgeDetected(lastEdgeSeen)) {
          Serial.println("right edge");
          manuever = TurningLeft;
          lastEdgeSeen = 0x0;
          // newManueverDetected = true;
          // turnComplete = false;
        } 
        else if(timeLimitReached && LeftFrontEdgeDetected(lastEdgeSeen)) {
          manuever = TurningRight;
          lastEdgeSeen = 0x0;
          // startTime = millis();
          // newManueverDetected = true;
          // turnComplete = false;
        }
        break;
      case TurningRight:
        SetPixelRGB(0, 100, 0, 0);
        if(error == 0) {
          manuever = DriveStraight;
          // newManueverDetected = true;
        }
        break;
      case TurningLeft:
        SetPixelRGB(0, 100, 100, 0);
        // Serial.println("turning left part one...");
        if(error == 0) {
          manuever = DriveStraight;
          // newManueverDetected = true;
        }
        break;
      default:
        break;
    }

    // Logic for how to change state
    if(currentManuever != manuever) {
      currentManuever = manuever;
      switch (manuever)
      {
        case DriveStraight:
          Serial.println("manuever is now DriveStraight");
          intendedHeading = PresentHeading();
          Kp = 1;
          Ki = 1;
          Kd = 1;
          baseSpeed = 30;
          turnAdjustment = 0;
          // error = 0;
          break;
        case Backup:
          Serial.println("manuever is now Backup");
          intendedHeading = PresentHeading();
          Kp = 1;
          Ki = 1;
          Kd = 1;
          baseSpeed = -30;
          turnAdjustment = 0;
          // error = 0;
          break;
        case TurningRight:
          SetPixelRGB(0, 255,20,147);
          Serial.println("manuever is now TurningRight");
          intendedHeading = PresentHeading() + (rightTurnAngle / 2);
          Kp = 0.4;
          Ki = 0;
          Kd = 1;
          baseSpeed = 15;
          // error = 0;
          turnAdjustment = baseSpeed;
          break;
        case TurningLeft:
          Serial.println("manuever is now TurningLeft");
          intendedHeading = PresentHeading() + (leftTurnAngle / 2);
          Kp = 0.4;
          Ki = 0;
          Kd = 1;
          baseSpeed = 15;
          // error = 0;
          turnAdjustment = baseSpeed;
          break;
        default:
          break;
      }
    }

    vTaskDelay(guidancePeriod / portTICK_PERIOD_MS);
  }

}



void TaskController(void *pvParameters) {
  int P, I = 0, D = 0;
  int currentHeading;
  // int error = 0;
  int lastError = 0;
  int speedLeft = 50;
  int speedRight = 50;
  Manuever currentManuever;

  int u;

  for(;;) {

    if(currentManuever != manuever) {
      currentManuever = manuever;
      OffEyes();
      // error = 0;
      lastError = 0;
      P = 0;
      I = 0;
      D = 0;
    }

    SimpleGyroNavigation();
    currentHeading = PresentHeading();

    error = intendedHeading - currentHeading;

    P = error;
    I = I + error;
    D = error - lastError;

    u = (Kp*P) + (Ki*I) + (Kd*D);
  
    // The Ringo is veering right, so we need to turn a bit to the left
    if(error < 0) {
      OnEyes(100,0,0);
      speedRight = -u + baseSpeed + motorBias;
      speedLeft = baseSpeed - turnAdjustment;
    }
    // Vice versa
    else if(error > 0) {
      OnEyes(0,100,0);
      speedLeft = u + baseSpeed;
      speedRight = baseSpeed - turnAdjustment;
    }
    else {
      OffEyes();
      speedLeft = max(baseSpeed - motorBias - turnAdjustment, 0);
      speedRight = max(baseSpeed - turnAdjustment, 0);
      // turnComplete = true;
    }

    lastError = error;

    speedLeft = min(speedLeft, maxSpeed);
    speedRight = min(speedRight, maxSpeed);
    Motors(speedLeft, speedRight);
    vTaskDelay(controllerPeriod / portTICK_PERIOD_MS);
  }
}

void TaskSensing(void *pvParameters) {
    Serial.println("sensing task");

    for(;;)
    {
        edge = LookForEdge();

        vTaskDelay( edgeDetectorPeriod / portTICK_PERIOD_MS);
    }
}
