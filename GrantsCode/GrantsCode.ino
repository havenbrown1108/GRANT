/* 
Lab4
Navigates a maze with two obstacles by following the policy "always turn left" when it reaches edges until it reaches the goal. We turn on lights to represent the state 
the Ringo is in so we can debug our bug.
*/
#include <Arduino_FreeRTOS.h>
#include "RingoHardware.h"

void TaskNavigateMaze(void *pvParameters);
void TaskController(void *pvParameters);
void TaskSensing(void *pvParameters);

// Globals
TaskHandle_t xControllerHandle;
TaskHandle_t xSensingHandle;
TaskHandle_t xNavigateMazeHandle;

float guidancePeriod = 300;
float controllerPeriod = 150;
int sensingPeriod = 50;

int intendedHeading;
int error = 0;

int baseSpeed = 30;
int maxSpeed = 100;
int motorBias = 5; //Our left motor is stronger than our right motor so this should account for that

int rightTurnAngle = 71;
int leftTurnAngle = -67;

char lastEdge = 0x0;
char edge;

enum Manuever { DriveStraight, Backup, TurningLeft};
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
  delay(500);
  NavigationBegin();
  ResumeNavigation();

  ResetLookAtEdge();

  xTaskCreate(
  TaskNavigateMaze
  ,  (const portCHAR *)"maze guidance task"
  ,  128 
  ,  NULL
  ,  1 
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

  intendedHeading = PresentHeading();
  manuever = DriveStraight;
  edge = 0x0;
}

void loop(){}

void TaskNavigateMaze(void *pvParameters) {
  unsigned long startTime = millis();
  int backingupTimeLimit = 1500;
  unsigned long time = millis() - startTime;
  SetAllPixelsRGB(0,0,0);

  for(;;) {
    time = millis() - startTime;

    // Logic for when to change state
    if (manuever == DriveStraight) {
      SetPixelRGB(TAIL_TOP, 0, 0, 100); // Blue
      if(FrontEdgeDetected(edge)) {
        manuever = Backup;
        startTime = millis();
      }
    } else if (manuever == Backup) {
        SetPixelRGB(TAIL_TOP, 0, 100, 0); // Green

        bool timeLimitReached = (time >= backingupTimeLimit) ? true : false;
        if(timeLimitReached) {
            manuever = TurningLeft;
            lastEdge = 0x0;
        }
    }
      else if (manuever == TurningLeft) {
        SetPixelRGB(TAIL_TOP, 100, 100, 0); // Purple
        if(error == 0) {
          manuever = DriveStraight;
        }
    }

    vTaskDelay(guidancePeriod / portTICK_PERIOD_MS);
  }

}

void TaskController(void *pvParameters) {
  float Kp = 1;
  float Ki = 1;
  float Kd = 1;

  int P, I = 0, D = 0;
  int currentHeading;
  int lastError = 0;
  int speedLeft = 50;
  int speedRight = 50;
  Manuever currentManuever;

  int u;

  for(;;) {
    if(currentManuever != manuever) {
      currentManuever = manuever;
      switch (manuever)
      {
        case DriveStraight:
          SetPixelRGB(BODY_TOP, 0, 0, 100);
          intendedHeading = PresentHeading();
          Kp = 1;
          Ki = 1;
          Kd = 1;
          baseSpeed = 30;
          break;
        case Backup:
          SetPixelRGB(BODY_TOP, 0, 100, 0);
          intendedHeading = PresentHeading();
          Kp = 1;
          Ki = 1;
          Kd = 1;
          baseSpeed = -30;
          break;
        case TurningLeft:
          SetPixelRGB(BODY_TOP, 100, 100, 0);
          intendedHeading = PresentHeading() + (leftTurnAngle / 4);
          Kp = 1;
          Ki = 0;
          Kd = 0;
          baseSpeed = 20;
          break;
        default:
          break;
      }
      
      OffEyes();
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

    if(error < 0) {
      OnEyes(100,0,0); // Red
      speedRight = -u + baseSpeed + motorBias;
      speedLeft = manuever == TurningLeft ? -(-u + baseSpeed) : baseSpeed;
    }
    else if(error > 0) {
      OnEyes(0,100,0); // Green
      speedLeft = u + baseSpeed;
      speedRight = manuever == TurningLeft ? -(-u + baseSpeed + motorBias) : baseSpeed;
    }
    else {
      OffEyes();
      if(currentManuever != Backup) {
        speedLeft = max(baseSpeed - motorBias, 0);
        speedRight = max(baseSpeed, 0);
      }
      else {
        speedLeft = baseSpeed - motorBias;
        speedRight = baseSpeed;
      }
    }

    lastError = error;

    speedLeft = min(speedLeft, maxSpeed);
    speedRight = min(speedRight, maxSpeed);
    Motors(speedLeft, speedRight);
    vTaskDelay(controllerPeriod / portTICK_PERIOD_MS); // put check to current time, delay until period - executed time
  }
}

void TaskSensing(void *pvParameters) {
    for(;;)
    {
        edge = LookForEdge();
        if(FrontEdgeDetected(edge)) {
            lastEdge = edge;
        }
        vTaskDelay(sensingPeriod / portTICK_PERIOD_MS);
    }
}