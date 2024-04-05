/* 
Lab4

*/
#include <Arduino_FreeRTOS.h>
#include "RingoHardware.h"


void TaskNavigateMaze(void *pvParameters);
void TaskController(void *pvParameters);
void TaskSensing(void *pvParameters);


// Globals
TaskHandle_t xControllerHandle;
// TaskHandle_t xDoSquaresHandle;
// TaskHandle_t xPlanningAndGuidanceHandle;
TaskHandle_t xSensingHandle;
// TaskHandle_t xAvoidObstacleHandle;
TaskHandle_t xNavigateMazeHandle;

float guidancePeriod = 300;
float controllerPeriod = 150;
int sensingPeriod = 50;

int intendedHeading;
int error = 0;



int baseSpeed = 30;
int maxSpeed = 100;
int motorBias = 5; //Our left motor is stronger than our right motor so this should account for that
int turnAdjustment = 0;

int rightTurnAngle = 71;
int leftTurnAngle = -67;
bool turnComplete = false;

int numTurns;

float Kp = 1;
float Ki = 1;
float Kd = 1;

char lastEdge = 0x0;
char edge;
bool edgeDetected = false;

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

  // Serial.println("setup");
  SetAllPixelsRGB(0,100,100);

  intendedHeading = PresentHeading();
  manuever = DriveStraight;
  edge = 0x0;

  // vTaskSuspend(xControllerHandle);
}

void loop(){}

void TaskNavigateMaze(void *pvParameters) {
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  bool drivingStraight = true;
  unsigned long startTime = millis();
  Manuever currentManuever = manuever;
  int backingupTimeLimit = 2000;
  unsigned long time = millis() - startTime;
  SetAllPixelsRGB(0,0,0);
  Serial.println("NavigateMaze");

  for(;;) {
    time = millis() - startTime;

    // SetPixelRGB(TAIL_TOP, 0, 0 ,0);
    Serial.println(manuever);
    Serial.println(error);
    // Logic for when to change state

    if (manuever == DriveStraight) {
      SetPixelRGB(TAIL_TOP, 0, 0, 100); // Blue
      if(FrontEdgeDetected(edge)) {
        manuever = Backup;
        startTime = millis();
      }
    } else if (manuever == Backup) {
        SetPixelRGB(TAIL_TOP, 0, 100, 0); // Green
        edgeDetected = false;

        bool timeLimitReached = (time >= backingupTimeLimit) ? true : false;
        if(timeLimitReached) {
          if(RightFrontEdgeDetected(lastEdge)) {
            manuever = TurningLeft;
            lastEdge = 0x0;
          } 
          else if(LeftFrontEdgeDetected(lastEdge)) {
            manuever = TurningRight;
            lastEdge = 0x0;
          }
        }
    } else if (manuever == TurningRight) {
        Serial.println("right");
        SetPixelRGB(TAIL_TOP, 100, 0, 100); 
        if(error == 0) {
          manuever = DriveStraight;
        }

    } else if (manuever == TurningLeft) {
        Serial.println("left");
        SetPixelRGB(TAIL_TOP, 100, 100, 0); 
        if(error == 0) {
          manuever = DriveStraight;
        }
    }
    // switch (manuever)
    // {
    //   case DriveStraight:
        // SetPixelRGB(TAIL_TOP, 0, 0, 100); // Blue
        // if(FrontEdgeDetected(edge)) {
        //   manuever = Backup;
        //   startTime = millis();
        // }
        // break;
    //   case Backup:
    //     SetPixelRGB(TAIL_TOP, 0, 100, 0); // Green
    //     edgeDetected = false;

    //     bool timeLimitReached = (time >= backingupTimeLimit) ? true : false;
    //     if(timeLimitReached) {
    //       if(RightFrontEdgeDetected(lastEdge)) {
    //         manuever = TurningLeft;
    //         lastEdge = 0x0;
    //       } 
    //       else if(LeftFrontEdgeDetected(lastEdge)) {
    //         manuever = TurningRight;
    //         lastEdge = 0x0;
    //       }
    //     }
    //     break;
    //   case TurningRight:
    //     Serial.println("right");
    //     SetPixelRGB(TAIL_TOP, 100, 0, 100); 
    //     if(error == 0) {
    //       manuever = DriveStraight;
    //     }
    //     break;
    //   case TurningLeft:
    //     Serial.println("left");
    //     SetPixelRGB(TAIL_TOP, 100, 100, 0);
    //     if(error == 0) {
    //       manuever = DriveStraight;
    //     }
    //     break;
    //   default:
    //     SetAllPixelsRGB(100, 100, 100);
    //     break;
    // }
    Serial.println("left switch statement");



    // vTaskDelayUntil(&xLastWakeTime, guidancePeriod);
    vTaskDelay(guidancePeriod / portTICK_PERIOD_MS);
  }

}

void TaskController(void *pvParameters) {
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

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
      switch (manuever)
      {
        case DriveStraight:
          // Serial.println("manuever is now DriveStraight");
          SetPixelRGB(BODY_TOP, 0, 0, 100);
          intendedHeading = PresentHeading();
          Kp = 1;
          Ki = 1;
          Kd = 1;
          baseSpeed = 30;
          turnAdjustment = 0;
          // error = 0;
          break;
        case Backup:
          // Serial.println("manuever is now Backup");
          SetPixelRGB(BODY_TOP, 0, 100, 0);
          intendedHeading = PresentHeading();
          Kp = 1;
          Ki = 1;
          Kd = 1;
          baseSpeed = -30;
          turnAdjustment = 0;
          // error = 0;
          break;
        case TurningRight:
          // Serial.println("manuever is now TurningRight");
          SetPixelRGB(BODY_TOP, 100, 0, 100);
          intendedHeading = PresentHeading() + (rightTurnAngle / 3);
          Kp = 0.4;
          Ki = 0;
          Kd = 1;
          baseSpeed = 30;
          // error = 0;
          turnAdjustment = baseSpeed;
          break;
        case TurningLeft:
          SetPixelRGB(BODY_TOP, 100, 100, 0);
          // Serial.println("manuever is now TurningLeft");
          intendedHeading = PresentHeading() + (leftTurnAngle / 3);
          Kp = 0.4;
          Ki = 0;
          Kd = 1;
          baseSpeed = 30;
          // error = 0;
          turnAdjustment = baseSpeed;
          break;
        default:
          break;
      }
            OffEyes();
      // error = 0;
      lastError = 0;
      P = 0;
      I = 0;
      D = 0;
    }
    // Logic for how to change state
    // if(currentManuever != manuever) {
    //   currentManuever = manuever;

    //   // newManueverDetected = false;
    // }

    // if(NavigationOn){
    //   CalibrateNavigationSensors();
    //   ResumeNavigation();
    // }
    // else
    //   NavigationBegin();  

    SimpleGyroNavigation();
    currentHeading = PresentHeading();

    error = intendedHeading - currentHeading;
    // if(error == 0) {
    //   Serial.println("error is 0 in the controller task!");

    // }

    P = error;
    I = I + error;
    D = error - lastError;

    u = (Kp*P) + (Ki*I) + (Kd*D);
  
    // The Ringo is veering right, so we need to turn a bit to the left
    if(error < 0) {
      OnEyes(100,0,0); // Red
      speedRight = -u + baseSpeed + motorBias;
      speedLeft = turnAdjustment != 0 ? -(-u + baseSpeed + motorBias) : baseSpeed - turnAdjustment;
    }
    // Vice versa
    else if(error > 0) {
      OnEyes(0,100,0); // Green
      speedLeft = u + baseSpeed;
      speedRight = turnAdjustment != 0 ? -(-u + baseSpeed + motorBias) : baseSpeed - turnAdjustment;
    }
    else {
      OffEyes();
      if(currentManuever != Backup) {
        speedLeft = max(baseSpeed - motorBias - turnAdjustment, 0);
        speedRight = max(baseSpeed - turnAdjustment, 0);
      }
      else {
        speedLeft = baseSpeed - motorBias;
        speedRight = baseSpeed;
      }
      // turnComplete = true;
    }

    lastError = error;

    speedLeft = min(speedLeft, maxSpeed);
    speedRight = min(speedRight, maxSpeed);
    // Serial.print(speedRight);
    // Serial.println(speedLeft);
    Motors(speedLeft, speedRight);
    vTaskDelay(controllerPeriod / portTICK_PERIOD_MS);
    // vTaskDelayUntil(&xLastWakeTime, controllerPeriod);
  }
}

void TaskSensing(void *pvParameters) {
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    // char edge = 0x0;
    // Serial.println("sensing task");
    for(;;)
    {
        edge = LookForEdge();
        // Serial.print("edge ");
        // Serial.println(edge, HEX);
        if(FrontEdgeDetected(edge)) {
            // Serial.println("Edge Detected");
            lastEdge = edge;
            edgeDetected = true;
            OnEyes(100, 100, 0);
            Serial.println(lastEdge, HEX);
        }
        // vTaskDelayUntil(&xLastWakeTime, sensingPeriod);
        vTaskDelay(sensingPeriod / portTICK_PERIOD_MS);
    }
}