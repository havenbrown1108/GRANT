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
int sensingPeriod = 100;

int intendedHeading;
int error = 0;



int baseSpeed = 30;
int maxSpeed = 50;
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

  // Serial.println("setup");
  SetAllPixelsRGB(0,100,100);

  intendedHeading = PresentHeading();
  manuever = DriveStraight;

  // vTaskSuspend(xControllerHandle);
}

void loop(){}

void TaskNavigateMaze(void *pvParameters) {
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  bool drivingStraight = true;
  unsigned long startTime = millis();
  Manuever currentManuever = manuever;
  int backingupTimeLimit = 1000;
  unsigned long time = millis() - startTime;
  SetAllPixelsRGB(0,0,0);
  Serial.println("NavigateMaze");

  for(;;) {
    time = millis() - startTime;

    SetPixelRGB(TAIL_TOP, 0, 0 ,0);
    // Logic for when to change state
    switch (manuever)
    {
      case DriveStraight:
        SetPixelRGB(TAIL_TOP, 0, 0, 100); // Blue
        if(edgeDetected) {
          // Serial.println("Backup");
          manuever = Backup;
          
          startTime = millis();
        }
        // vTaskResume(xControllerHandle);
        break;
      case Backup:
        SetPixelRGB(TAIL_TOP, 0, 100, 0); // Green
        edgeDetected = false;
        // Serial.println("Waiting");
        vTaskDelay(backingupTimeLimit / portTICK_PERIOD_MS);
        // Serial.println("Waited");
        // Serial.println(lastEdgeSeen,HEX);
        // Serial.println(LeftFrontEdgeDetected(lastEdgeSeen));
        if(RightFrontEdgeDetected(lastEdge)) {
          // Serial.println("right edge");
          SetPixelRGB(BODY_TOP, 100, 0, 0);
          manuever = TurningLeft;
          lastEdge = 0x0;
        } 
        else if(LeftFrontEdgeDetected(lastEdge)) {
          // Serial.println("right edge");
          SetPixelRGB(BODY_TOP, 0, 0, 100);
          manuever = TurningRight;
          lastEdge = 0x0;
        }
        
        break;
      case TurningRight:
        SetPixelRGB(TAIL_TOP, 100, 0, 0); // Red
        if(error == 0) {
          manuever = DriveStraight;
        }
        // vTaskResume(xControllerHandle);
        break;
      case TurningLeft:
        SetPixelRGB(TAIL_TOP, 100, 100, 0); // Yellow
        if(error == 0) {
          manuever = DriveStraight;
        }
        // vTaskResume(xControllerHandle);
        break;
      default:
        break;
    }

    if(currentManuever != manuever) {
      currentManuever = manuever;
      switch (manuever)
      {
        case DriveStraight:
          // Serial.println("manuever is now DriveStraight");
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
          intendedHeading = PresentHeading() + (rightTurnAngle / 2);
          Kp = 0.4;
          Ki = 0;
          Kd = 1;
          baseSpeed = 15;
          // error = 0;
          turnAdjustment = baseSpeed;
          break;
        case TurningLeft:
          // Serial.println("manuever is now TurningLeft");
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

    vTaskDelayUntil(&xLastWakeTime, guidancePeriod);
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
    // Logic for how to change state
    if(currentManuever != manuever) {
      currentManuever = manuever;
      OffEyes();
      // error = 0;
      lastError = 0;
      P = 0;
      I = 0;
      D = 0;
      // newManueverDetected = false;
    }

    if(NavigationOn){
      CalibrateNavigationSensors();
      ResumeNavigation();
    }
    else
      NavigationBegin();  

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
      speedLeft = baseSpeed - turnAdjustment;
    }
    // Vice versa
    else if(error > 0) {
      OnEyes(0,100,0); // Green
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
    // Motors(speedLeft, speedRight);
    // vTaskDelay(controllerPeriod / portTICK_PERIOD_MS);
    vTaskDelayUntil(&xLastWakeTime, controllerPeriod);
  }
}

void TaskSensing(void *pvParameters) {
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    char edge = 0x0;
    // Serial.println("sensing task");
    for(;;)
    {
        edge = LookForEdge();
        if(FrontEdgeDetected(edge)) {
            // Serial.println("Edge Detected");
            lastEdge = edge;
            edgeDetected = true;
            // Serial.println(lastEdge, HEX);
        }
        vTaskDelayUntil(&xLastWakeTime, sensingPeriod);
    }
}