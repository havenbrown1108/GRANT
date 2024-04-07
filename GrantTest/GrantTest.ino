/* 
Grant Testing Framework
*/
#include <Arduino_FreeRTOS.h>
#include "RingoHardware.h"

/***** TEST DRIVER GLOBALS ******/
#define ITERATIONS 20
void TestDriver(void *pvParameters);
void updateWCET();
TaskHandle_t xMasterHandle;
unsigned long startTime = 0, endTime = 0, wcet = 0;
/***** END TEST DRIVER GLOBALS ******/

/***** TASK-UNDER-TEST GLOBALS ******/
void TaskNavigateMaze(void *pvParameters);
void TaskController(void *pvParameters);
void TaskSensing(void *pvParameters);

TaskHandle_t xSensingHandle, xControllerHandle, xNavigateMazeHandle;
char edge, lastEdge;
float sensingPeriod = 1000, guidancePeriod = 300, controllerPeriod = 150;
int intendedHeading=0, error = 0;
int baseSpeed = 30, maxSpeed = 100, motorBias = 5;
int rightTurnAngle = 71, leftTurnAngle = -67;

enum Manuever { DriveStraight, Backup, TurningLeft};
Manuever manuever = Backup;
/***** END TASK-UNDER-TEST GLOBALS ******/

void setup(){
  /*** Begin Ringo Init  ***/
  HardwareBegin();
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
  Serial.begin(57600);
  /*** End Ringo Init  ***/
  
  // Test Driver Task
  xTaskCreate(
  TestDriver
  ,  (const portCHAR *)"Test Driver"
  ,  128
  ,  NULL
  ,  1
  ,  &xMasterHandle
  );

  // Task-under-test
  // xTaskCreate(
  // TaskNavigateMaze
  // ,  (const portCHAR *)"maze guidance task"
  // ,  128 
  // ,  NULL
  // ,  3
  // ,  &xNavigateMazeHandle );
  // vTaskSuspend(xNavigateMazeHandle);

  xTaskCreate(
  TaskController
  ,  (const portCHAR *)"Drive in direction of intended heading"
  ,  128 
  ,  NULL
  ,  3
  ,  &xControllerHandle );
  vTaskSuspend(xControllerHandle);

  // xTaskCreate(
  // TaskSensing
  // ,  (const portCHAR *)"Check for edges"
  // ,  128 
  // ,  NULL
  // ,  3 
  // ,  &xSensingHandle );
  // vTaskSuspend(xSensingHandle);

}

void loop(){}

void TestDriver(void *pvParameters)
{
    for (;;){
      // vTaskResume(xNavigateMazeHandle);
        vTaskResume(xControllerHandle);
      // vTaskResume(xSensingHandle);
      Serial.println(wcet);

      vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
  


  
}

void TaskNavigateMaze(void *pvParameters) {
  unsigned long startTime = millis();
  int backingupTimeLimit = 1500;
  unsigned long time = millis() - startTime;
  SetAllPixelsRGB(0,0,0);

  for(int i=0; i<ITERATIONS; i++) {
    startTime = micros();
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

    endTime = micros();
    wcet = max(endTime - startTime, wcet);
    vTaskDelay(guidancePeriod / portTICK_PERIOD_MS);
  }

  vTaskSuspend(xNavigateMazeHandle);
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

  for(int i=0; i<ITERATIONS; i++) {
    startTime = micros();
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
    // Motors(speedLeft, speedRight);

    endTime = micros();
    wcet = max(endTime - startTime, wcet);
    vTaskDelay(controllerPeriod / portTICK_PERIOD_MS);
  }

  vTaskSuspend(xControllerHandle);
}

void TaskSensing(void *pvParameters) {
  for(int i=0; i<ITERATIONS; i++) {
    startTime = micros();

    edge = LookForEdge();
    if(FrontEdgeDetected(edge))
    {
      // Handle right front edge detected
       if(FrontEdgeDetected(edge)) {
            lastEdge = edge;
        }
    }

    endTime = micros();
    wcet = max(endTime - startTime, wcet);
    vTaskDelay( sensingPeriod / portTICK_PERIOD_MS);
  }

  
  vTaskSuspend(xSensingHandle);
}