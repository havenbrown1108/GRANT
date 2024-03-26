/* 
Lab3
"Always keep moving forward." -Attack on Titan
"Keep moving forward." - Meet the Robinsons, the movie EVERYONE else would reference, Ani
Moves forward until an edge is detected then chirps, backs up, changes direction and continues forward. 
During movement the eyes blink periodically. The color of the eyes can be changed by the IRRemote.
Version 6.2 03/2018
*/
#include <Arduino_FreeRTOS.h>
#include "RingoHardware.h"

void TaskDriveForwardController(void *pvParameters);
void TaskTurnController(void *pvParameters);


// Globals
TaskHandle_t xStraightHandle;
TaskHandle_t xTurnHandle;
int intendedHeading;

unsigned long startTime = millis();
int drivingStraightTime = 5000;

int baseSpeed = 30;
int turnSpeed = 15;
int speedLeft = 50;
int speedRight = 50;

int motorBias = 5; //Our left motor is stronger than our right motor so this should account for that

int rightTurnAngle = 71;
int leftTurnAngle = -67;
bool turningRight = true;
int turnAngle = rightTurnAngle;

int numTurns;


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
  
  xTaskCreate(
  TaskDriveForwardController
  ,  (const portCHAR *)"Drive straight"
  ,  128 
  ,  NULL
  ,  3 
  ,  &xStraightHandle );

  xTaskCreate(
  TaskTurnController
  ,  (const portCHAR *)"Drive straight"
  ,  128 
  ,  NULL
  ,  3 
  ,  &xTurnHandle );

  vTaskSuspend(xTurnHandle);
  numTurns = 0;

  intendedHeading = PresentHeading();
}

void loop(){}

void TaskDriveForwardController(void *pvParameters) {
  int Kp = 1;
  int Ki = 1;
  int Kd = 1;
  int P, I = 0, D = 0;
  int currentHeading;
  int error = 0;
  int lastError = 0;

  int u;


  startTime = millis();
  for(;;) {
    unsigned long time = millis() - startTime;

    if( time >= drivingStraightTime) {
      if(numTurns >= 3) {
        numTurns = 0;
        if(turningRight) {
          turningRight = false;
          turnAngle = leftTurnAngle;
        }
        else {
          turningRight = true;
          turnAngle = rightTurnAngle;
        }
      }
      else {
        numTurns++;
      }

      intendedHeading = PresentHeading() + turnAngle;

      P = 0;
      I = 0;
      D = 0;
      lastError = 0;
      vTaskResume(xTurnHandle);
      vTaskSuspend(xStraightHandle);
      startTime = millis();
    }

    SimpleGyroNavigation();
    currentHeading = PresentHeading();

    // Error is positive when we turn  right and negative when we turn left
    error = intendedHeading - currentHeading;

    P = error;
    I = I + error;
    D = error - lastError;

    u = (Kp*P) + (Ki*I) + (Kd*D);
  
    // The Ringo is veering right, so we need to turn a bit to the left
    if(error < 0) {
      OnEyes(100,0,0);
      speedRight = -u + baseSpeed;
      speedLeft = baseSpeed;
    }
    // Vice versa
    else if(error > 0) {
      OnEyes(0,100,0);
      speedLeft = u + baseSpeed;
      speedRight = baseSpeed;
    }
    else {
      OffEyes();
      speedLeft = baseSpeed - motorBias;
      speedRight = baseSpeed;
    }

    lastError = error;
    Motors(speedLeft, speedRight);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void TaskTurnController(void *pvParameters) {
  float Kp = 0.4;
  int Ki = 0;
  int Kd = 1;
  int P, I = 0, D = 0;
  int currentHeading;
  int error = 0;
  int lastError = 0;
  int u;

  for(;;) {
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
      speedRight = -u + turnSpeed + motorBias;
      speedLeft = 0;
    }
    // Vice versa
    else if(error > 0) {
      OnEyes(0,100,0);
      speedLeft = u + turnSpeed;
      speedRight = 0;
    }
    else {
      OffEyes();
      speedLeft = 0;
      speedRight = 0;
      startTime = millis();
      
      P = 0;
      I = 0;
      D = 0;
      lastError = 0;
      vTaskResume(xStraightHandle);
      vTaskSuspend(xTurnHandle);
    }

    lastError = error;
    Motors(speedLeft, speedRight);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}