/* 
Lab2
"Always keep moving forward." -Attack on Titan
"Keep moving forward." - Meet the Robinsons, the movie EVERYONE else would reference, Ani
Moves forward until an edge is detected then chirps, backs up, changes direction and continues forward. 
During movement the eyes blink periodically. The color of the eyes can be changed by the IRRemote.
Version 6.2 03/2018
*/
#include <Arduino_FreeRTOS.h>
#include "RingoHardware.h"

void TaskBlinkEyes(void *pvParameters);
void TaskChangeEyeColor(void *pvParameters);
void TaskEdgeDetector(void *pvParameters);
void TaskSpinLilGuy(void *pvParameters);


// Globals
// int noteLength = 150;
// int volume = 40; // Controls amplitude/volume of notes played

// int eyeColor[3] = {255, 0, 0}; 
// int blinkPeriod = 1500;

// int remoteInputCheckPeriod = 1000;

// char edge;
// int edgeDetectorPeriod = 100;
TaskHandle_t xStraightHandle;
TaskHandle_t xTurnHandle;
int intendedHeading;
unsigned long startTime = millis();
int baseSpeed = 30;
int turnSpeed = 15;

int rightTurnAngle = 71;
int leftTurnAngle = -64;
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

  // Edge Detection setup
  // ResetLookAtEdge();
  
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
  MaintainHeadingReset();
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
  int speedLeft = 50;
  int speedRight = 50;

  int u;


  startTime = mills();
  for(;;) {
    unsigned long time = millis() - startTime;
    if( time >= 4000) {
      if(numTurns > 3) {
        // if(turningRight) {
        turningRight = false;
        turnAngle = leftTurnAngle;
        // }
      }
      intendedHeading = PresentHeading() + turnAngle;

      P = 0;
      I = 0;
      D = 0;
      numTurns++;
      vTaskResume(xTurnHandle);
      vTaskSuspend(xStraightHandle);
      startTime = millis();
    }
    // Serial.println("Driving forward");

    SimpleGyroNavigation();
    currentHeading = PresentHeading();

    error = intendedHeading - currentHeading;
    // error is positive when we turn  right and negative when we turn left

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
      speedLeft = baseSpeed - 6;
      speedRight = baseSpeed;

    }

    lastError = error;
    Motors(speedLeft, speedRight);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void TaskTurnController(void *pvParameters) {
  // OnEyes(100, 0, 100);

  float Kp = 0.4;
  int Ki = 0;
  int Kd = 1;
  int P, I = 0, D = 0;
  int currentHeading;
  int error = 0;
  int lastError = 0;
  int speedLeft = 50;
  int speedRight = 50;
  int u;

  for(;;) {
    // Serial.println("Turning right");
    SimpleGyroNavigation();
    currentHeading = PresentHeading();

    error = intendedHeading - currentHeading;
    
    // error is positive when we turn  right and negative when we turn left

    P = error;
    I = I + error;
    D = error - lastError;

    u = (Kp*P) + (Ki*I) + (Kd*D);
    // Serial.print(error);
    // Serial.print(" , ");
    // Serial.print(I);
    // Serial.print(" , ");
    // Serial.println(D);
  
    // The Ringo is veering right, so we need to turn a bit to the left
    if(error < 0) {
      OnEyes(100,0,0);
      speedRight = -u + turnSpeed + 5;
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
      vTaskResume(xStraightHandle);
      vTaskSuspend(xTurnHandle);
    }

    lastError = error;
    Motors(speedLeft, speedRight);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}