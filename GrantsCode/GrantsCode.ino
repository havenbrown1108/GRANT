/* 
Lab4

*/
#include <Arduino_FreeRTOS.h>
#include "RingoHardware.h"

void TaskDriveForwardController(void *pvParameters);
void TaskTurnController(void *pvParameters);


// Globals
TaskHandle_t xControllerHandle;
TaskHandle_t xDoSquaresHandle;
TaskHandle_t xPlanningAndGuidanceHandle;

float guidancePeriod = 150;
float controllerPeriod = 100;

int intendedHeading;
int error = 0;



int baseSpeed = 30;
int motorBias = 5; //Our left motor is stronger than our right motor so this should account for that
int turnAdjustment = 0;

int rightTurnAngle = 71;
int leftTurnAngle = -67;
bool turningRight = true;
int turnAngle = rightTurnAngle;

int numTurns;

float Kp = 1;
float Ki = 1;
float Kd = 1;
bool newManoevreDetected;


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
  TaskPlanningAndGuidance
  ,  (const portCHAR *)"Drive straight"
  ,  128 
  ,  NULL
  ,  3 
  ,  &xPlanningAndGuidanceHandle );

  xTaskCreate(
  TaskDoSquares
  ,  (const portCHAR *)"Drive in squares"
  ,  128 
  ,  NULL
  ,  3 
  ,  &xDoSquaresHandle );

  xTaskCreate(
  TaskController
  ,  (const portCHAR *)"Drive in direction of intended heading"
  ,  128 
  ,  NULL
  ,  3 
  ,  &xControllerHandle );

  vTaskSuspend(xDoSquaresHandle);
  numTurns = 0;

  intendedHeading = PresentHeading();
}

void loop(){}

void TaskPlanningAndGuidance(void *pvParameters)
{
  for(;;) {
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    vTaskResume(xDoSquaresHandle);
  }
}

void TaskDoSquares(void *pvParameters) {
  bool drivingStraight = true;
  unsigned long startTime = millis();
  int drivingStraightTime = 5000;
  unsigned long time = millis() - startTime;

  for(;;) {
    time = millis() - startTime;
    if(drivingStraight) {
      if( time >= drivingStraightTime) { // We need to do a turn
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

          drivingStraight = false;
          intendedHeading = PresentHeading() + turnAngle;
          turnAdjustment = baseSpeed;
          startTime = millis();
          Kp = 0.4;
          Ki = 0;
          Kd = 1;
          baseSpeed = 7;

          newManoevreDetected = true;
        }
    }
    else if(error == 0) { // We should drive straight again. 
      drivingStraight = true;
      startTime = millis();
      Kp = 1;
      Ki = 1;
      Kd = 1;
      baseSpeed = 30;
      turnAdjustment = 0;
      newManoevreDetected = true;
    }

    vTaskDelay(guidancePeriod / portTICK_PERIOD_MS);
  }

}

void TaskController(void *pvParameters) {
  int P, I = 0, D = 0;
  int currentHeading;
  int error = 0;
  int lastError = 0;
  int speedLeft = 50;
  int speedRight = 50;

  int u;

  for(;;) {

    if(newManoevreDetected) {
      OffEyes();
      lastError = 0;
      P = 0;
      I = 0;
      D = 0;
      newManoevreDetected = false;
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
    }

    lastError = error;
    Motors(speedLeft, speedRight);
    vTaskDelay(controllerPeriod / portTICK_PERIOD_MS);
  }
}
