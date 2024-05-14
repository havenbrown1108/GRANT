/* 
Final Project
Navigates a maze
*/
#include <Arduino_FreeRTOS.h>
#include "RingoHardware.h"

void TaskFollowLine(void *pvParameters);
void TaskController(void *pvParameters);
bool OurLeftEdgeDetected();
bool OurRightEdgeDetected();

// Globals
TaskHandle_t xControllerHandle;
TaskHandle_t xFollowLineHandle;

float guidancePeriod = 50;
float controllerPeriod = 50;

int intendedHeading;
int error = 0;

int baseSpeed = 30;
int maxSpeed = 80;
int motorBias = 15; //Our left motor is stronger than our right motor so this should account for that

int rightTurnAngle = 71;
int leftTurnAngle = -67;

enum Manuever { DriveStraight, Backup, TurningLeft, SwivelRight, SwivelLeft};
Manuever manuever;

void setup(){
  HardwareBegin();        //initialize Ringo's brain to work with his circuitry
  SetAllPixelsRGB(0, 0, 0);
  SwitchButtonToPixels(); //set shared line to control NeoPixel lights
  SwitchMotorsToSerial(); //Call "SwitchMotorsToSerial()" before using Serial.print functions as motors & serial share a line

  // Setup Navigation
  delay(500);
  NavigationBegin();
  ResumeNavigation();
  ResetLookAtEdge();

  xTaskCreate (
    TaskFollowLine
  ,  (const portCHAR *)"Follow line guidance task"
  ,  128 
  ,  NULL
  ,  2 
  ,  &xFollowLineHandle
  );

  xTaskCreate(
  TaskController
  ,  (const portCHAR *)"Drive in direction of intended heading"
  ,  128 
  ,  NULL
  ,  1 
  ,  &xControllerHandle
  );

  intendedHeading = PresentHeading();
  manuever = DriveStraight;
  SetAllPixelsRGB(0, 0, 0);
}

void loop(){}

void TaskFollowLine(void *pvParameters) {
    int minManueverTime = 750;
    int startTime = millis();
    Manuever lastManuever;
    for(;;) {
        // Logic for when to change state

        LookForEdge();
        if (manuever == SwivelRight && (millis() - startTime) < minManueverTime){
            manuever = SwivelRight;
        } else if (OurRightEdgeDetected() ) {
            manuever = SwivelRight;
            startTime = millis();
        } else if (OurLeftEdgeDetected()) {
            manuever = DriveStraight;
        } else  {
            manuever = SwivelLeft;
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

  int maxLeft = 1.5 * leftTurnAngle;
  int maxRight = 2.1 * rightTurnAngle;

  int u;

  for(;;) {
    SimpleGyroNavigation();

    if(currentManuever != manuever) {
      currentManuever = manuever;
      lastError = 0;
      switch (manuever)
      {
        case DriveStraight:
          SetPixelRGB(BODY_TOP, 0, 0, 100);
          intendedHeading = PresentHeading();
          Kp = 1;
          Ki = 1;
          Kd = 1;
          baseSpeed = 80;
          break;
        case SwivelRight:
          SetPixelRGB(BODY_TOP, 100, 0, 100);
          intendedHeading = PresentHeading() + 30;
          Kp = 2;
          Ki = 1;
          Kd = 0;
          baseSpeed = 15;
          break;
        case SwivelLeft:
          SetPixelRGB(BODY_TOP, 0, 100, 0);
          intendedHeading = PresentHeading() - 10;
          Kp = 2;
          Ki = 1;
          Kd = 0;
          baseSpeed = 25;
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

    currentHeading = PresentHeading();

    error = intendedHeading - currentHeading;

    P = error;
    I = I + error;
    D = error - lastError;

    u = (Kp*P) + (Ki*I) + (Kd*D);

    if(error < 0) {
      OnEyes(100,0,0); // Red
        speedRight = -u + baseSpeed + motorBias;
        if (manuever == DriveStraight) {
            speedLeft = baseSpeed - motorBias;
        } else if (manuever == SwivelLeft) {
            speedLeft = -baseSpeed * 0.75;
        } else {
            speedLeft = -baseSpeed * 0.5;
        }
    }
    else if(error > 0) {
        OnEyes(0,100,0); // Green
        if (manuever == DriveStraight) {
            speedLeft = -u + baseSpeed;
            speedRight = baseSpeed + motorBias;
        } else if (manuever == SwivelLeft) {
            intendedHeading = PresentHeading();
            speedLeft = u + baseSpeed - motorBias;
            speedRight = baseSpeed + motorBias;
        } else if (manuever == SwivelRight) {
            speedLeft = u + baseSpeed - motorBias;
            speedRight = -(baseSpeed + motorBias);
        }
    }
    else {
        if (manuever == DriveStraight) {
            speedRight = baseSpeed + motorBias;
            speedLeft = baseSpeed;
        }
        if (manuever == SwivelLeft) {
            intendedHeading = PresentHeading() - 10;
            constrain(intendedHeading, maxLeft, -maxLeft);
        }
        if (manuever == SwivelRight) {
            intendedHeading = PresentHeading() + 10;
            constrain(intendedHeading, -maxRight, maxRight);
        }
    }

    lastError = error;

    speedLeft = constrain(speedLeft, -65, 65);
    speedRight = constrain(speedRight, -maxSpeed, maxSpeed);
    Motors(speedLeft, speedRight);
    vTaskDelay(controllerPeriod / portTICK_PERIOD_MS);
  }
}

bool OurLeftEdgeDetected() {
    if(LeftEdgeSensorAverage > 150 && LeftEdgeSensorAverage < 550) {
        return true;
    }
    return false;
}

bool OurRightEdgeDetected() {
    if(RightEdgeSensorAverage > 150 && RightEdgeSensorAverage < 750) {
        return true;
    }
    return false;
}