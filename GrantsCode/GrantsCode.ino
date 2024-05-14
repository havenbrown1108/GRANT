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
bool RedEdgeDetected(char edge);
// Globals
TaskHandle_t xControllerHandle;
TaskHandle_t xSensingHandle;
// TaskHandle_t xNavigateMazeHandle;
TaskHandle_t xFollowLineHandle;

float guidancePeriod = 50;
float controllerPeriod = 300;
int sensingPeriod = 50;

int intendedHeading;
int error = 0;

int baseSpeed = 30;
int maxSpeed = 50;
int motorBias = 10; //Our left motor is stronger than our right motor so this should account for that

int rightTurnAngle = 71;
int leftTurnAngle = -67;

char lastEdge = 0x0;
char edge;

enum Manuever { DriveStraight, Backup, TurningLeft, SwivelRight, SwivelLeft};
Manuever manuever;
// enum Antennae {Left, Right};


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

  // xTaskCreate(
  // TaskNavigateMaze
  // ,  (const portCHAR *)"maze guidance task"
  // ,  128 
  // ,  NULL
  // ,  1 
  // ,  &xNavigateMazeHandle );

  xTaskCreate (
    TaskFollowLine
  ,  (const portCHAR *)"follow line guidance task"
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
  ,  &xControllerHandle );

//   xTaskCreate(
//   TaskSensing
//   ,  (const portCHAR *)"Check for edges"
//   ,  128 
//   ,  NULL
//   ,  3 
//   ,  &xSensingHandle );

  intendedHeading = PresentHeading();
  manuever = DriveStraight;
  edge = 0x0;
  SetAllPixelsRGB(0, 0, 0);
}

void loop(){}


void TaskFollowLine(void *pvParameters) {
    int minManueverTime = 500;
    int startTime = millis();
    Manuever lastManuever;
    for(;;) {
        // Logic for when to change state
        LookForEdge();
        if (manuever == DriveStraight) {
            SetPixelRGB(TAIL_TOP, 0, 0, 100); // Blue
            lastManuever = DriveStraight;
            if(OurLeftEdgeDetected()) {
                manuever = SwivelLeft;
                lastManuever = SwivelLeft;
                startTime = millis();
                // lastEdge = 0x0;
                // Serial.println("left edge");
            }
            else if (OurRightEdgeDetected())
            {
                manuever = SwivelRight;
                lastManuever = SwivelRight;
                startTime = millis();
                // lastEdge = 0x0;
                // Serial.println("Right edge");
            }

            // if(!OurBackEdgeDetected()) {
            //      if(lastManuever == SwivelLeft) {
            //             manuever = SwivelRight;
            //             lastManuever = SwivelRight;
            //         }
            //         else {
            //             manuever = SwivelLeft;
            //             lastManuever = SwivelLeft;
            //         }
            // }
        
        } else if (manuever == SwivelLeft) {
            SetPixelRGB(TAIL_TOP, 0, 100, 0); // Green
            // Serial.println("swivelleft");

            // add if right edge detected swivel right
            if(millis() - startTime > minManueverTime) {
                if (OurRightEdgeDetected()) {
                    manuever = SwivelRight;
                    startTime = millis();
                    // lastEdge = 0x0;
                } else if(!(OurLeftEdgeDetected() || OurRightEdgeDetected())) {
                    manuever = DriveStraight;
                } else if((OurLeftEdgeDetected() && OurRightEdgeDetected())) {
                    // manuever = lastManuever;
                    if(lastManuever == SwivelLeft) {
                        manuever = SwivelRight;
                        // lastManuever = SwivelRight;
                    }
                    else {
                        manuever = SwivelLeft;
                        // lastManuever = SwivelLeft;
                    }
                }
            }
        } else if (manuever == SwivelRight) {
            SetPixelRGB(TAIL_TOP, 100, 0, 100); // Purple
            // Serial.println("swivelright");
            
            // add if left edge detected swivel left
            //   if(LeftFrontEdgeDetected(lastEdge)) {
            if(millis() - startTime > minManueverTime) {

                // if(!OurBackEdgeDetected()) {

                // }

                if(OurLeftEdgeDetected()) {
                    manuever = SwivelLeft;
                    startTime = millis();
                    // lastEdge = 0x0;
                } else if(!(OurLeftEdgeDetected() || OurRightEdgeDetected())) {
                    manuever = DriveStraight;
                } else if((OurLeftEdgeDetected() && OurRightEdgeDetected())) {
                    // manuever = lastManuever;
                    if(lastManuever == SwivelLeft) {
                        manuever = SwivelRight;
                    }
                    else {
                        manuever = SwivelLeft;
                    }
                }
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

  int maxLeft = 2 * leftTurnAngle;
  int maxRight = 2 * rightTurnAngle;

  int u;

  for(;;) {
    // Serial.println("Hello");
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
          baseSpeed = 40;
          break;
        case SwivelRight:
          SetPixelRGB(BODY_TOP, 100, 0, 100);
          intendedHeading = PresentHeading() + 25;
          Kp = 0.75;
          Ki = 0;
          Kd = 0;
          baseSpeed = 10;
          break;
        case SwivelLeft:
          SetPixelRGB(BODY_TOP, 0, 100, 0);
          intendedHeading = PresentHeading() - 25;
          Kp = 0.75;
          Ki = 0;
          Kd = 0;
          baseSpeed = 10;
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
    //   speedRight = -u + baseSpeed + motorBias;
    //   speedRight = manuever == SwivelRight ? -(-u + baseSpeed - motorBias) : baseSpeed + motorBias;
    //   speedLeft = manuever == SwivelLeft ? -(-u + baseSpeed) : baseSpeed;
        speedRight = -u + baseSpeed + motorBias;
      speedLeft = manuever == SwivelLeft ? -(-u + baseSpeed) : baseSpeed;
    }
    else if(error > 0) {
      OnEyes(0,100,0); // Green
    //   speedLeft = u + baseSpeed;
    //     speedLeft = manuever == SwivelRight ? u + baseSpeed : baseSpeed;
    //   speedRight = manuever == SwivelLeft ? -(-u + baseSpeed - motorBias) : baseSpeed + motorBias;
        speedLeft = u + baseSpeed;
      speedRight = manuever == SwivelRight ? -(-u + baseSpeed + motorBias) : baseSpeed;
    }
    else {
    //   OffEyes();
      if(currentManuever != Backup) {
        speedLeft = max(baseSpeed - motorBias, 0);
        speedRight = max(baseSpeed, 0);
      }
      else {
        speedLeft = baseSpeed;
        speedRight = baseSpeed + motorBias;
      }
    }

    if(error == 0) {
        if(manuever == SwivelLeft) {
            intendedHeading = PresentHeading() - 5;
            constrain(intendedHeading, maxLeft, -maxLeft);
        }
        if(manuever == SwivelRight) {
            intendedHeading = PresentHeading() + 5;
            constrain(intendedHeading, -maxRight, maxRight);
        }
    }

    lastError = error;

    speedLeft = constrain(speedLeft, -maxSpeed, maxSpeed);
    speedRight = constrain(speedRight, -maxSpeed, maxSpeed);
    // Serial.print("Hello: ");
    // Serial.print(speedLeft);
    // Serial.print(", ");
    // Serial.println(speedRight);
    Motors(speedLeft, speedRight);
    vTaskDelay(controllerPeriod / portTICK_PERIOD_MS);
  }
}

void TaskSensing(void *pvParameters) {
    for(;;)
    {
        edge = LookForEdge();
        Serial.print(LeftEdgeSensorValue);
        Serial.print(", ");
        Serial.println(RightEdgeSensorValue);

        if(FrontEdgeDetected(edge)) {
            lastEdge = edge;
        }
        vTaskDelay(sensingPeriod / portTICK_PERIOD_MS);
    }
}

bool OurLeftEdgeDetected() {
    if(LeftEdgeSensorValue > 150 && LeftEdgeSensorValue < 550) {
        Serial.println("Left edge detected");
        return true;
    }

    return false;
}

bool OurRightEdgeDetected() {
    if(RightEdgeSensorValue > 150 && RightEdgeSensorValue < 750) {
        Serial.println("Right edge detected");
        return true;
    }

    return false;
}

bool OurBackEdgeDetected() {
    if(RearEdgeSensorValue > 150 && RearEdgeSensorValue < 550) {
        Serial.println("Rear edge detected");
        return true;
    }

    return false;
}

bool RedEdgeDetected() {
  if(LeftEdgeSensorValue > 950 && LeftEdgeSensorValue < 1020) {
    return true;
  }
  if(RightEdgeSensorValue > 950 && RightEdgeSensorValue < 1020) {
    return true;
  }

  return false;
}