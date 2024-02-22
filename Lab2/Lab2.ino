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
// TaskHandle_t xSpinHandle;
int intendedHeading;


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
  ResetLookAtEdge();
  
  // xTaskCreate(
  // TaskBlinkEyes
  // ,  (const portCHAR *)"Blink"
  // ,  64 
  // ,  NULL
  // ,  1 
  // ,  NULL );


  // xTaskCreate(
  // TaskChangeEyeColor
  // ,  (const portCHAR *)"Change Eye Colors"
  // ,  128 
  // ,  NULL
  // ,  2 
  // ,  NULL );

  // xTaskCreate(
  // TaskSpinLilGuy
  // ,  (const portCHAR *)"Change direction"
  // ,  128 
  // ,  NULL
  // ,  4 
  // ,  &xSpinHandle );


  // xTaskCreate(
  // TaskEdgeDetector
  // ,  (const portCHAR *)"Check for edges"
  // ,  128 
  // ,  NULL
  // ,  3 
  // ,  NULL );

  // vTaskSuspend(xSpinHandle);
  // Motors(50,50);
  xTaskCreate(
  TaskDriveForwardController
  ,  (const portCHAR *)"Drive straight"
  ,  128 
  ,  NULL
  ,  3 
  ,  NULL );

  intendedHeading = PresentHeading();
  MaintainHeadingReset();
}

void loop(){}

void TaskDriveForwardController(void *pvParameters) {
  int Kp = 1;
  int Ki = 0;
  int Kd = 0;
  int P, I = 0, D = 0;
  int currentHeading;
  int error = 0;
  int lastError = 0;
  int speedLeft = 50;
  int speedRight = 50;
  int baseSpeed = 50;
  int u;

  for(;;) {
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
      speedLeft = baseSpeed;
      speedRight = baseSpeed;
    }

    lastError = error;
    Motors(speedLeft, speedRight);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// // This is a periodic task that makes the bug go beep beep.
// void TaskBlinkEyes(void *pvParameters)
// {
//   TickType_t xLastWakeTime;
//   xLastWakeTime = xTaskGetTickCount();

//   for(;;)
//   {
//     OnEyes(eyeColor[0],eyeColor[1],eyeColor[2]);
//     vTaskDelayUntil( &xLastWakeTime, blinkPeriod / portTICK_PERIOD_MS);
//     OffEyes();
//     vTaskDelayUntil( &xLastWakeTime, blinkPeriod / portTICK_PERIOD_MS);
//   }
// }

// // This is a periodic tasks that checks for edges
// void TaskEdgeDetector(void *pvParameters)
// {
//   TickType_t xLastWakeTime;
//   xLastWakeTime = xTaskGetTickCount();

//   for(;;)
//   {
//     edge = LookForEdge();
//     if(FrontEdgeDetected(edge))
//     {
//       // Unsuspend aperiodic "spin maneuver" task
//       vTaskResume(xSpinHandle);
//     } 

//     vTaskDelayUntil( &xLastWakeTime, edgeDetectorPeriod / portTICK_PERIOD_MS);
//   }
// }


// // This is a sporadic task that changes the color of the blinking eyes based on IRButton press.
// void TaskChangeEyeColor(void *pvParameters)
// {
//   byte button;
//   TickType_t xLastWakeTime;
//   xLastWakeTime = xTaskGetTickCount();

//   for(;;)
//   {
//       button = GetIRButton();
//       if (button) {
//         switch (button) {
//           case 1: // If button 1 is pressed change eyes to red
//           eyeColor[0] = 255;
//           eyeColor[1] = 0;
//           eyeColor[2] = 0;
//           RxIRRestart(4);
//           break;
//           case 2: // If button 2 is pressed change eyes to green
//           eyeColor[0] = 0;
//           eyeColor[1] = 255;
//           eyeColor[2] = 0;
//           RxIRRestart(4);
//           break;
//           case 3: // If button 3 is pressed change eyes to blue
//           eyeColor[0] = 0;
//           eyeColor[1] = 0;
//           eyeColor[2] = 255;
//           RxIRRestart(4);
//           break;
//           default:
//           RxIRRestart(4);
//           break;
//         }
//       }

//     RxIRRestart(4);
//     vTaskDelayUntil(&xLastWakeTime, remoteInputCheckPeriod / portTICK_PERIOD_MS ); 
//   }
  
// }


// // This is an aperiodic task that spins the lil guy
// void TaskSpinLilGuy(void *pvParameters)
// {
//   for(;;)
//   {
//     // Stop and chirp to alert an edge detection
//     Motors(0,0);
//     PlayChirp(NOTE_D4, volume);
//     vTaskDelay(noteLength / portTICK_PERIOD_MS);
//     OffChirp();

//     // Back up
//     Motors(-50,-50);
//     vTaskDelay(2000 / portTICK_PERIOD_MS);
//     Motors(0,0);

//     // Turn
//     Motors(50, 0);  
//     vTaskDelay(500 / portTICK_PERIOD_MS);

//     // Continue forward
//     Motors(50,50);

//     vTaskSuspend(xSpinHandle);
//   }
// }

