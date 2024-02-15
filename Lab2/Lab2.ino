/* 
Lab2
Laughs in my face and keeps me up at night
Version 6.2 03/2018
*/
#include <Arduino_FreeRTOS.h>
#include "RingoHardware.h"

void TaskBeepBeep(void *pvParameters);
void TaskBlinkEyes(void *pvParameters);
void TaskChangeEyeColor(void *pvParameters);
void TaskMoveForward(void *pvParameters);
void TaskEdgeDetector(void *pvParameters);
void TaskSpinLilGuy(void *pvParameters);


// Globals
int noteLength = 150;
int beepbeepPeriod = 1500; // controls note length and will effect the speed of songs
int volume = 40; // Controls amplitude/volume of notes played

int eyeColor[3] = {255, 0, 0}; 
int blinkPeriod = 1500;

int remoteInputCheckPeriod = 5000;

char edge;
int edgeDetectorPeriod = 100;
int movePeriod = 2000;
TaskHandle_t xSpinHandle, xMoveHandle;

int spinPeriod = 100;


void setup(){
  HardwareBegin();        //initialize Ringo's brain to work with his circuitry
  SwitchButtonToPixels(); //set shared line to control NeoPixel lights
  SwitchMotorsToSerial(); //Call "SwitchMotorsToSerial()" before using Serial.print functions as motors & serial share a line
  // Set up for IR
  RxIRRestart(4);
  IsIRDone();
  GetIRButton();
  // RestartTimer();

  // Setup Navigation
  delay(1000);
  NavigationBegin();

  // Edge Detection setup
  ResetLookAtEdge();

  // Debugging
  // Serial.begin(9600);
  Serial.println(F("In Setup function"));

  // xTaskCreate(
  // TaskBeepBeep
  // ,  (const portCHAR *)"Beep"
  // ,  128 
  // ,  NULL
  // ,  1 
  // ,  NULL );

  
  xTaskCreate(
  TaskBlinkEyes
  ,  (const portCHAR *)"Blink"
  ,  128 
  ,  NULL
  ,  1 
  ,  NULL );


  // xTaskCreate(
  // TaskChangeEyeColor
  // ,  (const portCHAR *)"Change Eye Colors"
  // ,  128 
  // ,  NULL
  // ,  2 
  // ,  NULL );

  // Serial.println(F("setup change eye color"));

  xTaskCreate(
  TaskMoveForward
  ,  (const portCHAR *)"Move Forward"
  ,  128 
  ,  NULL
  ,  2 
  ,  &xMoveHandle );

  // Serial.println(F("Setup MoveForward"));

  xTaskCreate(
  TaskSpinLilGuy
  ,  (const portCHAR *)"Change direction"
  ,  128 
  ,  NULL
  ,  2 
  ,  &xSpinHandle );

  // Serial.println(F("setup spin lil guy"));

  xTaskCreate(
  TaskEdgeDetector
  ,  (const portCHAR *)"Check for edges"
  ,  128 
  ,  NULL
  ,  3 
  ,  NULL );

  Serial.println(F("Reached end of setup"));

  
  
  // vTaskSuspend(xSpinHandle);
}

void loop(){}

// This is a periodic task that makes the bug go beep beep.
void TaskBeepBeep(void *pvParameters)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  Serial.println(F("TaskBeepBeep"));
  Serial.println();

  for(;;)
  {
    
    PlayChirp(NOTE_D4, volume);
    delay(noteLength);
    // vTaskDelayUntil( &xLastWakeTime, beepbeepPeriod / portTICK_PERIOD_MS );
    OffChirp();
    vTaskDelayUntil( &xLastWakeTime, beepbeepPeriod / portTICK_PERIOD_MS );
  }
}

// This is a periodic task that makes the bug go beep beep.
void TaskBlinkEyes(void *pvParameters)
{
  Serial.println(F("TaskBlinkEyes"));
  Serial.println();
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();


  for(;;)
  {
    OnEyes(eyeColor[0],eyeColor[1],eyeColor[2]);
    vTaskDelayUntil( &xLastWakeTime, blinkPeriod / portTICK_PERIOD_MS);
    OffEyes();
    vTaskDelayUntil( &xLastWakeTime, blinkPeriod / portTICK_PERIOD_MS);
  }
}


// This is a sporadic task that changes the color of the blinking eyes based on IRButton press.
void TaskChangeEyeColor(void *pvParameters)
{
  Serial.println(F("TaskChangeEyeColor"));
  Serial.println();
  byte button;
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  for(;;)
  {
    // if (IsIRDone()) {
      button = GetIRButton();
      if (button) {
        switch (button) {
          case 1: // If button 1 is pressed change eyes to red
          // OnEyes(255, 0, 0);
          // eyeColor = {255, 0, 0};
          eyeColor[0] = 255;
          eyeColor[1] = 0;
          eyeColor[2] = 0;
          RxIRRestart(4);
          break;
          case 2: // If button 2 is pressed change eyes to green
          // OnEyes(0, 255, 0);
          // eyeColor = {0, 255, 0};
          eyeColor[0] = 0;
          eyeColor[1] = 255;
          eyeColor[2] = 0;
          RxIRRestart(4);
          break;
          case 3: // If button 3 is pressed change eyes to blue
          // OnEyes(0, 0, 255);
          eyeColor[0] = 0;
          eyeColor[1] = 0;
          eyeColor[2] = 255;
          RxIRRestart(4);
          break;
          case 4:
          OffEyes();
          RxIRRestart(4);
          default:
          RxIRRestart(4);
          break;
        }
      }
    // }
    RxIRRestart(4);
    vTaskDelayUntil(&xLastWakeTime, remoteInputCheckPeriod / portTICK_PERIOD_MS ); 
  }
  
}

// This is a periodic task that scoots the bug forward
void TaskMoveForward(void *pvParameters)
{
  Serial.println(F("TaskMoveForward"));
  Serial.println();
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  for(;;)
  {
    
    Motors(50, 50);
    vTaskDelayUntil( &xLastWakeTime, movePeriod / portTICK_PERIOD_MS);
  }
}

// This is a periodic tasks that checks for edges
void TaskEdgeDetector(void *pvParameters)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  Serial.println("TaskEdgeDetector");
  Serial.println();

  for(;;)
  {
    edge = LookForEdge();
    if(FrontEdgeDetected(edge))
    {
      // unsuspend sporadic spinning task
      vTaskSuspend(xMoveHandle);
      vTaskResume(xSpinHandle);
    } else {
      vTaskSuspend(xSpinHandle);
      vTaskResume(xMoveHandle);
    }
    vTaskDelayUntil( &xLastWakeTime, edgeDetectorPeriod / portTICK_PERIOD_MS);
  }
}

// This is an aperiodic task that spins the lil guy
void TaskSpinLilGuy(void *pvParameters)
{
  Serial.println(F("TaskSpinLilGuy"));
  Serial.println();
  for(;;)
  {
    // Back up
    Motors(-50,-50);
    delay(2000);
    Motors(0,0);
    // Turn
    Motors(50, 0);
    delay(500);
    // Stop
    Motors(0,0);

    vTaskResume(xMoveHandle);
    
  }
}

