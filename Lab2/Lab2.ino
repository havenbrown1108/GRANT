/* 
Lab1
Play songs and blinks 
Version 6.2 03/2018
*/
#include <Arduino_FreeRTOS.h>
#include "RingoHardware.h"

void TaskPeriodicSound(void *pvParameters);
void TaskPeriodicLights(void *pvParameters);
void TaskAperiodic(void *pvParameters);

// Globals
int noteLength = 150; // controls note length and will effect the speed of songs
int volume = 40; // Controls amplitude/volume of notes played
int eyeColor[3] = {255, 0, 0};


void setup(){
  HardwareBegin();        //initialize Ringo's brain to work with his circuitry
  SwitchButtonToPixels(); //set shared line to control NeoPixel lights
  SwitchMotorsToSerial(); //Call "SwitchMotorsToSerial()" before using Serial.print functions as motors & serial share a line
  // Set up for IR
  RxIRRestart(4);
  IsIRDone();
  GetIRButton();
  // RestartTimer();

  xTaskCreate(
  TaskPeriodicSound
  ,  (const portCHAR *)"Beep"
  ,  128 
  ,  NULL
  ,  1 
  ,  NULL );

  xTaskCreate(
  TaskPeriodicLights
  ,  (const portCHAR *)"Blink"
  ,  128 
  ,  NULL
  ,  2 
  ,  NULL );

  xTaskCreate(
  TaskAperiodic
  ,  (const portCHAR *)"Change Colors"
  ,  128 
  ,  NULL
  ,  3 
  ,  NULL );
}

void loop(){}


void TaskPeriodicSound(void *pvParameters)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  for(;;)
  {
    PlayChirp(NOTE_D4, volume);
    vTaskDelayUntil( &xLastWakeTime, noteLength / portTICK_PERIOD_MS );
    // vTaskDelay(10000 / portTICK_PERIOD_MS);
    OffChirp();
    // vTaskDelay(10000 / portTICK_PERIOD_MS);
    vTaskDelayUntil( &xLastWakeTime, noteLength / portTICK_PERIOD_MS );
  }
}

void TaskPeriodicLights(void *pvParameters)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  for(;;)
  {
    SetPixelRGB(EYE_LEFT,eyeColor[0],eyeColor[1],eyeColor[2]);
    SetPixelRGB(EYE_RIGHT,eyeColor[0],eyeColor[1],eyeColor[2]);
    vTaskDelayUntil( &xLastWakeTime, 500 / portTICK_PERIOD_MS);
    OffEyes();
    vTaskDelayUntil( &xLastWakeTime, 500 / portTICK_PERIOD_MS);
  }
}

void TaskAperiodic(void *pvParameters)
{
  byte button;
  for(;;)
  {
    // if (IsIRDone()) {
      button = GetIRButton();
      if (button) {
        switch (button) {
          case 1: // If button 1 is pressed change eyes to red
          eyeColor[0] = 255;
          eyeColor[1] = 0;
          eyeColor[2] = 0;
          // OnEyes(255, 0, 0);
          RxIRRestart(4);
          break;
          case 2: // If button 2 is pressed change eyes to green
          eyeColor[0] = 0;
          eyeColor[1] = 255;
          eyeColor[2] = 0;
          // OnEyes(0, 255, 0);
          RxIRRestart(4);
          break;
          case 3: // If button 3 is pressed change eyes to blue
          eyeColor[0] = 0;
          eyeColor[1] = 0;
          eyeColor[2] = 255;
          // OnEyes(0, 0, 255);
          RxIRRestart(4);
          break;
          default:
          RxIRRestart(4);
          break;
        }
      }
    // }
    
    RxIRRestart(4);
    vTaskDelay( 100 / portTICK_PERIOD_MS ); 
  }
  
}
