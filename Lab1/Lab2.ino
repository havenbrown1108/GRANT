/* 
Lab1
Play songs and blinks 
Version 6.2 03/2018
*/
#include <Arduino_FreeRTOS.h>
#include "RingoHardware.h"

void TaskSing( void *pvParameters );
void playNote(int note);
void restNote(int restLength);

// Globals
int noteLength = 150; // controls note length and will effect the speed of songs
int volume = 40; // Controls amplitude/volume of notes played

// Current color
int[] color = [0, 0, 0]

void setup(){
  HardwareBegin();        //initialize Ringo's brain to work with his circuitry
  SwitchButtonToPixels(); //set shared line to control NeoPixel lights
  SwitchMotorsToSerial(); //Call "SwitchMotorsToSerial()" before using Serial.print functions as motors & serial share a line
  // Set up for IR
  RxIRRestart(4);
  IsIRDone();
  GetIRButton();
  RestartTimer();

  xTaskCreate(
  TaskSing
  ,  (const portCHAR *)"Sing"
  ,  128 
  ,  NULL
  ,  2 
  ,  NULL );
}

void loop(){}

void TaskSing(void *pvParameters) 
{
  byte button;
  for (;;) 
  {
    // While idle and waiting for input blink eyes randomly
    RandomEyes();
    vTaskDelay( 500 / portTICK_PERIOD_MS ); 
    OffEyes();

    // Check for input
    if (IsIRDone()) {
      button = GetIRButton();
      if (button) {
        switch (button) {
          case 1: // If button 1 is pressed then play Megalovania
          OffEyes();
          PlayMegalovania();
          RxIRRestart(4);
          break;
          case 2: // If button 2 is pressed then play Mii Music
          OffEyes();
          PlayMiiSong();
          RxIRRestart(4);
          break;
          default:
          RxIRRestart(4);
          break;
        }
      }
    }
    RxIRRestart(4);
    vTaskDelay( 500 / portTICK_PERIOD_MS ); // Check every 0.5s
  }
}

void TaskPeriodicSound(void *pvParameters)
{
  PlayChirp(note, volume);
  vTaskDelay(noteLength / portTICK_PERIOD_MS);
  OffChirp();
  vTaskDelay( noteLength * restLength/ portTICK_PERIOD_MS );
}

void TaskPeriodicLights(void *pvParameters)
{
  
}

void TaskAperiodic(void xpvParameters)
{
  if (IsIRDone()) {
      button = GetIRButton();
      if (button) {
        switch (button) {
          case 1: // If button 1 is pressed then play Megalovania
          OffEyes();
          PlayMegalovania();
          RxIRRestart(4);
          break;
          case 2: // If button 2 is pressed then play Mii Music
          OffEyes();
          PlayMiiSong();
          RxIRRestart(4);
          break;
          default:
          RxIRRestart(4);
          break;
        }
      }
    }
    RxIRRestart(4);
}
