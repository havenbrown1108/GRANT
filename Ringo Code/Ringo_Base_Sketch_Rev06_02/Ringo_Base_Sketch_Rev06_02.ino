#include <Arduino_FreeRTOS.h>
/* 

Ringo Robot
Ringo_Base_Sketch_Rev06
Version 6.2 03/2018

This is a basic sketch that can be used as a starting point
for various functionality of the Ringo robot.

Significant portions of this code written by
Dustin Soodak for Plum Geek LLC. Some portions
contributed by Kevin King.
Portions from other open source projects where noted.
This code is licensed under:
Creative Commons Attribution-ShareAlike 2.0 Generic (CC BY-SA 2.0)
https://creativecommons.org/licenses/by-sa/2.0/
Visit http://www.plumgeek.com for Ringo information.
Visit http://www.arduino.cc to learn about the Arduino.

*/

#include "RingoHardware.h"

//int i;    //declaire any global variables here
void TaskBlink( void *pvParameters );
void TaskSing( void *pvParameters );

void setup(){
  HardwareBegin();        //initialize Ringo's brain to work with his circuitry
  PlayStartChirp();       //Play startup chirp and blink eyes
  SwitchMotorsToSerial(); //Call "SwitchMotorsToSerial()" before using Serial.print functions as motors & serial share a line

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }

  RestartTimer();

    xTaskCreate(
    TaskBlink
    ,  (const portCHAR *)"Blink"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

    xTaskCreate(
    TaskSing
    ,  (const portCHAR *)"Sing"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );
  
}

    
void loop(){ 
  // we don't want to use this, we'll be using tasks
} // end of loop() function

void TaskBlink(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;) // A Task shall never return or exit.
  {
    SetPixelRGB(EYE_LEFT, 0,0,255);
    SetPixelRGB(EYE_RIGHT, 0,255,0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    SetPixelRGB(EYE_LEFT, 0,0,0);
    SetPixelRGB(EYE_RIGHT, 0,0,0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void TaskSing(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  int volume = 40;

  for (;;) // A Task shall never return or exit.
  {
    PlayChirp(NOTE_FS4, 40);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    OffChirp();
    vTaskDelay(400 / portTICK_PERIOD_MS);

    PlayChirp(NOTE_A4, 40);
    vTaskDelay(300 / portTICK_PERIOD_MS);
    OffChirp();
    vTaskDelay(500 / portTICK_PERIOD_MS);

    PlayChirp(NOTE_CS4, 40);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    OffChirp();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    PlayChirp(NOTE_A4, 40);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    OffChirp();
    vTaskDelay(300 / portTICK_PERIOD_MS);

    PlayChirp(NOTE_FS4, 40);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    OffChirp();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    PlayChirp(NOTE_D4, 40);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    OffChirp();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    PlayChirp(NOTE_D4, 40);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    OffChirp();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    PlayChirp(NOTE_D4, 40);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    OffChirp();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}


