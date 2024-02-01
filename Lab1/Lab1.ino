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
#include <Arduino_FreeRTOS.h>
#include "RingoHardware.h"

//int i;    //declaire any global variables here

void TaskBlinkRandomly( void *pvParameters );
void TaskSing( void *pvParameters );
void playNote(int note);
void restNote(int restLength);

int noteLength = 150;
int volume = 40;


void setup(){
  HardwareBegin();        //initialize Ringo's brain to work with his circuitry
  SwitchButtonToPixels(); //set shared line to control NeoPixel lights
  // PlayStartChirp();       //Play startup chirp and blink eyes
  SwitchMotorsToSerial(); //Call "SwitchMotorsToSerial()" before using Serial.print functions as motors & serial share a line
  RxIRRestart(4);         //wait for 4 byte IR remote command
  IsIRDone();
  GetIRButton();
  RestartTimer();

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }

  // Now set up two tasks to run independently.
  // xTaskCreate(
  // TaskBlinkRandomly
  // ,  (const portCHAR *)"Blink"   // A name just for humans
  // ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
  // ,  NULL
  // ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
  // ,  NULL );

  xTaskCreate(
  TaskSing
  ,  (const portCHAR *)"Sing"   // A name just for humans
  ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
  ,  NULL
  ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
  ,  NULL );



}

void loop(){}

void TaskBlinkRandomly(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  for (;;) // A Task shall never return or exit.
  {
    RandomEyes();
    vTaskDelay( 500 / portTICK_PERIOD_MS ); 
    OffEyes();
    vTaskDelay( 500 / portTICK_PERIOD_MS ); 
  }
}

void TaskSing(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  byte button;
  for (;;) // A Task shall never return or exit.
  {
    RandomEyes();
    vTaskDelay( 500 / portTICK_PERIOD_MS ); 
    OffEyes();

    if (IsIRDone()) {
      button = GetIRButton();
      if (button) {
        switch (button) {
          case 1:
          OffEyes();
          PlayMegalovania();
          RxIRRestart(4);
          break;

          case 2:
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
    vTaskDelay( 500 / portTICK_PERIOD_MS );
  }
}

// Put source of sheet music
void PlayMegalovania() {
  SetAllPixelsRGB(100,0,0);

  MegFirstStanza();
  MegSecondStanza();
  MegThirdStanza();
  MegFourthStanza();
  MegFifthStanza();
  // MegSixthStanza();
  // MegSeventhStanza();

  SetAllPixelsRGB(0,0,0);
}

void MegFirstStanza() {
  playNote(NOTE_D4);
  playNote(NOTE_D4);
  playNote(NOTE_D5);

  restNote(1);

  playNote(NOTE_A4);

  restNote(2);

  playNote(NOTE_GS4);

  restNote(1);

  playNote(NOTE_G4);

  restNote(1);

  playNote(NOTE_F4);

  restNote(1);

  playNote(NOTE_D4);
  playNote(NOTE_F4);
  playNote(NOTE_G4);
  playNote(NOTE_C4);
  playNote(NOTE_C4);
  playNote(NOTE_D5);

  restNote(1);

  playNote(NOTE_A4);

  restNote(2);

  playNote(NOTE_GS4);

  restNote(1);

  playNote(NOTE_G4);

}

void MegSecondStanza() {
  restNote(1);

  playNote(NOTE_F4);

  restNote(1);

  playNote(NOTE_D4);
  playNote(NOTE_F4);
  playNote(NOTE_G4);
  playNote(NOTE_B3);
  playNote(NOTE_B3);
  playNote(NOTE_D5);

  restNote(1);

  playNote(NOTE_A4);

  restNote(2);

  playNote(NOTE_GS4);
  
  restNote(1);

  playNote(NOTE_G4);
  
  restNote(1);

  playNote(NOTE_F4);

  restNote(1);

  playNote(NOTE_D4);
  playNote(NOTE_F4);
  playNote(NOTE_G4);
  playNote(NOTE_AS3);
  playNote(NOTE_AS3);
  playNote(NOTE_D5);

  restNote(1);

}

void MegThirdStanza() {
  playNote(NOTE_A4);

  restNote(2);

  playNote(NOTE_GS4);

  restNote(1);
  
  playNote(NOTE_G4);
  
  restNote(1);

  playNote(NOTE_F4);
  
  restNote(1);

  playNote(NOTE_D4);
  playNote(NOTE_F4);
  playNote(NOTE_G4);
  playNote(NOTE_D4);
  playNote(NOTE_D4);
  playNote(NOTE_D5);

  restNote(1);

  playNote(NOTE_A4);

  restNote(2);

  playNote(NOTE_GS4);

  restNote(1);
  
  playNote(NOTE_G4);
  
  restNote(1);

  playNote(NOTE_F4);
  
  restNote(1);

  playNote(NOTE_D4);
}

void MegFourthStanza(){
  playNote(NOTE_F4);
  playNote(NOTE_G4);
  playNote(NOTE_C4);
  playNote(NOTE_C4);
  playNote(NOTE_D5);

  restNote(1);

  playNote(NOTE_A4);

  restNote(2);

  playNote(NOTE_GS4);

  restNote(1);
  
  playNote(NOTE_G4);
  
  restNote(1);

  playNote(NOTE_F4);
  
  restNote(1);

  playNote(NOTE_D4);
  playNote(NOTE_F4);
  playNote(NOTE_G4);
  playNote(NOTE_B3);
  playNote(NOTE_B3);
  playNote(NOTE_D5);

  restNote(1);

  playNote(NOTE_A4);

  restNote(2);

  playNote(NOTE_GS4);
}

void MegFifthStanza(){
  restNote(1);

  playNote(NOTE_G4);
  
  restNote(1);

  playNote(NOTE_F4);

  restNote(1);

  playNote(NOTE_D4);
  playNote(NOTE_F4);
  playNote(NOTE_G4);
  playNote(NOTE_AS3);
  playNote(NOTE_AS3);
  playNote(NOTE_D5);

  restNote(1);

  playNote(NOTE_A4);

  restNote(2);

  playNote(NOTE_GS4);

  restNote(1);
  
  playNote(NOTE_G4);
  
  restNote(1);

  playNote(NOTE_F4);
  
  restNote(1);

  playNote(NOTE_D4);
  playNote(NOTE_F4);
  playNote(NOTE_G4);
  playNote(NOTE_D4);
  playNote(NOTE_D4);
}

void MegSixthStanza(){
  playNote(NOTE_D5);

  restNote(1);

  playNote(NOTE_A4);

  restNote(2);

  playNote(NOTE_GS4);

  restNote(1);
  
  playNote(NOTE_G4);
  
  restNote(1);

  playNote(NOTE_F4);
  
  restNote(1);

  playNote(NOTE_D4);
  playNote(NOTE_F4);
  playNote(NOTE_G4);
  playNote(NOTE_C4);
  playNote(NOTE_C4);
  playNote(NOTE_D5);

  restNote(1);

  playNote(NOTE_A4);

  restNote(2);

  playNote(NOTE_GS4);

  restNote(1);
  
  playNote(NOTE_G4);
  
  restNote(1);

  playNote(NOTE_F4);
}

void MegSeventhStanza(){
  restNote(1);

  playNote(NOTE_D4);
  playNote(NOTE_F4);
  playNote(NOTE_G4);
  playNote(NOTE_B3);
  playNote(NOTE_B3);
  playNote(NOTE_D5);

  restNote(1);

  playNote(NOTE_A4);

  restNote(2);

  playNote(NOTE_GS4);

  restNote(1);
  
  playNote(NOTE_G4);
  
  restNote(1);

  playNote(NOTE_F4);
  
  restNote(1);

  playNote(NOTE_D4);
  playNote(NOTE_F4);
  playNote(NOTE_G4);
  playNote(NOTE_AS3);
  playNote(NOTE_AS3);
  playNote(NOTE_D5);

  restNote(1);

  playNote(NOTE_A4);
  
  restNote(1);
}

// Put source of sheet music
void PlayMiiSong() {
  SetAllPixelsRGB(0,100,0);

  MiiFirstStanza();
  MiiSecondStanza();
  MiiThirdStanza();
  MiiFourthStanza();

  SetAllPixelsRGB(0,0,0);
}

void MiiFirstStanza() {
  playNote(NOTE_FS4);

  restNote(3);

  playNote(NOTE_A4);

  restNote(1);

  playNote(NOTE_CS5);

  restNote(3);

  playNote(NOTE_A4);

  restNote(3);

  playNote(NOTE_FS4);

  restNote(1);

  playNote(NOTE_D4);

  restNote(1);

  playNote(NOTE_D4);

  restNote(1);

  playNote(NOTE_D4);

  restNote(5);
}

void MiiSecondStanza() {
  restNote(8);
  
  playNote(NOTE_FS4);
  
  restNote(1);

  playNote(NOTE_A4);

  restNote(1);

  playNote(NOTE_CS5);

  restNote(3);

  playNote(NOTE_A4);

  restNote(3);

  playNote(NOTE_FS4);

  restNote(1);

  playNote(NOTE_E5);

  restNote(3);
}

void MiiThirdStanza() {
  restNote(2);

  playNote(NOTE_DS5);

  restNote(1);

  playNote(NOTE_D5);

  restNote(7);

  playNote(NOTE_GS4);

  restNote(3);

  playNote(NOTE_CS5);

  restNote(1);

  playNote(NOTE_FS4);

  restNote(3);

  playNote(NOTE_CS5);

  restNote(3);
}

void MiiFourthStanza() {
  playNote(NOTE_GS4);
  restNote(3);
  playNote(NOTE_CS5);
  restNote(3);
  playNote(NOTE_G4);
  restNote(1);
  playNote(NOTE_FS4);
  restNote(3);
  playNote(NOTE_E4);
  restNote(3);
  playNote(NOTE_E4);
  restNote(1);
  playNote(NOTE_E4);
  restNote(1);
  playNote(NOTE_E4);
  restNote(7);
  playNote(NOTE_E4);
  restNote(1);
  playNote(NOTE_E4);
  restNote(1);
  playNote(NOTE_E4);
}

void playNote(int note){
  PlayChirp(note, volume);
  vTaskDelay(noteLength / portTICK_PERIOD_MS);
  OffChirp();
}

void restNote(int restLength) {
  vTaskDelay( noteLength * restLength/ portTICK_PERIOD_MS );
}