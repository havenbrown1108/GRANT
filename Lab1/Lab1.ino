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

// from https://pianoletternotes.blogspot.com/2017/10/megalovania-undertale-theme.html
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

// from https://pianoletternotes.blogspot.com/2017/11/mii-channel-theme.html
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