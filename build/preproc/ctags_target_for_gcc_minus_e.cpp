# 1 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\GrantsCode.ino"
/* 

Lab4



*/
# 5 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\GrantsCode.ino"
# 6 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\GrantsCode.ino" 2
# 7 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\GrantsCode.ino" 2


void TaskDriveForwardController(void *pvParameters);
void TaskTurnController(void *pvParameters);
void TaskSensing(void *pvParameters);


// Globals
TaskHandle_t xControllerHandle;
// TaskHandle_t xDoSquaresHandle;
// TaskHandle_t xPlanningAndGuidanceHandle;
TaskHandle_t xSensingHandle;
// TaskHandle_t xAvoidObstacleHandle;
TaskHandle_t xNavigateMazeHandle;

float guidancePeriod = 200;
float controllerPeriod = 150;
int edgeDetectorPeriod = 100;

int intendedHeading;
int error = 0;



int baseSpeed = 30;
int maxSpeed = 100;
int motorBias = 5; //Our left motor is stronger than our right motor so this should account for that
int turnAdjustment = 0;

int rightTurnAngle = 71;
int leftTurnAngle = -67;
bool turnComplete = false;
// bool turningRight = true;
// int turnAngle = rightTurnAngle;

int numTurns;

float Kp = 1;
float Ki = 1;
float Kd = 1;
// bool newManueverDetected;

char edge;

enum Manuever { DriveStraight, Backup, TurningRight, TurningLeft};
Manuever manuever;


void setup(){
  HardwareBegin(); //initialize Ringo's brain to work with his circuitry
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

  ResetLookAtEdge();

  xTaskCreate(
  TaskNavigateMaze
  , (const char *)"maze guidance task"
  , 128
  , 
# 75 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\GrantsCode.ino" 3 4
    __null
  
# 76 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\GrantsCode.ino"
 , 3
  , &xNavigateMazeHandle );

  xTaskCreate(
  TaskController
  , (const char *)"Drive in direction of intended heading"
  , 128
  , 
# 83 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\GrantsCode.ino" 3 4
    __null
  
# 84 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\GrantsCode.ino"
 , 2
  , &xControllerHandle );

  xTaskCreate(
  TaskSensing
  , (const char *)"Check for edges"
  , 128
  , 
# 91 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\GrantsCode.ino" 3 4
    __null
  
# 92 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\GrantsCode.ino"
 , 3
  , &xSensingHandle );

  edge = 0x0;
  Serial.println("setup");

  intendedHeading = PresentHeading();
  manuever = DriveStraight;
}

void loop(){}

void TaskNavigateMaze(void *pvParameters) {
  bool drivingStraight = true;
  unsigned long startTime = millis();
  // int drivingStraightTime = 5000;
  Manuever currentManuever = manuever;
  int backingupTimeLimit = 1000;
  unsigned long time = millis() - startTime;
  Serial.println("NavigateMaze");
  char lastEdgeSeen = 0x0;

  for(;;) {
    // Serial.println("start" + time);
    // Serial.print("manuever is ");
    // Serial.println(manuever);

    time = millis() - startTime;
    SetPixelRGB(0, 0, 0 ,0);
    // vTaskDelay(10)
    // Serial.println("error at start of for loop in guidance = " + error);


    // Logic for when to change state
    switch (manuever)
    {
      case DriveStraight:
        SetPixelRGB(0, 0, 0, 100);
        // Serial.println("driving straight part one....");
        if(((edge)&(0x01 | 0x02 | 0x10 | 0x20))) {
          manuever = Backup;
          lastEdgeSeen = edge;
          startTime = millis();
          // newManueverDetected = true;
        }
        break;
      case Backup:
        SetPixelRGB(0, 0, 100, 0);
        // Serial.println("backing up part one....");
        // Serial.println(time);
        bool timeLimitReached = (time >= backingupTimeLimit) ? true : false;
        if(timeLimitReached) {
          // Serial.println("time limit reached");
          Serial.println(lastEdgeSeen, 16);
        }
        if(timeLimitReached && ((lastEdgeSeen)&(0x01 | 0x02))) {
          Serial.println("right edge");
          manuever = TurningLeft;
          lastEdgeSeen = 0x0;
          // newManueverDetected = true;
          // turnComplete = false;
        }
        else if(timeLimitReached && ((lastEdgeSeen)&(0x10 | 0x20))) {
          manuever = TurningRight;
          lastEdgeSeen = 0x0;
          // startTime = millis();
          // newManueverDetected = true;
          // turnComplete = false;
        }
        break;
      case TurningRight:
        SetPixelRGB(0, 100, 0, 0);
        if(error == 0) {
          manuever = DriveStraight;
          // newManueverDetected = true;
        }
        break;
      case TurningLeft:
        SetPixelRGB(0, 100, 100, 0);
        // Serial.println("turning left part one...");
        if(error == 0) {
          manuever = DriveStraight;
          // newManueverDetected = true;
        }
        break;
      default:
        break;
    }

    // Logic for how to change state
    if(currentManuever != manuever) {
      currentManuever = manuever;
      switch (manuever)
      {
        case DriveStraight:
          Serial.println("manuever is now DriveStraight");
          intendedHeading = PresentHeading();
          Kp = 1;
          Ki = 1;
          Kd = 1;
          baseSpeed = 30;
          turnAdjustment = 0;
          // error = 0;
          break;
        case Backup:
          Serial.println("manuever is now Backup");
          intendedHeading = PresentHeading();
          Kp = 1;
          Ki = 1;
          Kd = 1;
          baseSpeed = -30;
          turnAdjustment = 0;
          // error = 0;
          break;
        case TurningRight:
          Serial.println("manuever is now TurningRight");
          intendedHeading = PresentHeading() + (rightTurnAngle / 2);
          Kp = 0.4;
          Ki = 0;
          Kd = 1;
          baseSpeed = 15;
          // error = 0;
          turnAdjustment = baseSpeed;
          break;
        case TurningLeft:
          Serial.println("manuever is now TurningLeft");
          intendedHeading = PresentHeading() + (leftTurnAngle / 2);
          Kp = 0.4;
          Ki = 0;
          Kd = 1;
          baseSpeed = 15;
          // error = 0;
          turnAdjustment = baseSpeed;
          break;
        default:
          break;
      }
    }

    vTaskDelay(guidancePeriod / ( (TickType_t) 
# 231 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\GrantsCode.ino" 3
                               (1 << (0 
# 231 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\GrantsCode.ino"
                               /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 231 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\GrantsCode.ino" 3
                               )) 
# 231 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\GrantsCode.ino"
                               ) /* Inaccurately assuming 128 kHz Watchdog Timer.*/);
  }

}



void TaskController(void *pvParameters) {
  int P, I = 0, D = 0;
  int currentHeading;
  // int error = 0;
  int lastError = 0;
  int speedLeft = 50;
  int speedRight = 50;
  Manuever currentManuever;

  int u;

  for(;;) {

    if(currentManuever != manuever) {
      currentManuever = manuever;
      OffEyes();
      // error = 0;
      lastError = 0;
      P = 0;
      I = 0;
      D = 0;
      // newManueverDetected = false;
    }

    SimpleGyroNavigation();
    currentHeading = PresentHeading();

    error = intendedHeading - currentHeading;
    // if(error == 0) {
    //   Serial.println("error is 0 in the controller task!");

    // }

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
      speedLeft = ((baseSpeed - motorBias - turnAdjustment)>(0)?(baseSpeed - motorBias - turnAdjustment):(0));
      speedRight = ((baseSpeed - turnAdjustment)>(0)?(baseSpeed - turnAdjustment):(0));
      // turnComplete = true;
    }

    lastError = error;

    speedLeft = ((speedLeft)<(maxSpeed)?(speedLeft):(maxSpeed));
    speedRight = ((speedRight)<(maxSpeed)?(speedRight):(maxSpeed));
    // Motors(speedLeft, speedRight);
    vTaskDelay(controllerPeriod / ( (TickType_t) 
# 301 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\GrantsCode.ino" 3
                                 (1 << (0 
# 301 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\GrantsCode.ino"
                                 /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 301 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\GrantsCode.ino" 3
                                 )) 
# 301 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\GrantsCode.ino"
                                 ) /* Inaccurately assuming 128 kHz Watchdog Timer.*/);
  }
}

void TaskSensing(void *pvParameters) {
    // TickType_t xLastWakeTime;
    // xLastWakeTime = xTaskGetTickCount();
    Serial.println("sensing task");

    for(;;)
    {
        edge = LookForEdge();

        vTaskDelay( edgeDetectorPeriod / ( (TickType_t) 
# 314 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\GrantsCode.ino" 3
                                        (1 << (0 
# 314 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\GrantsCode.ino"
                                        /* portUSE_WDTO to use the Watchdog Timer for xTaskIncrementTick*/ + 4
# 314 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\GrantsCode.ino" 3
                                        )) 
# 314 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\GrantsCode.ino"
                                        ) /* Inaccurately assuming 128 kHz Watchdog Timer.*/);
    }
}

// This is an aperiodic task that spins the lil guy
// void TaskAvoidObstacle(void *pvParameters)
// {
//   for(;;)
//   {
//     // Stop and chirp to alert an edge detection
//     Motors(0,0);

//     // Back up
//     Motors(-50,-50);
//     vTaskDelay(1000 / portTICK_PERIOD_MS);
//     Motors(0,0);

//     // Turn
//     if (RightFrontEdgeDetected(edge)) {
//         Motors(50, 0);
//         vTaskDelay(500 / portTICK_PERIOD_MS);
//     } else if (LeftFrontEdgeDetected(edge)) {
//         Motors(50, 0);
//         vTaskDelay(500 / portTICK_PERIOD_MS);
//     }


//     // Continue forward
//     Motors(50,50);

//     vTaskSuspend(xAvoidObstacleHandle);
//     // vTaskResume(xSensingHandle);
//   }
// }
# 1 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\FunStuff.ino"
/*



Ringo Robot:  Animation  Rev01.02  12/2015



This code includes various "animation" things Ringo can do. For example, sounds,

lighting sequences, dance moves, etc. Users are encouraged to build their own

"animations" and share their work on the forum.  http://forum.plumgeek.com/



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
# 22 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\FunStuff.ino"
# 23 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\FunStuff.ino" 2


// ***************************************************
// Short Animations
// ***************************************************


void PlayStartChirp(void){//Ver. 1.0, Kevin King   
  // This is the startup sequence that plays when Ringo is reset(put in begin())
  PlayChirp(1000, 50);
  OnEyes(0,0,50); // blue
  delay(50);
  PlayChirp(2000, 50);
  delay(50);
  PlayChirp(3000, 50);
  delay(50);
  PlayChirp(4000, 50);
  delay(100);
  OffChirp();
  OffEyes();
}


void PlayAck(void){//Ver. 1.0, Kevin King
  PlayChirp(2093,100);
  SetPixelRGB(3,0,30,0);//RefreshPixels();
  delay(50);
  OffChirp();
  SetPixelRGB(3,0,0,0);//RefreshPixels();
}


void PlayNonAck(void){//Ver. 1.0, Kevin King
  PlayChirp(1109,100);
  SetPixelRGB(3,30,0,0);//RefreshPixels();
  delay(75);
  OffChirp();
  SetPixelRGB(3,0,0,0);//RefreshPixels();
}


void PlayAnger(void){//Ver. 1.0, Dustin Soodak
  char i;
  for(i=0;i<10;i++){
    PlayChirp(1047,100);
    delay(10);
    PlayChirp(1109,100);
    delay(10);
  }
  OffChirp();
}


void PlayBoredom(void){//Ver. 1.0, Dustin Soodak
  char i;
  unsigned int freq,dfreq;
  dfreq=(1245 -1047)>>4;
  PlayChirp(1047,100);
  delay(50);
  for(i=0;i<16;i++){
    PlayChirp(1047 +dfreq*i,100);
    delay(10);
  }
  PlayChirp(1245,100);
  delay(100);
    for(i=0;i<16;i++){
    PlayChirp(1245 -dfreq*i,100);
    delay(10);
  }
  PlayChirp(1047,100);
  delay(50);
  OffChirp();
}


void PlayExcited(void){//Ver. 1.0, Dustin Soodak, Kevin King (note sequence)
  PlayChirp(1568,100);
  delay(100);
  PlayChirp(2093,100);
  delay(100);
  PlayChirp(1760,100);
  delay(100);
  PlayChirp(2093,100);
  delay(100);
  PlayChirp(1976,100);
  delay(100);
  PlayChirp(1568,100);
  delay(100);
  OffChirp();
}


// ***************************************************
// end Short Animations
// ***************************************************


// ***************************************************
// Chirp, Sound, Piezo Core Functions
// ***************************************************

void PlaySweep(int StartNote, int EndNote, int DwellTime){//Ver. 1.0, Kevin King
  if(StartNote<EndNote){
     for(StartNote; StartNote<=EndNote; StartNote++){
      PlayChirp(StartNote,100);
      delayMicroseconds(DwellTime);
     }
  }
   else{
     for(StartNote; StartNote>=EndNote; StartNote--){
      PlayChirp(StartNote,100);
      delayMicroseconds(DwellTime);
     }
  }
 OffChirp();
}

void PlayChirp(unsigned int Frequency, unsigned int Amplitude){//Ver. 1.0, Dustin Soodak
  //Frequency is in Hz
  //Amplitude is in units of UsOn
  uint16_t temp;
  uint16_t period,dutycycle;
  uint8_t prescalerbits;
  if(Amplitude>100)
    Amplitude=100;
  if (8000000L/Frequency/2>=0x10000){
    if(8000000L/Frequency/2>=0x10000*8){
        prescalerbits=0b011;//prescaler 64
        period=8000000L/Frequency/(2*16);
        dutycycle=8000000L/1000000*Amplitude/2/64;
    }
    else{
      prescalerbits = 0b010;// prescaler 8
      period=8000000L/Frequency/(2*8);
      dutycycle=8000000L/1000000*Amplitude/2/8;
    }
  }
  else{
    prescalerbits = 0b001; //on but no prescaling
    period=8000000L/Frequency/(2*1);
    dutycycle=8000000L/1000000*Amplitude/2/1;
  }
  if(Amplitude==0){
    
# 166 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\FunStuff.ino" 3
   (*(volatile uint8_t *)(0x80))
# 166 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\FunStuff.ino"
         =0;
    
# 167 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\FunStuff.ino" 3
   (*(volatile uint8_t *)(0x81))
# 167 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\FunStuff.ino"
         =0;
    pinMode(9 /*tone(pin, frequency) and noTone(),  or tone(pin, frequency, duration). also look at toneAC library*/,0x0);

    //Serial.println("off");    // debugging code
  }
  else{
    
# 173 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\FunStuff.ino" 3
   (*(volatile uint8_t *)(0x81))
# 173 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\FunStuff.ino"
         &=~0b00000111;//turn off timer
    
# 174 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\FunStuff.ino" 3
   (*(volatile uint16_t *)(0x86))
# 174 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\FunStuff.ino"
       =period;
    
# 175 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\FunStuff.ino" 3
   (*(volatile uint16_t *)(0x88))
# 175 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\FunStuff.ino"
        =dutycycle;
    
# 176 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\FunStuff.ino" 3
   (*(volatile uint8_t *)(0x80)) 
# 176 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\FunStuff.ino"
          = (0b10<<6) | 0b10;//COM1A1 COM1A0, and WGM11 WGM10
    
# 177 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\FunStuff.ino" 3
   (*(volatile uint8_t *)(0x81)) 
# 177 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\FunStuff.ino"
          = (0b10<<3) | prescalerbits;//WGM13 WGM12, and off/(on with prescaler)
    pinMode(9 /*tone(pin, frequency) and noTone(),  or tone(pin, frequency, duration). also look at toneAC library*/,0x1);//DDRB|=1;//make PortB pin1 an output 
  }
}

void OffChirp(void){//Ver. 1.0, Kevin King              //stops chirp tone
  PlayChirp(0,0); //turn off chirp
}

void OffPixels(void){//Ver. 1.0, Kevin King              //turns off all pixels
  SetAllPixelsRGB(0,0,0); //set all pixels to off
}

void OffPixel(byte Pixel){//Ver. 1.0, Kevin King    //turns off a specific pixel
  SetPixelRGB(Pixel,0,0,0); //set this pixel to off
}

void OnEyes(byte Red, byte Green, byte Blue){//Ver. 1.0, Kevin King     //makes eyes the given color, automatically calls RefreshPixels()
  SetPixelRGB(4,Red,Green,Blue); //set right eye
  SetPixelRGB(5,Red,Green,Blue); //set left eye
}

void LeftEye(byte Red, byte Green, byte Blue){//Ver. 1.0, Kevin King     //makes left eye the given color, automatically calls RefreshPixels()
  SetPixelRGB(5,Red,Green,Blue); //set left eye
}

void RightEye(byte Red, byte Green, byte Blue){//Ver. 1.0, Kevin King     //makes right eye the given color, automatically calls RefreshPixels()
  SetPixelRGB(4,Red,Green,Blue); //set right eye
}

void OffEyes(void){//Ver. 1.0, Kevin King                //turns off eye pixels
  SetPixelRGB(4,0,0,0); //blank right eye
  SetPixelRGB(5,0,0,0); //blank left eye
}

void RandomEyes(void){//Ver. 1.0, Kevin King             //Sets the pair of eyes to a random color
  int red, green, blue;
  red = random(120);
  green = random(120);
  blue = random(120);
  OnEyes(red,green,blue);
}

// ***************************************************
// end Chirp, Sound, Piezo Core Functions
// ***************************************************



// ***************************************************
// Additional Notes:  Chirp / Sound / Piezo
// ***************************************************

  //Terms:
  //TCNT1 is counter
  // goes from BOTTOM = 0x0000 to MAX = 0xFFFF in normal mode.
  //TOP can be either OCR1A or ICR1
  //
  //Table 15-4. Waveform Generation Mode Bit Description(16 different pwm modes, 2 of which are phase & freq correct)
  //Want "phase and frequency correct PWM mode" on p.130
  //Mode is set by WGM 13:0 bits.
  //Mode WGM: 13 12 11 10
  //10         1  0  1  0 PWM, Phase Correct, TOP is ICR1  [so can use OCR1A as duty cycle and OC1A as pin output]
  //
  //TCNT1 goes from BOTTOM to TOP then TOP to BOTTOM
  //TOP can be OCR1A or ICR1
  //Output Compare OC1x cleared when TCNT1=OCR1x while upcounting, and set when down counting.
  //
  // "The Output Compare Registers contain a 16-bit value that is continuously compared with the
  //counter value (TCNT1). A match can be used to generate an Output Compare interrupt, or to
  //generate a waveform output on the OC1x pin."
  //
  //Table 15-3. Compare Output Mode, when in Phase Correct and Phase and Frequency Correct mode
  //(what these bits do depends on what WGM 13:0 are set to)
  //COM1A1 COM1A0
  //     1      0    Clear OC1A on Compare Match when upcounting. Set OC1AB on Compare Match when downcounting.
  //If we want this single pin mode, can't use OCR1A for TOP since will be used for duty cycle of PWM
  //
  //"...note that the Data Direction Register (DDR) bit corresponding
  //to the OC1A or OC1B pin must be set in order to enable the output driver."
  //
  //CS12 CS11 CS10
  //off mode(all 0), set prescvaler, or external clock source
  //
  //COM1A1 COM1A0 are bits 7:6 in                   TCCR1A
  //data direction register: bit 1 of               DDRB 
  //WGM13 WGM12 are bits 4:3 in                     TCCR1B
  //WGM11 WGM10 are bits 1:0 in                     TCCR1A
  //CS12,11,10 are bits 2:0 in                      TCCR1B
  //total count (/2-1 to get period of waveform)is  ICR1
  //duty cycle (/2 to get period it is on) is       OCR1A
  //
  //TCCR1A=(TCCR1A&0b00111111)|0b01000000;
  //
  //
  //If we had the pwm output to the chirper on another pin:
  //"Using the ICR1 Register for defining TOP works well when using fixed TOP values. By using
  //ICR1, the OCR1A Register is free to be used for generating a PWM output on OC1A. However,
  //if the base PWM frequency is actively changed by changing the TOP value, using the OCR1A as
  //TOP is clearly a better choice due to its double buffer feature."
  //
  //Another doc for R/W of 16 bit registers: 
  //http://www.embedds.com/programming-16-bit-timer-on-atmega328/

// ***************************************************
// end Additional Notes:  Chirp / Sound / Piezo
// ***************************************************
# 1 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\Navigation.ino"
/*



Ringo Robot:  Navigation  Rev01.02  12/2015



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
# 18 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\Navigation.ino"
# 19 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\Navigation.ino" 2

//Ver. 1.0, Dustin Soodak

// ***************************************************
// Ringo I2C
// ***************************************************

# 27 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\Navigation.ino" 2
//put "RingoWireLibrary\RingoWire" into "C:\Program Files\Arduino\libraries"

//Accelerometer:
//http://cache.freescale.com/files/sensors/doc/data_sheet/MMA8451Q.pdf
//p.18
//
//Gyroscope:
//http://www.st.com/web/en/resource/technical/document/datasheet/DM00036465.pdf
//p.23-24
//
//Gyro: "Use same format for read/write single/multiple bytes"
//Accel: "The MMA8451Q automatically increments the received register address commands after a write command is received"


void I2CBegin(void){//Ver. 1.0, Dustin Soodak
  Wire.begin();
}

uint8_t I2CReadRegs(uint8_t Device, uint8_t Reg, uint8_t *RxBuffer, uint8_t Length){//Ver. 1.0, Dustin Soodak
  uint8_t i;
  Wire.beginTransmission(Device); // transmit to device (note: actually just starts filling of buffer)
  Wire.write(Reg);
  Wire.endTransmission(0);//send data without stop at end
  Wire.requestFrom(Device, Length);
  i=0;
  while(Wire.available()){
    RxBuffer[i]=Wire.read();
    i++;
  }
  return i;
}

uint8_t I2CReadReg(uint8_t Device, uint8_t Reg){//Ver. 1.0, Dustin Soodak
  uint8_t dat=0;
  I2CReadRegs(Device, Reg, &dat, 1);
  return dat;
}

void I2CWriteRegs(uint8_t Device, uint8_t Reg, uint8_t *TxBuffer, uint8_t Length){//Ver. 1.0, Dustin Soodak
  char i;
  Wire.beginTransmission(Device); // transmit to device (note: actually just starts filling of buffer)
  Wire.write(Reg); // sends one byte 
  for(i=0;i<Length;i++){
    Wire.write(TxBuffer[i]);
  }
  Wire.endTransmission(); //send data with stop at end 
}

void I2CWriteReg(uint8_t Device, uint8_t Reg, uint8_t TxData){//Ver. 1.0, Dustin Soodak
  I2CWriteRegs( Device, Reg, &TxData, 1);
}

// ***************************************************
// end Ringo I2C
// ***************************************************




// ***************************************************
// Navigation
// ***************************************************

float GyroscopeCalibrationMultiplier=1.0;

char IsStationary=1;//used in NavigationXY()
int NonStationaryValue=0;
char NonStationaryAxis=0;
char IsStationaryCount=0;


char XYMode=0;//added to tell GetAccelerationX(), etc. which navigation function was called: SimpleNavigation() or NavigationXY().

char GyroFifoOverflow=0;
int GyroZeroes[3]={0,0,0};
char AccelFifoOverflow=0;
int AccelZeroes[3]={0,0,0};
char NavigationOutOfRange=0;

char PauseNavigationInterrupt=1;
char NavigationOn=0;
int32_t count;

extern volatile char IRReceiving;
int nav_data[3];//made global so compiler doesn't optimize it out of code
int nav_accel[3];
int nav_data3[3];

signed char GyroEdgeDetection=0;
int GyroEdgeDetectionLevel=100;
char GyroEdgeDetectionRepeats=10;
char GyroEdgeDetected;


void ZeroGyroEdgeDetection(void){//Ver. 1.0, Dustin Soodak
  //works but already too late by the time edge is detected
  GyroEdgeDetection=0;
}

void UpdateGyroEdgeDetection(int GyroYAxisRawZeroed){//Ver. 1.0, Dustin Soodak
  //works but already too late by the time edge is detected
  if(!GyroEdgeDetected){
    if(GyroYAxisRawZeroed>10){
      if(GyroEdgeDetection>=0)
        GyroEdgeDetection=-1;
      else
        GyroEdgeDetection--;
      if(GyroEdgeDetection<-GyroEdgeDetectionLevel){
        GyroEdgeDetection=0;//so ready for next time when GyroEgeDetected set to 0
        GyroEdgeDetected=2;//bit 1: left
      }
    }
    else if(GyroYAxisRawZeroed<-10){
      if(GyroEdgeDetection<=0)
        GyroEdgeDetection=1;
      else
        GyroEdgeDetection++;
      if(GyroEdgeDetection>GyroEdgeDetectionLevel){
        GyroEdgeDetection=0;//so ready for next time when GyroEgeDetected set to 0
        GyroEdgeDetected=1;//bit 0: right
      }
    }
  }//end if(!GyroEdgeDetected)
}//end UpdateGyroEdgeDetection

void SimpleGyroNavigation(void){//Ver. 1.0, Dustin Soodak
  char n;
  //int nav_data[3];//Compiler error:  when nav_data[] local, it was not actually reserving space for this since it isn't explicitly set (is set by pointers)
  char i,j;
  ConvertNavigationCoordinates(0);
  if(IRReceiving){
    if(IsIRDone())
     IRReceiving=0;
  }
  if(!PauseNavigationInterrupt && !IRReceiving){

    //Get Gyroscope data
    n=GyroBufferSize();
    //if(n>0)
    //digitalWrite(Light_Bus_BTN1,0);
    for(i=0;i<n;i++){
      GyroGetAxes(nav_data);
      //digitalWrite(Light_Bus_BTN1,0);
      for(j=1;j<3;j++){//just y & z axis      
        GyroVelocity[j]=((int32_t)nav_data[j])-GyroZeroes[j];
        GyroPosition[j]+=(GyroVelocity[j]);
      }
      //UpdateGyroEdgeDetection(GyroVelocity[1]);//works but already too late by the time edge is detected

    }//end for(i=0;i<n;i++)
    //get current rotational velocity for x & y axes
    for(j=0;j<2;j++){
      GyroVelocity[j]=((int32_t)nav_data[j])-GyroZeroes[j];
    }
    if(n>=31)
      GyroFifoOverflow=1;
  }
}
int32_t AverGyroVelocity;

int AccelPositionXOffset=0,AccelPositionYOffset;
//Can remove this feature by commenting out body of
//this function(replace with "XYMode=NewXYMode;")
//and removing AccelPositionYOffset from GetPositionY().
void ConvertNavigationCoordinates(char NewXYMode){//Ver. 1.0, Dustin Soodak
  float Theta,conversion;
  if(XYMode==(!NewXYMode)){
    Theta=((float)90-GetDegrees())*3.14159/180;//(((float)GyroPosition[2])*((0.0000355*2000))*3.14159/180);
    conversion=(-((float)32768)*380*380/(2*9800));//since GetPositionX() in 
    //XYMode is ((float)AccelPosition[0])*(-(float)(2*9800)/32768/380/380)+AccelPositionXOffset;
    //GetPositionY() is same in XYMode but is ((float)AccelPosition[1])*(-(float)(2*9800)/32768/400) 
    //when using SimpleNagivation()
    AccelPositionXOffset=cos(Theta)*30 /*30 mm forward from center between wheels*/;
    AccelPositionYOffset=sin(Theta)*30 /*30 mm forward from center between wheels*/;
    if(NewXYMode){
      AccelPosition[1]=((float)AccelPosition[1])*(((float)380)*380/400);//scale to XYMode
      AccelPosition[1]+=((float)AccelPositionYOffset)*conversion;//add offset    
    }
    else{
      AccelPosition[1]-=((float)AccelPositionYOffset)*conversion;//get rid of offset
      AccelPosition[1]=((float)AccelPosition[1])*(((float)400)/380/380);//scale to Simple mode
    }
    XYMode=NewXYMode;
  }//if XY mode changed

}


void SimpleNavigation(void){//Ver. 1.0, Dustin Soodak
  //can't be put in interrupt since I2C functions use an interrupt(update: new I2C functions don't use an interrupt)
  char i,j,n,gn,an;
  //int nav_data[3];//Compiler error:  when nav_data[] local, it was not actually reserving space for this since it isn't explicitly set (is set by pointers)
  int32_t AccelPositionNewDat[3]={0,0,0};
  int32_t AccelVelocityNewDat[3]={0,0,0};
  int32_t GyroPositionNewDat[3]={0,0,0};
  AverGyroVelocity=0;
  count++;
  ConvertNavigationCoordinates(0);


  if(IRReceiving){
    if(IsIRDone())
     IRReceiving=0;
  }
  if(!PauseNavigationInterrupt && !IRReceiving){

    //Get Gyroscope data
    gn=GyroBufferSize();

    for(i=0;i<gn;i++){
      GyroGetAxes(nav_data);

      for(j=1;j<3;j++){//just y,z axis      
        GyroVelocity[j]=((int32_t)nav_data[j])-GyroZeroes[j];
        GyroPosition[j]+=(GyroVelocity[j]);
      }
      //UpdateGyroEdgeDetection(GyroVelocity[1]);//works but already too late by the time edge is detected

      AverGyroVelocity+=((GyroVelocity[2])>0?(GyroVelocity[2]):-(GyroVelocity[2]));

    }
    //get current rotational velocity for x & y axes
    for(j=0;j<2;j++){
      GyroVelocity[j]=((int32_t)nav_data[j])-GyroZeroes[j];
    }
    AverGyroVelocity/=gn;
    if(gn>=31)
      GyroFifoOverflow=1;

    //Get Accelerometer data
    an=AccelBufferSize();
    for(i=0;i<an;i++){
      AccelGetAxes(nav_accel);
      //digitalWrite(Light_Bus_BTN1,0);
      /*

      //less efficient but easier to understand:

      for(j=0;j<2;j++){   //just x and y axis

        AccelAcceleration[j]=((int32_t)nav_accel[j])-AccelZeroes[j];  

        AccelVelocity[j]+=AccelAcceleration[j];        

        AccelPosition[j]+=AccelVelocity[j]/400;//so in range and same as velocity

      }      

      */
# 269 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\Navigation.ino"
      //more efficient:
      for(j=0;j<3;j++)//get rav values for all 3 in case another function wants to reference them
        AccelAcceleration[j]=((int32_t)nav_accel[j])-AccelZeroes[j];
      for(j=1;j<2;j++){ //just y axis
        //AccelAcceleration[1]+=(10*(AverGyroVelocity))/8;//assuming that it always rotates about the same point (around where wheels are), experimentally putting this in
        AccelVelocity[j]+=AccelAcceleration[j];
        AccelPositionNewDat[j]+=AccelVelocity[j];//so in range and same as velocity
      }
      if(i==an-1){
        for(j=1;j<2;j++){//just y axis
          AccelPosition[j]+=AccelPositionNewDat[j]/400;
        }
      }
    }//end for(i=0;i<n;i++
     //get current acceleration for x,y, and z axes
    if(an>0){
      for(j=0;j<3;j++){
        AccelAcceleration[j]=((int32_t)nav_accel[j])-AccelZeroes[j];
      }
    }
    if(an>=31)
      AccelFifoOverflow=1;
  }


}//end NavigationHandler()



const int SinFunctionTable[]={0,286,572,857,1143,1428,1713,1997,2280,2563,2845,3126,3406,3686,3964,4240,4516,4790,5063,5334,5604,5872,6138,6402,6664,6924,7182,7438,7692,7943,8192,8438,8682,8923,9162,9397,9630,9860,10087,10311,10531,10749,10963,11174,11381,11585,11786,11982,12176,12365,12551,12733,12911,13085,13255,13421,13583,13741,13894,14044,14189,14330,14466,14598,14726,14849,14968,15082,15191,15296,15396,15491,15582,15668,15749,15826,15897,15964,16026,16083,16135,16182,16225,16262,16294,16322,16344,16362,16374,16382,16384};
int SineFunction(int degr){//Ver. 1.0, Dustin Soodak
  int sign=1;
  if(degr<0){
    sign=-1;
    degr=-degr;
  }
  if(degr>360)
    degr%=360;
  if(degr<=90)
    return sign*SinFunctionTable[degr];
  else if(degr<=180)
    return sign*SinFunctionTable[180-degr];
  else if(degr<=270)
    return -sign*SinFunctionTable[degr-180];
  else
    return -sign*SinFunctionTable[270-degr];
}
int CosineFunction(int degr){//Ver. 1.0, Dustin Soodak
   if(degr<0)
     degr=-degr;
   if(degr>360)
     degr%=360;
   if(degr<=90)
     return SinFunctionTable[90-degr];
   else if(degr<=180)
     return -SinFunctionTable[degr-90];
   else if(degr<=270)
     return -SinFunctionTable[270-degr];
   else
     return SinFunctionTable[360-degr];
}


extern int GyroVelocityZPrev;//(defined below)
//int32_t accelxraw,accelyraw;
//int CosDegr,SinDegr;
int32_t N_accelx,N_accely;//made global so compiler doesn't "optimize" them out.


//Notes on NavigationXY()
//
//This assumes flat level surface and no tilting forwards & backwards.
//Recently added code to account (in GetPositionX/Y() functions) for 
//fact that accelerometer is actually 3cm in front of the rotation point
//of the robot. 
//
//Possible improvements:
//
//I created a couple of versions of NavigationXYTilt() which were supposed to
//take forward/backward tilt into account (since gravity is a much stronger force
//than the motors can produce, even a degree or 2 tilt can make a huge difference)
//but the drift of x-axis of the gyroscope ended up being too big to simply plug
//its value into the "degrees of tilt" variable. Since the accelerometer is in 
//front it might also experience negative acceleration from the robot vibrating
//along its x-axis (though I haven't measured this directly so don't know if
//it is actually significant).
//
//Another possible improvement is to take into account that this is a wheeled
//vehicle and not a hovercraft. If the robot is swerving slightly to the 
//right & left as it travels, then it should be possible to correct for
//drift in calculated velocity since the sideways force exerted as it turns
//should be proportional to its forward velocity. Of course, the first step
//must be to calculate what the sideways acceleration is at a point 30mm 
//behind the accelerometer(I currently only have POSITION corrected for in
//this way...acceleration might be more complicated and has to be done real-time). 
//
//If you do plan on making functions that directly access gyroscope and
//accelerometer functions, keep in mind that the vibrations from movement
//are much larger than the general linear or rotational acceleration of the
//robot as a whole. Also: the functions in the "Recorded Data" section of
//RingoHardware.h might be useful.
//Dustin Soodak
//

void NavigationXY(int GyroSensitivity, int AccelSensitivity){ //Ver. 1.0, Dustin Soodak
  //Note: (50,800) are good input values
  signed char i,j,n,gn,an,m;
  int32_t N_accelxraw,N_accelyraw;//32 bit since zeroes being subtracted might make a negative value out of range
  static int32_t N_accelxrawprev,N_accelyrawprev;//,N_gyrorawprev;





  //int N_accelxdif,N_accelydif,N_gyrodif;
  //int32_t N_accelx,N_accely;
  int degr;
  int CosDegr,SinDegr;

  ConvertNavigationCoordinates(1);
  count++;
  if(IRReceiving){
    if(IsIRDone())
     IRReceiving=0;
  }
  if(!PauseNavigationInterrupt && !IRReceiving){

    gn=GyroBufferSize(); //Reads how many buffer positions are full in the Gyro (the Gyro has 32 buffer 
                                      //positions/readings and must be read prior to reaching 32 to avoid inaccurate calculation)
    an=AccelBufferSize(); //Reads how many buffer positions are full in the Accelerometer (the Accel has 32 buffer
                                      //positions/readings and must be read prior to reaching 32 to avoid inaccurate calculation)

    if(an>gn) //Because the accel reads 400 Hz and gryo reads 380 Hz, this section keeps them in sync by
      m=gn/(an-gn); //occasionally averaging 2 consecutive readings from the accel
    else
      m=99;
    //digitalWrite(Light_Bus_BTN1,0);                               //<-- uncomment to spit values out the serial debug
    for(i=0;i<gn;i++){
      GyroGetAxes(nav_data); //380 hz       
      if(an)//generally, an>=gn, unless only one reading has been received
        AccelGetAxes(nav_accel); //400 hz
      //otherwise, use last reading
      //Serial.print(i);Serial.print(" ");Serial.print(gn);Serial.print(an);Serial.println(m,DEC);    //<-- uncomment to spit values out the serial debug
      if(((i+1)%m)==0){ //to keep buffers in sync
         AccelGetAxes(nav_data3);
         for(j=0;j<3;j++){
            nav_accel[j]=(((int32_t)nav_accel[j])+nav_data3[j])/2;
         }
      }


      N_accelxrawprev=N_accelxraw;//Used in code that makes sure sensor drift or small 
      N_accelyrawprev=N_accelyraw;//changes in orientation don't make it think its moving.
      N_accelxraw=((int32_t)nav_accel[0])-AccelZeroes[0];//non-rotated value
      N_accelyraw=((int32_t)nav_accel[1])-AccelZeroes[1];//non-rotated value

      GyroVelocity[2]=((nav_data[2]-GyroZeroes[2]));

      //UpdateGyroEdgeDetection(nav_data[1]-GyroZeroes[1]);//works but already too late by the time edge is detected

                                                                   //to do the math using integer values rather than float values.  The float version
                                                                   //takes about 1000 uS to run.
      //see N_accelxYGData tab of Ringo spreadsheet
      //average .7276ms with all integer operations except for theta/sin/cos
      //average .3272ms with lookup array trig functions SinFunction() and CosFunction()
      //RestartTimer();for(i=0;i<100;i++){                         //<-- uncomment this line and "Serial.println(GetTime(),DEC);" below to measure time to run this function

      GyroPosition[2]+=GyroVelocity[2];
      GyroAccelerationZ=GyroVelocity[2]-GyroVelocityZPrev;
      GyroVelocityZPrev=GyroVelocity[2];
      degr=(((float)GyroPosition[2])*(0.0000355*2000/380));//proper direction but pi/2 offset
      SinDegr=SineFunction(degr);//integer trig functions scaled by 0x4000
      CosDegr=CosineFunction(degr);//integer trig functions scaled by 0x4000

      N_accelx=(N_accelxraw*CosDegr-N_accelyraw*SinDegr)/0x4000;//rotated value
      N_accely=(N_accelyraw*CosDegr+N_accelxraw*SinDegr)/0x4000;//rotated value

      //AccelDistance+=sqrt(((AccelVelocity[0]/380*AccelVelocity[0]/380)+(AccelVelocity[1]/380*AccelVelocity[1]/380))/380);
      //Serial.print(nav_accel[0]);Serial.print("\t");Serial.print(nav_accel[1]);Serial.print("\t");Serial.println(nav_data[2]);//remember: motors must be switched to serial to use this
      //}  Serial.println(GetTime(),DEC);  //<-- uncomment this line and "RestartTimer();for(i=0;i<100;i++){" above to measure time to run function & echo out serial

      //
      //
# 467 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\Navigation.ino"
      if(((N_accelxraw)>0?(N_accelxraw):-(N_accelxraw))>AccelSensitivity || ((N_accelyraw)>0?(N_accelyraw):-(N_accelyraw))>AccelSensitivity || ((GyroVelocity[2])>0?(GyroVelocity[2]):-(GyroVelocity[2]))>GyroSensitivity){//is moving (should catch this immediately)      
        if(((N_accelxrawprev-N_accelxraw)>0?(N_accelxrawprev-N_accelxraw):-(N_accelxrawprev-N_accelxraw))>AccelSensitivity || ((N_accelyrawprev-N_accelyraw)>0?(N_accelyrawprev-N_accelyraw):-(N_accelyrawprev-N_accelyraw))>AccelSensitivity){
          if(IsStationary){
            NonStationaryValue=(((N_accelxraw)>0?(N_accelxraw):-(N_accelxraw))>AccelSensitivity?N_accelxraw:((N_accelyraw)>0?(N_accelyraw):-(N_accelyraw))>AccelSensitivity?N_accelyraw:GyroVelocity[2]);
            NonStationaryAxis=(((N_accelxraw)>0?(N_accelxraw):-(N_accelxraw))>AccelSensitivity?0x1:((N_accelyraw)>0?(N_accelyraw):-(N_accelyraw))>AccelSensitivity?0x2:0x4);
          }//what if gyro but not accel?
          IsStationaryCount=0;
          IsStationary=0;
      }
      }
      else{//doesn't appear to be changing its velocity or orientation very much
        if(!IsStationary){
          if(IsStationaryCount<10){
            IsStationaryCount++;
          }
          else{//definitely not moving if still of 10 consecutive readings
            IsStationary=1;
            AccelVelocity[0]=0;
            AccelVelocity[1]=0;
          }
        }
      }
      if(IsStationary==0){
        AccelVelocity[0]+=N_accelx;
        AccelVelocity[1]+=N_accely;
        AccelPosition[0]+=AccelVelocity[0];
        AccelPosition[1]+=AccelVelocity[1];
        //Note: theta has a Pi/2 offset

        AccelPositionXOffset=-((int32_t)30 /*30 mm forward from center between wheels*/)*SinDegr/0x4000;
        AccelPositionYOffset=((int32_t)30 /*30 mm forward from center between wheels*/)*CosDegr/0x4000;





      }

    }//end for(i=0;i<gn;i++)

    if(an>0){

      AccelAcceleration[0]=N_accelx;
      AccelAcceleration[1]=N_accely;

      AccelAcceleration[2]=((int32_t)nav_accel[2])-AccelZeroes[2];
    }

    //digitalWrite(Light_Bus_BTN1,1);
    if(an>=31)
      AccelFifoOverflow=1;
    if(gn>=31)
      GyroFifoOverflow=1;

  }//end if(!PauseNavigationInterrupt && !IRReceiving)

}//end NavigationXY()

// calibrates out stationary drift in sensors. MUST BE RUN WHEN RINGO IS PERFECTLY STILL!!
void CalibrateNavigationSensors(void){//Ver. 1.0, Dustin Soodak
  char i,j,n,prev;
  int32_t totals[3];
  //int nav_data[3];//Compiler error:  when nav_data[] local, it was not actually reserving space for this since it isn't explicitly set (is set by pointers)
  prev=PauseNavigationInterrupt;
  PauseNavigationInterrupt=1;//PauseNavigationInterrupt=1;//noInterrupts();
  //Gyro average to find zero value (assuming not currently moving)
  for(i=0;i<3;i++)
    totals[i]=0;
  i=1000;
  while(GyroBufferSize()<20 && i)i--;
  for(i=0;i<20;i++){
    GyroGetAxes(nav_data);
    for(j=0;j<3;j++){
      totals[j]+=nav_data[j];
    }
  }
  for(i=0;i<3;i++){
    GyroZeroes[i]=totals[i]/20;
  }
  //Accel average to find zero value (assuming not currently moving)
  for(i=0;i<3;i++)
    totals[i]=0;
  i=1000;
  while(AccelBufferSize()<20 && i)i--;
  for(i=0;i<20;i++){
    AccelGetAxes(nav_accel);
    for(j=0;j<3;j++){
      totals[j]+=nav_accel[j];
    }
  }
  for(i=0;i<3;i++){
    AccelZeroes[i]=totals[i]/20;
    AccelVelocity[i]=0;//assumes it isn't moving when this function called
  }
  //Clear buffers
  n=AccelBufferSize();
  for(i=0;i<n;i++){
    AccelGetAxes(nav_data);
  }
  n=GyroBufferSize();
  for(i=0;i<n;i++){
    GyroGetAxes(nav_data);
  }
  AccelFifoOverflow=0;
  PauseNavigationInterrupt=prev;//interrupts();//PauseNavigationInterrupt=0;
  for(i=0;i<3;i++){
    AccelVelocity[i]=0;
  }

  //Zero sensors with arcsin:
  //9800*sin(GyroXRaw*((0.0000355*2000/380)*3.14159/180))=AccelYRaw*(-(float)(2*9800)/32768
  //9800*sin(GyroPosition[0]*((0.0000355*2000/380)*3.14159/180))=AccelZeroes[1]*(-(float)(2*9800)/32768
  //GyroPosition[0]=-arcsin(AccelZeroes[1]*((float)(2)/32768))/((0.0000355*2000/380)*3.14159/180)
  //Zero sensors with arctan:
  //atan2(x,y) (instead of y,x) so theta of 90 degrees -> 0 degrees.
  //GyroPosition[0]=-atan2(AccelZeroes[1],AccelZeroes[2])/((0.0000355*2000/380)*3.14159/180);
  //Calculate new:
  //Theta=GyroPosition[0]*((0.0000355*2000/380)*3.14159/180);
  //AccelAcceleration[1]-=sin(Theta)*16384;//raw to mm/s^2 is ((float)(2*9800)/32768);
  //Since sin(theta)=theta for small theta, just need one float and no trig:
  //AccelAcceleration[1]-=GyroPosition[0]*(((0.0000355*2000/380)*3.14159/180)*16384)

  //For gyroX correction to accelY:
  GyroPosition[0]=-asin(AccelZeroes[1]*((float)(2)/32768))/((0.0000355*2000/380)*3.14159/180);
  //Serial.print(AccelZeroes[1],DEC);Serial.print(" ");Serial.println(AccelZeroes[2]);
  //Serial.println(atan2(AccelZeroes[1],AccelZeroes[2]));
  //Serial.println(atan2(AccelZeroes[1],AccelZeroes[2])*180/3.14159);

  //9800*sin(GyroPosition[0]*((0.0000355*2000/380)*3.14159/180))

  //Serial.print("init gyro raw after zeroed ");
  //Serial.println(GyroPosition[0],DEC);
  //Serial.print(" to degr ");
  //Serial.println(GyroRawToDegrees(GyroPosition[0]),DEC);

  IsStationary=1;IsStationaryCount=0;//IsMovingCount=0;
  GyroVelocityZPrev=0;
  NavigationOutOfRange=0;
  //ZeroGyroEdgeDetection();//works, but already too late by the time an edge is detected 
  //The following initializes accel in case gyro
  //reading but not accel reading so accel has an initial
  //default value for NavigationXY()
  nav_accel[0]=AccelZeroes[0];
  nav_accel[1]=AccelZeroes[1];
  //Though already set above, put here explicitly in case
  //other code changed for an unrelated reason.

}//end CalibrateNavigationSensors()

//re-sets Ringo's X, Y, and Heading coordinates to zeros
void ZeroNavigation(void){//Ver. 1.0, Dustin Soodak
  char i;
   //Zero total changes in position, velocity, and orientation
  for(i=0;i<3;i++){
    AccelPosition[i]=0;
    AccelVelocity[i]=0;
    if(i>0)//for part in CalibrateNavigationSensors() which 
      GyroPosition[i]=0;
  }
  XYMode=0;
  AccelDistance=0;
  IsStationary=1;IsStationaryCount=0;//IsMovingCount=0;
  AccelFifoOverflow=0;
  GyroFifoOverflow=0;
  NavigationOutOfRange=0;
  AccelPositionYOffset=30;
  AccelPositionXOffset=0;
}


void PauseNavigation(void){//Ver. 1.0, Dustin Soodak
  char i;
  PauseNavigationInterrupt=1;
  for(i=0;i<3;i++){
    AccelVelocity[i]=0;
  }
}

void ResumeNavigation(void){//Ver. 1.0, Dustin Soodak
  char i,n;
  //clear gyro buffer and save last values
  n=GyroBufferSize();
  for(i=0;i<n;i++){
    GyroGetAxes(GyroVelocity);
  }
  //subtract gyro zeroes
  for(i=0;i<3;i++){
   GyroVelocity[i]-=GyroZeroes[i];
  }
  //clear accel buffer and save last values
  n=AccelBufferSize();
  for(i=0;i<n;i++){
   AccelGetAxes(AccelAcceleration);
  }
  //subtract accel zeroes
  for(i=0;i<3;i++){
   AccelAcceleration[i]-=AccelZeroes[i];
  }
  for(i=0;i<2;i++){
   AccelVelocity[i]=0;
  }
  PauseNavigationInterrupt=0;
  IsStationary=1;IsStationaryCount=0;//IsMovingCount=0;
  GyroVelocityZPrev=0;
}

char NavigationPaused(void){//Ver. 1.0, Dustin Soodak
  return PauseNavigationInterrupt;
}

void NavigationBegin(void){ //Ver. 1.0, Dustin Soodak   
  char i;
  I2CBegin();//needs to be called before writing Gryro or Accel registers
  PauseNavigationInterrupt=1;
  //
  //GyroWriteRegister(GYR_CTRL_REG3,0x34);//open drain (bit4), watermark interrupt on DRDY/INT2 (bit2). bit5 is INT1 high, but seems to work for INT2 as well
  GyroWriteRegister(0x22,0x10); //0x10 eliminates excess current drain from Accel via the shared interrupt line
  GyroWriteRegister(0x2E,0x54);//watermark FIFO threshold of 20 and fifo stream mode (bits7:6)
  GyroWriteRegister(0x20,0x0f);//all axes on(0:2) and auto-power off disabled
  GyroWriteRegister(0x24,0x40);//fifo enable
  //
  AccelWriteRegister(0x2A,0x0);// 0x2a inactive but can set registers (bit1)
  AccelWriteRegister(0x2C,0x3);// 0x2c interrupt is OD(bit0), active high(bit1)
  AccelWriteRegister(0x2D,0x40);// 0x2d fifo interrupt enable
  //AccelWriteRegister(ACC_CTRL_REG4,0x00);
  AccelWriteRegister(0x2E,0x40);// 0x2e fifo interrupt on INT1 pin
  AccelWriteRegister(0x09,0x14);// 0x9 watermark at 20 measurements
  AccelWriteRegister(0x2A,0x9);// 0x2a 400 Hz and active (bit1)
  AccelWriteRegister(0x09,0x54);// 0x09 circular buffer mode (now at 0x54=01010100)
  //

  GyroSetFrequency(380);
  GyroSetRange(2000);

  delay(200); // let nav sensors buffer fill some before calibrating
  CalibrateNavigationSensors();
  ZeroNavigation();

  pinMode(2 /*used by both Gyro and Accel chips.*/, 0x2);

  NavigationOn=1;
  PauseNavigationInterrupt=0;

}//end void NavigationBegin(void)


int PresentHeading(void){//Ver. 1.0, Kevin King
  return -GyroRawToDegrees(GyroPosition[2]);// z axis
}
int GetDegrees(void){//Ver. 1.0, Dustin Soodak
  return -GyroRawToDegrees(GyroPosition[2]);// z axis 
}
int GetDegreesX(void){//Ver. 1.0, Dustin Soodak
  return GyroRawToDegrees(GyroPosition[0]);// x axis
}
int GetDegreesPerSecond(void){//Ver. 1.0, Dustin Soodak
  return -GyroRawToDegreesPerSec(GyroVelocity[2]);// z axis
}
int GetDegreesPerSecondX(void){//Ver. 1.0, Dustin Soodak
  return GyroRawToDegreesPerSec(GyroVelocity[0]);
}
int GetDegreesPerSecondY(void){//Ver. 1.0, Dustin Soodak
  return GyroRawToDegreesPerSec(GyroVelocity[1]);
}
int GetDegreesToStop(void){//Ver. 1.0, Dustin Soodak
  return -GyroDegreesToStopFromRaw(GyroVelocity[2]);
}

int GetAccelerationX(void){//Ver. 1.0, Dustin Soodak
  return ((float)AccelAcceleration[0])*(-(float)(2*9800)/32768);
}
int GetAccelerationY(void){//Ver. 1.0, Dustin Soodak
  return ((float)AccelAcceleration[1])*(-(float)(2*9800)/32768);
}
int GetAccelerationYUnZeroed(void){//Ver. 1.0, Dustin Soodak
  //for front/back tilt analysis
  return ((float)AccelAcceleration[1]+AccelZeroes[1])*(-(float)(2*9800)/32768);
}
int GetAccelerationZ(void){//Ver. 1.0, Dustin Soodak
  return ((float)AccelAcceleration[2])*(-(float)(2*9800)/32768);
}
//The following 4 functions have different cases depending on if 
//NavigationXY() or one of the simpler functions are used.
int GetVelocityX(void){//Ver. 1.0, Dustin Soodak
  if(XYMode)
    return ((float)AccelVelocity[0])*(-(float)(2*9800)/32768/380);
  else
    return 0;
}
int GetVelocityY(void){//Ver. 1.0, Dustin Soodak
  if(XYMode)
    return ((float)AccelVelocity[1])*(-(float)(2*9800)/32768/380);
  else
    return ((float)AccelVelocity[1])*(-(float)(2*9800)/32768/400);
}
int GetPositionX(void){//Ver. 1.0, Dustin Soodak
  //SimpleNavigation() and SimpleGyroNavigation() leave X unchanged, so decided
  //not to have an XYMode==0 option.
  //if(XYMode)
    return ((float)AccelPosition[0])*(-(float)(2*9800)/32768/380/380)-AccelPositionXOffset;
  //else
    //return ((float)AccelPosition[0])*(-(float)(2*9800)/32768/380/380);
}
int GetPositionY(void){//Ver. 1.0, Dustin Soodak
  if(XYMode) //put these sections back to use auto offset
    return ((float)AccelPosition[1])*(-(float)(2*9800)/32768/380/380)-AccelPositionYOffset;
  else
    return ((float)AccelPosition[1])*(-(float)(2*9800)/32768/400);//note: already an extra "/400" in handler to keep in range (400 for 400hz)

}

void DelayWithSimpleNavigation(int ms){//Ver. 1.0, Dustin Soodak
  int32_t total=millis()+ms;
  while(millis()<total){
    SimpleNavigation();
  }
}

void DelayWithNavigation(int ms){//Ver. 1.0, Dustin Soodak
  int32_t total=millis()+ms;
  while(millis()<total){
    NavigationXY(80,800);
  }
}

// ***************************************************
// end Navigation
// ***************************************************

// ***************************************************
// Accelerometer
// ***************************************************

//Accelerometer code based on code by Jim Lindblom
//https://github.com/sparkfun/MMA8452_Accelerometer/tree/master/Firmware



int32_t AccelPosition[3]={0,0,0};//running total
int32_t AccelDistance=0;
int32_t AccelVelocity[3]={0,0,0};//running total
int AccelAcceleration[3]={0,0,0};

uint8_t AccelReadRegisters(uint8_t Reg, uint8_t *RxBuffer, uint8_t Length){//Ver. 1.0, Dustin Soodak
  return I2CReadRegs(0x1C,0x80|Reg,RxBuffer,Length);//"In order to read multiple bytes, it is necessary to assert the most significant bit of the subaddress field"
}
uint8_t AccelReadRegister(uint8_t Reg){//Ver. 1.0, Dustin Soodak
  return I2CReadReg(0x1C,Reg);
}
void AccelWriteRegisters(uint8_t Reg, uint8_t *TxBuffer, uint8_t Length){//Ver. 1.0, Dustin Soodak
   I2CWriteRegs(0x1C, Reg, TxBuffer, Length);
}
void AccelWriteRegister(uint8_t Reg, uint8_t TxData){//Ver. 1.0, Dustin Soodak
  I2CWriteReg(0x1C, Reg, TxData);
}
//Put in standby to change register settings
void AccelStandby(void){//Ver. 1.0, Dustin Soodak
  byte c = AccelReadRegister(0x2A);
  AccelWriteRegister(0x2A, c & ~(0x01));
}
//Needs to be in this mode to output data
void AccelActive(void){//Ver. 1.0, Dustin Soodak
  byte c = AccelReadRegister(0x2A);
  AccelWriteRegister(0x2A, c | 0x01);
}

uint8_t AccelBufferSize(void){//Ver. 1.0, Dustin Soodak
  return AccelReadRegister(0x00)&0x3F;
}
int AccelGetAxis(char Axis){//Ver. 1.0, Dustin Soodak
  //Axis=0,1,2 (see p.22)
  uint8_t ar[2];
  int16_t val;//OUT_X_MSB, etc.
  if(Axis>2)
    Axis=2;
  AccelReadRegisters((0x01 +2*Axis),ar,2);
  val=(ar[0]>>2)+(((uint8_t)(ar[1]&0x7F))<<6)+(((uint8_t)(ar[1]&0x80))<<8);
  return val;
}
void AccelGetAxes(int *Axes){//Ver. 1.0, Dustin Soodak
  //Axis=0,1,2 (see p.22) [int is 16 bit in adruino]
  uint8_t ar[6];
  int i;
  AccelReadRegisters((0x01),ar,6);
  for(i=0;i<6;i+=2){
     //Axes[i>>1]=(int)(ar[i]>>2)+(((uint8_t)(ar[i+1]&0x7F))<<6)+(((uint8_t)(ar[i+1]&0x80))<<8);
     Axes[i>>1]=(((signed short)ar[i])<<8)+ar[i+1];//((((unsigned int)ar[i])&0x10)<<8)|((((unsigned int)ar[i])&0x7F)<<6);
  }
}

//Serial.print(" v= ");Serial.print(-(GetVelocityRaw(1))*(2*9800)/(32768)/400,DEC);  // debugging code
//Serial.print(" p= ");Serial.println(-(GetPositionRaw(1))*(2*9800)/(32768)/400/* /400 in handler instead*/,DEC);  // debugging code

// ***************************************************
// end Accelerometer
// ***************************************************

// ***************************************************
// Gyro
// ***************************************************

//Note: referenced code from "L3G4200D 3-axis gyro example code" by Jim Lindblom at
//https://www.sparkfun.com/products/10612

int32_t GyroPosition[3]={0,0,0};//running total
int GyroVelocity[3]={0,0,0};
int GyroAccelerationZ=0;//rate of change in angular speed (degrees pre second)
int GyroVelocityZPrev=0;//used in NavigationXY() to calculate GyroAccelerationZ





int GyroRange=250;//default value for chip




int GyroFrequency=95;
float GyroRawToDegreesMult=1;
float GyroDegreesToRawMult=1;
float GyroRawToDegreesPerSecMult=1;
float GyroDegreesPerSecToRawMult=1;
float GyroRawToSkidMult=0;

uint8_t GyroReadRegisters(uint8_t Reg, uint8_t *RxBuffer, uint8_t Length){//Ver. 1.0, Dustin Soodak
  return I2CReadRegs(0x6B,Reg,RxBuffer,Length);
}
uint8_t GyroReadRegister(uint8_t Reg){//Ver. 1.0, Dustin Soodak
  return I2CReadReg(0x6B,Reg);
}
void GyroWriteRegisters(uint8_t Reg, uint8_t *TxBuffer, uint8_t Length){//Ver. 1.0, Dustin Soodak
   I2CWriteRegs(0x6B, Reg, TxBuffer, Length);
}
void GyroWriteRegister(uint8_t Reg, uint8_t TxData){//Ver. 1.0, Dustin Soodak
  I2CWriteReg(0x6B, Reg, TxData);
}

uint8_t GyroBufferSize(void){//Ver. 1.0, Dustin Soodak
  return GyroReadRegister(0x2F)&0x1F;
}

int16_t GyroGetAxis(char Axis){//Ver. 1.0, Dustin Soodak
  //Axis=0,1,2 (p.36)
  int16_t val;//OUT_X_L=0x28, OUT_X_H=0x29 (y and z follow)
  if(Axis>2)
    Axis=2;
  GyroReadRegisters((0x28 +2*Axis)|0x80,(uint8_t*)val,2);//"In order to read multiple bytes, it is necessary to assert the most significant bit of the subaddressfield."
  return val;
}
void GyroGetAxes(int *Axes){//Ver. 1.0, Dustin Soodak
//Axis=0,1,2
  //OUT_X_L=0x28, OUT_X_H=0x29 (y and z follow)
  GyroReadRegisters((0x28)|0x80,(uint8_t*)Axes,6);//"In order to read multiple bytes, it is necessary to assert the most significant bit of the subaddressfield."
}

//gyro:
//range	calc dps/dig	"typ" in datasheet	to get to "typ"		
//250	0.007629395	0.00875           	1.14687993         
//500	0.015258789	0.0175           	1.146880005
//2000	0.061035156	0.070            	1.146880005

void UpdateGyroConversionVars(void){//Ver. 1.0, Dustin Soodak
  GyroRawToDegreesPerSecMult=((float)GyroRange)*0.0000355/GyroscopeCalibrationMultiplier;// 1/2^15=1/32768=0.000030517578125      // 1/2^15*1.14688=exactly .000035
  GyroRawToDegreesMult=GyroRawToDegreesPerSecMult/GyroFrequency;
  GyroDegreesPerSecToRawMult=((float)28169)/GyroRange*GyroscopeCalibrationMultiplier; //2^15=32768                             //1/0.000035=28571.42857
  GyroDegreesToRawMult=GyroDegreesPerSecToRawMult*GyroFrequency;
  GyroRawToSkidMult=GyroRawToDegreesPerSecMult*0.1029;//used in GyroDegreesToStopFromRaw()
}

float EEPROM_CalMultiplier;
void GetGyroCalibrationMultiplier(void){//Ver. 1.0, Kevin King   //Gets gyro cal mult from EEPROM if present
  EEPROM_readAnything(1020 /*EEPROM address where Gyroscope_Calibration_Multipler is located*/, EEPROM_CalMultiplier);
  if((EEPROM_CalMultiplier >= 0.85) && (EEPROM_CalMultiplier <= 1.15)){
  GyroscopeCalibrationMultiplier=EEPROM_CalMultiplier; //use calibration value if within bounds
  }
}

void GyroSetRange(int Range){//Ver. 1.0, Dustin Soodak
  char RangeByte;
  if(Range==2000)
    RangeByte=0x20;
  else if(Range==500)
    RangeByte=0x10;
  else
    RangeByte=0x00;
  GyroWriteRegister(0x23,RangeByte);
  //FS=250dps: 8.75 mdps/digit
  //   500:    17.5
  //   2000:   70 
  GyroRange=Range;
  UpdateGyroConversionVars();
}

int GyroGetRangeFromChip(void){//Ver. 1.0, Dustin Soodak
  char d=GyroReadRegister(0x23)&0x30;
  if(d==0x00)
    return 250;
  else if(d==0x10)
    return 500;
  else if(d==0x20)
    return 2000;
  else
    return 250;
}

int GyroGetFrequencyFromChip(void){//Ver. 1.0, Dustin Soodak
  char d=GyroReadRegister(0x20)&0xC0;
  if(d==0b00000000 /*wg 20 0f*/)
    return 95;
  else if(d==0b01000000 /*wg 20 4f*/)
    return 190;
  else if(d==0b10000000 /*wg 20 8f*/)
    return 380;
  else if(d==0b11000000 /*wg 20 cf*/)
    return 760;
  else
    return 95;
}

//GyroSetFrequencyByte(GyroReadRegister(GYR_CTRL_REG1)&0xC0);
//GyroSetRangeByte(GyroReadRegister(GYR_CTRL_REG4)&0x30); 

void GyroSetFrequency(int Frequency){//Ver. 1.0, Dustin Soodak
  char FrequencyByte;
  char r=GyroReadRegister(0x20);
  if(Frequency==190)
    FrequencyByte=0b01000000 /*wg 20 4f*/;
  else if(Frequency==380)
    FrequencyByte=0b10000000 /*wg 20 8f*/;
  else if(Frequency==760)
    FrequencyByte=0b11000000 /*wg 20 cf*/;
  else if(Frequency==95)
    FrequencyByte=0b00000000 /*wg 20 0f*/;
  GyroWriteRegister(0x20,FrequencyByte|(r&~0b11000000));
  GyroFrequency=Frequency;
  UpdateGyroConversionVars();
}


int32_t GyroDegreesToRaw(int Degrees){//Ver. 1.0, Dustin Soodak
  int32_t raw=1;
  return GyroDegreesToRawMult*Degrees;
  /*if(GyroRangeByte==GR_250dps){

    raw=-Degrees*100000/875;

    //Serial.print("_250_");

  }

  else if(GyroRangeByte==GR_500dps){

    //Serial.print("_500_");

    raw=-Degrees*10000/175;

  }

  else if(GyroRangeByte==GR_2000dps){

    raw=-Degrees*1000/70;

    //Serial.print("_2000_");

  }

  if(GyroFrequencyByte==GF_95Hz)

    return raw*95;

  else if(GyroFrequencyByte==GF_190Hz)

    return raw*190;

  else if(GyroFrequencyByte==GF_380Hz)

    return raw*380;

  else if(GyroFrequencyByte==GF_760Hz){

    //Serial.print("_760_");

    return raw*760;

  }

  else

    return 0;

  */
# 1035 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\Navigation.ino"
}
int GyroDegreesPerSecToRaw(int Degrees){//Ver. 1.0, Dustin Soodak
  return Degrees*GyroDegreesPerSecToRawMult;
}
int GyroRawToDegrees(int32_t Raw){//Ver. 1.0, Dustin Soodak
  int32_t deg=1;
  return Raw*GyroRawToDegreesMult;
  /*

  if(GyroRangeByte==GR_250dps)

    deg=-Raw*875/100000;

  else if(GyroRangeByte==GR_500dps)

    deg=-Raw*175/10000;

  else if(GyroRangeByte==GR_2000dps){

    deg=-Raw*70/1000;

  }

  if(GyroFrequencyByte==GF_95Hz)

    return deg/95;

  else if(GyroFrequencyByte==GF_190Hz)

    return deg/190;

  else if(GyroFrequencyByte==GF_380Hz)

    return deg/380;

  else if(GyroFrequencyByte==GF_760Hz)

    return deg/760;

  else

    return 0;

  */
# 1061 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\Navigation.ino"
}

int GyroRawToDegreesPerSec(int Raw){//Ver. 1.0, Dustin Soodak
  return Raw*GyroRawToDegreesPerSecMult;
}

int GyroDegreesToStopFromRaw(int DegreesPerSecondRaw){//Ver. 1.0, Dustin Soodak
 int mx=((float)DegreesPerSecondRaw)*GyroRawToSkidMult;
 if(-24<=mx && mx<=24)
   return 0;
 else if(mx>24)
   return mx-24;
 else //mx<24
   return mx+24;
}

int GyroDegreesToStop(int DegreesPerSecond){//Ver. 1.0, Dustin Soodak
  int mx=((float)DegreesPerSecond)*0.1029;
  if(-24<=mx && mx<=24)
   return 0;
 else if(mx>24)
   return mx-24;
 else //mx<24
   return mx+24;
}

// ***************************************************
// end Gyro
// ***************************************************

// ***************************************************
// Computation
// ***************************************************

int MinTurn(int ChangeInDegrees){//Ver. 1.0, Dustin Soodak
  int degr=ChangeInDegrees;
  char IsNegative=0;
  if(degr<0){
    IsNegative=1;
    degr=-degr;
  }
  if(degr>360)
      degr=degr%360;
  if(degr<=180)
    degr;
  else
    -(360-degr);
  if(IsNegative)
    degr=-degr;
  return degr;
}
int MinTurnToHeading(int DesiredHeading){//Ver. 1.0, Dustin Soodak
  return MinTurn(DesiredHeading-GetDegrees())+GetDegrees();
}

int VectorToDegrees(int x,int y){//Ver. 1.0, Dustin Soodak
  return 90-(int)(atan2(y,x)*180/3.14159265359);
}

// ***************************************************
// end Computation
// ***************************************************
# 1 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino"
/*



Ringo Robot:  RingoHardware  Rev06.01  12/2015



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
# 18 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino"
# 19 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino" 2
# 20 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino" 2
# 21 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino" 2
# 22 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino" 2

// ***************************************************
// Ringo Hardware
// ***************************************************


void HardwareBegin(void){//Ver. 1.0, Dustin Soodak
  I2CBegin(); // startup I2C
  GyroWriteRegister(0x22,0x10); // turn off gyro open drain (drains current via int line from accel if not set)
  GyroWriteRegister(0x20,0x0f); // place gyro in power down mode (saves 6mA of current)
                                            // call NavigationBegin(); to wake up and configure Gyro and Accelerometer  
  MotorsBegin();
  pinMode(8 /*turn on IR_FRNT_LEFT_BTM and IR_FRNT_RGHT_BTM*/,0x1);
  digitalWrite(8 /*turn on IR_FRNT_LEFT_BTM and IR_FRNT_RGHT_BTM*/,0);
  pinMode(4,0x1);
  digitalWrite(4,0);

  pinMode(7 /*for 6 neo pixel RGB*/,0x2);
  Serial.begin(57600);//Serial.begin(9600);

  pinMode(3,0x2);

  pinMode(10,0x1);
  digitalWrite(10,0);
  pinMode(13,0x1);
  digitalWrite(13,1);
  pinMode(12,0x1);
  digitalWrite(12,1);
  pinMode(11,0x1);
  digitalWrite(11,1);
  GetGyroCalibrationMultiplier();
}


char Mode_ButtonOrPixels=99;
void SwitchButtonToPixels(void){//Ver. 1.0, Dustin Soodak
  //if(!(Mode_ButtonOrPixels==1)){
    pinMode(7 /*for 6 neo pixel RGB*/,0x1);
    digitalWrite(7 /*for 6 neo pixel RGB*/, 0x0);
    //pixels.begin();      //pixels.begin() does exactly what above two lines of code do, so not required to call
    Mode_ButtonOrPixels=1;
  //}
}
void SwitchPixelsToButton(void){//Ver. 1.0, Dustin Soodak
  //if(!(Mode_ButtonOrPixels==0)){
    pinMode(7 /*for 6 neo pixel RGB*/,0x2);
    delay(1);
    Mode_ButtonOrPixels=0;
  //}
}
char Mode_SerialOrMotors=99;
void SwitchSerialToMotors(void){//Ver. 1.0, Dustin Soodak
  //if(!(Mode_SerialOrMotors==1)){
    Serial.end();
    pinMode(0, 0x1);
    pinMode(1, 0x1);
    Mode_SerialOrMotors=1;
  //}
}
void SwitchMotorsToSerial(void){//Ver. 1.0, Dustin Soodak
  //if(!(Mode_SerialOrMotors==0)){
    pinMode(0, 0x0);
    Serial.begin(57600);//Serial.begin(9600);
    Mode_SerialOrMotors=0;
  //}
}

char ButtonPressed(void){//Ver. 1.0, Dustin Soodak
  SwitchPixelsToButton();
  return (digitalRead(7 /*for 6 neo pixel RGB*/)==0);
}


int LeftMotor;
int RightMotor;
void MotorsBegin(void){//Ver. 1.0, Dustin Soodak
  LeftMotor=0;
  RightMotor=0;
  pinMode(0, 0x1);
  pinMode(1, 0x1);
  analogWrite(6,0);
  analogWrite(5,0);
}


void Motors(int LeftMotorSpeed, int RightMotorSpeed){//Ver. 1.0, Dustin Soodak
  if(!(Mode_SerialOrMotors==1)){
     SwitchSerialToMotors();
  }
  if(LeftMotorSpeed>255 /*motor goes from -255 to 255*/)
    LeftMotorSpeed=255 /*motor goes from -255 to 255*/;
  if(LeftMotorSpeed<-255 /*motor goes from -255 to 255*/)
    LeftMotorSpeed=-255 /*motor goes from -255 to 255*/;
  if(RightMotorSpeed>255 /*motor goes from -255 to 255*/)
    RightMotorSpeed=255 /*motor goes from -255 to 255*/;
  if(RightMotorSpeed<-255 /*motor goes from -255 to 255*/)
    RightMotorSpeed=-255 /*motor goes from -255 to 255*/;
  LeftMotor=LeftMotorSpeed;
  RightMotor=RightMotorSpeed;
  if(LeftMotor<0){
    digitalWrite(0,0);
    //Serial.print("left - ");
  }
  else
    digitalWrite(0,1);
  if(RightMotor<0){
    digitalWrite(1,0);
    //Serial.print("right - ");
  }
  else
    digitalWrite(1,1);

  analogWrite(6,((LeftMotor)>0?(LeftMotor):-(LeftMotor)));
  analogWrite(5,((RightMotor)>0?(RightMotor):-(RightMotor)));
}

void EdgeLightsOn(void){//Ver. 1.0, Dustin Soodak
  digitalWrite(8 /*turn on IR_FRNT_LEFT_BTM and IR_FRNT_RGHT_BTM*/, 0x1);
}

void EdgeLightsOff(void){//Ver. 1.0, Dustin Soodak
  digitalWrite(8 /*turn on IR_FRNT_LEFT_BTM and IR_FRNT_RGHT_BTM*/, 0x0);
}

void SwitchAmbientToEdge(void){//Ver. 1.0, Dustin Soodak
  digitalWrite(4, 0x0);
}

void SwitchEdgeToAmbient(void){//Ver. 1.0, Dustin Soodak
  digitalWrite(4, 0x1);
}

int ReadLeftLightSensor(void){//Ver. 1.0, Dustin Soodak
  return analogRead(2 /*AD2 //Source_Select LOW=AMB_FRNT_LEFT, HIGH=EDGE_FRNT_LEFT*/);
}

int ReadRightLightSensor(void){//Ver. 1.0, Dustin Soodak
  return analogRead(1 /*AD1 //Source_Select LOW=AMB_FRNT_RIGHT, HIGH=EDGE_FRNT_RIGHT*/);
}

int ReadRearLightSensor(void){//Ver. 1.0, Dustin Soodak
  return analogRead(3 /*AD3*/);
}

extern int LeftEdgeSensorValue,RightEdgeSensorValue,RearEdgeSensorValue;//defined below
extern uint32_t LookAtEdgePrevTimeUs;//defined below
void ReadEdgeLightSensors(char Averages){//Ver. 1.2, Dustin Soodak
int i;
  int templeft,tempright,temprear;
  uint32_t us;
  us=micros();
  if(us-LookAtEdgePrevTimeUs<200 /*us*/){
    delayMicroseconds(200 /*us*/-(us-LookAtEdgePrevTimeUs));
  }
  SwitchAmbientToEdge();
  LeftEdgeSensorValue=0;
  RightEdgeSensorValue=0;
  RearEdgeSensorValue=0;
  for(i=0;i<Averages;i++){
    templeft=ReadLeftLightSensor();
    tempright=ReadRightLightSensor();
    temprear=ReadRearLightSensor();
    EdgeLightsOn();
    delayMicroseconds(200 /*us*/);
    LeftEdgeSensorValue+=ReadLeftLightSensor()-templeft-0 /*usually close enough to zero that it makes no difference*/;
    RightEdgeSensorValue+=ReadRightLightSensor()-tempright-0 /*usually close enough to zero that it makes no difference*/;
    RearEdgeSensorValue+=ReadRearLightSensor()-temprear-20 /*usually around 30-40*/;
    EdgeLightsOff();
  }
  LookAtEdgePrevTimeUs=micros();
  if(LeftEdgeSensorValue<0) LeftEdgeSensorValue=0; else LeftEdgeSensorValue/=Averages;
  if(RightEdgeSensorValue<0) RightEdgeSensorValue=0; else RightEdgeSensorValue/=Averages;
  if(RearEdgeSensorValue<0) RearEdgeSensorValue=0; else RearEdgeSensorValue/=Averages;
}
# 205 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino"
float LookAtEdgeStabilizationTime=200. /*in ms (used if TIMER_ADJUSTED_AVERAGE is defined).*/;
int LeftEdgeSensorValuePrevious,RightEdgeSensorValuePrevious,RearEdgeSensorValuePrevious;
int LeftEdgeSensorAverageTimes32=0,RightEdgeSensorAverageTimes32=0,RearEdgeSensorAverageTimes32=0;


int LeftEdgeSensorAverage=0,RightEdgeSensorAverage=0,RearEdgeSensorAverage=0;
int LeftEdgeSensorValue=0,RightEdgeSensorValue=0,RearEdgeSensorValue=0;
uint32_t LookAtEdgePrevTimeUs=0;//so LookAtEdge() can be paused until sensors have stabilized from previous time
float LookAtEdgeTimeBetweenReadings=1.0;//can be used to check how long since the last time LookAtEdge() was called.
void ResetLookAtEdge(void){//Ver. 1.1, Dustin Soodak
  char i;
# 231 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino"
  ReadEdgeLightSensors(1);
  LeftEdgeSensorValuePrevious=LeftEdgeSensorValue;
  RearEdgeSensorValuePrevious=RearEdgeSensorValue;
  RightEdgeSensorValuePrevious=RightEdgeSensorValue;
  LeftEdgeSensorAverageTimes32=LeftEdgeSensorValue<<5;
  RearEdgeSensorAverageTimes32=RearEdgeSensorValue<<5;
  RightEdgeSensorAverageTimes32=RightEdgeSensorValue<<5;

  LookAtEdgePrevTimeUs=micros();
  for(i=0;i<8;i++){
    //LookAtEdge(); 
  }
}



void LookAtEdge(void){//Ver. 1.2, Dustin Soodak
  //note: ir remote control signals don't usually get into the edge sensors
  uint32_t us;

  float r1,r2;
  LeftEdgeSensorValuePrevious=LeftEdgeSensorValue;
  RearEdgeSensorValuePrevious=RearEdgeSensorValue;
  RightEdgeSensorValuePrevious=RightEdgeSensorValue;

  us=micros();
  if(us-LookAtEdgePrevTimeUs<200 /*us*/){
    delayMicroseconds(200 /*us*/-(us-LookAtEdgePrevTimeUs));
  }

  //Serial.println(micros()-us);//remove!!!
  SwitchAmbientToEdge();
  //Measure
  LeftEdgeSensorValue=ReadLeftLightSensor();
  RightEdgeSensorValue=ReadRightLightSensor();
  RearEdgeSensorValue=ReadRearLightSensor();
  EdgeLightsOn();
  delayMicroseconds(200 /*us*/); //originally 5us
  LeftEdgeSensorValue=ReadLeftLightSensor()-LeftEdgeSensorValue-0 /*usually close enough to zero that it makes no difference*/;
  RightEdgeSensorValue=ReadRightLightSensor()-RightEdgeSensorValue-0 /*usually close enough to zero that it makes no difference*/;
  RearEdgeSensorValue=ReadRearLightSensor()-RearEdgeSensorValue-20 /*usually around 30-40*/;
  EdgeLightsOff();
  LookAtEdgeTimeBetweenReadings=((float)us-LookAtEdgePrevTimeUs);
  LookAtEdgePrevTimeUs=micros();
  if(LookAtEdgeTimeBetweenReadings<(200 /*us*/))
    LookAtEdgeTimeBetweenReadings=(200 /*us*/);
  LookAtEdgeTimeBetweenReadings*=0.001;
  //Process data
  if(LeftEdgeSensorValue<0)//so can always use "/2" -> ">>1" trick so no divisions have to be done.
    LeftEdgeSensorValue=0;
  if(RightEdgeSensorValue<0){//so can always use "/2" -> ">>1" trick so no divisions have to be done.
    RightEdgeSensorValue=0;
  }
  if(RearEdgeSensorValue<0){//so can always use "/2" -> ">>1" trick so no divisions have to be done.
    RearEdgeSensorValue=0;
  }
# 303 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino"
  //TimeAdjustedAverage=NewValue*(dt/totalT)+AverageValue*((totalT-dt)/totalT)
  //                   =(1/totalT)*(NewValue*dt+AverageValue*(totalT-dt))
  //                   dt=LookAtEdgeTimeBetweenReadings
  //                   totalT=LookAtEdgeStabilizationTime
  //                   AverageValue=, for example, LeftEdgeSensorAverage
  //                   NewValue=, for example, LeftEdgeSensorValue
  //                   generally make sure totalT/dt>=8 so average isn't changed too much by new value
  //                   
  r2=1/LookAtEdgeStabilizationTime;
  r1=LookAtEdgeTimeBetweenReadings*r2;
  if(r1>.125){
    r1=.125;
    r2=.875;
  }
  else
    r2=(LookAtEdgeStabilizationTime-LookAtEdgeTimeBetweenReadings)*r2;
  r1*=32;//since using "Times32" averages
  LeftEdgeSensorAverageTimes32=LeftEdgeSensorValue*r1+LeftEdgeSensorAverageTimes32*r2;
  RearEdgeSensorAverageTimes32=RearEdgeSensorValue*r1+RearEdgeSensorAverageTimes32*r2;
  RightEdgeSensorAverageTimes32=RightEdgeSensorValue*r1+RightEdgeSensorAverageTimes32*r2;
  LeftEdgeSensorAverage=LeftEdgeSensorAverageTimes32>>5;
  RearEdgeSensorAverage=RearEdgeSensorAverageTimes32>>5;
  RightEdgeSensorAverage=RightEdgeSensorAverageTimes32>>5;


}


char IsOverEdge(void){//Ver 1.0, Dustin Soodak
  char edge=0;
  int Dark1,Dark2,Bright1,Bright2;
  //Absolute endpoint tests:

  edge|=LeftEdgeSensorValue<5 /*default of 5 is mostly disabled*/?0x10:0;
  edge|=RightEdgeSensorValue<5 /*default of 5 is mostly disabled*/?0x01:0;
  edge|=RearEdgeSensorValue<5 /*default of 5 is mostly disabled*/?0x04:0;
  if(edge!=0)
    edge|=0x40;


  edge|=LeftEdgeSensorValue>1000 /*default of 1000 is mostly disabled*/?0x20:0;
  edge|=RearEdgeSensorValue>1000 /*default of 1000 is mostly disabled*/?0x08:0;
  edge|=RightEdgeSensorValue>1000 /*default of 1000 is mostly disabled*/?0x02:0;


  //More sensitive tests:

  if(!((edge)&0x10)){
    edge|=((LeftEdgeSensorValue<((((float)(LeftEdgeSensorAverage))*.50/*.85 default*/)-10 /*default: 10.  usually in 5 to 15 range*/) && (LeftEdgeSensorValuePrevious)<((((float)(LeftEdgeSensorAverage))*.50/*.85 default*/)-10 /*default: 10.  usually in 5 to 15 range*/))?0x10:0);
    if(!((edge)&0x10))
      edge|=((LeftEdgeSensorValue>((((float)(LeftEdgeSensorAverage))*2.00/*1.15 default*/)+10 /*default: 10.  usually in 5 to 15 range*/) && (LeftEdgeSensorValuePrevious)>((((float)(LeftEdgeSensorAverage))*2.00/*1.15 default*/)+10 /*default: 10.  usually in 5 to 15 range*/))?0x20:0);
  }
  if(!((edge)&0x04)){
    edge|=((RearEdgeSensorValue<((((float)(RearEdgeSensorAverage))*.50/*.85 default*/)-10 /*default: 10.  usually in 5 to 15 range*/) && (RearEdgeSensorValuePrevious)<((((float)(RearEdgeSensorAverage))*.50/*.85 default*/)-10 /*default: 10.  usually in 5 to 15 range*/))?0x04:0);
    if(!((edge)&0x04))
      edge|=((RearEdgeSensorValue>((((float)(RearEdgeSensorAverage))*2.00/*1.15 default*/)+10 /*default: 10.  usually in 5 to 15 range*/) && (RearEdgeSensorValuePrevious)>((((float)(RearEdgeSensorAverage))*2.00/*1.15 default*/)+10 /*default: 10.  usually in 5 to 15 range*/))?0x08:0);
  }
  if(!((edge)&0x01)){
    edge|=((RightEdgeSensorValue<((((float)(RightEdgeSensorAverage))*.50/*.85 default*/)-10 /*default: 10.  usually in 5 to 15 range*/) && (RightEdgeSensorValuePrevious)<((((float)(RightEdgeSensorAverage))*.50/*.85 default*/)-10 /*default: 10.  usually in 5 to 15 range*/))?0x01:0);
    if(!((edge)&0x01))
      edge|=((RightEdgeSensorValue>((((float)(RightEdgeSensorAverage))*2.00/*1.15 default*/)+10 /*default: 10.  usually in 5 to 15 range*/) && (RightEdgeSensorValuePrevious)>((((float)(RightEdgeSensorAverage))*2.00/*1.15 default*/)+10 /*default: 10.  usually in 5 to 15 range*/))?0x02:0);
  }
# 383 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino"
  return edge;
}//end IsOverEdge


// calls LookAtEdge(), and uses running average to detect edges or white tape.
// Output bit num: 0: right dark edge (0x01)(0b000001)  1: right bright edge (0x02)(0b000010)   
//                 2: rear dark edge (0x04)(0b000100)   3: rear bright edge (0x08)(0b001000)  
//                 4: left dark edge (0x10)(0b010000)   5: left bright edge (0x20)(0b100000)
char LookForEdge(void){//Ver. 1.1, Dustin Soodak
  //Ver. 1.1:
  LookAtEdge();
  return IsOverEdge();

}



int RightLightLevel,LeftLightLevel,RearLightLevel;
int RightLightLevelPrev,LeftLightLevelPrev,RearLightLevelPrev;
float RightLightLevelAverage=0,LeftLightLevelAverage=0,RearLightLevelAverage=0;//float instead of int so enough precision for 
                                                                               //the averaging algorithm to still work with 
                                                                               //extra long stabilization times.
int RearAmbientLightLevel,RightAmbientLightLevel,LeftAmbientLightLevel;
uint32_t ReadSideSensorsPrevTimeUs=0;//note: in future, maybe make this static inside ReadSideSensors (unless we want
                                     //      a "ResetReadSideSensors()" function).
void ReadSideSensors(void){//Ver. 1.1, Dustin Soodak
  uint32_t dt=micros()-ReadSideSensorsPrevTimeUs;
  float a1,b1,a2,b2;
  if(dt<200 /*us*/){
    delayMicroseconds(200 /*us*/-dt);
  }
  digitalWrite(10,0);
  digitalWrite(13,1);
  digitalWrite(12,1);
  digitalWrite(11,1);
  //ReadEdgeLightSensors();
  LeftLightLevelPrev=LeftLightLevel;
  RearLightLevelPrev=RearLightLevel;
  RightLightLevelPrev=RightLightLevel;
  SwitchEdgeToAmbient();
  LeftAmbientLightLevel=ReadLeftLightSensor();
  RightAmbientLightLevel=ReadRightLightSensor();
  RearAmbientLightLevel=ReadRearLightSensor();
  digitalWrite(10,1);
  delayMicroseconds(200 /*us*/);
  LeftLightLevel=ReadLeftLightSensor()-LeftAmbientLightLevel;
  RightLightLevel=ReadRightLightSensor()-RightAmbientLightLevel;
  RearLightLevel=ReadRightLightSensor()-RearAmbientLightLevel;
  digitalWrite(10,0);
  if(LeftLightLevel<0) LeftLightLevel=0;
  if(RightLightLevel<0) RightLightLevel=0;
  if(RearLightLevel<0) RearLightLevel=0;
  //  
  //TimeAdjustedAverage=NewValue*(dt/totalT)+AverageValue*((totalT-dt)/totalT)
  //totalT=SIDE_SENSOR_AVER_RISE_TIME us or SIDE_SENSOR_AVER_FALL_TIME us.
  if(dt>1000000 /*us*/){
    a1=1;
    b1=0;
  }
  else{
    a1=((float)dt)/1000000 /*us*/;
    b1=((float)1000000 /*us*/-dt)/1000000 /*us*/;
  }
  if(dt>100000 /*us*/){
    a2=1;
    b2=0;
  }
  else{
    a2=((float)dt)/100000 /*us*/;
    b2=((float)100000 /*us*/-dt)/100000 /*us*/;
  }
  if(LeftLightLevel>LeftLightLevelAverage)
    LeftLightLevelAverage=a1*LeftLightLevel+b1*LeftLightLevelAverage;
  else
    LeftLightLevelAverage=a2*LeftLightLevel+b2*LeftLightLevelAverage;
  if(RightLightLevel>RightLightLevelAverage)
    RightLightLevelAverage=a1*RightLightLevel+b1*RightLightLevelAverage;
  else
    RightLightLevelAverage=a2*RightLightLevel+b2*RightLightLevelAverage;
  if(RearLightLevel>RearLightLevelAverage)
    RearLightLevelAverage=a1*RearLightLevel+b1*RearLightLevelAverage;
  else
    RearLightLevelAverage=a2*RearLightLevel+b2*RearLightLevelAverage;
  ReadSideSensorsPrevTimeUs=micros();
}
//Some test code for ReadSideSensors():
//HardwareBegin();
//RestartTimer();
//while(!ButtonPressed()){
//  if(GetTime()>200){
//    ReadSideSensors();
//    //Try printing something like this:
//    Serial.print(LeftLightLevel,DEC);Serial.print(" ");Serial.print(RightLightLevel,DEC);Serial.print(" Amb: ");
//    Serial.print(LeftAmbientLightLevel,DEC);Serial.print(" ");Serial.print(RightAmbientLightLevel,DEC);
//    //or this (if these values > 50, it usually means a barrier is coming up):
//    //Serial.print(LeftLightLevel-LeftLightLevelAverage,DEC);Serial.print(" ");Serial.print(RightLightLevel-RightLightLevelAverage,DEC);
//    Serial.println();
//    RestartTimer();
//  }
//}

void TxIR(unsigned char *Data, int Length){//Ver. 1.2, Dustin Soodak
    int i;
    char j;
    const uint16_t Freq=38000,UsOn=5; //For R2=2k pull up, 8 us delay before pin falls. Inputs (28000,5) give a decent square wave in this case. 
    RxIRStop();
    EnableIROutputs(1);
    ModulateIR(Freq,UsOn);
    delayMicroseconds(9000);
    EnableIROutputs(0); //EnableIROutputs(0) turns off IR 38khz out but this makes receiver voltage go high.
    delayMicroseconds(4500);
    EnableIROutputs(1);
    delayMicroseconds(520);
    for(i=0;i<Length;i++){
      for(j=0;j<8;j++){
        EnableIROutputs(0);
        if((Data[i])&(1<<j))
          delayMicroseconds(1610);
        else
          delayMicroseconds(580);
        EnableIROutputs(1);
        delayMicroseconds(520);
      }
    }//end for(i=0;i<Length;i++)    
    ModulateIR(38000, 0);
    EnableIROutputs(1);
}//end TxIR()

void TxIRKey(byte key){//Ver. 1.0, Kevin King
  if(key<1){
    // do nothing
  }
  else if(key>21){
    // do nothing
  }
  else{ // actually send IR key if it was within correct range
  key-=1; //subract 1 from key number so it matches array IRRemoteButtons
  irData[0]=0x00;irData[1]=0xFF; // all remote keys begin with 0x00, 0xFF
  irData[2]=IRRemoteButtons[key][0]; // add 3rd value
  irData[3]=IRRemoteButtons[key][1]; // add 4th value
  TxIR(irData,4); // actually transmit via any enabled IR sources
  }
}//end TxIRKey()

int IRTransitionCount=0;
char IRBitNum=0,IRByte=0,IRNumOfBytes=0;
unsigned char IRBytes[20];
uint16_t IRPrevTime=0,IRTime;
uint32_t MsAtLastIR=0;//used for end of communication timeout
volatile char IRReceiving=0;
char IRActive=0;
char IRMaxByteCount=4;


void RxIRRestart(char BytesToLookFor){//Ver. 1.2, Dustin Soodak
  int i;
  detachInterrupt(1);//interrupt 1 is I/O 3 which is _38kHz_R
  
# 540 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino" 3
 (*(volatile uint8_t *)(0x80)) 
# 540 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino"
        = 0x00; // COM1A1=0, COM1A0=0 => Disconnect Pin OC1 from Timer/Counter 1 -- PWM11=0,PWM10=0 => PWM Operation disabled
  // ICNC1=0 => Capture Noise Canceler disabled -- ICES1=0 => Input Capture Edge Select (not used) -- CTC1=0 => Clear Timer/Counter 1 on Compare/Match
  // CS12=0 CS11=1 CS10=1 => Set prescaler to clock/64
  
# 543 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino" 3
 (*(volatile uint8_t *)(0x81)) 
# 543 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino"
        = 0x03; // 8MHz clock with prescaler 0x03 means TCNT1 increments every 8uS
  // ICIE1=0 => Timer/Counter 1, Input Capture Interrupt Enable -- OCIE1A=0 => Output Compare A Match Interrupt Enable -- OCIE1B=0 => Output Compare B Match Interrupt Enable
  // TOIE1=0 => Timer 1 Overflow Interrupt Enable
  
# 546 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino" 3
 (*(volatile uint8_t *)(0x6F)) 
# 546 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino"
        = 0x00;
  pinMode(3, 0x2);
  IRReceiving=0;
  IRTransitionCount=0;
  IRPrevTime=0;
  MsAtLastIR=0;IRBitNum=0;IRByte=0;IRNumOfBytes=0;
  IRActive=1;
  IRMaxByteCount=BytesToLookFor;
  attachInterrupt(1, IRHandler, 1);//interrupt 1 is I/O 3 which is _38kHz_Rx  
}
void RxIRStop(void){//Ver. 1.2, Dustin Soodak
  detachInterrupt(1);//interrupt 1 is I/O 3 which is _38kHz_Rx    
  
# 558 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino" 3
 (*(volatile uint8_t *)(0x81))
# 558 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino"
       =0;//turn off timer        
  pinMode(3, 0x2);

  MsAtLastIR=0;//so IRHandler() recognizes it as first falling edge of next transition
  IRReceiving=0;
  IRActive=0;
  IRTransitionCount=0;//so IsIRDone() does not expect anything just because RxIRStop() called.
}

// Infrared transmit and recieve functions are based on:
//http://playground.arduino.cc/Code/InfraredReceivers by Paul Malmsten 
//https://github.com/z3t0/Arduino-IRremote by Ken Shirriff

void IRHandler(void){//Ver. 2.0, Dustin Soodak, Kevin King
  //using interrupt 1 is I/O 3 which is _38kHz_Rx  
  int16_t dTime;
  char Level;
  
# 575 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino" 3
 __asm__ __volatile__ ("cli" ::: "memory")
# 575 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino"
               ;
  IRTime=
# 576 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino" 3
        (*(volatile uint16_t *)(0x84))
# 576 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino"
             ;

  Level=digitalRead(3);

  if(!Level){//note; 38khz IR signal makes level go low
    IRReceiving=1;
  }
  if((millis()-MsAtLastIR>15) && (IRNumOfBytes<IRMaxByteCount)) {//should never be more than 9 inside valid signal
    
# 584 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino" 3
   (*(volatile uint16_t *)(0x84))
# 584 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino"
        =0;
    IRPrevTime=0;
    IRTransitionCount=0;
    IRBitNum=0;IRNumOfBytes=0;IRByte=0;
    //IRReceiving=1;  
  }
  else{
    if(IRTime>IRPrevTime)
      dTime=IRTime-IRPrevTime;
    else
      dTime=0xFFFF-IRPrevTime+1+IRTime;
    IRPrevTime=IRTime;
    dTime=dTime<<3;
    if(IRTransitionCount>=3 && (IRTransitionCount&1)){//should be high
      if(dTime>1000){
        IRByte|=(1<<IRBitNum);
      }
      if(dTime<300){//error
         IRNumOfBytes=0;
         IRReceiving=0;
      }
      IRBitNum++;
      if(IRBitNum>7){
        if(IRNumOfBytes<IRMaxByteCount){
          IRBytes[IRNumOfBytes]=IRByte;
          IRNumOfBytes++;
        }
        else{
           IRReceiving=0;
        }
        IRBitNum=0;
        IRByte=0;
      }
    }
    IRTransitionCount++;
  }
  MsAtLastIR=millis();
  
# 621 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino" 3
 __asm__ __volatile__ ("sei" ::: "memory")
# 621 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino"
             ;
}

char IsIRDone(void){//Ver. 1.0, Dustin Soodak
  return ((millis()-MsAtLastIR>40 || IRNumOfBytes==IRMaxByteCount) && IRTransitionCount);
}

//Note: first and second bytes are always 0x00 and 0xFF. IRRemoteButtons[] contains the third and fourth bytes.
const byte IRRemoteButtons[][2]={
  {0x0C,0xF3},//"1" key: 1
  {0x18,0xE7},//"2" key: 2
  {0x5E,0xA1},//"3" key: 3
  {0x08,0xF7},//"4" key: 4
  {0x1C,0xE3},//"5" key: 5
  {0x5A,0xA5},//"6" key: 6
  {0x42,0xBD},//"7" key: 7
  {0x52,0xAD},//"8" key: 8
  {0x4A,0xB5},//"9" key: 9
  {0x16,0xE9},//"0" key: 10
  {0x40,0xBF},//"FORWARD" key: 11
  {0x07,0xF8},//"LEFT" key: 12
  {0x09,0xF6},//"RIGHT" key: 13
  {0x19,0xE6},//"BACKWARD" key: 14
  {0x45,0xBA},//"POWER" key: 15
  {0x46,0xB9},//"PLUM LOGO" key: 16
  {0x47,0xB8},//"MENU" key: 17
  {0x44,0xBB},//"A" key: 18
  {0x43,0xBC},//"B" key: 19
  {0x15,0xEA},//"PLAY" key: 20
  {0x0D,0xF2}//"X" key: 21
};
byte GetIRButton(void){//Ver. 2.0, Dustin Soodak, Kevin King
  byte ButtonNumber=0,i;

    for(i=0;i<sizeof(IRRemoteButtons)/2;i++){
      if(IRBytes[2]==IRRemoteButtons[i][0] && IRBytes[3]==IRRemoteButtons[i][1]){
        ButtonNumber=i+1;
        break;
      }
    }

    return ButtonNumber; //return the button number that was pressed

}// end GetIRButton()


// ***************************************************
// end Ringo Hardware
// ***************************************************

// ***************************************************
// Pixels
// ***************************************************

// These functions use the Adafruit NeePixel Libraray https://github.com/adafruit/Adafruit_NeoPixel

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(6, 7 /*for 6 neo pixel RGB*/, ((1 << 6) | (1 << 4) | (0 << 2) | (2)) /* 0x52*/ + 0x0000 /* 800 KHz datastream*/);
void SetPixelRGB(int Pixel, int Red, int Green, int Blue){//Ver. 1.0, Dustin Soodak
  SwitchButtonToPixels(); //make sure pixel line is selected
  if(Pixel>6)
    Pixel=6;
  if(Pixel<0)
    Pixel=0;
  pixels.setPixelColor(Pixel, pixels.Color(Red,Green,Blue));
  pixels.show();//Remove this line and use RefreshPixels() below 
  delayMicroseconds(300); //so newer model pixels can reset / update correctly
  //              if you want to set several at once for high-speed patterns.
}
void SetAllPixelsRGB(int Red, int Green, int Blue){//Ver. 1.1, Dustin Soodak
  SwitchButtonToPixels();
  char i;
  for(i=0;i<6;i++){
    pixels.setPixelColor(i, pixels.Color(Red,Green,Blue));
  }
  pixels.show();//added Ver.1.1
  delayMicroseconds(300); //so newer model pixels can reset / update correctly
}
void RefreshPixels(void){//Ver. 1.0, Dustin Soodak
  pixels.show();
  delayMicroseconds(300); //so newer model pixels can reset / update correctly
}
// ***************************************************
// end Pixels
// ***************************************************


// ***************************************************
// MovementFunctions
// ***************************************************

//
//MaintainHeading:
//
//This is a simplified PID (proportiona-Integral-Derivative) function for
//maintaining your present heading. A complete example can be found below
//the MaintainHeading(degrees,speed,wiggle) function.
//
int MaintTainHeadingOffsetDir=1;
int MaintainHeadingIntegral=0;
uint32_t MaintainHeadingPrevTimeUs;

void MaintainHeadingReset(){//Ver. 1.0, Dustin Soodak
  MaintainHeadingIntegral=0;
  MaintainHeadingPrevTimeUs=micros();
}

char MaintainHeading(int Heading, int Speed, int Wiggle){//Ver. 1.0, Dustin Soodak
  int Input,Output,Proportional,left,right;
  char ret=0;
  float dt;
  if(Wiggle>0){
    if(MaintTainHeadingOffsetDir>0){
      if(GetDegrees()>=Heading+Wiggle)
        MaintTainHeadingOffsetDir=-1;
    }
    else{
      if(GetDegrees()<=Heading-Wiggle)
        MaintTainHeadingOffsetDir=1;
    }
  }
  Input=GetDegrees()+GetDegreesToStop();
  Proportional=(Heading+MaintTainHeadingOffsetDir*Wiggle-Input);//make it try to turn to the wiggle value
  //Note: "MaintainHeadingAverageDerivative" no longer needs to be needed. Code left here in case necessary for modded bot.
  //TimeAdjustedAverage=NewValue*(dt/totalT)+AverageValue*((totalT-dt)/totalT)
  //In this case, we are doing a 1/10th second (100000us) time average.
  //if(dt>100000)
  //  MaintainHeadingAverageDerivative=GetDegreesPerSecond();
  //else
  //  MaintainHeadingAverageDerivative=(((float)dt)*.00001)*GetDegreesPerSecond()+((100000-(float)dt)*.00001)*MaintainHeadingAverageDerivative;
  dt=micros()-MaintainHeadingPrevTimeUs;
  MaintainHeadingPrevTimeUs=MaintainHeadingPrevTimeUs+dt;
  if(dt>100000)
    dt=100000;
    MaintainHeadingIntegral+=(((float)dt)*.0001)*Proportional;
  if(MaintainHeadingIntegral>20*20)
    MaintainHeadingIntegral=20*20;
  if(MaintainHeadingIntegral<-20*20)
    MaintainHeadingIntegral=-20*20;
  Output=Proportional+MaintainHeadingIntegral/20;
  //Use this version in case MaintainHeadingAverageDerivative has to brought back:
  //Output=Proportional+MaintainHeadingIntegral/20-(Wiggle==0?1*MaintainHeadingAverageDerivative/4:0*MaintainHeadingAverageDerivative/4);
  if(Output>150)
    Output=150;
  if(Output<-150)
    Output=-150;
  left=Speed+Output;
  right=Speed-Output;
  //Note: only include this section if you know what your robots min motor speed is (below which it doesn't move at all):
  if(((left)>0?(left):-(left))<50){
    left=(left>0?50:left<0?-50:0);
  }
  if(((right)>0?(right):-(right))<50){
    right=(right>0?50:right<0?-50:0);
  }
  Motors(left,right);
  //a debug/test section:
  /*RecordedDataRow.Prop=Proportional;

  RecordedDataRow.Int=MaintainHeadingIntegral;

  RecordedDataRow.Der=MaintainHeadingAverageDerivative;

  RecordedDataRefresh();*/
# 781 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino"
  return ret;
}
//A complete example: (CTRL-/ to un-comment)
//#include "RingoHardware.h"
//int Heading;
//int Speed=100;
//int Wiggle=0;
//void setup(){
//  HardwareBegin();
//  PlayStartChirp(); 
//  while(!ButtonPressed());
//  delay(1000);
//  NavigationBegin();
//  ResumeNavigation();
//  Heading=PresentHeading();
//  MaintainHeadingReset();
//}
//void loop(){
//  SimpleGyroNavigation();//or SimpleNavigation(), or NavigationXY()
//  MaintainHeading(Heading,Speed,Wiggle);
//  if(PresentHeading()>Heading)
//    SetPixelRGB(BODY_TOP,0,0,10);
//  else if(PresentHeading()==Heading)
//    SetPixelRGB(BODY_TOP,0,10,0);
//  else
//    SetPixelRGB(BODY_TOP,10,0,0);
//}




void DriveArc(int TurnDegrees, int left, int right, int MaxExpectedTurnTime, int MaxExpectedSkidTime){//Ver. 1.0, Dustin Soodak
  uint32_t timeout;
  int degr,DegrPredict,DegrInit;
  SwitchSerialToMotors();
  if(NavigationOn){
    CalibrateNavigationSensors();
    ResumeNavigation();
  }
  else
    NavigationBegin();

  DegrInit=GetDegrees();
  Motors(left,right);
  timeout=millis()+MaxExpectedTurnTime;
  RestartTimer();
  while(millis()<timeout){
    SimpleGyroNavigation();
    degr=GetDegrees()-DegrInit;
    DegrPredict=degr+GetDegreesToStop(); //DegrPredict=degr+GyroDegreesToStopFromRaw(rate);//(((float)(-rate))*2000/32768)*0.1029-(24);        
    if(TurnDegrees>=0?(DegrPredict>=TurnDegrees):(DegrPredict<=TurnDegrees)){
      break;
    }
  }
  Motors(0,0);
  timeout=millis()+MaxExpectedSkidTime;
  while(millis()<timeout){
    SimpleNavigation();
  }
  PauseNavigation();
}

char RotateAccurate(int Heading, int MaxExpectedTurnTime){//Ver. 1.0, Dustin Soodak
  uint32_t timeout,timestart;
  int degr,skid,motor,degrprev;
  char res=0;
  char reverses=0;
  //RecordedDataReset(100000);
  if(NavigationOn){
    CalibrateNavigationSensors();
    ResumeNavigation();
  }
  else
    NavigationBegin();

  timestart=millis();
  timeout=timestart+MaxExpectedTurnTime;
  degrprev=GetDegrees()-Heading;
  SwitchSerialToMotors();
  while(1){
    SimpleGyroNavigation();//NavigationXY(100,800);//SimpleGyroNavigation();
    //Data.degr=GetDegrees();
    //Data.ms=RecordedDataTime();
    //if(!RecordedDataFull())
    //  RecordedDataRefresh();
    degr=GetDegrees()-Heading;
    if((degr>0)!=(degrprev>0) && millis()>timestart+300)
      reverses++;
    degrprev=degr;
    skid=GetDegreesToStop();
    if(((degr)>0?(degr):-(degr))<=1){
      Motors(0,0);
      if(GetDegreesPerSecond()==0){
        timeout=millis()+50;
        while(millis()<timeout){
          SimpleGyroNavigation();//NavigationXY(100,800);//SimpleGyroNavigation();
        }
        degr=GetDegrees()-Heading;
        if(((degr)>0?(degr):-(degr))<=1){
          res=1;
          break;
        }
      }
    }
    else{
      motor=50 +((degr+skid)>0?(degr+skid):-(degr+skid));
      if(motor>200)
        motor=200;
      if(degr+skid>0)
        Motors(-motor,motor);
      else
        Motors(motor,-motor);
    }
    if(millis()>timeout || reverses>3){
      res=0;
      Motors(0,0);
      while(GetDegreesPerSecond()!=0 && millis()<timeout+500){
        SimpleGyroNavigation();//NavigationXY(100,800);//SimpleGyroNavigation(); 
      }
      break;
    }
  }//end while(1)
  return res;
}//end Rotate()



void MoveWithOptions(int Heading, int Distance, int Speed, int MaxExpectedRunTime, int MaxExpectedSkidTime, void (*EdgeFunction)(char), char Wiggle){//Ver. 1.0, Dustin Soodak

  uint32_t timeout;
  int xrelative,yrelative,xrelativeinit,yrelativeinit,X,Y;
  int Input,Output,Proportional,Integral=0,Derivative;
  int MinLeftEdge,MinRightEdge;
  char left,right,edge;
  signed char BackAway=0;
  signed char OffsetDir=1;
  char StartedInRightDirection=0;
  char DirectionCount=0;
  float theta;

  if(!NavigationOn){//put this back (so doesn't reset navigation each time) when it works with accurate rotation function.
    NavigationBegin();
    PauseNavigation();
  }
  ResetLookAtEdge();

  theta=((float)(90-Heading))*3.14159/180;
  X=cos(theta)*Distance+GetPositionX();
  Y=sin(theta)*Distance+GetPositionY();


  xrelative=GetPositionX()-X;
  xrelativeinit=xrelative;
  yrelative=GetPositionY()-Y;
  yrelativeinit=yrelative;

  ResumeNavigation();
  SwitchSerialToMotors();
  Motors(Speed,Speed);
  timeout=millis()+MaxExpectedRunTime;
  Input=GetDegrees()+GetDegreesToStop();

  RestartTimer();
  while(millis()<timeout){
    NavigationXY(100,1000);

    xrelative=GetPositionX()-X;
    yrelative=GetPositionY()-Y;
    if(((GetDegrees())>0?(GetDegrees()):-(GetDegrees()))-Heading>30 && Wiggle==0){
      Heading=90-atan2(-yrelative,-xrelative)*180/3.14159;
      xrelativeinit=xrelative;//finish line is in new direction
      yrelativeinit=yrelative;
    }

    MaintainHeading(Heading,Speed,Wiggle);


    /*

    if(OffsetDir>0){

      if(GetDegrees()>Heading+Wiggle)

        OffsetDir=-1;    

    }

    else{

      if(GetDegrees()<Heading-Wiggle)

        OffsetDir=1;

    }

    Input=GetDegrees()+GetDegreesToStop();

    Proportional=(Heading+OffsetDir*Wiggle-Input);//make it try to turn to the wiggle value

    Derivative=GetDegreesPerSecond();

    Integral+=Proportional;

    if(Integral/20>100)

      Integral=20*100;

    if(Integral/20<-100)

      Integral=-20*100; 

    Output=Proportional+Integral/20-(Wiggle==0?3*Derivative/4:Derivative/4);

    if(Output>100)

      Output=100;

    if(Output<-100)

      Output=-100; 

    Motors(Speed+Output,Speed-Output);  

    */
# 982 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino"
    //If dot product of initial and current relative positions is negative, then their orientations differ by more than 90 degrees.
    //This is how we determine if we have passed the imaginary "finish line".
    if(((int32_t)xrelative)*xrelativeinit+((int32_t)yrelative)*yrelativeinit<0){

      break;
    }

    if(EdgeFunction){
      edge=LookForEdge();
      if(edge){
        EdgeFunction(edge);
        break;//exit
      }
    }
  }//end while(millis()<timeout)
  Motors(0,0);
  //SwitchButtonToPixels();SetPixelRGB(5,H_Bright,H_Bright/3,H_Bright/3);SetPixelRGB(6,H_Bright,H_Bright/3,H_Bright/3);RefreshPixels();//for behavior 030
  timeout=millis()+MaxExpectedSkidTime;
  while(millis()<timeout){
    NavigationXY(80,800);//lower values to be sure it really is stationary
    if(IsStationary)
      break;
  }

}


void MoveXYWithOptions(int X, int Y, int Speed, int MaxExpectedRunTime, int MaxExpectedSkidTime, void (*EdgeFunction)(char), char Wiggle){//Ver. 1.0, Dustin Soodak
  int32_t xrelative=GetPositionX()-X;
  int32_t yrelative=GetPositionY()-Y;
  int Heading=90-atan2(-yrelative,-xrelative)*180/3.14159;
  int Distance=sqrt(((int32_t)xrelative)*xrelative+((int32_t)yrelative)*yrelative);
  Heading=MinTurn(Heading-GetDegrees())+GetDegrees();
  //SwitchMotorsToSerial();Serial.print("heading ");Serial.print(Heading);Serial.print(" Distance ");Serial.println(Distance);
  if(Distance>0)
    MoveWithOptions(Heading,Distance,Speed,MaxExpectedRunTime,MaxExpectedSkidTime,EdgeFunction,Wiggle);
}



// ***************************************************
// end MovementFunctions
// ***************************************************


// ***************************************************
// Timer
// ***************************************************

int32_t Timer_InitTime=0,Timer_StoppedTime=0;
char Timer_Running=0;
int32_t GetTime(void){//Ver. 1.0, Dustin Soodak
  if(Timer_Running){
    return millis()-(uint32_t)Timer_InitTime;
  }
  else
    return Timer_StoppedTime;
}
void RestartTimer(void){//Ver. 1.0, Dustin Soodak
  Timer_InitTime=millis();
  Timer_Running=1;
}
void StopTimer(void){//Ver. 1.0, Dustin Soodak
  if(Timer_Running){
    Timer_StoppedTime=millis()-(uint32_t)Timer_InitTime;
    Timer_Running=0;
  }
}

// ***************************************************
// end Timer
// ***************************************************


// ***************************************************
// IR Data Sending
// ***************************************************

void EnableIROutputs(char Level){//Ver. 1.0, Dustin Soodak
  if(Level){
    digitalWrite(13,1);
    digitalWrite(12,1);
    digitalWrite(11,1);
  }
  else{
    digitalWrite(13,0);
    digitalWrite(12,0);
    digitalWrite(11,0);
  }
}

void ModulateIR(unsigned int Frequency, unsigned int OnTime){//Ver. 1.0, Dustin Soodak 
  //ModulateIR(38000,6) seems to produce best square wave for 38kHz.
  //Frequency is in Hz
  //OnTime is in units of UsOn
  uint16_t temp;
  uint16_t period,dutycycle;
  uint8_t prescalerbits;
  if(OnTime>100)
    OnTime=100;
  if (8000000L/Frequency/2>=0x10000){
    if(8000000L/Frequency/2>=0x10000*8){
        prescalerbits=0b011;//prescaler 64
        period=8000000L/Frequency/(2*16);
        dutycycle=8000000L/1000000*OnTime/2/64;
    }
    else{
      prescalerbits = 0b010;// prescaler 8
      period=8000000L/Frequency/(2*8);
      dutycycle=8000000L/1000000*OnTime/2/8;
    }
  }
  else{
    prescalerbits = 0b001; //on but no prescaling
    period=8000000L/Frequency/(2*1);
    dutycycle=8000000L/1000000*OnTime/2/1;
  }
  if(OnTime==0){
    
# 1100 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino" 3
   (*(volatile uint8_t *)(0x80))
# 1100 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino"
         =0;
    
# 1101 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino" 3
   (*(volatile uint8_t *)(0x81))
# 1101 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino"
         =0;
    pinMode(9 /*tone(pin, frequency) and noTone(),  or tone(pin, frequency, duration). also look at toneAC library*/,0x0);

    //Serial.println("off");
  }
  else{
    
# 1107 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino" 3
   (*(volatile uint8_t *)(0x81))
# 1107 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino"
         &=~0b00000111;//turn off timer    
    
# 1108 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino" 3
   (*(volatile uint16_t *)(0x86))
# 1108 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino"
       =period;
    
# 1109 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino" 3
   (*(volatile uint16_t *)(0x8A))
# 1109 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino"
        =dutycycle;
    
# 1110 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino" 3
   (*(volatile uint8_t *)(0x80)) 
# 1110 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino"
          = (0b10<<4) | 0b10;//COM1B1 COM1B0, and WGM11 WGM10
    
# 1111 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino" 3
   (*(volatile uint8_t *)(0x81)) 
# 1111 "C:\\Users\\hbrown28\\Documents\\Arduino\\GrantsCode\\RingoHardware.ino"
          = (0b10<<3) | prescalerbits;//WGM13 WGM12, and off/(on with prescaler)    
  }
}

void PlayChirpIR(unsigned int Frequency, unsigned int OnTime){//Ver. 1.0, Dustin Soodak
  // ModulateIR used to be called PlayChirpIR, left in for backward compatibility.
   ModulateIR(Frequency,OnTime);
}

char CheckMenuButton(void){
  byte button;
  if(IsIRDone()){ //wait for an IR remote control command to be received
      button = GetIRButton(); // read which button was pressed, store in "button" variable
      RxIRRestart(4); // restart looking for another buttom press
      if(button == 17){ // button 17 is the "MENU" button on the IR remote
      return 1;
      }
      else{
      return 0;
      }
  }
  return 0;
}

// ***************************************************
// end IR Data Sending
// ***************************************************


// ***************************************************
// Recorded Data
// ***************************************************

  //Ver. 1.0, Dustin Soodak
  //Global variables:
  RecordedDataStruct RecordedDataRow;
  RecordedDataStruct RecordedDataArray[1 /*set to 1 if not using*/];
  unsigned char RecordedDataLength=0;
  unsigned char RecordedDataPosition=0;
  int RecordedDataN;
  uint32_t RecordedDataStart,RecordedDataPrev;
  uint16_t RecordedDataMinDelay;

  //RecordedDataRow.ms=RecordedDataTime()/1000;RecordedDataRow.degr=GetDegrees();RecordedDataRefresh();

// ***************************************************
// end Recorded Data
// ***************************************************
