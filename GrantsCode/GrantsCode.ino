/* 
Lab4
Navigates a maze with two obstacles by following the policy "always turn left" when it reaches edges until it reaches the goal. We turn on lights to represent the state 
the Ringo is in so we can debug our bug.
*/
#include "RingoHardware.h"

void TaskNavigateMaze(void *pvParameters);
void TaskController(void *pvParameters);
void TaskSensing(void *pvParameters);

// Globals

float guidancePeriod = 300;
float controllerPeriod = 150;
int sensingPeriod = 50;

// queue<int> readyTasks;

int intendedHeading;
int error = 0;

int baseSpeed = 30;
int maxSpeed = 100;
int motorBias = 5; //Our left motor is stronger than our right motor so this should account for that

int rightTurnAngle = 71;
int leftTurnAngle = -67;

char lastEdge = 0x0;
char edge;

enum Manuever { DriveStraight, Backup, TurningLeft};
Manuever manuever;

unsigned long startTime;
int backingupTimeLimit = 1500;
unsigned long time;
// SetAllPixelsRGB(0,0,0);

// Controller
float Kp;
float Ki;
float Kd;

int P, I, D;
int currentHeading;
int lastError;
int speedLeft;
int speedRight;
Manuever currentManuever;

int u;

// Measured wcets in Lab 4
float guidance_wcet = 0.576;
float controller_wcet = 12.4;
float sensing_wcet = 1.32;

bool guidance_executed_this_frame;
bool controller_executed_this_frame;
bool sensing_executed_this_frame;

unsigned long taskStartTime;
unsigned long currentframeExecutionTime;

int frame = 30;
int hyperFrame = 300;


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

  intendedHeading = PresentHeading();
  manuever = DriveStraight;
  edge = 0x0;

  // Initialization of once local variables 
  // NavigateMaze
  startTime = millis();
  backingupTimeLimit = 1500;
  time = millis() - startTime;
  SetAllPixelsRGB(0,0,0);

  // Controller
  Kp = 1;
  Ki = 1;
  Kd = 1;

  I = 0, D = 0;
  currentHeading;
  lastError = 0;
  speedLeft = 50;
  speedRight = 50;
  currentManuever;

}

void loop() {
  // Serial.println("looping");
  guidance_executed_this_frame = false;
  controller_executed_this_frame = false;
  sensing_executed_this_frame = false;
  bool allTasksExecuted = false;
  unsigned long hyperframeStartTime = millis();

  while(!allTasksExecuted) {
    taskStartTime = millis();
    ScheduleNextPriorityTask();
    currentframeExecutionTime = millis() - taskStartTime;
    // if extra time in frame execute a task that fits - maybe i should test what i have before getting too fancy here

    if(guidance_executed_this_frame && controller_executed_this_frame && sensing_executed_this_frame) {
      allTasksExecuted = true;
    }
  }
  unsigned long currentHyperframeExecTime = millis() - hyperframeStartTime;

  delay(hyperFrame - currentHyperframeExecTime); // might need to add port tick microseconds


}

void ScheduleNextPriorityTask() {
  if(!sensing_executed_this_frame) {
    TaskSensing();
    sensing_executed_this_frame = true;
  }
  else if(!controller_executed_this_frame) {
    TaskController();
    controller_executed_this_frame = true;
  }
  else if(!guidance_executed_this_frame) {
    TaskNavigateMaze();
    guidance_executed_this_frame = true;
  }
  else {
    Serial.println("Something is wrong, there are no tasks to schedule");
  }
}

void ScheduleNextTaskThatFits() {
  if(!sensing_executed_this_frame) {
    TaskSensing();
    sensing_executed_this_frame = true;
  }
  else if(!controller_executed_this_frame) {
    TaskController();
    controller_executed_this_frame = true;
  }
  else if(!guidance_executed_this_frame) {
    TaskNavigateMaze();
    guidance_executed_this_frame = true;
  }
}

void TaskNavigateMaze() {
  // Serial.println("navigating");
  // OnEyes(0, 0, 100);
  // unsigned long startTime = millis();
  // int backingupTimeLimit = 1500;
  // unsigned long time = millis() - startTime;
  // SetAllPixelsRGB(0,0,0);

  time = millis() - startTime;

  // Logic for when to change state
  if (manuever == DriveStraight) {
    SetPixelRGB(TAIL_TOP, 0, 0, 100); // Blue
    if(FrontEdgeDetected(edge)) {
      manuever = Backup;
      startTime = millis();
    }
  } else if (manuever == Backup) {
      SetPixelRGB(TAIL_TOP, 0, 100, 0); // Green

      bool timeLimitReached = (time >= backingupTimeLimit) ? true : false;
      if(timeLimitReached) {
          manuever = TurningLeft;
          lastEdge = 0x0;
      }
  }
    else if (manuever == TurningLeft) {
      SetPixelRGB(TAIL_TOP, 100, 100, 0); // Purple
      if(error == 0) {
        manuever = DriveStraight;
      }
  }

}

void TaskController() {
  // Serial.println("controlling");
  // OnEyes(100, 0, 0);
  // float Kp = 1;
  // float Ki = 1;
  // float Kd = 1;

  // int P, I = 0, D = 0;
  // int currentHeading;
  // int lastError = 0;
  // int speedLeft = 50;
  // int speedRight = 50;
  // Manuever currentManuever;

  // int u;

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
        baseSpeed = 30;
        break;
      case Backup:
        SetPixelRGB(BODY_TOP, 0, 100, 0);
        intendedHeading = PresentHeading();
        Kp = 1;
        Ki = 1;
        Kd = 1;
        baseSpeed = -30;
        break;
      case TurningLeft:
        SetPixelRGB(BODY_TOP, 100, 100, 0);
        intendedHeading = PresentHeading() + (leftTurnAngle / 4);
        Kp = 1;
        Ki = 0;
        Kd = 0;
        baseSpeed = 20;
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
    speedRight = -u + baseSpeed + motorBias;
    speedLeft = manuever == TurningLeft ? -(-u + baseSpeed) : baseSpeed;
  }
  else if(error > 0) {
    OnEyes(0,100,0); // Green
    speedLeft = u + baseSpeed;
    speedRight = manuever == TurningLeft ? -(-u + baseSpeed + motorBias) : baseSpeed;
  }
  else {
    OffEyes();
    if(currentManuever != Backup) {
      speedLeft = max(baseSpeed - motorBias, 0);
      speedRight = max(baseSpeed, 0);
    }
    else {
      speedLeft = baseSpeed - motorBias;
      speedRight = baseSpeed;
    }
  }

  lastError = error;

  speedLeft = min(speedLeft, maxSpeed);
  speedRight = min(speedRight, maxSpeed);
  Motors(speedLeft, speedRight);
}

void TaskSensing() {
  // Serial.println("sensing");
  // OnEyes(0, 100, 0);
  edge = LookForEdge();
  if(FrontEdgeDetected(edge)) {
      lastEdge = edge;
  }
}