/* 
Grant Testing Framework
*/
#include <Arduino_FreeRTOS.h>
#include "RingoHardware.h"

/***** TEST DRIVER GLOBALS ******/
void TestDriver(void *pvParameters);
TaskHandle_t xMasterHandle;
unsigned long startTime = 0, endTime = 0, wcet = 0;
#define ITERATIONS 10
/***** END TEST DRIVER GLOBALS ******/

/***** TASK-UNDER-TEST GLOBALS ******/
void TaskSensing(void *pvParameters);
TaskHandle_t xSensingHandle, xSpinHandle;
char edge;
int edgeDetectorPeriod = 100;
/***** END TASK-UNDER-TEST GLOBALS ******/

void setup(){
  /*** Begin Ringo Init  ***/
  HardwareBegin();
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
  /*** End Ringo Init  ***/
  
  // Test Driver Task
  xTaskCreate(
  TestDriver
  ,  (const portCHAR *)"Test Driver"
  ,  128
  ,  NULL
  ,  1
  ,  &xMasterHandle
  );

  // Task-under-test
  xTaskCreate(
  TaskSensing
  ,  (const portCHAR *)"Check for edges"
  ,  128 
  ,  NULL
  ,  3 
  ,  &xSensingHandle
  );

  vTaskSuspend(xSensingHandle);
}

void loop(){}

void TestDriver(void *pvParameters)
{
  for(int i=0; i<ITERATIONS; i++) {
    vTaskResume(xSensingHandle);
    updateWCET();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }

  Serial.print("Task Under Test WCET: ");
  Serial.println(wcet);
}

void TaskSensing(void *pvParameters) {
  startTime = micros();

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  edge = LookForEdge();
  if(FrontEdgeDetected(edge))
  {
    // Handle right front edge detected
    vTaskResume(xSpinHandle);
    // vTaskSuspend(xSensingHandle);
  }

  endTime = micros();
  vTaskSuspend(xSensingHandle);
}

void updateWCET() {
  wcet = max(endTime - startTime, wcet);
}