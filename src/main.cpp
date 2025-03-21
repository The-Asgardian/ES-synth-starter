#include <Arduino.h>

#include <bitset>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>
#include "debugDefinitions.h"

#include "loopTimeMeasurement.h"

#include "pinDefinitions.h"

#include "globalDefinitions.h"

#include "keyReading.h"

#include "knobs.h"

#include "CANstuff.h"

#include "waveformStuff.h"

#include "displayStuff.h"








TaskHandle_t myPrintHandle = NULL;
void myPrintTask(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(1000);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    #ifdef PRINT_TIMING
      MY_PRINT_LTM.run1();
    #endif
    /*
    Serial.print(allSetUp);
    Serial.print(" ");
    Serial.print(amTheEastMost);
    Serial.print(" ");
    Serial.print(amTheWestMost);
    Serial.print(" ");
    Serial.print(amPREVIOUSLYTheEastMost);
    Serial.print(" ");
    Serial.print(amPREVIOUSLYTheWestMost);
    Serial.println(" ");
    */

    Serial.println("Hi");
    #ifdef PRINT_TIMING
      MY_PRINT_LTM.run2();
    #endif
  }
}

void setup() {

  //Set pins
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setupDisplay();

  #ifdef ALLOW_SAMPLE_ISR
    sampleTimer.setOverflow(22000, HERTZ_FORMAT);
    sampleTimer.attachInterrupt(sampleISR);
    sampleTimer.resume();
  #endif
  //Initialise UART
  Serial.begin(115200);
  delay(2000);
  //while(!Serial){}
  Serial.println("Hello World");

  for(int i = 0; i < 12; i++) stepSizes[i] = stepSizeGenerator(i);

  for(int i = 0; i < 12; i++) Serial.println(stepSizes[i]);

  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);

  initSineTable();

  msgInQ = xQueueCreate(36,8);
  msgOutQ = xQueueCreate(36,8);
  sampleISRtimingQ = xQueueCreate(100,sizeof(long));

  CAN_Init(false);
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  //setCANFilter(0x123,0x7ff);
  setCANFilter(7,0);
  CAN_Start();

  UBaseType_t scanKeysTask_priority = 15; //most important, largest number
  UBaseType_t knobActionTask_priority = 10;
  UBaseType_t receiveQueueTask_priority = 8;
  UBaseType_t CAN_TX_Task_priority = 5;
  UBaseType_t myPrintTask_priority = 2;
  UBaseType_t displayUpdateTask_priority = 1;//least important, smallest number

  #ifdef ALLOW_KEY_MEASUREMENT_THREAD
  xTaskCreate(
              scanKeysTask,		// Function that implements the task 
              "scanKeys",		// Text name for the task 
              256,      		// Stack size in words, not bytes 
              NULL,			// Parameter passed into the task 
              scanKeysTask_priority,			// Task priority 
              &scanKeysHandle );	// Pointer to store the task handle 
  #endif
  
  #ifdef ALLOW_DISPLAY_THREAD
  xTaskCreate(
              displayUpdateTask,		// Function that implements the task 
              "displayUpdate",		// Text name for the task 
              512,      		// Stack size in words, not bytes 
              NULL,			// Parameter passed into the task 
              displayUpdateTask_priority,			// Task priority 
              &displayUpdateHandle );	// Pointer to store the task handle 
  #endif

  #ifdef ALLOW_MY_PRINT_THREAD
  xTaskCreate(
              myPrintTask,		// Function that implements the task 
              "myPrint",		// Text name for the task 
              256,      		// Stack size in words, not bytes 
              NULL,			// Parameter passed into the task 
              myPrintTask_priority,			// Task priority 
              &myPrintHandle );	// Pointer to store the task handle 
  #endif

  #ifdef ALLOW_CAN_RX_THREAD
  xTaskCreate(
              receiveQueueTask,		// Function that implements the task 
              "receiveQueue",		// Text name for the task 
              256,      		// Stack size in words, not bytes 
              NULL,			// Parameter passed into the task 
              receiveQueueTask_priority,			// Task priority 
              &receiveQueueHandle );	// Pointer to store the task handle 
  #endif

  #ifdef ALLOW_CAN_TX_THREAD
  xTaskCreate(
              CAN_TX_Task,		// Function that implements the task 
              "transmitQueue",		// Text name for the task 
              512,      		// Stack size in words, not bytes 
              NULL,			// Parameter passed into the task 
              CAN_TX_Task_priority,			// Task priority 
              &transmitQueueHandle );	// Pointer to store the task handle 
  #endif

  #ifdef ALLOW_KNOB_ACTION_THREAD
  knobActionQueue = xQueueCreate(12, sizeof(KnobActionClass));
  xTaskCreate(
              knobActionTask,  
              "knobAction",     
              256,             
              NULL,             
              knobActionTask_priority,         
              &knobActionHandle          );

  #endif

  #ifdef ALLOW_SAMPLE_ISR
  #ifdef PRINT_TIMING
  xTaskCreate(
              SAMPLE_ISR_TIME_MEASUREMENT_Task,  
              "SAMPLE_ISR_TIME_MEASUREMENT",     
              256,             
              NULL,             
              3,         
              &sampleISRtimeMeasurementTaskHandle          );
  #endif
  #endif
  

  sysState.mutex = xSemaphoreCreateMutex();
  vTaskStartScheduler();
}

#ifdef ALLOW_KNOB_ACTION_THREAD
#ifdef WORST_CASE_TIMING_TESTING
std::bitset<8> knobVals;
int knobStateCounter = 0;

bool knobStates[4][2] = {
    {0, 0},
    {0, 1}, 
    {1, 1},
    {1, 0}  
};
#endif
#endif 

void loop() {
  #ifdef WORST_CASE_TIMING_TESTING
    #ifdef ALLOW_CAN_TX_THREAD
      addkeyPressMessageToTxQueue('R', 3, 4);
    #endif
    #ifdef ALLOW_KNOB_ACTION_THREAD
      knobStateCounter = (knobStateCounter+1)%4;
      knobVals[0] = knobStates[knobStateCounter][0];
      knobVals[1] = knobStates[knobStateCounter][1];
      knobVals[2] = knobStates[knobStateCounter][0];
      knobVals[3] = knobStates[knobStateCounter][1];
      knobVals[4] = knobStates[knobStateCounter][0];
      knobVals[5] = knobStates[knobStateCounter][1];
      knobVals[6] = knobStates[knobStateCounter][0];
      knobVals[7] = knobStates[knobStateCounter][1];
      decodeKnobs(knobVals);
    #endif
  #endif
}