#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>
#include "loopTimeMeasurement.h"

#include "pinDefinitions.h"

#include "globalDefinitions.h"

#include "keyReading.h"

#include "knobs.h"

#include "CANstuff.h"


loopTimeMeasurement ltm("key reading", 100);

HardwareTimer sampleTimer(TIM1);

SystemState sysState;

void sysStateTake() {
  xSemaphoreTake(sysState.mutex, portMAX_DELAY);
}

void sysStateGive() {
  xSemaphoreGive(sysState.mutex);
}

int stepSizeGenerator(int index) {
  float Fs = 22000;
  float FA = 440*float(pow(2.0, float(octave)-4.0));
  float factor = pow(2.0, (float) (float(index)-9.0)*(1.0/12.0));
  float f = FA*factor;
  int step = int((float) pow(2.0, 32.0)*(f/Fs));
  return step;
}

int stepSize(int index, int octaveLocal) {
  int step = stepSizes[index];
  int octaveDiff = octaveLocal - 4;
  step = step*pow(2,octaveDiff);
  //if(octaveDiff < 0) step >> -octaveDiff;
  //if(octaveDiff > 0) step << octaveDiff;
  return step;
}

void sampleISR() {
  static uint32_t phaseAccs[MAX_POLYPHONY] = {0};
  int32_t mixedOutput = 0;
  uint8_t activeVoices = 0;

  int volume_local = __atomic_load_n(&volume, __ATOMIC_RELAXED);
  uint8_t waveform_local = __atomic_load_n(&sysState.currentWaveform, __ATOMIC_RELAXED);

  ActiveNote activeNotesLocal[MAX_POLYPHONY];

  // ISR-safe atomic copy of notes (no mutex)
  for (uint8_t i = 0; i < MAX_POLYPHONY; i++) {
    activeNotesLocal[i].active = __atomic_load_n(&sysState.activeNotes[i].active, __ATOMIC_RELAXED);
    activeNotesLocal[i].stepSize = __atomic_load_n(&sysState.activeNotes[i].stepSize, __ATOMIC_RELAXED);
  }

  // Mix all active notes clearly without relying on contiguous indices
  for (uint8_t i = 0; i < MAX_POLYPHONY; i++) {
    if (activeNotesLocal[i].active) {
      phaseAccs[i] += activeNotesLocal[i].stepSize;
      uint8_t phase = (phaseAccs[i] >> 24) & 0xFF;

      int32_t voiceOutput = 0;

      switch (sysState.currentWaveform) {
        case 0: // Sine wave approximation
        voiceOutput = (((int32_t)sineTable[phase]) - 128) * 3;
          break;

        case 1: // Square wave
          voiceOutput = (phase < 128) ? 127 : -128;
          break;

        case 2: // Triangle wave
          voiceOutput = (phase < 128) ? (-128 + (phase * 2)) : (127 - ((phase - 128) * 2)) * 3;   
          break;         
        case 3: // Sawtooth wave
          voiceOutput = phase - 128;
          break;

        default:
          voiceOutput = phase - 128; // Default to sawtooth
      }

      mixedOutput += voiceOutput;
      activeVoices++;
    } else {
      phaseAccs[i] = 0; // Reset inactive voice
    }
  }

  if (activeVoices > 0) {
    mixedOutput /= activeVoices; // Normalize based on active voices
  } else {
    mixedOutput = 0; // No active notes
  }

  mixedOutput = mixedOutput >> (8 - volume_local);

  analogWrite(OUTR_PIN, mixedOutput + 128);
}









TaskHandle_t scanKeysHandle = NULL;
void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  std::bitset<32> prevKeyState;

    // Initialize activeNotes array
    if (xSemaphoreTake(sysState.mutex, portMAX_DELAY) == pdTRUE)
    {
      for (uint8_t i = 0; i < MAX_POLYPHONY; i++)
      {
        sysState.activeNotes[i].active = false;
        sysState.activeNotes[i].noteIndex = 0;
        sysState.activeNotes[i].stepSize = 0;
      }
      sysState.activeNoteCount = 0;
      xSemaphoreGive(sysState.mutex);
    }


  while(1) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    ltm.run1();
    // Track notes that need to be activated/deactivated
    std::bitset<12> notesToActivate;   // New notes to turn on
    std::bitset<12> notesToDeactivate; // Notes to turn off
    std::bitset<32> sysState_inputs_local;
    for (int i = 0; i < 7; i++) {
      std::bitset<4> rowInputs = readRow(i);
      for(int j = 0; j < 4; j++) sysState_inputs_local[i*4+j] = rowInputs[j];
    }

    //keys decoding
    /*
    int currentStepSize_local = 0;
    int currentKeyIndex_local = -1;
    for(int i = 0; i < 12; i++) {
      if(sysState_inputs_local[i] == true) {
        currentStepSize_local = stepSizes[i];
        currentKeyIndex_local = i;
      }
    }
      */
    std::bitset<8> knobABs;
    knobABs[0] = sysState_inputs_local[18];
    knobABs[1] = sysState_inputs_local[19];
    knobABs[2] = sysState_inputs_local[16];
    knobABs[3] = sysState_inputs_local[17];
    knobABs[4] = sysState_inputs_local[14];
    knobABs[5] = sysState_inputs_local[15];
    knobABs[6] = sysState_inputs_local[12];
    knobABs[7] = sysState_inputs_local[13];
    decodeKnobs(knobABs);

    bool WestHS = sysState_inputs_local[23];
    bool EastHS = sysState_inputs_local[27];

    bool amTheWestMost_local = !WestHS;
    bool amTheEastMost_local = !EastHS;
   
    __atomic_store_n(&amTheWestMost, amTheWestMost_local, __ATOMIC_RELAXED);
    
    __atomic_store_n(&amTheEastMost, amTheEastMost_local, __ATOMIC_RELAXED);

    if(!WestHS) __atomic_store_n(&amPREVIOUSLYTheWestMost, true, __ATOMIC_RELAXED);
    if(!EastHS) __atomic_store_n(&amPREVIOUSLYTheEastMost, true, __ATOMIC_RELAXED);



    //if(amPREVIOUSLYTheWestMost && !amTheWestMost && amTheEastMost) requestPreviousEndpointOctave('E');
    //if(amPREVIOUSLYTheEastMost && !amTheEastMost && amTheWestMost) requestPreviousEndpointOctave('W');
    if(!allSetUp && amTheEastMost_local && !amTheWestMost_local) requestPreviousEndpointOctave('E');
    if(!allSetUp && amTheWestMost_local && !amTheEastMost_local) requestPreviousEndpointOctave('W');
    if(amTheWestMost_local && amTheEastMost_local) allSetUp = true;

    //prevWestHS = WestHS;
    //prevEastHS = EastHS;


    // Scanning Musical Keys
    sysStateTake();
    for(int i = 0; i < 12; i++) {
      bool wasPressed = prevKeyState[i];
      bool isPressed = sysState_inputs_local[i];
  
      // Key just pressed
      if(!wasPressed && isPressed) {
        if(!amTheWestMost_local)
          addkeyPressMessageToTxQueue('P', i, octave);
        else {
          bool added = false;
          for(uint8_t j = 0; j < MAX_POLYPHONY; j++) {
            if(!sysState.activeNotes[j].active) {
              sysState.activeNotes[j].active = true;
              sysState.activeNotes[j].noteIndex = i;
              sysState.activeNotes[j].stepSize = stepSize(i, octave);
              sysState.activeNoteCount++;
              added = true;
              break;
            }
          }
          if(!added) {
            Serial.println("Max Polyphony reached!");
          }
        }
      }
  
      // Key just released
      if(wasPressed && !isPressed) {
        if(!amTheWestMost_local)
          addkeyPressMessageToTxQueue('R', i, octave);
        else {
          for(uint8_t j = 0; j < MAX_POLYPHONY; j++) {
            if(sysState.activeNotes[j].active && sysState.activeNotes[j].noteIndex == i) {
              sysState.activeNotes[j].active = false;
              sysState.activeNotes[j].stepSize = 0;
              sysState.activeNoteCount--;
              break;
            }
          }
        }
      }
    }
  
    prevKeyState = sysState_inputs_local;
    sysStateGive();
  
    ltm.run2();
  }
}


TaskHandle_t displayUpdateHandle = NULL;
void displayUpdateTask(void * pvParameters) {
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    //Update display
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(2,10,"Hello World!");  // write something to the internal memory
    u8g2.setCursor(2,20);
    //u8g2.print(inputs.to_ulong(),HEX); 
    int currentKeyIndex_local = __atomic_load_n(&currentKeyIndex, __ATOMIC_RELAXED);
    if(currentKeyIndex_local == -1) u8g2.print("-"); 
    else u8g2.print((keyNames[currentKeyIndex]).c_str()); 
    int volume_local = __atomic_load_n(&volume, __ATOMIC_RELAXED);
    //u8g2.setCursor(2,20);
    u8g2.drawStr(2, 30, (std::to_string(volume_local)).c_str()); 

    u8g2.setCursor(110,10);
    u8g2.print(octave);


    u8g2.setCursor(66,30);
    
    u8g2.print((char) RX_Message[0]);
    u8g2.print((char) RX_Message[1]);
    u8g2.print(RX_Message[2]);
    
    u8g2.sendBuffer();          // transfer internal memory to the display
    //Serial.println(aaaa);
    //Toggle LED
    digitalToggle(LED_BUILTIN);
  }
}




TaskHandle_t myPrintHandle = NULL;
void myPrintTask(void *pvParameters) {
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1) {
      vTaskDelayUntil( &xLastWakeTime, xFrequency );
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
      //Serial.println(uxTaskGetStackHighWaterMark(myPrintHandle));//__atomic_load_n(&volume, __ATOMIC_RELAXED)); // Get stack usage for this task
  }
}

void setup() {
  // put your setup code here, to run once:

  //Set pin directions
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
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  initSineTable();

  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();

  //Initialise UART
  Serial.begin(9600);
  delay(2000);
  //while(!Serial){}
  Serial.println("Hello World");

  for(int i = 0; i < 12; i++) stepSizes[i] = stepSizeGenerator(i);

  for(int i = 0; i < 12; i++) Serial.println(stepSizes[i]);

  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);

  msgInQ = xQueueCreate(36,8);
  msgOutQ = xQueueCreate(36,8);

  CAN_Init(false);
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  //setCANFilter(0x123,0x7ff);
  setCANFilter(7,0);//);
  CAN_Start();

  UBaseType_t scanKeysTask_priority = 15; //most important, largest number
  UBaseType_t knobActionTask_priority = 10;
  UBaseType_t receiveQueueTask_priority = 8;
  UBaseType_t CAN_TX_Task_priority = 5;
  UBaseType_t myPrintTask_priority = 2;
  UBaseType_t displayUpdateTask_priority = 1;//least important, smallest number


  xTaskCreate(
              scanKeysTask,		// Function that implements the task 
              "scanKeys",		// Text name for the task 
              256,      		// Stack size in words, not bytes 
              NULL,			// Parameter passed into the task 
              scanKeysTask_priority,			// Task priority 
              &scanKeysHandle );	// Pointer to store the task handle 
  
  xTaskCreate(
              displayUpdateTask,		// Function that implements the task 
              "displayUpdate",		// Text name for the task 
              512,      		// Stack size in words, not bytes 
              NULL,			// Parameter passed into the task 
              displayUpdateTask_priority,			// Task priority 
              &displayUpdateHandle );	// Pointer to store the task handle 
  
  xTaskCreate(
              myPrintTask,		// Function that implements the task 
              "myPrint",		// Text name for the task 
              256,      		// Stack size in words, not bytes 
              NULL,			// Parameter passed into the task 
              myPrintTask_priority,			// Task priority 
              &myPrintHandle );	// Pointer to store the task handle 
  xTaskCreate(
              receiveQueueTask,		// Function that implements the task 
              "receiveQueue",		// Text name for the task 
              256,      		// Stack size in words, not bytes 
              NULL,			// Parameter passed into the task 
              receiveQueueTask_priority,			// Task priority 
              &receiveQueueHandle );	// Pointer to store the task handle 
  xTaskCreate(
              CAN_TX_Task,		// Function that implements the task 
              "transmitQueue",		// Text name for the task 
              512,      		// Stack size in words, not bytes 
              NULL,			// Parameter passed into the task 
              CAN_TX_Task_priority,			// Task priority 
              &transmitQueueHandle );	// Pointer to store the task handle 
  
  knobSetup(knobActionTask_priority);

  sysState.mutex = xSemaphoreCreateMutex();
  vTaskStartScheduler();
}

void loop() {
}