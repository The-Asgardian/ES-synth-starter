#ifndef KEY_READING_H
#define KEY_READING_H

#include "pinDefinitions.h"
#include "debugDefinitions.h"

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
    digitalWrite(REN_PIN,LOW);
    digitalWrite(RA0_PIN, bitIdx & 0x01);
    digitalWrite(RA1_PIN, bitIdx & 0x02);
    digitalWrite(RA2_PIN, bitIdx & 0x04);
    digitalWrite(OUT_PIN,value);
    digitalWrite(REN_PIN,HIGH);
    delayMicroseconds(2);
    digitalWrite(REN_PIN,LOW);
}

std::bitset<4> readCols() {
    std::bitset<4> result;
    result[0] = !digitalRead(C0_PIN);
    result[1] = !digitalRead(C1_PIN);
    result[2] = !digitalRead(C2_PIN);
    result[3] = !digitalRead(C3_PIN);
    return result;
}

std::bitset<4> readRow(const uint8_t bitIdx) {
    digitalWrite(REN_PIN,LOW);
    digitalWrite(RA0_PIN, bitIdx & 0x01);
    digitalWrite(RA1_PIN, bitIdx & 0x02);
    digitalWrite(RA2_PIN, bitIdx & 0x04);

    //digitalWrite(OUT_PIN, outValue[bitIdx]);
    digitalWrite(OUT_PIN, HIGH);

    digitalWrite(REN_PIN,HIGH);
    delayMicroseconds(3);
    std::bitset<4> inputs = readCols();
    digitalWrite(REN_PIN,LOW);
    return inputs;
}


void decodeKnobs(std::bitset<8> vals);
void requestPreviousEndpointOctave(char direction);
void addkeyPressMessageToTxQueue(char command, byte keyIndex, byte octaveIndex);
int stepSize(int index, int octaveLocal);


void processPressedKey(int index, int octaveLocal) {
    if(!__atomic_load_n(&amTheWestMost, __ATOMIC_RELAXED)) addkeyPressMessageToTxQueue('P', index, octaveLocal);
    else {
        bool added = false;
        for(uint8_t j = 0; j < MAX_POLYPHONY; j++) {
        if(!sysState.activeNotes[j].active) {
            sysState.activeNotes[j].active = true;
            sysState.activeNotes[j].noteIndex = index;
            sysState.activeNotes[j].stepSize = stepSize(index, octaveLocal);
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

void processReleasedKey(int index, int octaveLocal) {
    if(!__atomic_load_n(&amTheWestMost, __ATOMIC_RELAXED)) addkeyPressMessageToTxQueue('R', index, octaveLocal);
    else {
      for(uint8_t j = 0; j < MAX_POLYPHONY; j++) {
        if(sysState.activeNotes[j].active && sysState.activeNotes[j].noteIndex == index) {
          sysState.activeNotes[j].active = false;
          sysState.activeNotes[j].stepSize = 0;
          sysState.activeNoteCount--;
          break;
        }
      }
    }
}


TaskHandle_t scanKeysHandle = NULL;
void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  std::bitset<32> prevKeyState;
      // initialize activeNotes
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
    #ifdef PRINT_TIMING
        KEY_MEASUREMENT_LTM.run1();
    #endif

std::bitset<12> notesToActivate;  
std::bitset<12> notesToDeactivate;
std::bitset<32> sysState_inputs_local;
for (int i = 0; i < 7; i++) {
  std::bitset<4> rowInputs = readRow(i);
  for(int j = 0; j < 4; j++) sysState_inputs_local[i*4+j] = rowInputs[j];
}


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

if(!allSetUp && amTheEastMost_local && !amTheWestMost_local) requestPreviousEndpointOctave('E');
if(!allSetUp && amTheWestMost_local && !amTheEastMost_local) requestPreviousEndpointOctave('W');
if(amTheWestMost_local && amTheEastMost_local) allSetUp = true;

// scanning Musical Keys
sysStateTake();
for(int i = 0; i < 12; i++) {
  bool wasPressed = prevKeyState[i];
  bool isPressed = sysState_inputs_local[i];

  // key just pressed
  #ifndef WORST_CASE_TIMING_TESTING
  if(!wasPressed && isPressed) {
    processPressedKey(i, octave.get());
  }

  // key just released
  if(wasPressed && !isPressed) {
    processReleasedKey(i, octave.get());
  }
  #endif

  #ifdef WORST_CASE_TIMING_TESTING
  processPressedKey(i, octave.get());
  processReleasedKey(i, octave.get());
  #endif
}

prevKeyState = sysState_inputs_local;
sysStateGive();
    #ifdef PRINT_TIMING
        KEY_MEASUREMENT_LTM.run2();
    #endif
  }
}

#endif