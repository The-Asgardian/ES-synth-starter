#ifndef GLOBAL_DEFINITIONS_H
#define GLOBAL_DEFINITIONS_H

#include "threadSafetyClasses.h"

uint8_t RX_Message[8] = {0};


//active Notes for POLYPHONY
struct ActiveNote {
    bool active;
    uint8_t noteIndex;
    uint32_t stepSize;
};



#define TABLE_SIZE 256
uint8_t sineTable[TABLE_SIZE];



void initSineTable() {
    for (int i = 0; i < TABLE_SIZE; i++) {
        sineTable[i] = (uint8_t)(127.0f * sinf(2 * PI * i / TABLE_SIZE));
    }
}



#define MAX_POLYPHONY 10
std::string waveformNames[] = {"Sine", "Square", "Triangle", "Sawtooth"};



struct SystemState {

    std::bitset<32> inputs;

    ActiveNote activeNotes[MAX_POLYPHONY]; 

    uint8_t activeNoteCount = 0;        

    uint8_t currentWaveform = 0;

    SemaphoreHandle_t mutex; 

};


SystemState sysState;



uint32_t stepSizes [12];
volatile uint32_t currentStepSize;
volatile uint32_t currentKeyIndex;
ThreadSafeBoundedInteger volume(6, 0, 8);
ThreadSafeBoundedInteger octave(4);
const std::string keyNames[12] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};

bool amTheWestMost = false, amTheEastMost = false, amPREVIOUSLYTheWestMost = false, amPREVIOUSLYTheEastMost = false;
bool allSetUp = false;

int uniqueID = 5;

  void sysStateTake() {
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
  }
  
  void sysStateGive() {
    xSemaphoreGive(sysState.mutex);
  }


#endif