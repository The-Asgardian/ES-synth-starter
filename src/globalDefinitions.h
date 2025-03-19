#ifndef GLOBAL_DEFINITIONS_H
#define GLOBAL_DEFINITIONS_H

#include "threadSafetyClasses.h"

//Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

//uint8_t TX_Message[8] = {0};
uint8_t RX_Message[8] = {0};

// Active Notes for POLYPHONY
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


#define MAX_POLYPHONY 4

struct SystemState {
    std::bitset<32> inputs;
    ActiveNote activeNotes[MAX_POLYPHONY]; // Track currently active notes
    uint8_t activeNoteCount = 0;           // Count of active notes
    uint8_t currentWaveform = 0;
    SemaphoreHandle_t mutex; 
};

extern SystemState sysState;

void sysStateTake();
void sysStateGive();

uint32_t stepSizes [12];
volatile uint32_t currentStepSize;
volatile uint32_t currentKeyIndex;
volatile uint32_t volume = 6;
volatile uint32_t octave = 4;
const std::string keyNames[12] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};

//std::bitset<8> outValue = {false, false, false, true, true, true, true, true};
bool amTheWestMost = false, amTheEastMost = false, amPREVIOUSLYTheWestMost = false, amPREVIOUSLYTheEastMost = false;
bool allSetUp = false;

int uniqueID = 5;


#endif