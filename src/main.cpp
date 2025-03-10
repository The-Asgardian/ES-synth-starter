#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <cmath>
#include <HardwareTimer.h>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>


// Test Mode Selection - Uncomment to enable specific test
// #define DISABLE_THREADS        // Uncomment to disable thread creation
// #define TEST_SCANKEYS          // Uncomment to test scanKeysTask
// #define TEST_DISPLAY_UPDATE    // Uncomment to test displayUpdateTask
// #define TEST_CAN_TX            // Uncomment to test CAN_TX_Task
// #define TEST_SAMPLE_ISR        // Uncomment to test sampleISR
// #define TEST_FREE_RTOS_STATS   // Uncomment to enable FreeRTOS statistics (requires config)

// Test configuration
#define TEST_ITERATIONS 32    // Number of iterations for timing measurements

#define MAX_POLYPHONY 4


struct ActiveNote {
  bool active;
  uint8_t noteIndex;
  uint32_t stepSize;
};

HardwareTimer sampleTimer(TIM1); // Create global timer object using TIM1

//Constants
const uint32_t interval = 100; //Display update interval
const float sampleRate = 22000.0f; // Sample rate in Hz
const float baseFrequency = 440.0f; // Frequency of A4
const uint8_t WAVEFORM_COUNT = 4; // Number of waveforms
const uint32_t stepSizes[12] = {
  51076056,  // Note 1
  54113197,  // Note 2
  57330935,  // Note 3
  60740009,  // Note 4
  64351798,  // Note 5
  68178356,  // Note 6
  72232452,  // Note 7
  76527617,  // Note 8
  81078186,  // Note 9
  85899345,  // Note 10
  91007186,  // Note 11
  96418755   // Note 12
};

const char* waveformNames[] = {
  "Sine", "Square", "Triangle", "Sawtooth"
};

const char* noteNames[12] = {
  "C4", "C#4", "D4", "D#4", "E4", "F4",
  "F#4", "G4", "G#4", "A4", "A#4", "B4"
};

//Pin definitions
//Row select and enable
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

//Matrix input and output
const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

//Audio analogue out
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

//Joystick analogue in
const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

//Output multiplexer bits
const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

// Global state shared between tasks
struct {
  std::bitset<32> inputs;
  int8_t selectedNote = -1;
  uint8_t volume = 8;
  int8_t knobRotation = 0;
  uint8_t currentWaveform = 0; // 0:Sine, 1:Square, 2:Triangle, 3:Sawtooth
  ActiveNote activeNotes[MAX_POLYPHONY]; // Array of currently active notes
  uint8_t activeNoteCount = 0;       // Number of currently active notes
  SemaphoreHandle_t mutex;
} sysState;

// Mutex for protecting sysState
SemaphoreHandle_t sysStateSemaphore;
SemaphoreHandle_t CAN_TX_Semaphore;

//Global variables
volatile uint32_t currentStepSize = 0; //Current step size for encoder
volatile int8_t lastVout = 0; // To debug audio output
QueueHandle_t msgInQ; //Queue for incoming messages
QueueHandle_t msgOutQ; //Queue for outgoing messages
uint8_t RX_Message[8] = {0}; //Incoming message buffer
uint8_t TX_Message[8] = {0}; // Outgoing CAN message buffer
// Add debounce counter for waveform changes
volatile uint8_t waveformDebounceCounter = 0;
const uint8_t WAVEFORM_DEBOUNCE_THRESHOLD = 2; // Adjust as needed

#define SENDER_MODE // Uncomment this for sender mode, comment for receiver mode




// Task handles for control
TaskHandle_t scanKeysHandle = NULL;
TaskHandle_t displayUpdateHandle = NULL;
TaskHandle_t canTxHandle = NULL;

//Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);



void updateVolume() {
  int8_t knobValue = sysState.knobRotation;
  uint8_t newVolume = map(knobValue, 0, 1023, 0, 255);
  if (xSemaphoreTake(sysState.mutex, portMAX_DELAY) == pdTRUE) {
    sysState.volume = newVolume;
    xSemaphoreGive(sysState.mutex);
  }
}

void setRow(uint8_t rowIdx) {
  // Disable row selection to prevent glitches
  digitalWrite(REN_PIN, LOW);
  
  // Set row select address
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, rowIdx & 0x02);
  digitalWrite(RA2_PIN, rowIdx & 0x04);
  
  // Enable row selection
  digitalWrite(REN_PIN, HIGH);
}

std::bitset<4> readCols(){
  std::bitset<4> result;

  // Read column inputs
  result[0] = !digitalRead(C0_PIN);
  result[1] = !digitalRead(C1_PIN);
  result[2] = !digitalRead(C2_PIN);
  result[3] = !digitalRead(C3_PIN);
  
  return result;
}

// ISR Test Function - separated to allow direct calling for timing
// 4. Modified sampleISR to support polyphony
void sampleISR_test() {
  static uint32_t phaseAccs[MAX_POLYPHONY] = {0}; // Phase accumulators for each voice
  int32_t mixedOutput = 0;
  bool anyActiveNotes = false;
  
  // Get current waveform type and volume - using atomics for minimal ISR overhead
  uint8_t waveform = __atomic_load_n(&sysState.currentWaveform, __ATOMIC_RELAXED);
  uint8_t volume = __atomic_load_n(&sysState.volume, __ATOMIC_RELAXED);
  
  // Safety check for waveform
  if (waveform > 3) waveform = 0;
  
  // Process each voice using mutex-protected access
  if (xSemaphoreTakeFromISR(sysState.mutex, NULL) == pdTRUE) {
    // Process each active voice
    for (uint8_t i = 0; i < MAX_POLYPHONY; i++) {
      if (sysState.activeNotes[i].active && sysState.activeNotes[i].stepSize > 0) {
        anyActiveNotes = true;
        
        // Update phase accumulator for this voice
        phaseAccs[i] += sysState.activeNotes[i].stepSize;
        
        // Generate waveform based on the phase
        uint8_t phase = phaseAccs[i] >> 24;
        int32_t voiceOutput = 0;
        
        // Generate the appropriate waveform
        switch (waveform) {
          case 0: // Sine wave (using lookup table)
          voiceOutput = (uint8_t)(127.5f + 127.5f * sinf(2.0f * M_PI * phase / 256.0f));
            break;
            
          case 1: // Square wave
            voiceOutput = (phase < 128) ? 120 : -120;
            break;
            
          case 2: // Triangle wave
            if (phase < 128) {
              voiceOutput = -120 + (phase * 240 / 128);
            } else {
              voiceOutput = 120 - ((phase - 128) * 240 / 128);
            }
            break;
            
          case 3: // Sawtooth wave
            voiceOutput = ((int32_t)phase * 240 / 256) - 120;
            break;
        }
        
        // Add this voice to the mix (with scaling to prevent overflow)
        mixedOutput += voiceOutput / MAX_POLYPHONY;
      } else if (sysState.activeNotes[i].active && sysState.activeNotes[i].stepSize == 0) {
        // Reset phase accumulator for inactive notes to prevent issues
        phaseAccs[i] = 0;
      }
    }
    xSemaphoreGiveFromISR(sysState.mutex, NULL);
  }
  
  // If no notes are active, output silence
  if (!anyActiveNotes) {
    #ifndef TEST_SAMPLE_ISR
    analogWrite(OUTL_PIN, 128);
    analogWrite(OUTR_PIN, 128);
    #endif
    return;
  }
  
  // Apply volume
  if (volume > 0) {
    mixedOutput = (mixedOutput * volume) / 8;
  } else {
    mixedOutput = 0;
  }
  
  // Convert to DAC range and output
  uint8_t dacOut = constrain(mixedOutput + 128, 0, 255);
  
  #ifndef TEST_SAMPLE_ISR
  analogWrite(OUTL_PIN, dacOut);
  analogWrite(OUTR_PIN, dacOut);
  #endif
}

// Actual ISR that will be attached to timer
void sampleISR() {
  sampleISR_test();
}

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, bitIdx & 0x01);
  digitalWrite(RA1_PIN, bitIdx & 0x02);
  digitalWrite(RA2_PIN, bitIdx & 0x04);
  digitalWrite(OUT_PIN, value);
  digitalWrite(REN_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(REN_PIN, LOW);
}

void CAN_RX_ISR (void) {
  uint8_t RX_Message_ISR[8];
  uint32_t ID;
  CAN_RX(ID, RX_Message_ISR);
  xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

// Modified CAN_TX_Task for testing
void CAN_TX_Task(void * pvParameters) {
  uint8_t msgOut[8];
  
  #ifdef TEST_CAN_TX
  // For testing: don't block, process all available messages
  while (xQueueReceive(msgOutQ, msgOut, 0) == pdTRUE) {
    if (xSemaphoreTake(CAN_TX_Semaphore, 0) == pdTRUE) {
      CAN_TX(0x123, msgOut);
    }
  }
  return;
  #endif
  
  // Normal operation
  while (1) {
    xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
    xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
    CAN_TX(0x123, msgOut);
  }
}

void CAN_TX_ISR (void) {
  xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}


int8_t decodeKnobRotation(uint8_t prevState, uint8_t currentState) {
  // Combined states: previous state in high bits, current state in low bits
  uint8_t combinedState = (prevState << 2) | currentState;
  
  switch (combinedState) {
    // Clockwise rotation patterns
    case 0b0001:
    case 0b1110:
      return 1;
      
    // Counter-clockwise rotation patterns
    case 0b0100:
    case 0b1011:
      return -1;
      
    // Indeterminate but likely continuation patterns
    case 0b0010:
    case 0b1001:
    case 0b1100:
    case 0b0110:
      // Continue previous trend to smooth out rotation
      return 0;
      
    // All other transitions - no rotation
    default:
      return 0;
  }
}
// Modified scanKeysTask for testing
void scanKeysTask(void * pvParameters) {
  // Timing constants
  const TickType_t SCAN_FREQUENCY = 10 / portTICK_PERIOD_MS;
  const uint8_t KEY_DEBOUNCE_TIME = 3; // milliseconds
  
  // State variables for encoders
  static uint8_t prevVolumeKnobState = 0;
  static uint8_t prevWaveformKnobState = 0;
  
  // Local working variables
  TickType_t lastWakeTime = xTaskGetTickCount();
  int8_t volumeKnobRotation = 0;
  int8_t waveformKnobRotation = 0;
  std::bitset<32> currentKeyState;
  std::bitset<32> prevKeyState;
  uint8_t TX_Message[8] = {0};

  // Test mode handler
  #ifdef TEST_SCANKEYS
  // Generate test messages for all keys
  for (uint8_t row = 0; row < 3; row++) {
    for (uint8_t col = 0; col < 4; col++) {
      uint8_t keyIndex = (row * 4) + col;
      TX_Message[0] = 'P';
      TX_Message[1] = 4;
      TX_Message[2] = keyIndex;
      xQueueSend(msgOutQ, TX_Message, 0);
    }
  }
  return;
  #endif

  // Initialize activeNotes array
  if (xSemaphoreTake(sysState.mutex, portMAX_DELAY) == pdTRUE) {
    for (uint8_t i = 0; i < MAX_POLYPHONY; i++) {
      sysState.activeNotes[i].active = false;
      sysState.activeNotes[i].noteIndex = 0;
      sysState.activeNotes[i].stepSize = 0;
    }
    sysState.activeNoteCount = 0;
    xSemaphoreGive(sysState.mutex);
  }

  // Main task loop
  while (1) {
    #ifndef TEST_SCANKEYS
    vTaskDelayUntil(&lastWakeTime, SCAN_FREQUENCY);
    #endif
    
    // Reset per-cycle variables
    volumeKnobRotation = 0;
    waveformKnobRotation = 0;
    int8_t mostRecentNote = -1;
    
    // Track notes that need to be activated/deactivated
    std::bitset<12> notesToActivate;   // New notes to turn on
    std::bitset<12> notesToDeactivate; // Notes to turn off
    
    // 1. SCAN MUSICAL KEYS (rows 0-2)
    for (uint8_t row = 0; row < 3; row++) {
      setRow(row);
      delayMicroseconds(KEY_DEBOUNCE_TIME);
      std::bitset<4> rowKeys = readCols();

      for (uint8_t col = 0; col < 4; col++) {
        uint8_t keyIndex = (row * 4) + col;
        bool keyPressed = rowKeys[col];
        currentKeyState[keyIndex] = keyPressed;

        // Detect key state changes
        if (currentKeyState[keyIndex] != prevKeyState[keyIndex]) {
          // Prepare CAN message for key state change
          TX_Message[0] = keyPressed ? 'P' : 'R';
          TX_Message[1] = 4;
          TX_Message[2] = keyIndex;
          
          #ifdef SENDER_MODE
          xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
          #endif
          
          // Track note activation/deactivation
          if (keyPressed) {
            notesToActivate[keyIndex] = true;
            mostRecentNote = keyIndex;
          } else {
            notesToDeactivate[keyIndex] = true;
          }
        }
        
        // Always track the most recent pressed key for UI feedback
        if (keyPressed && mostRecentNote == -1) {
          mostRecentNote = keyIndex;
        }
      }
    }
    
    // Save key state for next cycle
    prevKeyState = currentKeyState;

    // 2. SCAN CONTROL KNOBS (row 3)
    setRow(3);
    delayMicroseconds(KEY_DEBOUNCE_TIME);
    std::bitset<4> controlRow = readCols();
    
    // Extract knob states
    uint8_t volumeKnobState = (controlRow[1] << 1) | controlRow[0];     // {B,A} for volume
    uint8_t waveformKnobState = (controlRow[3] << 1) | controlRow[2];   // {B,A} for waveform
    
    // Process volume knob
    volumeKnobRotation = decodeKnobRotation(prevVolumeKnobState, volumeKnobState);
    prevVolumeKnobState = volumeKnobState;
    
    // Process waveform knob
    waveformKnobRotation = decodeKnobRotation(prevWaveformKnobState, waveformKnobState);
    prevWaveformKnobState = waveformKnobState;

    // 3. UPDATE SYSTEM STATE with acquired mutex
    if (xSemaphoreTake(sysState.mutex, 10 / portTICK_PERIOD_MS) == pdTRUE) {
      // Volume control
      sysState.knobRotation += volumeKnobRotation;
      sysState.knobRotation = constrain(sysState.knobRotation, 0, 8);
      sysState.volume = sysState.knobRotation;
      
      // Waveform selection
      if (waveformKnobRotation != 0) {
        int16_t newWaveform = sysState.currentWaveform + waveformKnobRotation;
        sysState.currentWaveform = constrain(newWaveform, 0, 3);
      }
      
      // Update selected note for UI display
      if (mostRecentNote >= 0) {
        sysState.selectedNote = mostRecentNote;
      } else if (sysState.activeNoteCount == 0) {
        sysState.selectedNote = -1; // No notes playing
      }
      
      // Handle polyphony - Deactivate notes first
      for (uint8_t i = 0; i < 12; i++) {
        if (notesToDeactivate[i]) {
          // Find and remove this note from active notes
          for (uint8_t j = 0; j < MAX_POLYPHONY; j++) {
            if (sysState.activeNotes[j].active && sysState.activeNotes[j].noteIndex == i) {
              sysState.activeNotes[j].active = false;
              sysState.activeNotes[j].stepSize = 0;
              sysState.activeNoteCount--;
              break;
            }
          }
        }
      }
      
      // Then activate new notes (up to polyphony limit)
      for (uint8_t i = 0; i < 12; i++) {
        if (notesToActivate[i] && sysState.activeNoteCount < MAX_POLYPHONY) {
          // Find an empty slot
          for (uint8_t j = 0; j < MAX_POLYPHONY; j++) {
            if (!sysState.activeNotes[j].active) {
              sysState.activeNotes[j].active = true;
              sysState.activeNotes[j].noteIndex = i;
              sysState.activeNotes[j].stepSize = stepSizes[i];
              sysState.activeNoteCount++;
              break;
            }
          }
        }
      }
      
      xSemaphoreGive(sysState.mutex);
    }
  }
}

// Modified displayUpdateTask for testing
void displayUpdateTask(void * pvParameters) {
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // Declare buffer for note names here to avoid stack issues
  char activeNoteNames[16] = ""; 

  #ifdef TEST_DISPLAY_UPDATE
  // For testing: just update the display once
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(2, 10, "Poly Synthesizer");
  u8g2.setCursor(2, 20);
  u8g2.print("Vol: 8 Wave: Sine");
  u8g2.setCursor(2, 30);
  u8g2.print("No notes playing");
  u8g2.sendBuffer();
  return; // Exit after one iteration during testing
  #endif

  while (1) {
    #ifndef TEST_DISPLAY_UPDATE
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    #endif

    // Process CAN messages
    xQueueReceive(msgInQ, RX_Message, 0);
    
    // Local variables
    uint8_t localVolume = 0;
    uint8_t localWaveform = 0;
    int8_t localSelectedNote = -1;
    uint8_t localActiveNoteCount = 0;
    activeNoteNames[0] = '\0'; // Clear buffer
    
    // Fetch system state with timeout
    if (xSemaphoreTake(sysState.mutex, 10 / portTICK_PERIOD_MS) == pdTRUE) {
      localVolume = sysState.volume;
      localWaveform = sysState.currentWaveform;
      localSelectedNote = sysState.selectedNote;
      localActiveNoteCount = sysState.activeNoteCount;
      
      // Prepare string with active note names
      int strPos = 0;
      for (uint8_t i = 0; i < MAX_POLYPHONY; i++) {
        if (sysState.activeNotes[i].active && strPos < 14) { // Leave room for null terminator
          uint8_t noteIdx = sysState.activeNotes[i].noteIndex;
          if (noteIdx < 12) {
            // Copy note name (up to 3 chars) + space
            for (uint8_t c = 0; noteNames[noteIdx][c] != '\0' && c < 3; c++) {
              activeNoteNames[strPos++] = noteNames[noteIdx][c];
            }
            activeNoteNames[strPos++] = ' ';
          }
        }
      }
      activeNoteNames[strPos] = '\0'; // Null-terminate
      
      xSemaphoreGive(sysState.mutex);
    }
    
    // Update display
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    
    // Title row
    u8g2.drawStr(2, 10, "Poly Synthesizer");
    
    // Controls row
    u8g2.setCursor(2, 20);
    u8g2.print("Vol:");
    u8g2.print(localVolume);
    
    u8g2.setCursor(50, 20);
    u8g2.print("Wave:");
    if (localWaveform < 4) {
      u8g2.print(waveformNames[localWaveform]);
    }
    
    // Notes row
    u8g2.setCursor(2, 30);
    if (localActiveNoteCount > 0) {
      u8g2.print(activeNoteNames);
    } else {
      u8g2.print("No notes playing");
    }
    
    u8g2.sendBuffer();
  }
}

#ifdef TEST_FREE_RTOS_STATS
// Function to print FreeRTOS statistics to OLED
void printTaskStats() {
  char buffer[400]; // Buffer to hold stats
  vTaskGetRunTimeStats(buffer);
  
  // Clear display
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  
  // Display stats (we'll need to show just a portion due to screen size)
  u8g2.setCursor(0, 10);
  u8g2.print("FreeRTOS Stats:");
  
  // Find first newline to break up the content
  char* nextLine = strchr(buffer, '\n');
  if (nextLine) {
    *nextLine = '\0'; // Terminate first line
    u8g2.setCursor(0, 20);
    u8g2.print(buffer); // Display header
    
    if (*(nextLine + 1) != '\0') { // If there's more content
      char* taskLine = nextLine + 1;
      nextLine = strchr(taskLine, '\n');
      if (nextLine) *nextLine = '\0';
      
      u8g2.setCursor(0, 30);
      u8g2.print(taskLine); // Display first task
    }
  }
  
  u8g2.sendBuffer();
  delay(2000); // Display for 2 seconds
  
  // Can add more screens for additional tasks if needed
}
#endif

void setup() {
  // Serial for debugging
  Serial.begin(115200);
  Serial.println("Synthesizer starting...");
  
  // Set pin directions
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

  // Initialize display
  setOutMuxBit(DRST_BIT, LOW);  // Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  // Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  // Enable display power supply

  // Create mutex for sysState
  sysState.mutex = xSemaphoreCreateMutex();
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);
  
  // Setup timer for audio generation
  #ifndef TEST_SAMPLE_ISR
  sampleTimer.setOverflow(sampleRate, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();
  #endif

  // Initialize CAN if not in a test mode
  #if !defined(TEST_SCANKEYS) && !defined(TEST_DISPLAY_UPDATE) && !defined(TEST_CAN_TX) && !defined(TEST_SAMPLE_ISR)
  CAN_Init(true);
  setCANFilter(0x123,0x7ff);
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  CAN_Start();
  #endif

  // Create message queues - larger for testing
  #ifdef TEST_SCANKEYS
  msgOutQ = xQueueCreate(384, sizeof(TX_Message)); // Enlarged for testing (12 keys * 32 iterations)
  #else
  msgOutQ = xQueueCreate(36, sizeof(TX_Message));  // Normal size
  #endif
  
  msgInQ = xQueueCreate(36, 8); // Incoming messages
  
  // Create FreeRTOS tasks if not disabled
  #ifndef DISABLE_THREADS
  #if !defined(TEST_SCANKEYS) && !defined(TEST_DISPLAY_UPDATE) && !defined(TEST_CAN_TX) && !defined(TEST_SAMPLE_ISR)
  xTaskCreate(scanKeysTask, "scanKeys", 128, NULL, 2, &scanKeysHandle);
  xTaskCreate(displayUpdateTask, "displayUpdate", 256, NULL, 1, &displayUpdateHandle);
  #ifdef SENDER_MODE
  xTaskCreate(CAN_TX_Task, "CAN_TX", 128, NULL, 3, &canTxHandle);
  #endif
  #endif
  #endif
  
  // Test sections for measuring execution time
  
  #ifdef TEST_SCANKEYS
  // Test scanKeysTask execution time
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(2, 10);
  u8g2.print("Testing scanKeys...");
  u8g2.sendBuffer();
  
  uint32_t startTime = micros();
  for (int iter = 0; iter < TEST_ITERATIONS; iter++) {
    scanKeysTask(NULL);
  }
  uint32_t elapsedTime = micros() - startTime;
  uint32_t perIteration = elapsedTime / TEST_ITERATIONS;
  
  u8g2.clearBuffer();
  u8g2.setCursor(0, 10);
  u8g2.print("scanKeys:");
  u8g2.setCursor(0, 20);
  u8g2.print(elapsedTime);
  u8g2.print(" us total");
  u8g2.setCursor(0, 30);
  u8g2.print(perIteration);
  u8g2.print(" us per iter");
  u8g2.sendBuffer();
  
  while(1) {
    // Blink LED to show test is complete
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
  #endif
  
  #ifdef TEST_DISPLAY_UPDATE
  // Test displayUpdateTask execution time
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(2, 10);
  u8g2.print("Testing display...");
  u8g2.sendBuffer();
  delay(1000); // Show the message before testing
  
  uint32_t startTime = micros();
  for (int iter = 0; iter < TEST_ITERATIONS; iter++) {
    displayUpdateTask(NULL);
  }
  uint32_t elapsedTime = micros() - startTime;
  uint32_t perIteration = elapsedTime / TEST_ITERATIONS;
  
  u8g2.clearBuffer();
  u8g2.setCursor(0, 10);
  u8g2.print("display:");
  u8g2.setCursor(0, 20);
  u8g2.print(elapsedTime);
  u8g2.print(" us total");
  u8g2.setCursor(0, 30);
  u8g2.print(perIteration);
  u8g2.print(" us per iter");
  u8g2.sendBuffer();
  
  while(1) {
    // Blink LED to show test is complete
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
  #endif
  
  #ifdef TEST_CAN_TX
  // Test CAN_TX_Task execution time
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(2, 10);
  u8g2.print("Testing CAN_TX...");
  u8g2.sendBuffer();
  
  // Prepare message queue with test data
  for (int i = 0; i < TEST_ITERATIONS; i++) {
    uint8_t testMsg[8] = {'P', 4, (uint8_t)(i % 12), 0, 0, 0, 0, 0};
    xQueueSend(msgOutQ, testMsg, 0);
  }
  
  uint32_t startTime = micros();
  CAN_TX_Task(NULL); // This processes all queued messages in test mode
  uint32_t elapsedTime = micros() - startTime;
  uint32_t perIteration = elapsedTime / TEST_ITERATIONS;
  
  u8g2.clearBuffer();
  u8g2.setCursor(0, 10);
  u8g2.print("CAN_TX:");
  u8g2.setCursor(0, 20);
  u8g2.print(elapsedTime);
  u8g2.print(" us total");
  u8g2.setCursor(0, 30);
  u8g2.print(perIteration);
  u8g2.print(" us per iter");
  u8g2.sendBuffer();
  
  while(1) {
    // Blink LED to show test is complete
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
  #endif
  
  #ifdef TEST_SAMPLE_ISR
  // Test sampleISR execution time
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(2, 10);
  u8g2.print("Testing sampleISR...");
  u8g2.sendBuffer();
  
  uint32_t startTime = micros();
  for (int iter = 0; iter < TEST_ITERATIONS; iter++) {
    sampleISR_test();
  }
  uint32_t elapsedTime = micros() - startTime;
  uint32_t perIteration = elapsedTime / TEST_ITERATIONS;
  
  u8g2.clearBuffer();
  u8g2.setCursor(0, 10);
  u8g2.print("sampleISR:");
  u8g2.setCursor(0, 20);
  u8g2.print(elapsedTime);
  u8g2.print(" us total");
  u8g2.setCursor(0, 30);
  u8g2.print(perIteration);
  u8g2.print(" us per iter");
  u8g2.sendBuffer();
  
  while(1) {
    // Blink LED to show test is complete
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
  #endif
  
  #if !defined(TEST_SCANKEYS) && !defined(TEST_DISPLAY_UPDATE) && !defined(TEST_CAN_TX) && !defined(TEST_SAMPLE_ISR)
  // Start the RTOS scheduler if we're not in test mode
  vTaskStartScheduler();
  
  // Code should never reach here if scheduler started properly
  Serial.println("ERROR: Scheduler failed to start!");
  #endif
  
  while(1) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
}

void loop() {
  // Empty - everything is handled by FreeRTOS tasks
}