#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <cmath>
#include <HardwareTimer.h>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>

HardwareTimer sampleTimer(TIM1); // Create global timer object using TIM1

// Enable Testing Modes
// #define TEST_SCANKEYS
// #define TEST_ISR
// #define TEST_DISPLAY_UPDATE
// #define TEST_CAN_TX
// #define TEST_CAN_RX
// #define TEST_FREE_RTOS_STATS

//Constants
const uint32_t interval = 100; //Display update interval
const float sampleRate = 22000.0f; // Sample rate in Hz
const float baseFrequency = 440.0f; // Frequency of A4
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

#define SENDER_MODE // Uncomment this for sender mode, comment for receiver mode

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

void sampleISR() {
  static uint32_t phaseAcc = 0;
  uint32_t localStepSize = __atomic_load_n(&currentStepSize, __ATOMIC_RELAXED);
  phaseAcc += localStepSize;

  int32_t Vout = (phaseAcc >> 24) - 128;

  // **Read Volume Atomically**
  uint8_t localVolume = __atomic_load_n(&sysState.volume, __ATOMIC_RELAXED);

  // **Apply Logarithmic Scaling**
  Vout = Vout >> (8 - localVolume); // Right shift by (8 - volume)

  uint8_t dacOut = Vout + 128;
  analogWrite(OUTL_PIN, dacOut);
  analogWrite(OUTR_PIN, dacOut);
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

void CAN_TX_Task (void * pvParameters) {
	uint8_t msgOut[8];
	while (1) {
		xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
		xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
		CAN_TX(0x123, msgOut);
	}
}

void CAN_TX_ISR (void) {
	xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}

// Task for scanning keys
void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS; // Faster scan for knob
  TickType_t xLastWakeTime = xTaskGetTickCount();

  static uint8_t prevKnobState = 0b00;  // Previous knob {B,A} state
  int8_t localKnobRotation = 0;
  uint32_t localCurrentStepSize = 0;
  bool notePressed = false;
  int8_t localSelectedNote = -1;
  std::bitset<32> localInputs;
  std::bitset<32> prevInputs;
  uint8_t TX_Message[8] = {0};

  while (1) {
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
      localKnobRotation = 0;
      localSelectedNote = -1;
      localCurrentStepSize = 0;
      notePressed = false;
      // Scan rows 0-2 for musical keys
      for (uint8_t row = 0; row < 3; row++) {
          setRow(row);
          delayMicroseconds(3);
          std::bitset<4> rowKeys = readCols();

          for (uint8_t col = 0; col < 4; col++) {
              uint8_t keyIndex = (row * 4) + col;
              bool keyPressed = rowKeys[col];
              localInputs[keyIndex] = keyPressed;

              if (localInputs[keyIndex] != prevInputs[keyIndex]) {
                TX_Message[0] = (keyPressed) ? 'P' : 'R'; // Key Pressed or Released
                TX_Message[1] = 4; // Octave Number
                TX_Message[2] = keyIndex; // Note Number
                #ifdef SENDER_MODE                
                xQueueSend( msgOutQ, TX_Message, portMAX_DELAY);
                #endif
              }

              if (keyPressed) {
                  localCurrentStepSize = stepSizes[keyIndex];
                  localSelectedNote = keyIndex;
                  notePressed = true;
              }
          }
      }
      prevInputs = localInputs;

      // Scan Row 3 for Knob 3 (Volume Control)
      setRow(3);
      delayMicroseconds(3);
      std::bitset<4> row3Keys = readCols();
      uint8_t knobState = (row3Keys[1] << 1) | row3Keys[0]; // Read {B, A}

      // **Decode Knob Rotation**
      switch ((prevKnobState << 2) | knobState) {
          case 0b0001: localKnobRotation = +1; break; // CW
          case 0b0100: localKnobRotation = -1; break; // CCW
          case 0b1011: localKnobRotation = -1; break; // CCW
          case 0b1110: localKnobRotation = +1; break; // CW
          case 0b0010: case 0b1001: case 0b1100: case 0b0110:
              localKnobRotation = (localKnobRotation == 0) ? 0 : (localKnobRotation > 0 ? +1 : -1);
              break;
          default:
              break;
      }

      prevKnobState = knobState; // Store state for next cycle

      // **Apply Volume Limits (0 to 8)**
      if (xSemaphoreTake(sysState.mutex, portMAX_DELAY) == pdTRUE) {
          sysState.knobRotation += localKnobRotation;
          sysState.knobRotation = constrain(sysState.knobRotation, 0, 8); // Keep in range
          sysState.volume = sysState.knobRotation; // Volume linked to knob
          xSemaphoreGive(sysState.mutex);
      }

      // Atomic update of step size for ISR
      __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
      __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
  }
}



// Task for updating the display
void displayUpdateTask(void * pvParameters) {
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t ID;

  while (1) {
      vTaskDelayUntil(&xLastWakeTime, xFrequency);

      // **Receive the latest CAN message from the queue**
      xQueueReceive(msgInQ, RX_Message, 0);
      

      int8_t localKnobRotation;
      uint8_t localVolume;

      if (xSemaphoreTake(sysState.mutex, portMAX_DELAY) == pdTRUE) {
          localKnobRotation = sysState.knobRotation;
          localVolume = sysState.volume;
          xSemaphoreGive(sysState.mutex);
      }

      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.drawStr(2, 10, "Synthesizer");

      // Display volume
      u8g2.setCursor(2, 20);
      u8g2.print("Volume: ");
      u8g2.print(localVolume);

      // **Display the latest received CAN message**
      u8g2.setCursor(66, 30);
      u8g2.print((char) RX_Message[0]); // 'P' or 'R'
      u8g2.print(RX_Message[1]); // Octave
      u8g2.print(RX_Message[2]); // Note number

      u8g2.sendBuffer();
  }
}




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
  sampleTimer.setOverflow(sampleRate, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();

  CAN_Init(true);
  setCANFilter(0x123,0x7ff);
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  CAN_Start();

  msgOutQ = xQueueCreate(36, sizeof(TX_Message)); // Outgoing messages
  msgInQ = xQueueCreate(36,8); // Incoming messages
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3, 3); // 3 mailboxes available
  
  // **Create Tasks**
  xTaskCreate(scanKeysTask, "scanKeys", 128, NULL, 2, NULL);
  xTaskCreate(displayUpdateTask, "displayUpdate", 256, NULL, 1, NULL);
  #ifdef SENDER_MODE
  xTaskCreate(CAN_TX_Task, "CAN_TX", 128, NULL, 3, NULL);
  #endif
  // Start the RTOS scheduler
  vTaskStartScheduler();
  
  // Code should never reach here if scheduler started properly
  Serial.println("ERROR: Scheduler failed to start!");
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