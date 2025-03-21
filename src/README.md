# Embedded Systems Music Synthesiser Report

## Table of Contents
- [Introduction](#introduction)
- [Functionality](#functionality)
- [Code Structure](#code-structure)
- [Real-Time System Analysis](#real-time-system-analysis)
- [Thread Safety](#thread-safety)
- [Resource Sharing](#resource-sharing)
- [Inter-task Dependencies](#inter-task-dependencies)
- [CPU Utilization](#cpu-utilization)
- [CAN Communication](#can-communication)
- [Audio Generation](#audio-generation)
- [Testing and Validation](#testing-and-validation)
- [Conclusion](#conclusion)

## Introduction

This report documents the implementation of a music synthesizer firmware for the Embedded Systems Coursework 2. The system provides key detection, audio generation, display updates, and inter-keyboard communication capabilities. The implementation emphasizes real-time performance, concurrency management, thread safety, and efficient resource utilization on an STM32 microcontroller using the FreeRTOS operating system.

## Functionality

### Core Functionality

The synthesizer successfully implements all core functionality requirements. It generates sawtooth wave audio when keys are pressed with very low latency between input and sound production. Volume control is implemented through rotary encoders with 8 discrete levels. The OLED display updates every 100ms, showing the current note, octave, volume level, and toggles an LED for visual feedback. The system supports configurable operation as either a sender or receiver module, enabling keyboard stacking through CAN bus communication for expanded musical range.

### Advanced Features

Beyond the core requirements, several advanced features enhance musical capabilities. The system supports polyphony with up to 10 simultaneous notes through a dynamic allocation system. Users can select from four distinct waveforms (sine, square, triangle, sawtooth) to achieve different timbres. Octave control extends the playable range beyond physical keys, while auto-detection of connected keyboards dynamically adjusts octave settings across modules. Thread-safety mechanisms ensure robust operation under all conditions, preventing data corruption during concurrent access.

## Code Structure

The project employs a modular architecture with specialized header files for distinct functionality domains. The `globalDefinitions.h` file contains shared state and constants, while `threadSafetyClasses.h` provides thread-safe data structures for concurrent access. Hardware interaction is managed through `pinDefinitions.h`, `keyReading.h`, and `knobs.h`. Communication facilities are implemented in `CANstuff.h`, while audio generation resides in `waveformStuff.h`. User interface updates are handled by `displayStuff.h`, and the `debugDefinitions.h` file provides testing configuration options.

The central `main.cpp` file configues hardware, initializes subsystems and the FreeRTOS task structure:

```cpp

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
```

This priority scheme ensures time-critical operations receive appropriate CPU allocation.

## Real-Time System Analysis

### Task Allocation

The system distributes functionality across concurrent tasks to ensure responsive operation. Audio generation runs as a 22kHz interrupt service routine. Key scanning executes every 20ms as a highest-priority task (15). Display updates run every 100ms at the lowest priority (1). Three event-driven tasks handle knob actions (priority 10), CAN message reception (priority 8), and CAN transmission (priority 5), each running only when required (queue based).

### Timing Analysis

Task execution times have been rigorously measured to verify real-time constraints:

| Task | Average Time (ms) | Largest Time (ms) | Period (ms) |
|------|-------------------|-------------------|-------------|
| Key reading | 0.264 | 0.264| 20 |
| Display | 16.23| 16.28 | 100 |
| Knob | 0.004 | 0.012 | 60 |
| RX (CAN) | 0.001 | 0.004 | 72 (est) |
| TX (CAN) | 0.0015 | 0.005 |72 (est) |

The display update task, as expected, consumes the most execution time (16.23ms average, 16.28ms maximum) due to I2C communication overhead and graphical rendering requirements. However, this remains well within its 100ms period, providing ample margin. Key reading executes in approximately 0.26ms, which is minimal compared to its 20ms period, ensuring responsive detection while leaving substantial CPU availability. All event-driven tasks execute in around 0.01ms, demonstrating excellent efficiency for their operations.
Timing estimation was made under assumtion of 10 key presses max (10 fingers)

### Critical Instant Analysis

Rate Monotonic Scheduling (RMS) analysis confirms that all tasks will meet their deadlines even under worst-case conditions. The key reading task utilizes 1.32% of CPU (0.26ms/20ms), while the display task uses 16.23% (16.23ms/100ms). The audio ISR, running at 22kHz, consumes approximately 11.1% (0.005ms/0.045ms). Event-driven tasks contribute minimally to the overall load: knob processing (0.096%), CAN reception (0.004%), and CAN transmission (0.003%).

The total CPU utilization of 28.8% remains well below the theoretical RMS limit of 71.2% for three periodic tasks, confirming that all deadlines will be met even under peak load conditions.

## Thread Safety

The system employs a comprehensive thread safety strategy utilizing multiple synchronization mechanisms. For managing bounded integers like volume and octave values, the `ThreadSafeBoundedInteger` class provides atomic operations with bounds checking:

```cpp
class ThreadSafeBoundedInteger {
    std::atomic<int> number{0};
    const int max;
    const int min;

public:
    ThreadSafeBoundedInteger(int initialValue, int min_, int max_) 
        : number(initialValue), min(min_), max(max_) {}

    int get() const {
        return number.load(std::memory_order_relaxed);
    }

    bool add(int delta) {
        int oldNumber = number.load(std::memory_order_relaxed);
        int newNumber;
        bool success = true;

        do {
            newNumber = oldNumber + delta;
            if (newNumber > max) {
                newNumber = max;
                success = false;
            } else if (newNumber < min) {
                newNumber = min;
                success = false;
            }
        } while (!number.compare_exchange_weak(oldNumber, newNumber, 
                                        std::memory_order_relaxed));
        return success;
    }
}
```

RAII principles are implemented through the `FreeRTOSlock` class, which automatically releases mutexes when control exits a scope, preventing potential deadlocks from forgotten releases:

```cpp
class FreeRTOSlock {
public:
    FreeRTOSlock(SemaphoreHandle_t sh) : sh_int{sh} {
        xSemaphoreTake(sh, portMAX_DELAY);
    }
    ~FreeRTOSlock() {
        xSemaphoreGive(sh_int);
    }
private:
    SemaphoreHandle_t sh_int;
};
```

The core `SystemState` structure uses mutex protection to ensure consistent access to the collection of active notes and system parameters. Status flags for module position detection use atomic operations for efficient, lock-free concurrent access. The audio generation interrupt service routine employs atomic loads when accessing shared state, avoiding mutex usage in this time-critical context.

For complex data structures, dedicated mutex wrapper functions provide a clean interface for protecting critical sections:

```cpp
void sysStateTake() {
  xSemaphoreTake(sysState.mutex, portMAX_DELAY);
}

void sysStateGive() {
  xSemaphoreGive(sysState.mutex);
}
```

This combination of atomic operations for simple data and mutex protection for complex structures prevents race conditions and data corruption.

## Resource Sharing

Several key resources are shared between tasks in the system. The `SystemState` structure contains active notes, waveform selection, and input information shared between key scanning, audio generation, display, and CAN communication tasks:

```cpp
struct SystemState {
    std::bitset<32> inputs;
    ActiveNote activeNotes[MAX_POLYPHONY];
    uint8_t activeNoteCount = 0;
    uint8_t currentWaveform = 0;
    SemaphoreHandle_t mutex; 
};
```

Volume and octave parameters, implemented as `ThreadSafeBoundedInteger` instances, are shared between knob processing, display, and audio generation. Message queues for CAN communication are shared between ISRs and processing tasks.

Different synchronization methods are employed based on resource characteristics. Mutex protection secures complex data structures like `SystemState`, ensuring atomic updates to related variables. Atomic operations provide efficient, lock-free access to simple flags and counters, particularly beneficial in ISR contexts:

```cpp
// In ISR context
activeNotesLocal[i].active = __atomic_load_n(&sysState.activeNotes[i].active, __ATOMIC_RELAXED);
activeNotesLocal[i].stepSize = __atomic_load_n(&sysState.activeNotes[i].stepSize, __ATOMIC_RELAXED);
```

Queue-based communication inherently decouples producers from consumers, providing built-in synchronization. Semaphores control access to hardware resources like the CAN transmitter. These mechanisms cooperatively prevent race conditions while maintaining system performance.

## Inter-task Dependencies

The system's task interactions have been carefully designed to be acyclic, preventing potential deadlock conditions. The key scanning task produces events that flow to audio generation, the knob action queue, and CAN transmission. The knob action task consumes events and may produce CAN messages:

```cpp
void knobActionTask(void *pvParameters) {
    KnobActionClass receivedAction;
    
    while (1) {
        if (xQueueReceive(knobActionQueue, &receivedAction, portMAX_DELAY)) {
            int knobID = receivedAction.knobID;
            int action = receivedAction.action;

            switch (knobID) {
                case 3: // volume adjustment
                    volume.add(action);
                    break;
                case 0: // waveform selection
                    sysStateTake();
                    int currentWaveform_local = sysState.currentWaveform;
                    currentWaveform_local = constrain(currentWaveform_local + action, 0, 3);
                    sysState.currentWaveform = currentWaveform_local;
                    sysStateGive();
                    break;
                case 2: // octave change
                    if (amTheWestMost_local) {
                        if (sendOctaveAction(action)) {
                            octave.add(action);
                        }
                    }
                    break;
            }
        }
    }
}
```

CAN transmission depends on hardware availability through semaphore control, while CAN reception may update system state affecting audio generation. The display task reads system state but creates no dependencies for other tasks.

Several strategies prevent deadlocks in this architecture. Resource acquisition predominantly uses either non-blocking methods or timeouts rather than indefinite blocking. When multiple resources must be acquired, they are always accessed in consistent order to prevent hold-and-wait scenarios. Queue management includes overflow handling to prevent deadlocks from full message queues:

```cpp
if (uxQueueSpacesAvailable(msgOutQ) == 0) {
  uint8_t dump[8]; 
  xQueueReceive(msgOutQ, dump, 0); 
}
```

These approaches collectively ensure reliable operation even under stress conditions.

## CPU Utilization

Detailed timing measurements reveal efficient CPU utilization across all tasks. The display update task consumes the most CPU time proportionally (16.23%), while the audio ISR utilizes approximately 11.1% due to its high frequency despite brief execution time. Key reading uses 1.32%, with all other tasks collectively using around 0.1%. The combined CPU utilization of approximately 28.8% demonstrates efficient resource usage.

## CAN Communication

The CAN bus enables multi-module operation through structured message formats and efficient queue management. Key presses transmit note events with command type, key index, and octave information:

```cpp
void addkeyPressMessageToTxQueue(char command, byte keyIndex, byte octaveIndex) {
    uint8_t TX_Message[8] = {0};
    TX_Message[0] = int(command); // 'P' for press, 'R' for release
    TX_Message[1] = keyIndex;
    TX_Message[2] = octaveIndex;
  
    xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
}
```

Octave control messages synchronize settings across modules:

```cpp
bool sendOctaveAction(int direction) {
  uint8_t TX_Message[8] = {0};
  TX_Message[0] = int('O');
  TX_Message[1] = direction == 1 ? 1 : 0;
  
  if (xQueueSend(msgOutQ, TX_Message, 0) == pdPASS) {
      lastSendTime = micros();
      return true;
  }
  return false;
}
```

Module discovery messages facilitate automatic detection and configuration of stacked keyboards:

```cpp
void requestPreviousEndpointOctave(char direction) {
  uint8_t TX_Message[8] = {0};
  TX_Message[0] = int(direction); // 'W' or 'E'
  TX_Message[1] = int('?');
  xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
}
```

Message handling uses FreeRTOS queues to buffer transmissions and receptions, decoupling generation from actual bus communication. A semaphore controls access to the CAN transmitter, ensuring exclusive hardware usage. Overflow handling prevents queue saturation by discarding oldest messages when necessary. ISR-safe queue operations ensure reliable message passing between interrupt and task contexts:

```cpp
void CAN_RX_ISR(void) {
  uint8_t RX_Message_ISR[8];
  uint32_t ID;
  CAN_RX(ID, RX_Message_ISR);
  xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}
```

This communication architecture enables the synthesizer to function seamlessly as part of a larger, multi-module keyboard setup with automatic octave negotiation and consistent event propagation across units.

## Audio Generation

The synthesizer implements sophisticated audio generation using phase accumulation and multiple waveform algorithms. Each waveform type employs a specific generation approach:

```cpp
void sampleISR() {
    static uint32_t phaseAccs[MAX_POLYPHONY] = {0};
    int32_t mixedOutput = 0;
    
    int volume_local = volume.get();
    uint8_t waveform_local = __atomic_load_n(&sysState.currentWaveform, __ATOMIC_RELAXED);
    
    ActiveNote activeNotesLocal[MAX_POLYPHONY];
    for (uint8_t i = 0; i < MAX_POLYPHONY; i++) {
      activeNotesLocal[i].active = __atomic_load_n(&sysState.activeNotes[i].active, __ATOMIC_RELAXED);
      activeNotesLocal[i].stepSize = __atomic_load_n(&sysState.activeNotes[i].stepSize, __ATOMIC_RELAXED);
    }
    
    for (uint8_t i = 0; i < MAX_POLYPHONY; i++) {
      if (activeNotesLocal[i].active) {
        phaseAccs[i] += activeNotesLocal[i].stepSize;
        uint8_t phase = (phaseAccs[i] >> 24) & 0xFF;
        
        int32_t voiceOutput = 0;
        switch (sysState.currentWaveform) {
          case 0: // sine wave
            voiceOutput = (((int32_t)sineTable[phase]) - 128) * 3;
            break;
          case 1: // square wave
            voiceOutput = (phase < 128) ? 127 : -128;
            break;
          case 2: // triangle wave
            voiceOutput = (phase < 128) ? 
              (-128 + (phase * 2)) : (127 - ((phase - 128) * 2)) * 3;
            break;         
          case 3: // sawtooth wave
            voiceOutput = phase - 128;
            break;
        }
        
        mixedOutput += voiceOutput;
      } else {
        phaseAccs[i] = 0; 
      }
    }
    
    mixedOutput = mixedOutput >> (8 - volume_local);
    mixedOutput = constrain(mixedOutput, -128, 255-128);
    analogWrite(OUTR_PIN, mixedOutput + 128);
}
```

Polyphony is achieved through a dynamic slot allocation system that manages up to 10 simultaneous notes. When a key is pressed, the system searches for an available slot in the `activeNotes` array, storing the note index and calculated step size:

```cpp
void processPressedKey(int index, int octaveLocal) {
    if(!__atomic_load_n(&amTheWestMost, __ATOMIC_RELAXED)) {
        addkeyPressMessageToTxQueue('P', index, octaveLocal);
    } else {
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
```

Our keyboard connection algorithm allows for connecting any number of keyboards on the fly.

## Testing and Validation

The implementation includes comprehensive testing mechanisms to ensure reliable performance. The `loopTimeMeasurement` class provides detailed execution time tracking for performance analysis:

```cpp
struct loopTimeMeasurement {
  long LTM_time_of_start = 0;
  long LTM_time_of_end = 0;
  int LTM_loop_counter = 0;
  
  void run1() {
    runStartEvent();
  }

  void run2() {
    runEndEvent();
  }
  
  void runEndEvent() {
    long time_of_second_event = micros();
    long timeDiff = (time_of_second_event - time_of_first_event);
    if(timeDiff > largestTime) largestTime = timeDiff;
    time_difference_accumulated_sum += timeDiff;
    LTM_loop_counter++;
    
    if(LTM_loop_counter == number_of_cycles_to_wait) {
      LTM_loop_counter = 0;
      float avg_diff = (float)time_difference_accumulated_sum / 
                      number_of_cycles_to_wait;
      Serial.print("\n DTM: [");
      Serial.print(printName);
      Serial.print("] avg time: ");
      Serial.print(avg_diff/1000.0, 3);
      Serial.print("ms, largest: ");
      Serial.print(largestTime/1000.0, 3);
      Serial.print("ms\n");
      time_difference_accumulated_sum = 0;
      largestTime = 0;
    }
  }
};
```

A dedicated worst-case testing mode artificially generates simultaneous inputs to stress-test the system:

```cpp
#ifdef WORST_CASE_TIMING_TESTING
  addkeyPressMessageToTxQueue('R', 3, 4);
  
  knobStateCounter = (knobStateCounter+1)%4;
  knobVals[0] = knobStates[knobStateCounter][0];
  knobVals[1] = knobStates[knobStateCounter][1];
  decodeKnobs(knobVals);
#endif
```

Configurable debugging flags allow selective enabling of specific tasks:

```cpp
#define WORST_CASE_TIMING_TESTING
#define PRINT_TIMING

//#define ALLOW_KEY_MEASUREMENT_THREAD
//#define ALLOW_DISPLAY_THREAD
#define ALLOW_KNOB_ACTION_THREAD
//#define ALLOW_MY_PRINT_THREAD
//#define ALLOW_CAN_RX_THREAD
//#define ALLOW_CAN_TX_THREAD
```

These testing approaches collectively ensure that all system components operate correctly under both normal and stress conditions, validating the robustness of the implementation.

## Conclusion

The implemented music synthesizer successfully fulfills all core functional requirements while incorporating several advanced features. Real-time performance analysis confirms robust behavior with all tasks meeting their deadlines even under worst-case conditions. The comprehensive thread safety strategy, combining atomic operations, mutex protection, and queue-based communication, ensures reliable concurrent operation without data corruption or race conditions.
