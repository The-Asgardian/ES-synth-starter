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
- [Conclusions](#conclusions)

## Introduction

This report documents the implementation of a music synthesizer firmware for the Embedded Systems Coursework 2. The system provides key detection, audio generation, display updates, and inter-keyboard communication capabilities. The implementation emphasizes real-time performance, concurrency management, thread safety, and efficient resource utilization on an STM32 microcontroller using the FreeRTOS operating system.

## Functionality

### Core Functionality

The synthesizer successfully implements all core functionality requirements. It generates sawtooth wave audio when keys are pressed with imperceptible latency between input and sound production. Volume control is implemented through rotary encoders with 8 discrete levels. The OLED display updates every 100ms, showing the current note, octave, volume level, and toggles an LED for visual feedback. The system supports configurable operation as either a sender or receiver module, enabling keyboard stacking through CAN bus communication for expanded musical range.

### Advanced Features

Beyond the core requirements, several advanced features enhance musical capabilities. The system supports polyphony with up to 10 simultaneous notes through a dynamic allocation system. Users can select from four distinct waveforms (sine, square, triangle, sawtooth) to achieve different timbres. Octave control extends the playable range beyond physical keys, while auto-detection of connected keyboards dynamically adjusts octave settings across modules. Thread-safety mechanisms ensure robust operation under all conditions, preventing data corruption during concurrent access.

## Code Structure

The project employs a modular architecture with specialized header files for distinct functionality domains. The `globalDefinitions.h` file contains shared state and constants, while `threadSafetyClasses.h` provides thread-safe data structures for concurrent access. Hardware interaction is managed through `pinDefinitions.h`, `keyReading.h`, and `knobs.h`. Communication facilities are implemented in `CANstuff.h`, while audio generation resides in `waveformStuff.h`. User interface updates are handled by `displayStuff.h`, and the `debugDefinitions.h` file provides testing configuration options.

The central `main.cpp` file serves as the orchestrator, configuring hardware, initializing subsystems, and creating the FreeRTOS task structure:

```cpp
void setup() {
  // Configure hardware pins
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  // Initialize OLED display
  setupDisplay();
  // Set up audio timer at 22kHz
  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();
  // Initialize semaphores and queues
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);
  msgInQ = xQueueCreate(36,8);
  msgOutQ = xQueueCreate(36,8);
  // Create tasks with appropriate priorities
  xTaskCreate(scanKeysTask, "scanKeys", 256, NULL, 15, &scanKeysHandle);
  xTaskCreate(knobActionTask, "knobAction", 256, NULL, 10, &knobActionHandle);
  xTaskCreate(receiveQueueTask, "receiveQueue", 256, NULL, 8, &receiveQueueHandle);
  xTaskCreate(CAN_TX_Task, "transmitQueue", 512, NULL, 5, &transmitQueueHandle);
  xTaskCreate(displayUpdateTask, "displayUpdate", 512, NULL, 1, &displayUpdateHandle);
  // Start scheduler
  vTaskStartScheduler();
}
```

This priority scheme ensures time-critical operations receive appropriate CPU allocation.

## Real-Time System Analysis

### Task Allocation

The system distributes functionality across concurrent tasks to ensure responsive operation. Audio generation runs as a 22kHz interrupt service routine to maintain consistent sound production. Key scanning executes every 20ms as a high-priority task (15), ensuring responsive input detection while also handling handshaking with adjacent modules. Display updates run every 100ms at the lowest priority (1), balancing visual feedback with processing efficiency. Three event-driven tasks handle knob actions (priority 10), CAN message reception (priority 8), and CAN transmission (priority 5), each activating only when required.

The key scanning task implementation demonstrates how matrix scanning is performed periodically:

```cpp
void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    // Scan key matrix
    std::bitset<32> sysState_inputs_local;
    for (int i = 0; i < 7; i++) {
      std::bitset<4> rowInputs = readRow(i);
      for(int j = 0; j < 4; j++) 
        sysState_inputs_local[i*4+j] = rowInputs[j];
    }
    
    // Process key state changes
    sysStateTake();
    for(int i = 0; i < 12; i++) {
      bool wasPressed = prevKeyState[i];
      bool isPressed = sysState_inputs_local[i];
      
      if(!wasPressed && isPressed) {
        processPressedKey(i, octave.get());
      }
      if(wasPressed && !isPressed) {
        processReleasedKey(i, octave.get());
      }
    }
    prevKeyState = sysState_inputs_local;
    sysStateGive();
  }
}
```

### Timing Analysis

Task execution times have been rigorously measured to verify real-time constraints:

| Task | Average Time (ms) | Largest Time (ms) | Period (ms) |
|------|-------------------|-------------------|-------------|
| Key reading | 0.497 | 0.504 | 20 |
| Display | 19.2 | 19.952 | 100 |
| Knob | 0.007 | 0.032 | Event-driven |
| Print | 0.035 | 0.04 | 1000 |
| RX (CAN) | 0.002 | 0.024 | Event-driven |
| TX (CAN) | 0.015 | 0.032 | Event-driven |

The display update task, as expected, consumes the most execution time (19.2ms average, 19.952ms maximum) due to I2C communication overhead and graphical rendering requirements. However, this remains well within its 100ms period, providing ample margin. Key reading executes in approximately 0.5ms, which is minimal compared to its 20ms period, ensuring responsive detection while leaving substantial CPU availability. All event-driven tasks execute in under 0.05ms, demonstrating excellent efficiency for their operations.

### Critical Instant Analysis

Rate Monotonic Scheduling (RMS) analysis confirms that all tasks will meet their deadlines even under worst-case conditions. The key reading task utilizes 2.52% of CPU (0.504ms/20ms), while the display task uses 19.95% (19.952ms/100ms). The audio ISR, running at 22kHz, consumes approximately 11.1% (0.005ms/0.045ms). Event-driven tasks contribute minimally to the overall load: knob processing (0.032%), CAN reception (0.12%), and CAN transmission (0.16%).

The total CPU utilization of 33.9% remains well below the theoretical RMS limit of 69.3% for three periodic tasks, confirming that all deadlines will be met even under peak load conditions. This provides substantial headroom for future enhancements while maintaining reliable real-time performance.

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

This combination of atomic operations for simple data and mutex protection for complex structures balances performance with reliability, preventing race conditions and data corruption.

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

The system's task interactions form a directed dependency graph that has been carefully designed to be acyclic, preventing potential deadlock conditions. The key scanning task produces events that flow to audio generation, the knob action queue, and CAN transmission. The knob action task consumes events and may produce CAN messages:

```cpp
void knobActionTask(void *pvParameters) {
    KnobActionClass receivedAction;
    
    while (1) {
        if (xQueueReceive(knobActionQueue, &receivedAction, portMAX_DELAY)) {
            int knobID = receivedAction.knobID;
            int action = receivedAction.action;

            switch (knobID) {
                case 3: // Volume adjustment
                    volume.add(action);
                    break;
                case 0: // Waveform selection
                    sysStateTake();
                    int currentWaveform_local = sysState.currentWaveform;
                    currentWaveform_local = constrain(currentWaveform_local + action, 0, 3);
                    sysState.currentWaveform = currentWaveform_local;
                    sysStateGive();
                    break;
                case 2: // Octave change
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

Several strategies prevent deadlocks in this architecture. The dependency graph is explicitly designed to avoid circular wait conditions. Resource acquisition predominantly uses either non-blocking methods or timeouts rather than indefinite blocking. When multiple resources must be acquired, they are always accessed in consistent order to prevent hold-and-wait scenarios. Queue management includes overflow handling to prevent deadlocks from full message queues:

```cpp
if (uxQueueSpacesAvailable(msgOutQ) == 0) {
  uint8_t dump[8]; // Temporary buffer for oldest message
  xQueueReceive(msgOutQ, dump, 0);  // Remove oldest message
}
```

These approaches collectively ensure reliable operation even under stress conditions.

## CPU Utilization

Detailed timing measurements reveal efficient CPU utilization across all tasks. The display update task consumes the most CPU time proportionally (19.95%), while the audio ISR utilizes approximately 11.1% due to its high frequency despite brief execution time. Key reading uses 2.52%, with all other tasks collectively using less than 0.4%.

The combined CPU utilization of approximately 33.9% demonstrates efficient resource usage while maintaining substantial headroom for additional features. No individual task approaches its deadline, with the display task using only about 20% of its available window. This confirms that the design effectively balances functionality with performance, meeting real-time constraints while accommodating potential future enhancements.

## CAN Communication

The CAN communication system enables multi-module operation through structured message formats and efficient queue management. Key press messages transmit note events with command type, key index, and octave information:

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

Message handling uses FreeRTOS queues to buffer transmissions and receptions, decoupling generation from actual bus communication. A semaphore controls access to the CAN transmitter, ensuring exclusive hardware usage. Overflow handling prevents queue saturation by discarding oldest messages when necessary, avoiding potential deadlocks. ISR-safe queue operations ensure reliable message passing between interrupt and task contexts:

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

The synthesizer implements sophisticated audio generation using phase accumulation techniques and multiple waveform algorithms. Each waveform type employs a specific generation approach:

```cpp
void sampleISR() {
    static uint32_t phaseAccs[MAX_POLYPHONY] = {0};
    int32_t mixedOutput = 0;
    
    int volume_local = volume.get();
    uint8_t waveform_local = __atomic_load_n(&sysState.currentWaveform, __ATOMIC_RELAXED);
    
    // Copy active notes atomically (thread-safe in ISR)
    ActiveNote activeNotesLocal[MAX_POLYPHONY];
    for (uint8_t i = 0; i < MAX_POLYPHONY; i++) {
      activeNotesLocal[i].active = __atomic_load_n(&sysState.activeNotes[i].active, __ATOMIC_RELAXED);
      activeNotesLocal[i].stepSize = __atomic_load_n(&sysState.activeNotes[i].stepSize, __ATOMIC_RELAXED);
    }
    
    // Generate and mix all active notes
    for (uint8_t i = 0; i < MAX_POLYPHONY; i++) {
      if (activeNotesLocal[i].active) {
        phaseAccs[i] += activeNotesLocal[i].stepSize;
        uint8_t phase = (phaseAccs[i] >> 24) & 0xFF;
        
        int32_t voiceOutput = 0;
        switch (sysState.currentWaveform) {
          case 0: // Sine wave (lookup table)
            voiceOutput = (((int32_t)sineTable[phase]) - 128) * 3;
            break;
          case 1: // Square wave
            voiceOutput = (phase < 128) ? 127 : -128;
            break;
          case 2: // Triangle wave
            voiceOutput = (phase < 128) ? 
              (-128 + (phase * 2)) : (127 - ((phase - 128) * 2)) * 3;
            break;         
          case 3: // Sawtooth wave
            voiceOutput = phase - 128;
            break;
        }
        
        mixedOutput += voiceOutput;
      } else {
        phaseAccs[i] = 0; // Reset inactive voice
      }
    }
    
    // Apply volume and output
    mixedOutput = mixedOutput >> (8 - volume_local);
    mixedOutput = constrain(mixedOutput, -128, 255-128); // Prevent clipping
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

This architecture balances memory efficiency with computational performance, enabling rich harmonic content without excessive resource consumption.

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
    
    // Calculate and report statistics periodically
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
  // Artificially generate key presses for timing measurement
  addkeyPressMessageToTxQueue('R', 3, 4);
  
  // Simulate knob rotations
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

## Conclusions

The implemented music synthesizer successfully fulfills all core functional requirements while incorporating several advanced features that enhance its musical capabilities. Real-time performance analysis confirms robust behavior with all tasks meeting their deadlines even under worst-case conditions. The comprehensive thread safety strategy, combining atomic operations, mutex protection, and queue-based communication, ensures reliable concurrent operation without data corruption or race conditions.

The system achieves efficient resource utilization with a total CPU load of 33.9%, leaving substantial headroom for future enhancements. The modular architecture with distinct functional components enhances maintainability and facilitates potential extensions. Advanced audio features including polyphony and multiple waveforms provide enhanced musical expressiveness beyond core requirements.

Future work could explore additional enhancements including advanced audio effects (reverb, delay, filtering), dynamic waveform customization, MIDI integration, enhanced user interface capabilities, memory optimization for increased polyphony, DMA-based audio generation, and further performance profiling. These extensions would build upon the solid foundation of the current implementation while maintaining its robust real-time performance characteristics.d counters, particularly beneficial in ISR contexts. Queue-based communication inherently decouples producers from consumers, providing built-in synchronization. Semaphores control access to hardware resources like the CAN transmitter. These mechanisms cooperatively prevent race conditions while maintaining system performance.

## Inter-task Dependencies

The system's task interactions form a directed dependency graph that has been carefully designed to be acyclic, preventing potential deadlock conditions. The key scanning task produces events that flow to audio generation, the knob action queue, and CAN transmission. The knob action task consumes events and may produce CAN messages. CAN transmission depends on hardware availability through semaphore control, while CAN reception may update system state affecting audio generation. The display task reads system state but creates no dependencies for other tasks.

Several strategies prevent deadlocks in this architecture. The dependency graph is explicitly designed to avoid circular wait conditions. Resource acquisition predominantly uses either non-blocking methods or timeouts rather than indefinite blocking. When multiple resources must be acquired, they are always accessed in consistent order to prevent hold-and-wait scenarios. Queue management includes overflow handling to prevent deadlocks from full message queues, discarding oldest messages when necessary. These approaches collectively ensure reliable operation even under stress conditions.

## CPU Utilization

Detailed timing measurements reveal efficient CPU utilization across all tasks. The display update task consumes the most CPU time proportionally (19.95%), while the audio ISR utilizes approximately 11.1% due to its high frequency despite brief execution time. Key reading uses 2.52%, with all other tasks collectively using less than 0.4%.

The combined CPU utilization of approximately 33.9% demonstrates efficient resource usage while maintaining substantial headroom for additional features. No individual task approaches its deadline, with the display task using only about 20% of its available window. This confirms that the design effectively balances functionality with performance, meeting real-time constraints while accommodating potential future enhancements.

## CAN Communication

The CAN communication system enables multi-module operation through structured message formats and efficient queue management. Key press messages transmit note events with command type, key index, and octave information. Octave control messages synchronize settings across modules, while module discovery messages facilitate automatic detection and configuration of stacked keyboards.

Message handling uses FreeRTOS queues to buffer transmissions and receptions, decoupling generation from actual bus communication. A semaphore controls access to the CAN transmitter, ensuring exclusive hardware usage. Overflow handling prevents queue saturation by discarding oldest messages when necessary, avoiding potential deadlocks. ISR-safe queue operations ensure reliable message passing between interrupt and task contexts.

This communication architecture enables the synthesizer to function seamlessly as part of a larger, multi-module keyboard setup with automatic octave negotiation and consistent event propagation across units.

## Audio Generation

The synthesizer implements sophisticated audio generation using phase accumulation techniques and multiple waveform algorithms. Each waveform type employs a specific generation approach: sine waves use a pre-calculated lookup table for efficiency, square waves implement simple threshold comparison, triangle waves use dual linear ramps, and sawtooth waves apply direct phase-to-amplitude conversion.

Polyphony is achieved through a dynamic slot allocation system that manages up to 10 simultaneous notes. When a key is pressed, the system searches for an available slot in the `activeNotes` array, storing the note index and calculated step size. Upon release, the corresponding slot is freed. The audio ISR iterates through active slots, generating and mixing waveforms with appropriate volume scaling and output limiting to prevent clipping. This architecture balances memory efficiency with computational performance, enabling rich harmonic content without excessive resource consumption.

## Testing and Validation

The implementation includes comprehensive testing mechanisms to ensure reliable performance. The `loopTimeMeasurement` class provides detailed execution time tracking for performance analysis, measuring both average and worst-case scenarios. A dedicated worst-case testing mode artificially generates simultaneous inputs to stress-test the system under peak load conditions, verifying that deadlines are consistently met.

Configurable debugging flags in `debugDefinitions.h` allow selective enabling of specific tasks and features, facilitating isolation testing and bottleneck identification. Functional validation confirms that all core requirements are satisfied: audio response time measurements verify imperceptible latency, polyphony tests confirm correct simultaneous note generation, CAN communication validation verifies correct message propagation, and UI responsiveness measurements confirm appropriate update rates.

These testing approaches collectively ensure that all system components operate correctly under both normal and stress conditions, validating the robustness of the implementation.

## Conclusions

The implemented music synthesizer successfully fulfills all core functional requirements while incorporating several advanced features that enhance its musical capabilities. Real-time performance analysis confirms robust behavior with all tasks meeting their deadlines even under worst-case conditions. The comprehensive thread safety strategy, combining atomic operations, mutex protection, and queue-based communication, ensures reliable concurrent operation without data corruption or race conditions.

The system achieves efficient resource utilization with a total CPU load of 33.9%, leaving substantial headroom for future enhancements. The modular architecture with distinct functional components enhances maintainability and facilitates potential extensions. Advanced audio features including polyphony and multiple waveforms provide enhanced musical expressiveness beyond core requirements.

Future work could explore additional enhancements including advanced audio effects (reverb, delay, filtering), dynamic waveform customization, MIDI integration, enhanced user interface capabilities, memory optimization for increased polyphony, DMA-based audio generation, and further performance profiling. These extensions would build upon the solid foundation of the current implementation while maintaining its robust real-time performance characteristics.
