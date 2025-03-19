#ifndef KNOBS_H
#define KNOBS_H

#include "globalDefinitions.h"

QueueHandle_t knobActionQueue;


struct KnobActionClass {
    int knobID;
    int action;
};


class knobClass {
    private:
      SemaphoreHandle_t mutex;
      bool prevB = false, prevA = false;
      int prevAction = 0;
  
    public:
      knobClass() {
        mutex = xSemaphoreCreateMutex();
      }
  
      int decodeKnob(bool A, bool B) {
          xSemaphoreTake(mutex, portMAX_DELAY);
          int action = 0;
          if((!prevB && !prevA && !B && A) || (prevB && prevA && B && !A)) action = 1;
          else if((!prevB && prevA && !B && !A) || (prevB && !prevA && B && A)) action = -1;
          //else if((prevB && prevA && !B && !A) || (!prevB && !prevA && B && A)) action = prevAction;
          else if(prevB != B && prevA != A) action = prevAction; //illegal transition - assume same action as the last
          else action = 0;
  
          if(action!=0) {
            prevAction = action;
          }
          prevB = B;
          prevA = A;
          xSemaphoreGive(mutex);
          return action;
        
      }
  };

  
void decodeKnobs(std::bitset<8> vals) {

    static knobClass knob0, knob1, knob2, knob3;

    KnobActionClass knobActions[] = {
        {0, knob0.decodeKnob(vals[0], vals[1])},
        {1, knob1.decodeKnob(vals[2], vals[3])},
        {2, knob2.decodeKnob(vals[4], vals[5])},
        {3, knob3.decodeKnob(vals[6], vals[7])}
    };

    for (int i = 0; i < 4; i++) {
        if (knobActions[i].action != 0) {
            xQueueSend(knobActionQueue, &knobActions[i], portMAX_DELAY);
        }
    }
  }

  bool sendOctaveAction(int action);

  void knobActionTask(void *pvParameters) {
    KnobActionClass receivedAction;
    
    while (1) {
        if (xQueueReceive(knobActionQueue, &receivedAction, portMAX_DELAY)) {
            int knobID = receivedAction.knobID;
            int action = receivedAction.action;

            switch (knobID) {
                case 3:
                    
                    {// adjust volume
                        int volume_local = __atomic_load_n(&volume, __ATOMIC_RELAXED);
                        volume_local = constrain(volume_local + action, 0, 8);
                        __atomic_store_n(&volume, volume_local, __ATOMIC_RELAXED);
                    }
                    break;
                    
                case 1:
                {
                    sysStateTake();
                    int currentWaveform_local = sysState.currentWaveform;
                    currentWaveform_local = constrain(currentWaveform_local + action, 0, 3);
                    if (currentWaveform_local != sysState.currentWaveform) {
                        sysState.currentWaveform = currentWaveform_local;
                        sendWaveformUpdate(currentWaveform_local); // Inform other boards!
                    }
                    sysStateGive();
                    Serial.print("Waveform changed to: ");
                    Serial.println(currentWaveform_local);
                }
                    break;

                case 2:
                    { //send octave change message
                        bool amTheWestMost_local = __atomic_load_n(&amTheWestMost, __ATOMIC_RELAXED);
                        if (amTheWestMost_local) {
                            if (sendOctaveAction(action)) {
                                int octave_local = __atomic_load_n(&octave, __ATOMIC_RELAXED);
                                octave_local += action;
                                __atomic_store_n(&octave, octave_local, __ATOMIC_RELAXED);
                            }
                        }
                    }
                    break;
                    
                case 0:
                    Serial.print("Knob 0 action: ");
                    Serial.println(action);
                    break;

                default:
                    Serial.println("invalid knob ID");
                    break;
            }
        }
    }
}

TaskHandle_t knobActionHandle = NULL;
void knobSetup(UBaseType_t knobActionTask_priority) {
    knobActionQueue = xQueueCreate(10, sizeof(KnobActionClass));
    xTaskCreate(
        knobActionTask,   // Function that implements the task
        "knobAction",     // Name for debugging
        256,              // Stack size in words
        NULL,             // No parameters
        knobActionTask_priority,         //priority
        &knobActionHandle              // No handle needed
    );
}



#endif