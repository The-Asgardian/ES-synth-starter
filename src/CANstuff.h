#ifndef CAN_STUFF_H
#define CAN_STUFF_H

#include "debugDefinitions.h"

QueueHandle_t msgOutQ;
SemaphoreHandle_t CAN_TX_Semaphore;
QueueHandle_t msgInQ;

int stepSize(int index, int octaveLocal);

bool sendOctaveAction(int direction) {
  bool success = false;
  long sendInterval = 200;  // limit to 10ms per message
  static long lastSendTime = micros();

  if (micros() - lastSendTime >= sendInterval) {
    uint8_t TX_Message[8] = {0};
    TX_Message[0] = int('O');
    TX_Message[1] = direction == 1 ? 1 : 0;

    if (uxQueueSpacesAvailable(msgOutQ) == 0) {
      uint8_t dump[8]; 
      xQueueReceive(msgOutQ, dump, 0); 
  }

  
      if (xQueueSend(msgOutQ, TX_Message,  0) == pdPASS) {
          lastSendTime = micros();
          success =  true;
      }
    
  }
  
  return success;
}

void addkeyPressMessageToTxQueue(char command, byte keyIndex, byte octaveIndex) {
    uint8_t TX_Message[8] = {0};
    TX_Message[0] = int(command);
    TX_Message[1] = keyIndex;
    TX_Message[2] = octaveIndex;
  
    xQueueSend( msgOutQ, TX_Message, portMAX_DELAY);
  }
  

  void requestPreviousEndpointOctave(char direction) {
    uint8_t TX_Message[8] = {0};
    TX_Message[0] = int(direction); // W or E
    TX_Message[1] = int('?');
  
    xQueueSend( msgOutQ, TX_Message, portMAX_DELAY);
  }
  
  void sendPreviousEndpointOctave(char direction) {
    uint8_t TX_Message[8] = {0};
    TX_Message[0] = int(direction); // W or E
    TX_Message[1] = int('!');
    TX_Message[2] = octave.get();
  
    xQueueSend( msgOutQ, TX_Message, portMAX_DELAY);
  }

  void CAN_RX_ISR (void) {
	uint8_t RX_Message_ISR[8];
	uint32_t ID;
	CAN_RX(ID, RX_Message_ISR);
	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

void processPressedKey(int index, int octaveLocal);
void processReleasedKey(int index, int octaveLocal);

TaskHandle_t receiveQueueHandle = NULL;
void receiveQueueTask(void *pvParameters) {
  while (1) {
    xQueueReceive(msgInQ, RX_Message, portMAX_DELAY);
    #ifdef PRINT_TIMING
        CAN_RX_LTM.run1();
    #endif
    bool amTheWestMost_local = __atomic_load_n(&amTheWestMost, __ATOMIC_RELAXED);
    if(RX_Message[0] == int('P') && amTheWestMost_local) {
        processPressedKey(RX_Message[1], RX_Message[2]);

    }
    if(RX_Message[0] == int('R') && amTheWestMost_local) {
        processReleasedKey(RX_Message[1], RX_Message[2]);
    }

    if(RX_Message[1] == int('?')) {
      bool amTheWestMost_local = __atomic_load_n(&amTheWestMost, __ATOMIC_RELAXED);
      bool amTheEastMost_local = __atomic_load_n(&amTheEastMost, __ATOMIC_RELAXED);
      bool amPREVIOUSLYTheWestMost_local = __atomic_load_n(&amPREVIOUSLYTheWestMost, __ATOMIC_RELAXED);
      bool amPREVIOUSLYTheEastMost_local = __atomic_load_n(&amPREVIOUSLYTheEastMost, __ATOMIC_RELAXED);

      bool allSetUp_local = __atomic_load_n(&allSetUp, __ATOMIC_RELAXED);
      
      if (RX_Message[0] == int('W') && amPREVIOUSLYTheWestMost_local && allSetUp_local) {
        sendPreviousEndpointOctave('W');
        __atomic_store_n(&amPREVIOUSLYTheWestMost, false ,__ATOMIC_RELAXED);
      }
      else if (RX_Message[0] == int('E') && amPREVIOUSLYTheEastMost_local && allSetUp_local) {
        sendPreviousEndpointOctave('E');
        __atomic_store_n(&amPREVIOUSLYTheEastMost, false ,__ATOMIC_RELAXED);
      }

    }

    if(RX_Message[1] == int('!')) {
      bool amTheWestMost_local = __atomic_load_n(&amTheWestMost, __ATOMIC_RELAXED);
      bool amTheEastMost_local = __atomic_load_n(&amTheEastMost, __ATOMIC_RELAXED);
      bool amPREVIOUSLYTheWestMost_local = __atomic_load_n(&amPREVIOUSLYTheWestMost, __ATOMIC_RELAXED);
      bool amPREVIOUSLYTheEastMost_local = __atomic_load_n(&amPREVIOUSLYTheEastMost, __ATOMIC_RELAXED);

      bool allSetUp_local = __atomic_load_n(&allSetUp, __ATOMIC_RELAXED);

      int receivedOctave = RX_Message[2];

      if (RX_Message[0] == int('E') && !allSetUp_local && amTheEastMost_local && !amTheWestMost_local) {
        octave.assign(receivedOctave+1);
        __atomic_store_n(&amPREVIOUSLYTheWestMost, false ,__ATOMIC_RELAXED);
        __atomic_store_n(&allSetUp, true, __ATOMIC_RELAXED);
      }
      else if (RX_Message[0] == int('W') && !allSetUp_local && amTheWestMost_local && !amTheEastMost_local) {
        octave.assign(receivedOctave-1);
        __atomic_store_n(&amPREVIOUSLYTheEastMost, false ,__ATOMIC_RELAXED);
        __atomic_store_n(&allSetUp, true, __ATOMIC_RELAXED);
      }
    }

    if(RX_Message[0] == int('O') && !amTheWestMost_local) {
      if(RX_Message[1] == 0) {
        octave.decrement();
      }
      if(RX_Message[1] == 1) {
        octave.increment();
      }
    }
    #ifdef PRINT_TIMING
        CAN_RX_LTM.run2();
    #endif
  }
  
}

TaskHandle_t transmitQueueHandle = NULL;

void CAN_TX_Task (void * pvParameters) {

	uint8_t msgOut[8];
	while (1) {
		xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
        #ifdef PRINT_TIMING
            CAN_TX_LTM.run1();
        #endif
		xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
        CAN_TX(uniqueID, msgOut);
        #ifdef PRINT_TIMING
            CAN_TX_LTM.run2();
        #endif
	}
}

void CAN_TX_ISR (void) {
	xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}


#endif