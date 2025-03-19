#ifndef CAN_STUFF_H
#define CAN_STUFF_H


QueueHandle_t msgOutQ;
SemaphoreHandle_t CAN_TX_Semaphore;
QueueHandle_t msgInQ;

int stepSize(int index, int octaveLocal);

bool sendOctaveAction(int direction) {
  bool success = false;
  long sendInterval = 200;  // Limit to 10ms per message
  static long lastSendTime = micros();

  if (micros() - lastSendTime >= sendInterval) {
    uint8_t TX_Message[8] = {0};
    TX_Message[0] = int('O');
    TX_Message[1] = direction == 1 ? 1 : 0;
    //xQueueSend( msgOutQ, TX_Message, pdMS_TO_TICKS(50));
    //if (uxQueueSpacesAvailable(msgOutQ) > 0) {

    if (uxQueueSpacesAvailable(msgOutQ) == 0) {
      uint8_t dump[8]; // Temporary buffer to remove the oldest message
      xQueueReceive(msgOutQ, dump, 0);  // Remove oldest message
  }
  //xQueueSend(msgOutQ, TX_Message, 0);
  
      if (xQueueSend(msgOutQ, TX_Message,  0) == pdPASS) {
          lastSendTime = micros();
          success =  true;
      }
    //}
  }
  
  return success;
}

void addkeyPressMessageToTxQueue(char command, byte keyIndex, byte octaveIndex) {
    uint8_t TX_Message[8] = {0};
    TX_Message[0] = int(command);
    TX_Message[1] = keyIndex;
    TX_Message[2] = octaveIndex;
  
    xQueueSend( msgOutQ, TX_Message, portMAX_DELAY);
    //CAN_TX(0x123, TX_Message);
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
    TX_Message[2] = octave;
  
    xQueueSend( msgOutQ, TX_Message, portMAX_DELAY);
  }

  // Added: Waveform synchronization helper
void sendWaveformUpdate(uint8_t waveform) {
  uint8_t TX_Message[8] = {'W', waveform};
  xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
}

  void CAN_RX_ISR (void) {
	uint8_t RX_Message_ISR[8];
	uint32_t ID;
	CAN_RX(ID, RX_Message_ISR);
	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}


TaskHandle_t receiveQueueHandle = NULL;
void receiveQueueTask(void *pvParameters) {
  while (1) {
    xQueueReceive(msgInQ, RX_Message, portMAX_DELAY);
    bool amTheWestMost_local = __atomic_load_n(&amTheWestMost, __ATOMIC_RELAXED);
    if(RX_Message[0] == int('P') && amTheWestMost_local) {
      int currentStepSize_local = stepSize(RX_Message[1], RX_Message[2]);
      Serial.println(RX_Message[2]);
      int currentKeyIndex_local = RX_Message[1];

      __atomic_store_n(&currentStepSize, currentStepSize_local, __ATOMIC_RELAXED);
      __atomic_store_n(&currentKeyIndex, currentKeyIndex_local, __ATOMIC_RELAXED);
    }
    if(RX_Message[0] == int('R') && amTheWestMost_local) {
      int currentStepSize_local = 0;
      int currentKeyIndex_local = RX_Message[1];

      __atomic_store_n(&currentStepSize, currentStepSize_local, __ATOMIC_RELAXED);
      __atomic_store_n(&currentKeyIndex, currentKeyIndex_local, __ATOMIC_RELAXED);
    }
    if(RX_Message[0] == int('W') && !amTheWestMost_local) {
      sysStateTake();
      sysState.currentWaveform = constrain(RX_Message[1], 0, 3);
      sysStateGive();
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
      /*
      else if (RX_Message[0] == int('E') && amPREVIOUSLYTheWestMost_local && amPREVIOUSLYTheEastMost_local) {
        sendPreviousEndpointOctave('E');
        __atomic_store_n(&amPREVIOUSLYTheEastMost, false ,__ATOMIC_RELAXED);
      }
      */
    }

    if(RX_Message[1] == int('!')) {
      bool amTheWestMost_local = __atomic_load_n(&amTheWestMost, __ATOMIC_RELAXED);
      bool amTheEastMost_local = __atomic_load_n(&amTheEastMost, __ATOMIC_RELAXED);
      bool amPREVIOUSLYTheWestMost_local = __atomic_load_n(&amPREVIOUSLYTheWestMost, __ATOMIC_RELAXED);
      bool amPREVIOUSLYTheEastMost_local = __atomic_load_n(&amPREVIOUSLYTheEastMost, __ATOMIC_RELAXED);

      bool allSetUp_local = __atomic_load_n(&allSetUp, __ATOMIC_RELAXED);

      int receivedOctave = RX_Message[2];

      if (RX_Message[0] == int('E') && !allSetUp_local && amTheEastMost_local && !amTheWestMost_local) {
        octave = receivedOctave+1;
        __atomic_store_n(&amPREVIOUSLYTheWestMost, false ,__ATOMIC_RELAXED);
        __atomic_store_n(&allSetUp, true, __ATOMIC_RELAXED);
      }
      else if (RX_Message[0] == int('W') && !allSetUp_local && amTheWestMost_local && !amTheEastMost_local) {
        octave = receivedOctave-1;
        __atomic_store_n(&amPREVIOUSLYTheEastMost, false ,__ATOMIC_RELAXED);
        __atomic_store_n(&allSetUp, true, __ATOMIC_RELAXED);
      }
    }

    if(RX_Message[0] == int('O') && !amTheWestMost_local) {
      int octaveShift = 0;
      if(RX_Message[1] == 0) {
        octaveShift = -1;
      }
      if(RX_Message[1] == 1) {
        octaveShift = 1;
      }
      int octave_local = __atomic_load_n(&octave, __ATOMIC_RELAXED);
      octave_local += octaveShift;

      //updateOctave(octave_local);
      __atomic_store_n(&octave, octave_local, __ATOMIC_RELAXED);
    }
  }
}

TaskHandle_t transmitQueueHandle = NULL;

void CAN_TX_Task (void * pvParameters) {
	uint8_t msgOut[8];
	while (1) {
		xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
		xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
    CAN_TX(uniqueID, msgOut);
	}
}

void CAN_TX_ISR (void) {
	xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}


#endif