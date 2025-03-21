#ifndef WAVEFORM_STUFF_H
#define WAVEFORM_STUFF_H

int stepSizeGenerator(int index) {
    float Fs = 22000;
    float FA = 440*float(pow(2.0, float(octave.get())-4.0));
    float factor = pow(2.0, (float) (float(index)-9.0)*(1.0/12.0));
    float f = FA*factor;
    int step = int((float) pow(2.0, 32.0)*(f/Fs));
    return step;
  }
  
  int stepSize(int index, int octaveLocal) {
    int step = stepSizes[index];
    int octaveDiff = octaveLocal - 4;
    step = step*pow(2,octaveDiff);
    return step;
  }




QueueHandle_t sampleISRtimingQ;
void sampleISR() {

    #ifdef PRINT_TIMING
        long startTime = micros();
    #endif
    static uint32_t phaseAccs[MAX_POLYPHONY] = {0};
    int32_t mixedOutput = 0;
    uint8_t activeVoices = 0;
  
    int volume_local = volume.get();
    uint8_t waveform_local = __atomic_load_n(&sysState.currentWaveform, __ATOMIC_RELAXED);
  
    ActiveNote activeNotesLocal[MAX_POLYPHONY];
  
    // ISR-safe atomic copy of notes (no mutex)
    for (uint8_t i = 0; i < MAX_POLYPHONY; i++) {
      activeNotesLocal[i].active = __atomic_load_n(&sysState.activeNotes[i].active, __ATOMIC_RELAXED);
      activeNotesLocal[i].stepSize = __atomic_load_n(&sysState.activeNotes[i].stepSize, __ATOMIC_RELAXED);
    }
  
    // Mix all active notes clearly without relying on contiguous indices
    for (uint8_t i = 0; i < MAX_POLYPHONY; i++) {
      if (activeNotesLocal[i].active) {
        phaseAccs[i] += activeNotesLocal[i].stepSize;
        uint8_t phase = (phaseAccs[i] >> 24) & 0xFF;
  
        int32_t voiceOutput = 0;
  
        switch (sysState.currentWaveform) {
          case 0: // sine
          voiceOutput = (((int32_t)sineTable[phase]) - 128) * 3;
            break;
  
          case 1: // square wave
            voiceOutput = (phase < 128) ? 127 : -128;
            break;
  
          case 2: //triangle
            voiceOutput = (phase < 128) ? (-128 + (phase * 2)) : (127 - ((phase - 128) * 2)) * 3;   
            break;         
          case 3: // sawtooth 
            voiceOutput = phase - 128;
            break;
  
          default:
            voiceOutput = phase - 128; // default = sawtooth
        }
  
        mixedOutput += voiceOutput;
        activeVoices++;
      } else {
        phaseAccs[i] = 0; // reset inactive voice
      }
    }
  
    if (activeVoices > 0) {
      //mixedOutput /= pow(activeVoices, 1/2); // do not to normalise
    } else {
      mixedOutput = 0; //no active notes
    }
  
    mixedOutput = mixedOutput >> (8 - volume_local);

    mixedOutput = constrain(mixedOutput, -128, 255-128); //cliping
  
    analogWrite(OUTR_PIN, mixedOutput + 128);

    #ifdef PRINT_TIMING
        long timeDiff = micros() - startTime;
        xQueueSendFromISR(sampleISRtimingQ, &timeDiff, NULL);
    #endif
  }
  
  
  HardwareTimer sampleTimer(TIM1);

#ifdef PRINT_TIMING
#ifdef ALLOW_SAMPLE_ISR
TaskHandle_t sampleISRtimeMeasurementTaskHandle = NULL;
void SAMPLE_ISR_TIME_MEASUREMENT_Task (void * pvParameters) {

	long timeDiff;
	while (1) {
		xQueueReceive(sampleISRtimingQ, &timeDiff, portMAX_DELAY);
        SAMPLE_ISR_LTM.run1(0);
        SAMPLE_ISR_LTM.run2(timeDiff);
	}
}
#endif
#endif

#endif