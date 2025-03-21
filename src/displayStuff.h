#ifndef DISPLAY_STUFF_H
#define DISPLAY_STUFF_H

#include <U8g2lib.h>
#include "debugDefinitions.h"
#include "keyReading.h"

//Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

TaskHandle_t displayUpdateHandle = NULL;
void displayUpdateTask(void * pvParameters) {
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    #ifdef PRINT_TIMING
        DISPLAY_LTM.run1();
    #endif

    #ifndef WORST_CASE_TIMING_TESTING
    bool amTheWestMost_local = __atomic_load_n(&amTheWestMost, __ATOMIC_RELAXED);
    //Update display
    u8g2.clearBuffer();         // clear the internal memory

    if(amTheWestMost_local) {
        u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
        u8g2.drawStr(2,10,"Wave:");  // write something to the internal memory
        u8g2.drawStr(60, 10, waveformNames[sysState.currentWaveform].c_str()); 
    }
    u8g2.setCursor(2,20);

    u8g2.drawStr(2, 20,"Octave:");
    u8g2.drawStr(60, 20, (std::to_string(octave.get())).c_str());
    
    if(amTheWestMost_local) {
        u8g2.drawStr(2, 30,"Volume:");
        u8g2.drawStr(60, 30, (std::to_string(volume.get())).c_str()); 
    }


    u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
    digitalToggle(LED_BUILTIN);

    #endif
    #ifdef WORST_CASE_TIMING_TESTING
        u8g2.clearBuffer();         // clear the internal memory


            u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
            u8g2.drawStr(2,10,"Wave:");  // write something to the internal memory
            u8g2.drawStr(60, 10, waveformNames[sysState.currentWaveform].c_str()); 
        
        u8g2.setCursor(2,20);

        u8g2.drawStr(2, 20,"Octave:");
        u8g2.drawStr(60, 20, (std::to_string(octave.get())).c_str());
        
   
            u8g2.drawStr(2, 30,"Volume:");
            u8g2.drawStr(60, 30, (std::to_string(volume.get())).c_str()); 



        u8g2.sendBuffer();          // transfer internal memory to the display

        //Toggle LED
        digitalToggle(LED_BUILTIN);
    #endif

    #ifdef PRINT_TIMING
        DISPLAY_LTM.run2();
    #endif
  }
}


void setupDisplay() {
    setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
    delayMicroseconds(2);
    setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
    u8g2.begin();
    setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

}

#endif