#ifndef KEY_READING_H
#define KEY_READING_H

#include "pinDefinitions.h"

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
    digitalWrite(REN_PIN,LOW);
    digitalWrite(RA0_PIN, bitIdx & 0x01);
    digitalWrite(RA1_PIN, bitIdx & 0x02);
    digitalWrite(RA2_PIN, bitIdx & 0x04);
    digitalWrite(OUT_PIN,value);
    digitalWrite(REN_PIN,HIGH);
    delayMicroseconds(2);
    digitalWrite(REN_PIN,LOW);
}

std::bitset<4> readCols() {
    std::bitset<4> result;
    result[0] = !digitalRead(C0_PIN);
    result[1] = !digitalRead(C1_PIN);
    result[2] = !digitalRead(C2_PIN);
    result[3] = !digitalRead(C3_PIN);
    return result;
}

std::bitset<4> readRow(const uint8_t bitIdx) {
    digitalWrite(REN_PIN,LOW);
    digitalWrite(RA0_PIN, bitIdx & 0x01);
    digitalWrite(RA1_PIN, bitIdx & 0x02);
    digitalWrite(RA2_PIN, bitIdx & 0x04);

    //digitalWrite(OUT_PIN, outValue[bitIdx]);
    digitalWrite(OUT_PIN, HIGH);

    digitalWrite(REN_PIN,HIGH);
    delayMicroseconds(3);
    std::bitset<4> inputs = readCols();
    digitalWrite(REN_PIN,LOW);
    return inputs;
}

#endif