#ifndef LOOPTIMEMEASUREMENT_H
#define LOOPTIMEMEASUREMENT_H

//objects of this class are used for easy measurement of how long each loop or just section of the code takes to run on average
//this is just a tool we made for debugging, diagnostics and development

struct loopTimeMeasurement {
  long LTM_time_of_start = 0;
  long LTM_time_of_end = 0;
  int LTM_loop_counter = 0;
  int const default_number_of_cycles_to_wait = 1000;
  int number_of_cycles_to_wait;


  long time_of_first_event = 0;
  long time_difference_accumulated_sum = 0;

  long largestTime = 0;

  bool forbidDifferenceMeasurements = false;
  bool forbidLoopMeasurements = false;

  String printName = "default name";

  loopTimeMeasurement(int num, String name) : number_of_cycles_to_wait(num), printName(name) {
  }

  loopTimeMeasurement(String name, int num) : printName(name), number_of_cycles_to_wait(num) {
  }

  loopTimeMeasurement(String name) : printName(name), number_of_cycles_to_wait(default_number_of_cycles_to_wait) {
  }

  loopTimeMeasurement(int num) : number_of_cycles_to_wait(num) {
  }

  loopTimeMeasurement() : number_of_cycles_to_wait(default_number_of_cycles_to_wait) {
  }

public:
  void run() {
    if (!forbidLoopMeasurements) {
      forbidDifferenceMeasurements = true;

      //LTM_loop_counter++;
      if (number_of_cycles_to_wait < 1) {
        number_of_cycles_to_wait = 1; // correct invalid cycle numbers
      }

      LTM_loop_counter++;

      if (LTM_loop_counter == number_of_cycles_to_wait) {
        LTM_time_of_end = micros();
        Serial.print("\n LTM: [");
        Serial.print(printName);
        Serial.print("] runs every: ");
        Serial.print(((float)float(int(LTM_time_of_end - LTM_time_of_start)) / float(number_of_cycles_to_wait))/1000.0, 3);
        Serial.print("ms.\n");
        LTM_loop_counter = 0;
        LTM_time_of_start = micros();
        //LTM_time_of_start = LTM_time_of_end;
      } 
    } 
    else Serial.println("Forbidden loop measurement on: " + String(printName));
  }

  void run(int num) {
    number_of_cycles_to_wait = num;
    run();
  }

  void runStartEvent(long timeNow) {
    if (!forbidDifferenceMeasurements) {
      forbidLoopMeasurements = true;
      time_of_first_event = timeNow;
    } 
    else Serial.println("Forbidden difference measurement on: " + String(printName));
  }

  void run1() {
    runStartEvent(micros());
  }

  void run1(long timeNow) {
    runStartEvent(timeNow);
  }

  void runEndEvent(long timeNow) {
    if (!forbidDifferenceMeasurements) {
      forbidLoopMeasurements = true;
      float time_of_second_event = float(int(timeNow));
      long timeDiff = (time_of_second_event - time_of_first_event);
      if(timeDiff > largestTime) largestTime = timeDiff;
      time_difference_accumulated_sum += (time_of_second_event > time_of_first_event) ? (time_of_second_event - time_of_first_event) : (time_of_first_event - time_of_second_event);
      LTM_loop_counter++;
      if(LTM_loop_counter == number_of_cycles_to_wait) {
        LTM_loop_counter = 0;
        float avg_diff = (float) float(int(time_difference_accumulated_sum)) / float(number_of_cycles_to_wait);
        time_difference_accumulated_sum = 0;
        Serial.print("\n DTM: [");
        Serial.print(printName);
        Serial.print("] has avg time between events:  ");
        Serial.print(avg_diff/1000.0, 3);
        Serial.print("ms.  ");
        Serial.print(" and largest time:  ");
        Serial.print(largestTime/1000.0, 3);
        Serial.print("ms. \n");
        largestTime = 0;
      }

    } 
    else Serial.println("Forbidden difference measurement on: " + String(printName));
  }

  void run2() {
    runEndEvent(micros());
  }

  void run2(long timeNow) {
    runEndEvent(timeNow);
  }
};


loopTimeMeasurement KEY_MEASUREMENT_LTM("key reading thread", 100);
loopTimeMeasurement DISPLAY_LTM("display thread", 20);
loopTimeMeasurement KNOB_ACTION_LTM("knob action thread", 100);
loopTimeMeasurement MY_PRINT_LTM("my print thread", 10);
loopTimeMeasurement CAN_RX_LTM("CAN RX thread", 100); 
loopTimeMeasurement CAN_TX_LTM("CAN TX thread", 2);
loopTimeMeasurement SAMPLE_ISR_LTM("SAMPLE ISR", 100000);



#endif