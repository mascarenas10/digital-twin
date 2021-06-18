#include <Arduino.h>

int read_vane_sensor(){
//Vane sensor
  analogReference(EXTERNAL);    //connect 3.3v to AREF
    
  int angle = (analogRead(A8) - 207) * 0.448;
  return angle;
}