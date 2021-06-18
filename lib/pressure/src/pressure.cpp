#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
int pressure;

void pressure_setup(void){
    bmp.begin();
}

int read_pressure_sensor(void){
//Pressure
/* Get a new sensor event */ 
sensors_event_t event;
//Serial.print(event.pressure);
bmp.getEvent(&event);

pressure=int(event.pressure*10);

return pressure;
}
