#include <Arduino.h>
#include <TimeLib.h>
time_t timefecha;
char bufferfecha [30];

void timestamp_setup(void){
    setTime(16, 20, 0, 25, 05, 2021);
}

void get_fecha(char* fecha){
    timefecha=now();
    char json_data[] = "\"%i/%i/%i %i:%i:%i\"";//Temperatura, Humedad, Dirección del viento
    snprintf(bufferfecha, sizeof(bufferfecha), json_data, year(timefecha), month(timefecha), day(timefecha), hour(timefecha), minute(timefecha), second(timefecha));
    strcpy(fecha, bufferfecha);
}