#include <Arduino.h>
#include <DHT.h>    // importa la Librerias DHT

int SENSOR = 28;     // pin DATA de DHT22 a pin digital 2
int TEMPERATURA;
int HUMEDAD;

DHT dht(SENSOR, DHT11);   // creacion del objeto, cambiar segundo parametro
// por DHT11 si se utiliza en lugar del DHT22

void temperature_humidity_setup(void){
    dht.begin();
}

int read_temperature(void){
    TEMPERATURA = dht.readTemperature();
    return TEMPERATURA;
}

int read_humidity(void){
    HUMEDAD = dht.readHumidity();
    return HUMEDAD;
}