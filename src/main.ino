#include <Arduino.h>

#include <accelerometer.h>
#include <anemometer.h>
#include <temperature_humidity.h>
#include <pressure.h>
#include <vane.h>
#include <gsm.h>
#include <TimeLib.h>
#include <timestamp.h>

//Variables that can be modified by the user
#define WINDMIN 400 //400cm/s This is the minimum velocity to start the movement in the Vortex
#define TIMETEMP 60000 //1 min to send the temperature data
#define TIMEWINDMIN 120000 //2 mins to send the wind data in the worse case. (Not used in prototype)
#define TIMEWINDMAX 30000 //30 secs to send the wind data

// MQTT details
const char* broker = "mqtt.swx.altairone.com";
const char* topicData = "set/vortex/collections/vortexnano/things/01F497X94GZYH2KY74TA17MKP7/data";

const char* user = "jTI2@vortex";
const char* password = "D3ejBOuKWNirIVpz";

uint32_t lastconnecttemp = 0;
uint32_t lastconnectwind = 0;
uint32_t t;
uint32_t ttemp;
uint32_t twind;
char fecha [30];


void setup() {
    //gsm_automatic_start();

    anemometer_setup();
    temperature_humidity_setup();
    pressure_setup();
    accelerometer_setup();
    gsm_setup(broker);
    timestamp_setup();

    mqttConnect(broker, user, password);

    Serial.println("Setup done");
}

void loop() {
    ttemp = millis();
    twind = millis();

    //check to see if too much time has since the last anamometer rising edge
    CheckPulseTimeout();
    //see if it's time to send a windspeed message
    CheckSpeed();
    int fWindSpeed = read_anemometer_sensor();

   if (fWindSpeed > WINDMIN) {
        int num = get_accelerometer_num();
        delay(10);
        
        if (num == 0) {

            get_fecha(fecha);
            accelerometer_start();
            
        }
        if (num > 99) {
            accelerometer_stop();

            mqttConnect(broker, user, password);
            
            char mybuffer[510];
            mqttConnect(broker, user, password);
            for (int i=0; i<4;i++){
                get_accelerometer_data(mybuffer, i, fecha);
                Serial.print("Buffer: ");Serial.print(i);Serial.println(mybuffer);

                if (mqttconnected()) {
                Serial.print("Mando Buffer: ");Serial.println(i);
                send_data_mqtt(topicData, mybuffer);
                }
            }

            restart_accelerometer_num();
        }

    }

    //Temperature data sent
    if ((ttemp - lastconnecttemp) > TIMETEMP) {//Entramos a medir
    accelerometer_stop();
    restart_accelerometer_num();
    get_fecha(fecha);

    //Temperature and humidity
    int temperature = read_temperature();
    int humidity = read_humidity();

    //Pressure
    int pressure = read_pressure_sensor();
    
    mqttConnect(broker, user, password);

        if (mqttconnected()) {
            char mybuffer3[200] = "{\"time\": ";
            strcat(mybuffer3, fecha);

            char json_data3[] = ", \"temperature\":%i, \"humidity\":%i, \"pressure\":%i}";//Temperature, Humidity, Pressure
            char mybuffer4[100] = "";
            snprintf(mybuffer4, sizeof(mybuffer4), json_data3, temperature, humidity, pressure);
            
            strcat(mybuffer3, mybuffer4);
            
            send_data_mqtt(topicData, mybuffer3);
        }

        lastconnecttemp = ttemp;
        delay(20);
    }

    //Wind data sent
    if (((fWindSpeed > WINDMIN) && ((twind - lastconnectwind) > TIMEWINDMIN))||((twind - lastconnectwind) > TIMEWINDMAX)) {
    accelerometer_stop();
    restart_accelerometer_num();
    get_fecha(fecha);

    //Weathervane
    int angle = read_vane_sensor();

    mqttConnect(broker, user, password);

        if (mqttconnected()) {
            char mybuffer5[200] = "{\"time\": ";
            strcat(mybuffer5, fecha);

            char json_data5[] = ", \"windorientation\":%i, \"windspeed\":%i}";//wind direction and wind speed
            char mybuffer6[100] = "";
            snprintf(mybuffer6, sizeof(mybuffer6), json_data5, angle, fWindSpeed);

            strcat(mybuffer5, mybuffer6);

            send_data_mqtt(topicData, mybuffer5);
        }

        lastconnectwind = twind;
        delay(20);
    }

    if (fWindSpeed < WINDMIN) {
        restart_accelerometer_num();
    }
}
