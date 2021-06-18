#include <Arduino.h>
#include <MsTimer2.h>
#include <String.h>


//Variables Acelerometro
//Pines analógicos
#define xPin  A9
#define yPin  A10
#define zPin  A11

#define refx 515
#define refy 502
#define refz 515

static int accel[120][3];

int num = 0;
int block;

char buffer_ac[510]="";

void accelerometer(void);

void accelerometer_setup(void){
    MsTimer2::set(10, accelerometer); // 10ms period
}

void accelerometer(void) {
//Entramos cada 8ms
    //Cada 102 pulsos es 1g (0.33*1023/3.3)
    const int steps = 102;

    //Valores de referencia, para obtener los Gs

    analogReference(EXTERNAL);    //connect 3.3v to AREF

//Calculo de Gs
    accel[num][0] = (analogRead(xPin) - refx) * 100 / steps;
    accel[num][1] = (analogRead(yPin) - refy) * 100 / steps;
    accel[num][2] = (analogRead(zPin) - refz) * 100 / steps;

//  Serial.print("Acc X: ");Serial.println(accel[num][0]);
//  Serial.print("Num: ");Serial.println(num);

    num = num + 1;

    if (num > 100) {
        num = 0;

    }

}

int get_accelerometer_num(void) {
    return num;
}



void accelerometer_start(void){
    MsTimer2::start();
}

void accelerometer_stop(void){
    MsTimer2::stop();
}

void get_accelerometer_data(char *buffer1, int j, char *fecha){

    char json_data[30] = "{\"b\": %i,\"time\": ";
    snprintf(buffer_ac, sizeof(buffer_ac), json_data, j);
    strcat(buffer_ac, fecha);
    char json_data2[10] = ", \"acc\":[";
    //Serial.println(json_data2);
    strcat(buffer_ac, json_data2);
    int i = 0;
    for(i=25*j; i<25*j+24; i++){
        char json_data_aux[14] ="[%i,%i,%i],";
        char buffer_aux[20]="";
        snprintf(buffer_aux, sizeof(buffer_aux), json_data_aux, accel[i][0], accel[i][1], accel[i][2]);
        //Serial.println(buffer_aux);
        strcat(buffer_ac, buffer_aux);
    }
    char json_data_aux[14] ="[%i,%i,%i]]}";
    char buffer_aux[20]="";
    snprintf(buffer_aux, sizeof(buffer_aux), json_data_aux, accel[i][0], accel[i][1], accel[i][2]);
    //Serial.println(buffer_aux);
    strcat(buffer_ac, buffer_aux);

    strcpy(buffer1, buffer_ac);
}

void restart_accelerometer_num(void){
    num = 0;
}