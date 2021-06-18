//Variables a modificar por el usuario
#define WINDMIN 400 //400cm/s para que se empiece a mover el vortex nano
#define TIME 60000 //10 mins para mandar los datos del resto de sensores
//#define TIME 600000 //10 mins para mandar los datos del resto de sensores

// Select your modem:
#define TINY_GSM_MODEM_SIM900

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Hardware Serial 1 (Mega)
#define SerialAT Serial1

//// DefineSoftware Serial on Uno, Nano
//#include <SoftwareSerial.h>
//SoftwareSerial SerialAT(8, 9); // RX, TX

// Add a reception delay - may be needed for a fast processor at a slow baud rate
 #define TINY_GSM_YIELD() { delay(2); }

// Define how you're planning to connect to the internet
#define TINY_GSM_USE_GPRS true

// set GSM PIN, if any
#define GSM_PIN ""

// Your GPRS credentials, if any
const char apn[] = "wlapn.com";
const char gprsUser[] = "";
const char gprsPass[] = "";

// MQTT details
const char* broker = "mqtt.swx.altairone.com";
const char* topicData = "set/vortex/collections/vortexnano/things/01F2CAD3GPHXXPKFWMHK7KKPPH/data";

#include <TinyGsmClient.h>
#include <PubSubClient.h>

TinyGsm modem(SerialAT);

TinyGsmClient client(modem);
PubSubClient mqtt(client);

boolean mqttConnect() {
  SerialMon.print(F("Connecting to "));
  SerialMon.print(broker);

  boolean status = mqtt.connect(broker, "O5r2@vortex", "uXMSJSLgV9Nq2SvV");

//  if (status == false) {
//    SerialMon.println(F(" fail"));
//    return false;
//  }
  SerialMon.println(F(" success"));
//  mqtt.publish(topicInit, "GsmClientTest started");
//  mqtt.subscribe(topicLed);
  return mqtt.connected();

}


//Variables Acelerometro
//Pines analógicos
const int xPin = A1;
const int yPin = A2;
const int zPin = A3;

int accel[125][3]; 

//Número de datos en 1 segundo (debería ser 100)
int num = 0;
#include <MsTimer2.h>
//Fin Variables Acelerometro

//Variables Anemometro
const byte pinAnamometer = 3;       //anamometer signal input

#define UPDATE_PERIOD   500         //send serial messages to the monitor every 500mS
#define PULSE_TOUT      1000ul      //time between pulses where we say wind speed is basically 0.0mph
#define CONST 0.0875                //Según datasheet de Amazon. El otro datasheet lo pone como 0,125
//#define CONST 0.125

void ISR_anamomPulse( void );       //prototype of the function used for the anamometer pin rising-edge interrupt
void CheckPulseTimeout( void );
unsigned long
    Period,                         //holds the time between rising edges in microseconds
    pulseTimeout;                   //holds the time of the last pulse (mS) use to determine if there's little/no wind
bool
    bFirstPulse,                    //indicates to ISR that this is the first pulse
    bGotPulse;                      //indication from ISR that a valid Period reading is ready

int fWindSpeed;                     //Valor de la magnitud del viento
//Fin Variables Anemometro  

//Variables Temperatura
#include <DHT.h>    // importa la Librerias DHT

int SENSOR = 2;     // pin DATA de DHT22 a pin digital 2
int TEMPERATURA;
int HUMEDAD;

DHT dht(SENSOR, DHT11);   // creacion del objeto, cambiar segundo parametro
        // por DHT11 si se utiliza en lugar del DHT22
//Fin Variables Temperatura

//Variables presion
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
int pressure;
//Fin Variables presion

uint32_t lastconnect = 0;
unsigned long t;

void setup() {

  // Set console baud rate
  SerialMon.begin(9600);
  delay(10);

  pinMode( pinAnamometer, INPUT_PULLUP );

  //set up to interrupt on rising edges of pin 2 (anamometer input) and to call function ISR_anamomPulse
  //on each interrupt
  attachInterrupt( digitalPinToInterrupt(pinAnamometer), ISR_anamomPulse, FALLING );
 
  //ready the ISR to receive its first pulse and indicate no period is ready yet
  bFirstPulse = true;   
  bGotPulse = false;

  dht.begin();      // inicializacion de sensor temperatura
  bmp.begin();      // inicializacion de sensor presion

  SerialAT.begin(9600);
  delay(1000);
  
  // Restart takes quite some time
  // To skip it, call init() instead of restart()
//  SerialMon.println(F("Initializing modem..."));
  // modem.restart();
  modem.init();

  String modemInfo = modem.getModemInfo();
  SerialMon.print(F("Modem Info: "));
  SerialMon.println(modemInfo);

#if TINY_GSM_USE_GPRS
  // Unlock your SIM card with a PIN if needed
  if ( GSM_PIN && modem.getSimStatus() != 3 ) {
    modem.simUnlock(GSM_PIN);
  }
#endif


#if TINY_GSM_USE_GPRS && defined TINY_GSM_MODEM_XBEE
  // The XBee must run the gprsConnect function BEFORE waiting for network!
  modem.gprsConnect(apn, gprsUser, gprsPass);
#endif

  SerialMon.print(F("Waiting for network..."));
  if (!modem.waitForNetwork()) {
    SerialMon.println(F(" fail"));
    delay(10000);
    return;
  }
  SerialMon.println(F(" success"));

  if (modem.isNetworkConnected()) {
    SerialMon.println(F("Network connected"));
  }

#if TINY_GSM_USE_GPRS
  // GPRS connection parameters are usually set after network registration
    SerialMon.print(F("Connecting to "));
    SerialMon.print(apn);
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
      SerialMon.println(F(" fail"));
      delay(10000);
      return;
    }
    SerialMon.println(F(" success"));

  if (modem.isGprsConnected()) {
    SerialMon.println(F("GPRS connected"));
  }
#endif

  // MQTT Broker setup
  mqtt.setServer(broker, 1883);
  mqttConnect();
 
  MsTimer2::set(10, accelerometer); // 10ms period
//  MsTimer2::start();

  Serial.println("Setup done");
}

void loop() {
  t=millis();
  //check to see if too much time has elapsed since the last anamometer rising edge   
  CheckPulseTimeout();
  //see if it's time to send a windspeed message
  CheckSpeed();

//  Serial.println(fWindSpeed);
//  delay(250);
  

  if(fWindSpeed>WINDMIN){
    Serial.print("Entro en el if");
    if(num==0){
      Serial.print("Inicio la adqui de datos");
      MsTimer2::start();
    }
      if(num>99){
        Serial.print("Me conecto para mandar datos");
      
      mqttConnect();
      
      if(mqtt.connected()){

      char json_data[] = "{\"acc34\":[%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i]}";//34 primeros
      char mybuffer[600]="";
      snprintf(mybuffer, sizeof(mybuffer), json_data, accel[0][0], accel[0][1], accel[0][2], accel[1][0], accel[1][1], accel[1][2],accel[2][0], accel[2][1], accel[2][2], accel[3][0], accel[3][1], accel[3][2],accel[4][0], accel[4][1], accel[4][2], accel[5][0], accel[5][1], accel[5][2],accel[6][0], accel[6][1], accel[6][2], accel[7][0], accel[7][1], accel[7][2],accel[8][0], accel[8][1], accel[8][2], accel[9][0], accel[9][1], accel[9][2], accel[10][0], accel[10][1], accel[10][2], accel[11][0], accel[11][1], accel[11][2],accel[12][0], accel[12][1], accel[12][2], accel[13][0], accel[13][1], accel[13][2],accel[14][0], accel[14][1], accel[14][2], accel[15][0], accel[15][1], accel[15][2],accel[16][0], accel[16][1], accel[16][2], accel[17][0], accel[17][1], accel[17][2],accel[18][0], accel[18][1], accel[18][2], accel[19][0], accel[19][1], accel[19][2], 
      accel[20][0], accel[20][1], accel[20][2], accel[21][0], accel[21][1], accel[21][2],accel[22][0], accel[22][1], accel[22][2], accel[23][0], accel[23][1], accel[23][2],accel[24][0], accel[24][1], accel[24][2], accel[25][0], accel[25][1], accel[25][2],accel[26][0], accel[26][1], accel[26][2], accel[27][0], accel[27][1], accel[27][2],accel[28][0], accel[28][1], accel[28][2], accel[29][0], accel[29][1], accel[29][2], accel[30][0], accel[30][1], accel[30][2], accel[31][0], accel[31][1], accel[31][2],accel[32][0], accel[32][1], accel[32][2], accel[33][0], accel[33][1], accel[33][2]);
     
      mqtt.publish(topicData, mybuffer);

      char json_data2[] = "{\"acc67\":[%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i]}";//35-67 
      char mybuffer2[600]="";
      snprintf(mybuffer2, sizeof(mybuffer2), json_data2,accel[34][0], accel[34][1], accel[34][2], accel[35][0], accel[35][1], accel[35][2],accel[36][0], accel[36][1], accel[36][2], accel[37][0], accel[37][1], accel[37][2],accel[38][0], accel[38][1], accel[38][2], accel[39][0], accel[39][1], accel[39][2], accel[40][0], accel[40][1], accel[40][2], accel[41][0], accel[41][1], accel[41][2],accel[42][0], accel[42][1], accel[42][2], accel[43][0], accel[43][1], accel[43][2],accel[44][0], accel[44][1], accel[44][2], accel[45][0], accel[45][1], accel[45][2],accel[46][0], accel[46][1], accel[46][2], accel[47][0], accel[47][1], accel[47][2],accel[48][0], accel[48][1], accel[48][2], accel[49][0], accel[49][1], accel[49][2], accel[50][0], accel[50][1], accel[50][2], accel[51][0], accel[51][1], accel[51][2],accel[52][0], accel[52][1], accel[52][2], accel[53][0], 
      accel[53][1], accel[53][2],accel[54][0], accel[54][1], accel[54][2], accel[55][0], accel[55][1], accel[55][2],accel[56][0], accel[56][1], accel[56][2], accel[57][0], accel[57][1], accel[57][2],accel[58][0], accel[58][1], accel[58][2], accel[59][0], accel[59][1], accel[59][2],accel[60][0], accel[60][1], accel[60][2], accel[61][0], accel[61][1], accel[61][2],accel[62][0], accel[62][1], accel[62][2], accel[63][0], accel[63][1], accel[63][2],accel[64][0], accel[64][1], accel[64][2], accel[65][0], accel[65][1], accel[65][2],accel[66][0], accel[66][1], accel[66][2]);
      
      mqtt.publish(topicData, mybuffer2);

      char json_data3[] = "{\"acc99\":[%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i][%i,%i,%i]}";//68-100
      char mybuffer3[600]="";
      snprintf(mybuffer3, sizeof(mybuffer3), json_data3, accel[67][0], accel[67][1], accel[67][2],accel[68][0], accel[68][1], accel[68][2], accel[69][0], accel[69][1], accel[69][2], accel[70][0], accel[70][1], accel[70][2], accel[71][0], accel[71][1], accel[71][2],accel[72][0], accel[72][1], accel[72][2], accel[73][0], accel[73][1], accel[73][2],accel[74][0], accel[74][1], accel[74][2], accel[75][0], accel[75][1], accel[75][2],accel[76][0], accel[76][1], accel[76][2], accel[77][0], accel[77][1], accel[77][2],accel[78][0], accel[78][1], accel[78][2], accel[79][0], accel[79][1], accel[79][2], accel[80][0], accel[80][1], accel[80][2], accel[81][0], accel[81][1], accel[81][2],accel[82][0], accel[82][1], accel[82][2], accel[83][0], accel[83][1], accel[83][2],accel[84][0], accel[84][1], accel[84][2], accel[85][0], accel[85][1], accel[85][2],accel[86][0], 
      accel[86][1], accel[86][2], accel[87][0], accel[87][1], accel[87][2],accel[88][0], accel[88][1], accel[88][2], accel[89][0], accel[89][1], accel[89][2], accel[90][0], accel[90][1], accel[90][2], accel[91][0], accel[91][1], accel[91][2],accel[92][0], accel[92][1], accel[92][2], accel[93][0], accel[93][1], accel[93][2],accel[94][0], accel[94][1], accel[94][2], accel[95][0], accel[95][1], accel[95][2],accel[96][0], accel[96][1], accel[96][2], accel[97][0], accel[97][1], accel[97][2],accel[98][0], accel[98][1], accel[98][2], accel[99][0], accel[99][1], accel[99][2]);
      
      mqtt.publish(topicData, mybuffer3);


      }
      mqtt.loop();

        num=0;
        Serial.print("Datos mandados");
        delay(5000); //Delay para analizar el Serial Monitor
    }
    
  }
    if(t-lastconnect>TIME){//Entramos a medir
    MsTimer2::stop();
    //Mandar todos los demas valores
    //Temperatura y humedad
    TEMPERATURA = dht.readTemperature();  // obtencion de valor de temperatura
    HUMEDAD = dht.readHumidity();   // obtencion de valor de humedad

    //Veleta
    analogReference(EXTERNAL);    //connect 3.3v to AREF
     // read the input on analog pin 0:
     //int sensorValue = analogRead(A0);
     //float voltage = sensorValue * (5.0 / 1023.0);
     int angle = (analogRead(A0) - 220) * 0.448;

     //Presion
       /* Get a new sensor event */ 
        sensors_event_t event;
        //Serial.print(event.pressure);
        bmp.getEvent(&event);
      
        pressure=int(event.pressure*10);
//        SerialMon.print("Pres");SerialMon.println(pressure);

     mqttConnect();

     if(mqtt.connected()){
      char json_data4[] = "{\"temperature\":%i, \"humidity\":%i, \"wspeed\":%i, \"wdir\":%i, \"pressure\":%i}";//Temperatura, Humedad, Dirección del viento
      char mybuffer4[200]="";
      snprintf(mybuffer4, sizeof(mybuffer4), json_data4, TEMPERATURA, HUMEDAD, fWindSpeed, angle, pressure);
     
      mqtt.publish(topicData, mybuffer4);
     }
     
    Serial.println("Manda el resto de datos");
    lastconnect = t;
  }
   if(fWindSpeed<WINDMIN){
    num=0;
   }

}

void accelerometer() {
//Entramos cada 8ms
  //Cada 102 pulsos es 1g (0.33*1023/3.3)
  const int steps = 102;

  //Valores de referencia, para obtener los Gs
  const int refx = 515;
  const int refy = 502;
  const int refz = 515;
  
  analogReference(EXTERNAL);    //connect 3.3v to AREF

//Calculo de Gs
  accel[num][0] = (analogRead(xPin)-refx)*100/steps;
  accel[num][1] = (analogRead(yPin)-refy)*100/steps;
  accel[num][2] = (analogRead(zPin)-refz)*100/steps;

//  Serial.print("Acc X: ");Serial.println(accel[num][0]);
//  Serial.print("Num: ");Serial.println(num);

  num = num + 1;

  if(num>100){
    MsTimer2::stop();
  }

}

void CheckSpeed(){
    char
        szSpd[20];
    float
        fP;
    static unsigned long
        timeUpdate = 0;
    unsigned long
        timeNow;

    //we send a message every UPDATE_PERIOD mS
    timeNow = millis();
    if( timeNow - timeUpdate < UPDATE_PERIOD )
        return;
    timeUpdate = timeNow;

    //if we haven't even gotten a single pulse yet (or it's been long enough since the last
    //one that we re-set this flag due to timeout) just print a windspeed of 0.0
    if( bFirstPulse )
        fWindSpeed = 0.0;
    else
    {
        //check the ISR has been able to measure a period yet
        if( bGotPulse )
        {
            //yes...stop interrupts for a moment while the flag is cleared and we get the
            //current value of the period, then re-enable them
            noInterrupts();
            bGotPulse = false;
            //period is measured internally in microseconds; divide by a million to get
            //a value in seconds suitable for use in the Hz calculation   
            fP = (float)Period/1000000.0;
            interrupts();

            //calculates the windspeed
            fWindSpeed = CONST*100/fP;
           
        }//if
//    }//else
//
//    //and send it out to the serial monitor
//    Serial.print( "Windspeed : " );Serial.print( fWindSpeed,2 );Serial.println( " m/s" );
    }     
}//CheckSpeed

void CheckPulseTimeout(){
    //don't check for a timeout if we've not even gotten a pulse since power-on/timeout
    if( !bFirstPulse )
    {
        //check if enough mS has elapsed since the last rising edge
        if( millis() - pulseTimeout > PULSE_TOUT )
        {
            //if so, halt interrupts, set the flag, and then re-enable ints
            noInterrupts();
            bFirstPulse = true;
            interrupts();           
           
        }//if
       
    }//if
   
}//CheckPulseTimeout

void ISR_anamomPulse(){
    unsigned long
        pulseNow;
    static unsigned long
        lastPulse;

    //get the time of this pulse (not exact using this method but close enough)
    pulseNow = micros();
    //and update the time of the timeout value for each rising edge
    pulseTimeout = millis();

    //if this is the first edge since power-on or timeout, we need to save its time as the "last" time
    //so we can compute the period on the following edge
    if( bFirstPulse )
    {
        lastPulse = pulseNow;
        //got a pulse, so clear this flag now
        bFirstPulse = false;
       
    }//if   
    else
    {
        //not the first edge; calculate the period between rising edges as the time of this
        //pulse (pulseNow) minus the time of the last pusle (lastPulse)
        Period = pulseNow - lastPulse;
        //now we save the current time as the last time so we can recalc the period on the
        //next rising edge
        lastPulse = pulseNow;
        //we've calculated a Period so indicate to CheckSpeed() that there's a good value
        bGotPulse = true;
       
    }//else
   
}//ISR_anamomPulse
