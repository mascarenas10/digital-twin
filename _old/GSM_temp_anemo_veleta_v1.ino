/**************************************************************
 *
 * For this example, you need to install PubSubClient library:
 *   https://github.com/knolleary/pubsubclient
 *   or from http://librarymanager/all#PubSubClient
 *
 * TinyGSM Getting Started guide:
 *   https://tiny.cc/tinygsm-readme
 *
 * For more MQTT examples, see PubSubClient library
 *
 **************************************************************
 * Use Mosquitto client tools to work with MQTT
 *   Ubuntu/Linux: sudo apt-get install mosquitto-clients
 *   Windows:      https://mosquitto.org/download/
 *
 * Subscribe for messages:
 *   mosquitto_sub -h test.mosquitto.org -t GsmClientTest/init -t GsmClientTest/ledStatus -q 1
 * Toggle led:
 *   mosquitto_pub -h test.mosquitto.org -t GsmClientTest/led -q 1 -m "toggle"
 *
 * You can use Node-RED for wiring together MQTT-enabled devices
 *   https://nodered.org/
 * Also, take a look at these additional Node-RED modules:
 *   node-red-contrib-blynk-ws
 *   node-red-dashboard
 *
 **************************************************************/
// Select your modem:
#define TINY_GSM_MODEM_SIM900

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// DefineSoftware Serial on Uno, Nano
#include <SoftwareSerial.h>
SoftwareSerial SerialAT(8, 9); // RX, TX

// See all AT commands, if wanted
//#define DUMP_AT_COMMANDS

// Define the serial console for debug prints, if needed
//#define TINY_GSM_DEBUG SerialMon

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

uint32_t lastReconnectAttempt = 0;
long lastPing = 0;
unsigned long t;

boolean mqttConnect() {
  SerialMon.print(F("Connecting to "));
  SerialMon.print(broker);

  boolean status = mqtt.connect(broker, "O5r2@vortex", "uXMSJSLgV9Nq2SvV");

  if (status == false) {
    SerialMon.println(F(" fail"));
    return false;
  }
  SerialMon.println(F(" success"));
//  mqtt.publish(topicInit, "GsmClientTest started");
//  mqtt.subscribe(topicLed);
  return mqtt.connected();
}

//Variables Anemómetro
const byte pinAnamometer = 3;       //anamometer signal input
#define UPDATE_PERIOD   500         //send serial messages to the monitor every 500mS
#define PULSE_TOUT      1000ul      //time between pulses where we say wind speed is basically 0.0mph
#define CONST 0.0875                //Según datasheet de Amazon. El otro datasheet lo pone como 0,125
//#define CONST 0.125
void ISR_anamomPulse( void );       //prototype of the function used for the anamometer pin rising-edge interrupt

unsigned long
    Period,                         //holds the time between rising edges in microseconds
    pulseTimeout;                   //holds the time of the last pulse (mS) use to determine if there's little/no wind
bool
    bFirstPulse,                    //indicates to ISR that this is the first pulse
    bGotPulse;                      //indication from ISR that a valid Period reading is ready

int fWindSpeed;                   //Variable que mide la intensidad del viento
//Fin Variables Anemómetro
#include <DHT.h>    // importa la Librerias DHT

int SENSOR = 2;     // pin DATA de DHT22 a pin digital 2
int TEMPERATURA;
int HUMEDAD;

DHT dht(SENSOR, DHT11);   // creacion del objeto, cambiar segundo parametro
        // por DHT11 si se utiliza en lugar del DHT22



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

  dht.begin();      // inicializacion de sensor

  SerialMon.println(F("Wait..."));

  SerialAT.begin(9600);
  delay(3000);
  
  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println(F("Initializing modem..."));
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
}

void loop() {
    //Cálculos anemómetro
    //check to see if too much time has elapsed since the last anamometer rising edge   
    CheckPulseTimeout();
    //see if it's time to send a windspeed message
    CheckSpeed();

    TEMPERATURA = dht.readTemperature();  // obtencion de valor de temperatura
    HUMEDAD = dht.readHumidity();   // obtencion de valor de humedad

    analogReference(EXTERNAL);    //connect 3.3v to AREF
   // read the input on analog pin 0:
   //int sensorValue = analogRead(A0);
   //float voltage = sensorValue * (5.0 / 1023.0);
   int angle = (analogRead(A0) - 220) * 0.448;
    
    t = millis();
  
  if (!mqtt.connected()) {
    SerialMon.println(F("MQTT not connected"));
    // Reconnect every 10 seconds
    if (t - lastReconnectAttempt > 5000L) {
      lastReconnectAttempt = t;
      if (mqttConnect()) {
        lastReconnectAttempt = 0;
      }
    }
    return;
  } else if (t - lastPing > 10000L) {
    lastPing = t;

    //char json_data[] = "{\"temperature\":%i, \"humidity\":%i, \"wdir\":%i, \"wspeed\":%i}";
    char mybuffer[220]="";
    //snprintf(mybuffer, sizeof(mybuffer), json_data, TEMPERATURA, HUMEDAD, angle, fWindSpeed);
    char json_data[] = "{\"temp\":%i, \"hum\":%i, \"wdir\":%i, \"ws\":%i}";
    snprintf(mybuffer, sizeof(mybuffer), json_data, TEMPERATURA, HUMEDAD, angle, fWindSpeed);
    mqtt.publish(topicData, mybuffer);

    t = millis();
  }
  mqtt.loop();
}

//Funciones anemómetro
void CheckSpeed( void )
{
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
    }//else

}//CheckSpeed

void CheckPulseTimeout( void )
{
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

void ISR_anamomPulse( void )
{
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
