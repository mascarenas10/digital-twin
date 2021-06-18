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
//#define INTERVAL 22.5
//String pos;

void setup() {
  // Set console baud rate
  SerialMon.begin(9600);
  delay(10);

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
    analogReference(EXTERNAL);    //connect 3.3v to AREF
   // read the input on analog pin 0:
   //int sensorValue = analogRead(A0);
   // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
   //float voltage = sensorValue * (5.0 / 1023.0);
   int angle = (analogRead(A0) - 198) * 0.453;

   
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

    char json_data[] = "{\"wdir\":%i}";
    char mybuffer[220]="";
    snprintf(mybuffer, sizeof(mybuffer), json_data, angle);
    
    mqtt.publish(topicData, mybuffer);

    t = millis();
  }
  mqtt.loop();
}
