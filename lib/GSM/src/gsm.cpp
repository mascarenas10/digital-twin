#include <Arduino.h>

void gsm_setup(const char* broker);
boolean mqttConnect(const char* broker, const char* user, const char* password);
void gsm_initialization(void);
void gsm_modeminfo(void);
void gsm_pin(int pin);
void gsm_xbee(void);
void gsm_network(void);
void gsm_gprs(void);
void gsm_setserver(const char* broker);
boolean mqttconnected(void);
void send_data_mqtt(const char* topicData, char* mybuffer);

// Select your modem:
#define TINY_GSM_MODEM_SIM900

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Hardware Serial 1 (Mega)
#define SerialAT Serial3

// See all AT commands, if wanted
//#define DUMP_AT_COMMANDS

// Define the serial console for debug prints, if needed
//#define TINY_GSM_DEBUG SerialMon

//// DefineSoftware Serial on Uno, Nano
// #include <SoftwareSerial.h>
// SoftwareSerial SerialAT(8, 9); // RX, TX

// Add a reception delay - may be needed for a fast processor at a slow baud rate
#define TINY_GSM_YIELD() { delay(2); }

// Define how you're planning to connect to the internet
#define TINY_GSM_USE_GPRS true

// set GSM PIN, if any
#define GSM_PIN ""

//LEAVE THESE LIBRARIES HERE. IF NOT IT DOES NOT WORK
#include <TinyGsmClient.h>
#include <PubSubClient.h>
TinyGsm modem(SerialAT);

TinyGsmClient client(modem);
PubSubClient mqtt(client);

// Your GPRS credentials, if any
const char apn[] = "wlapn.com";
const char gprsUser[] = "";
const char gprsPass[] = "";



boolean mqttConnect(const char* broker, const char* user, const char* password) {
    SerialMon.print(F("Connecting to "));
    SerialMon.print(broker);

    boolean status = mqtt.connect(broker, user, password);

    if (status == false) {
        SerialMon.println(F("MQTT fail"));
      return false;
      }
    SerialMon.println(F(" success"));
    return mqtt.connected();
}
void gsm_automatic_start(void){
  digitalWrite(52, HIGH);
  delay(1000);
  digitalWrite(52, LOW);
  delay(5000);
}
void gsm_setup(const char* broker){
  SerialMon.begin(9600);
  delay(10);
  SerialAT.begin(9600);
  delay(1000);
  gsm_initialization();
  //gsm_modeminfo();
  //gsm_pin(GSM_PIN);
  //gsm_xbee();
  gsm_network();
  gsm_gprs();
  gsm_setserver(broker);

}

void gsm_initialization(void){
    // Restart takes quite some time
    // To skip it, call init() instead of restart()
    //SerialMon.println(F("Initializing modem..."));
    // modem.restart();
    modem.init();
}

void gsm_modeminfo(void){
    
    String modemInfo = modem.getModemInfo();
    SerialMon.print(F("Modem Info: "));
    SerialMon.println(modemInfo);  
}

void gsm_pin(int pin){
  #if TINY_GSM_USE_GPRS
    // Unlock your SIM card with a PIN if needed
    if (GSM_PIN && modem.getSimStatus() != 3) {
        modem.simUnlock(GSM_PIN);
    }
  #endif
}

void gsm_xbee(void){
  #if TINY_GSM_USE_GPRS && defined TINY_GSM_MODEM_XBEE
        // The XBee must run the gprsConnect function BEFORE waiting for network!
        modem.gprsConnect(apn, gprsUser, gprsPass);
  #endif
}

void gsm_network(void){
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
}

void gsm_gprs(void){
  #if TINY_GSM_USE_GPRS
    // GPRS connection parameters are usually set after network registration
    SerialMon.print(F("Connecting to "));
    SerialMon.print(apn);
    if(!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        SerialMon.println(F("fail to connect to apn"));
        delay(10000);
        return;
    }
    SerialMon.println(F(" success"));

    if (modem.isGprsConnected()) {
        SerialMon.println(F("GPRS connected"));
    }
  #endif
}

void gsm_setserver(const char* broker){
    // MQTT Broker setup
    mqtt.setServer(broker, 1883);
}

boolean mqttconnected(void){
  return mqtt.connected();
}

void send_data_mqtt(const char* topicData, char* mybuffer){
  mqtt.publish(topicData, mybuffer);
}