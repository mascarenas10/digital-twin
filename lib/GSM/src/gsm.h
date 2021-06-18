#ifndef _gsm_h
#define _gsm_h

    void gsm_setup(const char* broker);
    void gsm_automatic_start(void);
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


#endif
