#ifndef _anemometer_h
#define _anemometer_h

    void anemometer_setup(void);
    void CheckSpeed(void);
    void CheckPulseTimeout(void);
    void ISR_anamomPulse(void);
    int read_anemometer_sensor(void);


#endif
