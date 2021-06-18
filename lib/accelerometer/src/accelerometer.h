#ifndef _accelerometer_h
#define _accelerometer_h

    void accelerometer_setup(void);
    void accelerometer(void);
    void accelerometer_start(void);
    void accelerometer_stop(void);
    int get_accelerometer_num(void);
    void get_accelerometer_data(char *buffer1, int j, char *fecha);
    void restart_accelerometer_num(void);

#endif
