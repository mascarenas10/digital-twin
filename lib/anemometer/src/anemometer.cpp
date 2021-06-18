#include <Arduino.h>

//Variables Anemometro
const byte pinAnamometer = 19;       //anamometer signal input

#define UPDATE_PERIOD   500         //send serial messages to the monitor every 500mS
#define PULSE_TOUT      1000ul      //time between pulses where we say wind speed is basically 0.0mph
#define CONST 0.0875                //Según datasheet de Amazon. El otro datasheet lo pone como 0,125

//#define CONST 0.125

void CheckSpeed(void);
void ISR_anamomPulse(void);       //prototype of the function used for the anamometer pin rising-edge interrupt
void CheckPulseTimeout(void);

unsigned long
        Period,                         //holds the time between rising edges in microseconds
pulseTimeout;                   //holds the time of the last pulse (mS) use to determine if there's little/no wind
bool
        bFirstPulse,                    //indicates to ISR that this is the first pulse
        bGotPulse;                      //indication from ISR that a valid Period reading is ready

int fWindSpeed;                     //Valor de la magnitud del viento
//Fin Variables Anemometro  

void anemometer_setup(void){
    pinMode(pinAnamometer, INPUT_PULLUP);

    //set up to interrupt on rising edges of pin 2 (anamometer input) and to call function ISR_anamomPulse
    //on each interrupt
    attachInterrupt(digitalPinToInterrupt(pinAnamometer), ISR_anamomPulse, FALLING);

    //ready the ISR to receive its first pulse and indicate no period is ready yet
    bFirstPulse = true;
    bGotPulse = false;
}
int read_anemometer_sensor(void){
    return fWindSpeed;
}
void CheckSpeed(void){
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
            fWindSpeed = (int) (CONST*100/fP);
           
        }//if
//    }//else
//
//    //and send it out to the serial monitor
//    Serial.print( "Windspeed : " );Serial.print( fWindSpeed,2 );Serial.println( " m/s" );
    }     
}//CheckSpeed

void CheckPulseTimeout(void){
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

void ISR_anamomPulse(void){
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