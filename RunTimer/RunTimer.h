/** RunTimer class.
 *  J. Bradshaw 20160519
 *  library for building a 10 millisecond running timer
 *
 * Example:
 * @code
 * #include "mbed.h"
 * #include "RunTimer.h"
 * 
 * Serial pc(USBTX,USBRX);
 * RunTimer runTime;
 * 
 * int main() {
 *     while(1){
 *          pc.printf("Time=day=%02d hour=%02d min=%02d sec=%02d ms=%02d \r\n", runTime.day,runTime.hour,runTime.min,runTime.sec,runTime.ms)
 *          wait(.02);
 *     }             
 * }
 * @endcode
 */
#ifndef MBED_RUNTIMER_H
#define MBED_RUNTIMER_H

#include "mbed.h"

class RunTimer{
    
public:    
    RunTimer();
    
    void timeAcc(void);
    void Reset(void);
    
    Ticker timer_10ms;          //Ticker for adding 10ms
    
    float ms_total;
    unsigned int ms;
    unsigned int sec;
    unsigned int min;
    unsigned int hour;
    unsigned int day;    
};

#endif