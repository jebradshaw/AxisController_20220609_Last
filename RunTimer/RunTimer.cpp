#include "RunTimer.h"

RunTimer::RunTimer(){
    this->ms=0;
    this->sec =0;
    this->min = 0;
    this->hour = 0;
    this->day = 0;
    
    this->timer_10ms.attach(this, &RunTimer::timeAcc, .01);
}

void RunTimer::timeAcc(void){
    ms_total += 10.0;
    this->ms +=10;
    if(this->ms == 1000){
        this->ms = 0;
        this->sec++;
        if(this->sec==60){
            this->sec=0;
            this->min++;
            if(this->min == 60){
                this->min=0;
                this->hour++;
                if(this->hour==24){
                    this->hour=0;
                    day++;
                }
            }
        }    
    }
}//timeAcc

void RunTimer::Reset(void){
    this->ms=0;
    this->sec =0;
    this->min = 0;
    this->hour = 0;
    this->day = 0;
}