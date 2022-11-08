

#ifndef MBED_AXIS_H
#define MBED_AXIS_H
 
#include "mbed.h"
#include "PID.h"        //library for software routine PID controller
#include "LS7366.h"     //library for quadrature encoder interface IC's
#include "MotCon.h"     //simple motor control routines

class Axis{
public:
    Axis(SPI& _spi, PinName _cs, PinName _pwm, PinName _dir, PinName _analog, int* limit);    
    void paramUpdate(void);
    void center(void);
    void init(float encCountsPerRev);
    void moveTrapezoid(float position, float time);
    void moveUpdate(void);
    float readCurrent(void);
    void axisOff(void);
    void axisOn(void);
    void zero(void);
    void writeEncoderValue(long value);
    float readEncoderValue(void);
    void updatePIDgains(float P, float I, float D);
    void changeMoveMode(int mode);
    
    long enc;       //used to return the data from the LS7366 encoder chip
    float co;       // = 0.0;
    float Tdelay;   // = .01;
    float Pk;       // 120.0 for scorbot
    float Ik;       // 55.0 for scorbot
    float Dk;
    float set_point;// = 0.0;
    float set_point_last;
    float pos, vel, acc;    //calculated position, velocity, and acceleration
    int stat;               //overall axis status
    float pos_last, vel_last, acc_last; //history variables used to calculate motion
    float pos_cmd, vel_cmd, vel_avg_cmd, acc_cmd;    
    float vel_max, acc_max;
    float vel_accum;
    float moveTime;    
    float p_higher, p_lower;
    int moveStatus;
    int moveState;
    int debug;
    int *ptr_limit;
    float motI;   //motor current read from readCurrent() function
    volatile float motI_last;
    float mot_I_lim;    //max current limit
    float dIdT;
    float mot_I_max, mot_I_max_last;
    int axisState;
    int motInvert;
    char dataFormat;    //'r'=radians (default), 'd'=degrees, 'e'=encoder counts
    float pos_rad, vel_rad;      //current position measurement in radians
    float pos_deg, vel_deg;  //current position measurement in degrees
    float ctsPerDeg;
    int busyflag;
    float test_acc;
    int moveMode;       //Movement Control Mode: 0-position, 1-velocity,2-acceleration
    float countsPerRev; //encoder counts per shaft revolution

    Ticker update;
    Ticker moveProfile;
    Timer t;
    PID *pid;
    LS7366 *ls7366;
    MotCon *motcon;
    //AnalogIn *motCurrent;
    
private:
    SPI _spi;
    DigitalOut _cs;
    PwmOut _pwm;
    DigitalOut _dir;
    AnalogIn _analog;
};

#endif


