// Test program for NXP LPC1768 processor Scorbot Interface
// J Bradshaw
// After compiling save to 

//    C:\Users\bradshaw\AppData\Roaming\Microsoft\Windows\Start Menu\Programs\Python 3.7\intelhex\scripts

//  and rename as axis.bin, then run the "python bin2hex.py axis.bin axis.hex" converter
//  Then use FlashMagic to dump to processor using bootloader by toggling the NRST and PRG pins

// 20150731 - Began prototyping motor controller software for encoder with limit switch and motor driver
// 20150827 - Got Ticker Trapeziodal profile command working (moveTrapezoid, moveUpdate) 
// 20150924 - Port from the mbed to the LPC1768 processor for the 6 axis robotic arm controller
// 20151218 - Eliminated PCF8574 I2C I/O expander and attached limit switches to P0_22 - P0_17
//             pins as external interrupts.
// 20160311 - Build first PCB prototype
// 20160518 - First Production PCB fab
// 20160906 - BSEPR class design to inherit Axis class
// 20161110 - Currently using python script to convert .bin to .hex
//             save file to C:\Python27\intelhex\scripts  then execute script AxisConvert.bat
//             on Desktop
// cd C:\Python27\intelhex\scripts
// rename axis*.bin axis.bin
// python bin2hex.py axis.bin axis.hex
//  
// Then use Flash Magic to dump .hex filoe using serial bootloader

// 20161115 - Fixed Home function where Home_elbow() second pass does not fail moving EW_up()
// 20170705 - Working theta2enc() function, backed out math to all joints in degrees
// 20170720 - Working enc2theta() function, pitch still seems just a little off from home position
//  (-84.89 instead of -88.81 at home position).  Still having problems manually setting PID gains
//  with serial/scanf() functions.
// 20170721 - FINISHED enc2theta and theta2enc routines using pitch ratio "magic number" derived from 
//   WRIST_ELBOW_OFFSET_RATIO = .230796           //MOT_4_5_CPD/((95.02*MOT_2_3_CPD)/88.81)
//   Also converted the elbow axis to negative (makes more sense). Home = -95.02
// 20170803 - made axisOn() function only re-zero set point when not already homed.  This would
//   otherwise cause joint angles with non-zero offests to automatically go to zero when turned back on
// 20171101 - Moved character sort to switch statement and added moveDone_or_qs() function
// 20180706 - Added '|' current sweep motor profile identity function and ESC char board reset.
// 20180724 - Added I2C EEPROM 24LC256 memory functions for storing motor currents, etc
// 20180725 - MOT_SUM_I_MAX in collision check for total current and start timer, disable 
//   all motor drivers if MOT_SUM_TIME_MAX has exceeded
// 20181116 - Added provisions to interrupt instantaneous movement routines with updates.
//   Added bit clear in stall_state in motor current measurement when axis stall NOT measured!!!
//   Added !PosCMDdtxx in the list of commands, 'q' to quit, 'l' to loop
//   Added Axis::readEncoderValue() for when the axisState is Off so you can still read the axis
//    position with the controller disabled
// 20181127 - Changes trapezoidal profile moveUpdate() to use current set point and not position
//    which was resulting in counts lag (never reaching final position) v1.03
// 20190108 - Added provisions for Axis Mode Control change (position, velocity, acceleration)
// 20190110 - Use theta2enc() for velocity Ticker function
// 20190205 - Velocity Ticker uses integrated velocity update to overwrite robotic
//    joint that was read only if velocity flag is set for that joint (works) v1.05
// 20190313 - Worked on porting velocity controller to a single command that works on updateing
//    velocity component on all robot joints simultaneously.  Vt,b,s,e,p,r,g\r
//    t=timeout in sec (max 10.0 sec, min 0.0)
//    b,s,e,p,r in rad/sec: b max/min is .4/-.4, s .5/-.5, e .5/-.5, p 1.8/-.1.8, r 1.8/-.1.8
//    g in mm/sec is max/min 12.0/-12.0
// 20190318 - Flashed all controllers in MU111 with v1.06 firmware.
// 20220609 Troubleshooting exercise bindup: qFlag=-1 failure during exercise
//   Line 2683 added axisX.moveState = 4; before exiting with -2 for qFlag return
//   to indicate that the motion had not stopped before dt timed out

/*
 Motor 1:  80 counts on encoder  *  127.1:1 (mot gearbox) = 10168 counts / rev

                  24 teeth on motor gear head, and 120 teeth on base gear

                 10168 * (120 tooth / 24 teeth) = 50840 counts / rev

                   50840 / 360 degrees = 141.222 counts / degree
-------------------------------------------------------------------------------------------------------------------

  Motor 2 and 3 :  80 counts on encoder  *  127.1:1 (mot gearbox) = 10168 counts / rev

                  18 teeth on motor gear head, and 72 teeth on base gear

                 10168 * (72 tooth / 18 teeth) = 40672 counts / rev

                    40672 counts / 360 degrees = 112.978 counts / degree
-------------------------------------------------------------------------------------------------------------------

  Motor 4 and 5: 80 counts on encoder  *  65.5:1 (mot gearbox) = 5240 counts / rev

                 12 teeth on motor gear head, and 23 teeth on elbow joint (directly coupled to another 23 tooth hear head on pitch/roll joint)

                5240 * (23 tooth / 12 teeth) = 10043.3 counts / rev

               10043.3 counts / 360 degrees = 27.8981 counts / degree

--------------------------------------------------------------------------------------------------------------------
  Motor 6 - Gripper
  
  Counts        millimeters         inches
   500              7.43                .292
  1000              16.83               .663
  1500              24.39               .960
  2000              32.73              1.288
  2500              40.60              1.598
  3000              48.31              1.902
  3500              54.84              2.159
  4000              62.88              2.475
  
*/

// 20190329 - Disable gripper current stall collision detect and add "HOMED\r"
//   after home complete
// 20191028 - Add "[" Velocity profile command
// 20191029 - First working test using JTAG/SWD debug port, fast programming
// 20191101 - Discovered that gripper mm to counts translation was causing the
//   zero offset value (73) to accumulate when called, removed zero offet val


#include "mbed.h"
#include "Axis.h"
#include "RunTimer.h"
#include "stdlib.h"
#include "eprom_address.h"

#define MODSERIAL_DEFAULT_RX_BUFFER_SIZE 1024
#define MODSERIAL_DEFAULT_TX_BUFFER_SIZE 1024

#define VERSION_STRING  "v1.09"
#include "MODSERIAL.h"

#include <string>

#define PI          (3.14159f)
#define TWOPI       (6.28319f)
#define RADPERDEG (.017453f)
#define SP_TOL      5  // SET POINT TOLERANCE is +/- tolerance for set point command (was 15 on 20181121)

//Changed 20191030
#define MOT_SUM_I_MAX           4.60f           // was 2.4 20190326 is total current over 1.5 amps
#define MOT_SUM_TIME_MAX        10.0f            // 3.0 seconds

//#define MOT_SUM_I_MAX           2.60f           // was 2.4 20190326 is total current over 1.5 amps
//#define MOT_SUM_TIME_MAX        10.0f            // 3.0 seconds
int sumAmpsTimeFlag = 0;                        //flag for testing when max sum amps have been on too long

// A percentage of the total stall current before stall condition is considered valid
#define MAX_CURRENT_SCALE1   .80f    //80% of max measured
#define MAX_CURRENT_SCALE2   .80f    //80% of max measured
#define MAX_CURRENT_SCALE3   .80f    //80% of max measured
#define MAX_CURRENT_SCALE4   .80f    //80% of max measured
#define MAX_CURRENT_SCALE5   .80f    //80% of max measured
#define MAX_CURRENT_SCALE6   .85f    //85% of max measured

// Initial values taken from many averages of 12 different scorbot arms
float STALL_I_1 = 1.117f * MAX_CURRENT_SCALE1;  //0.9f
float STALL_I_2 = 1.021f * MAX_CURRENT_SCALE2;  //0.9f        //1.28 stall measured on motors 2
float STALL_I_3 = 1.081f * MAX_CURRENT_SCALE3;  //0.8f         //1.28 stall measured on motor 3
float STALL_I_4 = 1.091f * MAX_CURRENT_SCALE4;  //1.0f
float STALL_I_5 = 1.294f * MAX_CURRENT_SCALE5;  //1.0f
float STALL_I_6 = 1.926f * MAX_CURRENT_SCALE6;  //1.5f        //1.7 stall measured for motor 6, about .4 amp running current


// OLD defaults values
#define STALL_I_1_DEFAULT       0.9f
#define STALL_I_2_DEFAULT       0.9f        //1.28 stall measured on motors 2
#define STALL_I_3_DEFAULT       0.9f         //1.28 stall measured on motor 3
#define STALL_I_4_DEFAULT       0.8f
#define STALL_I_5_DEFAULT       1.0f
#define STALL_I_6_DEFAULT       1.5f

int GRIPPER_COL_DETECT = 1;        //Initially enabled

#define CURRENT_CHECK_PERIOD    .1  //period of time in seconds to read currents

#define GRIPPER_STALL_I STALL_I_6  //measured amps stall current for gripper
float GRIPPER_BACKOFF  = 250;      // how many counts open after hard close when homeing becomes zero
                                   // 50 is may too tight, 150 still seemed tight bigger is looser. 
                                   // 260 seems just right.  mmmmm poorage
                                // well ... maybe 240, or 250

//collision detection parameters for collisionCheck()
#define MAX_COL_COUNTS  4
#define MAX_COL6_COUNTS  3600   //3600 min = (60sec * 3min) / .05sec
volatile int  axis1col_cnt=0,
            axis2col_cnt=0,
            axis3col_cnt=0,
            axis4col_cnt=0,
            axis5col_cnt=0,
            axis6col_cnt=0;
                                    

#define HOME_DELAY_GLOBAL_DEFAULT .04   //default home delay time between 100 enc counts

// Counts Per Degree of each Scorbot Axis
#define MOT_1_CPD                   (141.222f)           //360 deg / (80 counts * 127.1 * (120/24) = 141.222 counts per degree
#define MOT_2_3_CPD                 (112.978f)           //Counts/Deg - Counted teeth to obtain gear ratio
#define MOT_4_5_CPD                 (27.8981f)           //Counts/Deg - Counted teeth to obtain gear ratio
#define WRIST_ELBOW_RATIO           (.246934f)           //Ratio of wrist to elbow gear ratios - MOT_4_5_CPD/MOT_2_3_CPD
#define WRIST_ELBOW_OFFSET_RATIO    (.230796f)           //MOT_4_5_CPD/((-95.02*MOT_2_3_CPD)/-88.81)

// counts per 360 degree rotation for velocity controller
#define MOT_1_CPR                   (50839.9f)           // 141.222 * 360
#define MOT_2_CPR                   (40672.1f)           // 112.978 * 360
#define MOT_3_CPR                   (40672.1f)
#define MOT_4_CPR                   (10043.3f)           // 27.8981 * 360
#define MOT_5_CPR                   (10043.3f)
#define MOT_6_CPR                   (4000.0f)            // about 4000 counts for 62.88mm or 2.475" of throw (see notes on 2-9-17)

// Radians per Count of each Scorbot Axis
#define MOT_1_RPC                       (.000124f)           //TWOPI/MOT_1_CPR = .000124
#define MOT_2_3_RPC                     (.000154f)           //TWOPI/MOT_2_CPR = .000154
#define MOT_4_5_RPC                     (.000626f)           //TWOPI/MOT_3_CPR = .000626

char dev_mode_Global = 'r'; //initialize global device mode in counts

// Terminal command sequences.
#define CLR_SCR       "\33[;H\33[2J"
#define MOVE_L_C      "\33[%d;%dH\33[K"
#define RED_FONT      "\33[0;31m"
#define BLUE_FONT     "\33[0;34m"
#define GREEN_FONT    "\33[0;32m"
#define YELLOW_FONT   "\33[0;33m"
#define WHITE_FONT    "\e[97m"
#define DEFAULT_FONT  "\33[0m"
#define ERASE_LINE    "\33[K"

#define SW_DEBOUNCE_MS      5   // delay in milliseconds to debounce the limit switches
#define LED_ON  0               // Active low LED's on the 6-Axis Controller board
#define LED_OFF 1

// LEDs used for status indicators
//PwmOut led1(P1_18);    //GREEN LED - heartbeat
DigitalOut led1(P1_18, LED_OFF);    //GREEN LED - heartbeat
DigitalOut led2(P1_19, LED_OFF);    //YELLOW LED - Running mode
DigitalOut led3(P1_20, LED_OFF);    //RED LED - Error

DigitalOut enMotDrv(P4_28, 1);         //drives FET (100 ohm pullup required!) which pulls down EN pins on 
                                    // MC33926 motor driver IC's

//Serial pc(P0_2, P0_3);              //pc serial interface (USB)
MODSERIAL pc(P0_2, P0_3);              //pc serial interface (USB)
SPI spi(P0_9, P0_8, P0_7);          //MOSI, MISO, SCK

//Limit switch pins are declared as both DigitalIn's and Interrupt pins as provisions do not exist
// to interrupt on both edges, so the edge must be monitored and checked in the interrupt
DigitalIn limit1_pin(P0_22);
DigitalIn limit2_pin(P0_21);
DigitalIn limit3_pin(P0_20);
DigitalIn limit4_pin(P0_19);
DigitalIn limit5_pin(P0_18);
DigitalIn limit6_pin(P0_17);

InterruptIn limit1_int(P0_22);
InterruptIn limit2_int(P0_21);
InterruptIn limit3_int(P0_20);
InterruptIn limit4_int(P0_19);
InterruptIn limit5_int(P0_18);
InterruptIn limit6_int(P0_17);

//for EEPROM memory
#define EPROM_PAGE_SIZE 64
I2C i2c(P0_27, P0_28);
int EPROMaddress = 0xA0;
int pointerAdddress = 0;
char data_read[EPROM_PAGE_SIZE];
    
int limit1, limit2, limit3, limit4, limit5, limit6;         //global limit switch values
volatile float axis1_I,axis2_I,axis3_I,axis4_I,axis5_I,axis6_I;
int streamFlag=0;
int streamOutputActive_Flag=0;
float streamRate=.02;
volatile int stall_state = 0;
float serReadTimeout=.001;

float SHOULDER_SWITCH_OFFSET_COUNTS=0; //433.0;    //difference in counts between 120.28 deg home and
    // actual 0 count on the mechanical switch
    
float ELBOW_SWITCH_OFFSET_COUNTS=10735.2;     //difference in counts between 95.02 deg home and 0deg (95.02 * 112.878)
    // actual 0 enc count on mechanical switch

//changed 20191030  J. Bradshaw
float WRIST_SWITCH_OFFSET = -755;    //-1050.0; //default offset after wrist roll home     
//float WRIST_SWITCH_OFFSET = -700;    //-1050.0; //default offset after wrist roll home 

int DEBUG=0;

//Timer classes
Timer t;
Timer sumAmpsTimer;
//Run Timer for total runtime (regular timer class wraps too early)
RunTimer runTime;

// heartbeat pulse, collision checks, and current
Ticker pulse;
Ticker colCheck;
Ticker overCurrentTick;

//globals used for heartbeat indicator
int hb_flag=0;
int hb_cntr = 0;

volatile int human_or_PC=0;    //human keyboard interface is default, toggled with * command
float homeDelayGlobal = HOME_DELAY_GLOBAL_DEFAULT;  //initially set the global delay to default

//instantiate axis object NAME(spi bus, LS7366_cs, pwm, dir, analog current sensing, limitSwitchAddress)
Axis axis1(spi, P1_24, P2_5, P1_0, P0_23, &limit1);   //base
Axis axis2(spi, P1_25, P2_4, P1_1, P0_24, &limit2);     //shoulder
Axis axis3(spi, P1_26, P2_3, P1_4, P0_25, &limit3);     //elbow
Axis axis4(spi, P1_27, P2_2, P1_8, P0_26, &limit4);      //pitch/roll
Axis axis5(spi, P1_28, P2_1, P1_9, P1_30, &limit5);      //pitch/roll
Axis axis6(spi, P1_29, P2_0, P1_10, P1_31, &limit6);     //grip

// Prototypes
void limit1_irq(void);
void limit2_irq(void);
void limit3_irq(void);
void limit4_irq(void);
void limit5_irq(void);
void limit6_irq(void);
void init_limitSwitches(void);
void reset_stall_state(void);
void all_on(void);
void all_off(void);
void SEW_up(int counts);
void SEW_down(int counts);
void EW_up(int counts);
void EW_down(int counts);
void W_up(int counts);
void W_down(int counts);
void W_pitch(int counts);
void W_roll(int counts);
void G_close(int counts);
void G_open(int counts);
void zero_axis(int axis);
void zero_all(void);
void heartbeat(void);
void collisionCheck(void);
void print_all(void);
int home_base(void);
int home_shoulder(void);
int OLD_home_elbow(void);
int home_elbow(void);
int home_pitch_wrist(void);
int home_elbow(void);
int home_pitch_wrist(void);
int home_rotate_wrist(void);
int home_gripper(void);
int home_master(void);
int CMD_read(char *str, int numchars, float timeout);
void led_blink_sequence(void);
void panic_flash(int times);
void test_all_motors(void);
float read_Pk(int axis);
float read_Ik(int axis);
float read_Dk(int axis);
int check_stats(void);
void clear_motMaxI(void);
void print_motMaxI(void);
void print_motI(void);
int homed(void);
int axisMoving(void);
int theta2enc(float theta0, float theta1, float theta2, float theta3, float theta4);
int theta2enc_dt(float dt, float theta0, float theta1, float theta2, float theta3, float theta4, float grip);
void enc2theta(float *B_theta, float *S_theta, float *E_theta, float *P_theta, float *R_theta);
void printControllerOutputs(void);
int moveDone_or_qs(float dt);
void axisRampTestI(Axis &axisTest);
void setPointRun(Axis &axisRun, float temp_setpoint);
void writeEEPROM(int address, unsigned int eeaddress, char *data, int size);
void readEEPROM(int address, unsigned int eeaddress, char *data, int size);
void default_eprom_motI(void);
void read_eprom_motI(void);
void exerciseAllAxes(void);
void motCurrentProfileUpdate(void);
int checkBoundsRad(float *b,float *s,float *e,float *p,float *r,float *g);
void velUpdateTick(void);
//void velocityCmd(char RobotJoint, float velCmdRadSec, float timeout);
void velocityCmd(float timeout,float vb,float vs,float ve,float vp,float vr,float vg);

//Begin Scorbot Specific library functions
float theta1_deg, theta2_deg, theta3_deg, theta4_deg, theta5_deg, theta6_deg;
float theta1_rad, theta2_rad, theta3_rad, theta4_rad, theta5_rad, theta6_rad;
    
//Begin local class definitions for the Intelitek ScorBot Arm
#define BASE_CPD    MOT_1_CPD    //counts per degree
class Base{    
public:
    Base(float joint_cpd){
        _joint_cpd = joint_cpd;
    }        
    
    void set_deg(float degrees){
        this->deg_cmd_pos = degrees;
        
        if(axis1.stat==0){
            axis1.set_point = _joint_cpd * degrees;
        }
        else{
            pc.printf("Axes not homed for base position command.  Must home first.\r\n");
        }
    }
    float get_deg(void){
        return axis1.pos / _joint_cpd;
    }
    
    void set_rad(float radians){
        this->rad_cmd_pos = radians;
        
        this->set_deg(radians * 180.0/PI);
    }
    float get_rad(void){
        return this->get_deg() * (PI/180.0);
    }
    
    void set_counts(long counts){
        axis1.set_point = counts;        
    }    
    
    float deg_cmd_pos;
    float rad_cmd_pos;
private:    
    float _joint_cpd;    
};

Base sbBase(BASE_CPD);

#define SHOULDER_CPD        MOT_2_3_CPD     //0 cpr * 127.1 gear ratio * (72/18) external gears = 112.978 counts/degree
#define SHOULDER_OFFSET     13650.0    //0 degrees
#define SHOULDER_DEG_OFFSET 120.28     //0 enc

class Shoulder{    
public:
    Shoulder(float joint_cpd){
        _joint_cpd = joint_cpd;
    }        
    
    void set_deg(float degrees){
        if((axis2.stat==0) && (axis3.stat==0) && (axis4.stat==0) && (axis5.stat==0)){                  
            this->deg_pos_cmd = degrees;
            
            float counts = _joint_cpd * (this->deg_pos_cmd - SHOULDER_DEG_OFFSET);        
            float count_diff;
                
            count_diff = axis2.pos - counts;                        
            axis2.set_point = counts;
            axis3.set_point -= -count_diff;
            axis4.set_point -= -count_diff * WRIST_ELBOW_RATIO;
            axis5.set_point += -count_diff * WRIST_ELBOW_RATIO;

        }
        else{
            pc.printf("Axes not homed for shoulder position command.  Must home first.\r\n");
        }
    }
    float get_deg(void){
        return (axis2.pos + SHOULDER_OFFSET) / _joint_cpd;
    }
    
    void set_rad(float radians){
        this->rad_pos_cmd = radians;
        
        this->set_deg(radians * 180.0/PI) ;
    }
    float get_rad(void){
        return this->get_deg() * (PI/180.0);
    }
    
    void set_counts(long counts){
        axis2.set_point = counts;        
    }    
    float deg_pos_cmd;
    float rad_pos_cmd;
private: 
    float _joint_cpd;
};

Shoulder sbShoulder(SHOULDER_CPD);

#define ELBOW_CPD       MOT_2_3_CPD    //80 cpr * 127.1 gear ratio * (72/18) external gears = 112.978 counts/degree
#define ELBOW_OFFSET     10733.0    //0 degrees
#define ELBOW_DEG_OFFSET  95.02        //0 enc = 95.02 degrees
class Elbow{    
public:
    Elbow(float joint_cpd){
        _joint_cpd = joint_cpd;
    }        
    
    void set_deg(float degrees){
       if((axis3.stat==0) && (axis4.stat==0) && (axis5.stat==0)){            
            this->deg_pos_cmd = degrees;
            
            float counts = _joint_cpd * (this->deg_pos_cmd - ELBOW_DEG_OFFSET);    
            float count_diff;
                
            count_diff = axis3.pos- counts;
                        
            axis3.set_point = counts;
            axis4.set_point -= count_diff * WRIST_ELBOW_RATIO;
            axis5.set_point += count_diff * WRIST_ELBOW_RATIO;
        }
        else{
            pc.printf("Axes not homed for shoulder position command.  Must home first.\r\n");
        }
    }
    float get_deg(void){
        return ELBOW_DEG_OFFSET - ((ELBOW_OFFSET - axis3.pos) / _joint_cpd);
    }
    
    void set_rad(float radians){
        this->set_deg(radians * 180.0/PI) ;
    }
    float get_rad(void){
        return this->get_deg() * (PI/180.0);
    }
    
    void set_counts(long counts){
        axis3.set_point = counts;        
    }    
    float deg_pos_cmd;
    float rad_pos_cmd;
private:    
    float _joint_cpd;    
};

Elbow sbElbow(ELBOW_CPD);

#define PITCH_CPD           MOT_4_5_CPD      //counts per degree
#define PITCH_DEG_OFFSET    -88.81      //Home for pitch is -88.81 deg
#define PITCH_OFFSET        PITCH_DEG_OFFSET*MOT_4_5_CPD    //-2477.63 //-1058.0f

#define ROLL_CPD            MOT_4_5_CPD //counts per degree
#define ROLL_OFFSET         0.0


class Pitch{    
public:
    Pitch(float joint_cpd){
        _joint_cpd = joint_cpd;
    }
    
    void set_deg(float degrees){
       if((axis4.stat==0) && (axis5.stat==0)){
            float roll_counts = (axis4.pos + axis5.pos)/2.0;
            float roll_deg = roll_counts / ROLL_CPD;            
            pc.printf("roll deg = %7.2f  counts=%7.2f\r\n\r\n", roll_deg, roll_counts);
            
            float pitch_counts = ((axis5.pos - axis4.pos) / 2.0) + PITCH_OFFSET;
            float pitch_deg  = pitch_counts / PITCH_CPD;
            pc.printf("pitch deg = %7.2f  counts=%7.2f\r\n\r\n", pitch_deg, pitch_counts);
            
            float new_pitch_counts = (_joint_cpd * degrees);
            pc.printf("set degrees (%7.2f) command counts = %7.2f\r\n", degrees, new_pitch_counts);
            
            float a4, a5;
            //2P = A5 - A4
            //Current roll plus new pitch
            //      2*current roll counts = A4 + A5
            // +    2*new pitch counts    = A5 - A4
            //    = combined counts       =  2 * A5
            //    = combined counts / 2  = A5
            a5 = (roll_counts * 2.0) + (new_pitch_counts * 2.0) / 2.0;
            
            // New P = (A5 - A4) / 2.0
            
            // new_pitch_counts = (a5 - A4) / 2.0
            //
            //  so...   2*new_pitch_counts = a5 - a4
            
            a4 = a5 - (2.0 * new_pitch_counts);
            
            //Check calculation
            float p_test = ((a5 - a4) / 2.0) / PITCH_CPD;
            float r_test = ((a4 + a5) / 2.0) / ROLL_CPD;
            
            pc.printf("\r\n\r\n pitch calc test deg = %7.2f\r\n roll calc test deg = %7.2f\r\n", p_test, r_test);            
            
            axis4.set_point = a4;
            pc.printf("axis4.set_point = %7.2f\r\n", axis4.set_point);
            axis5.set_point = a5;
            pc.printf("axis5.set_point = %7.2f\r\n", axis5.set_point);
        }
        else{
            pc.printf("Axes not homed for shoulder position command.  Must home first.\r\n");
        }        
    }
    float get_deg(void){
        return ((axis5.pos - axis4.pos)/2.0) / _joint_cpd;
    }
    
    void set_rad(float radians){
        this->set_deg(radians * 180.0/PI) ;
    }
    float get_rad(void){
        return this->get_deg() * (PI/180.0);
    }
private:    
    float _joint_cpd;    
};

Pitch sbPitch(PITCH_CPD);

class Roll{    
public:
    Roll(float joint_cpd){
        _joint_cpd = joint_cpd;
    }        
    
    void set_deg(float degrees){
       if((axis4.stat==0) && (axis5.stat==0)){                                        
            float roll_counts = (axis4.pos + axis5.pos) / 2.0;
            float roll_deg = roll_counts / ROLL_CPD;            
            pc.printf("\r\nroll deg = %7.2f  counts=%7.2f\r\n", roll_deg, roll_counts);
            
            float pitch_counts = ((axis5.pos - axis4.pos) / 2.0) + PITCH_OFFSET;
            float pitch_deg  = pitch_counts / PITCH_CPD;
            pc.printf("pitch deg = %7.2f  counts=%7.2f\r\n", pitch_deg, pitch_counts);
            
            float new_roll_counts = (_joint_cpd * degrees);
            pc.printf("set degrees (%7.2f) command counts = %7.2f\r\n", degrees, new_roll_counts);
            
            float a4, a5;
            //2R = A4 + A5
            //Current pitch plus new roll
            //      2*current pitch counts = A5 - A4
            // +    2*new roll counts    = A4 + A5
            //    = combined counts       =  2 * A5
            //    = combined counts / 2  = A5
            a5 = (pitch_counts * 2.0) + (new_roll_counts * 2.0) / 2.0;
            
            // New R = (A4 + A5) / 2.0
            
            // new_roll_counts = (A4 + a5) / 2.0
            //
            //  so...   2*new_roll_counts = a4 + a5
            
            a4 = (2.0 * new_roll_counts) - a5;
            
            //Check calculation
            float p_test = ((a5 - a4) / 2.0) / PITCH_CPD;
            float r_test = ((a4 + a5) / 2.0) / ROLL_CPD;
            
            pc.printf("a4=%7.2f  a5=%7.2f  \r\n", a4, a5);
            
            pc.printf("\r\n\r\n pitch calc test deg = %7.2f\r\n roll calc test deg = %7.2f\r\n", p_test, r_test);            
            
            axis4.set_point = a4;
            pc.printf("axis4.set_point = %7.2f\r\n", axis4.set_point);
            axis5.set_point = a5;
            pc.printf("axis5.set_point = %7.2f\r\n", axis5.set_point);
        }
        else{
            pc.printf("Axes not homed for shoulder position command.  Must home first.\r\n");
        }        
    }
    float get_deg(void){
        return ((axis4.pos + axis5.pos) / 2.0) / _joint_cpd;
    }
    
    void set_rad(float radians){
        this->set_deg(radians * 180.0/PI) ;
    }
    float get_rad(void){
        return this->get_deg() * (PI/180.0);
    }
private:    
    float _joint_cpd;    
};

Roll sbRoll(ROLL_CPD);

//Gripper Class
class Gripper{
public:
    Gripper(){
    }
    
    void set_mm(float mm){
       if(axis6.stat==0){
            float counts = .13*mm*mm + 55.0*mm; //get rid of zero offset, it will grow initentionally otherwise JB 20191101  + 73.0;                
            
            axis6.set_point = counts;
        }
        else{
            pc.printf("Axes not homed for gripper position command.  Must home first.\r\n");
        }
    }
    
    float get_mm(void){
        int counts = axis6.pos;
        //For testing 20191104
        float returnVal = (-.0000005*(float)counts*(float)counts) + (.018*(float)counts);
        
        //pc.printf("counts read in mm from get_mm = %.4f\r\n", returnVal);
        
        return returnVal;
    }
        
    float get_inches(void){   
        int counts = axis6.pos;     
        return -.00000002*counts*counts + .0007*counts;
    }
    
    void set_inches(float inches){        
       if(axis6.stat==0){                        
            float counts = 83.0*inches*inches + 1400.0*inches;  //get rid of zero offest + 73.0;                
            
            axis6.set_point = counts;
        }
        else{
            pc.printf("Axes not homed for gripper position command.  Must home first.\r\n");
        }
    }
    
    float mm_to_counts(float mm){                                   
            float counts = .13*mm*mm + 55.0*mm; // + 73;
            return counts;
    }

    float counts_to_mm(float counts){             
        return -.00000002*counts*counts + .0007*counts;
    }
};

Gripper sbGripper;

void limit1_irq(void){
    wait_ms(SW_DEBOUNCE_MS);
    limit1 = limit1_pin;
    
    if(limit1)
        limit1_int.fall(&limit1_irq);
    else
        limit1_int.rise(&limit1_irq);        
}

void limit2_irq(void){
    wait_ms(SW_DEBOUNCE_MS);
    limit2 = limit2_pin;
    
    if(limit2)
        limit2_int.fall(&limit2_irq);
    else
        limit2_int.rise(&limit2_irq);
}

void limit3_irq(void){
    wait_ms(SW_DEBOUNCE_MS);
    limit3 = limit3_pin;
    
    if(limit3)
        limit3_int.fall(&limit3_irq);
    else
        limit3_int.rise(&limit3_irq);
}

void limit4_irq(void){
    wait_ms(SW_DEBOUNCE_MS);
    limit4 = limit4_pin;
    
    if(limit4)
        limit4_int.fall(&limit4_irq);
    else
        limit4_int.rise(&limit4_irq);
}

void limit5_irq(void){
    wait_ms(SW_DEBOUNCE_MS);
    limit5 = limit5_pin;
    
    if(limit5)
        limit5_int.fall(&limit5_irq);
    else
        limit5_int.rise(&limit5_irq);
}

void limit6_irq(void){
    wait_ms(SW_DEBOUNCE_MS);
    limit6 = limit6_pin;
    
    if(limit6)
        limit6_int.fall(&limit6_irq);
    else
        limit6_int.rise(&limit6_irq);
}

void init_limitSwitches(void){
    
    //Limit switch 1 initial state
    limit1 = limit1_pin;
    if(limit1)
        limit1_int.fall(&limit1_irq);
    else
        limit1_int.rise(&limit1_irq);
                
    //Limit switch 2 initial state
    limit2 = limit2_pin;
    if(limit2)
        limit2_int.fall(&limit2_irq);
    else
        limit2_int.rise(&limit2_irq);
        
    //Limit switch 3 initial state
    limit3 = limit3_pin;
    if(limit3)
        limit3_int.fall(&limit3_irq);
    else
        limit3_int.rise(&limit3_irq);

    //Limit switch 4 initial state
    limit4 = limit4_pin;
    if(limit4)
        limit4_int.fall(&limit4_irq);
    else
        limit4_int.rise(&limit4_irq);

    //Limit switch 5 initial state
    limit5 = limit5_pin;
    if(limit5)
        limit5_int.fall(&limit5_irq);
    else
        limit5_int.rise(&limit5_irq);

    //Limit switch 6 initial state
    limit6 = limit6_pin;
    if(limit6)
        limit6_int.fall(&limit6_irq);
    else
        limit6_int.rise(&limit6_irq);
}

void reset_stall_state(void){
    led3=LED_OFF; //Turn Off the RED LED
    stall_state=0;  //reset the stall_state
}
    
void all_on(void){
    axis1.axisOn();
    axis2.axisOn();
    axis3.axisOn();
    axis4.axisOn();
    axis5.axisOn();
    axis6.axisOn();            
}

void all_off(void){
    axis1.axisOff();
    axis2.axisOff();
    axis3.axisOff();
    axis4.axisOff();
    axis5.axisOff();
    axis6.axisOff();     
}

//move the base
void Base(int counts){
    axis1.set_point += counts;
}

//shoulder + elbow + wrist up
void SEW_up(int counts){
    axis2.set_point += counts;
    axis3.set_point -= counts;
    axis4.set_point -= counts * WRIST_ELBOW_RATIO;
    axis5.set_point += counts * WRIST_ELBOW_RATIO;
}

//shoulder + elbow + wrist down
void SEW_down(int counts){
    axis2.set_point -= counts;
    axis3.set_point += counts;
    axis4.set_point += counts * WRIST_ELBOW_RATIO;
    axis5.set_point -= counts * WRIST_ELBOW_RATIO;  
}

//elbow + wrist up
void EW_up(int counts){
    axis3.set_point -= counts;
    axis4.set_point -= counts * WRIST_ELBOW_RATIO;
    axis5.set_point += counts * WRIST_ELBOW_RATIO;    
}

//elbow + wrist down
void EW_down(int counts){
    axis3.set_point += counts;
    axis4.set_point += counts * WRIST_ELBOW_RATIO;
    axis5.set_point -= counts * WRIST_ELBOW_RATIO;        
}

//move Wrist up
void W_up(int counts){
    axis4.set_point -= counts;
    axis5.set_point += counts;
}    

//move Wrist down
void W_down(int counts){
    axis4.set_point += counts;
    axis5.set_point -= counts;
}

//positive counts go up, negative counts go down
void W_pitch(int counts){
    axis4.set_point -= counts;
    axis5.set_point += counts;
}

//move Wrist down
void W_roll(int counts){
    axis4.set_point += counts;
    axis5.set_point += counts;
}
    
//Gripper close
void G_close(int counts){
    axis6.set_point -= counts;
}

//Gripper open
void G_open(int counts){
    axis6.set_point += counts;
}
    
//zero axis
void zero_axis(int axis){
    all_off();
    switch(axis){                                        
        case 1:
            axis1.zero();
        break;
        
        case 2:
            axis2.zero();
        break;
        
        case 3:
            axis3.zero();
        break;
        
        case 4:
            axis4.zero();
        break;
        
        case 5:
            axis5.zero();             
        break;
        
        case 6:
            axis6.zero();
        break;                                                                                                    
    }    
    all_on();
}

void zero_all(void){    
    for(int i=1;i<=6;i++){
        zero_axis(i);
        wait(.005);
    }
}

void heartbeat(void){
    led1 = !led1;
    /*    
    switch(hb_flag){        
        case 0:        
            led1 = led1 - .2;
            if(led1 <= 0.0)
                hb_flag = 1;
            break;
        case 1:    
            led1 = led1 + .2;
            if(led1 >= 1.0){
                hb_flag = 2;            
            }
            break;
        case 2:        
            led1 = led1 - .2;
            if(led1 <= 0.0)
                hb_flag = 3;
            break;
        case 3:    
            led1 = led1 + .2;
            if(led1 >= 1.0){
                hb_flag = 4;
            }
            break;
        case 4:
            hb_cntr++;
            if(hb_cntr>40){
                hb_cntr = 0;
                hb_flag = 0;
            }
            break;
        default:
            hb_cntr = 0;
            hb_flag = 0;
            break;
    }//switch   
    */
    
    char c = pc.rxGetLastChar() & 0xFF;        
    if(c == 27){      // Pressed ESC to reset
        NVIC_SystemReset();     //Restart the System
    }
}

void stop_all_movement(void){
    axis1.moveState = 0;
    axis2.moveState = 0;
    axis3.moveState = 0;
    axis4.moveState = 0;
    axis5.moveState = 0;
    axis6.moveState = 0;
    
    axis1.stat=1;
    axis2.stat=1;
    axis3.stat=1;
    axis4.stat=1;
    axis5.stat=1;
    axis6.stat=1;        
}
            
void collisionCheck(void){
    axis1_I = axis1.readCurrent();
    axis2_I = axis2.readCurrent();
    axis3_I = axis3.readCurrent();
    axis4_I = axis4.readCurrent();        
    axis5_I = axis5.readCurrent();
    axis6_I = axis6.readCurrent();
        
    if(axis1.motI > STALL_I_1){
        led3=LED_ON; //set the RED LED on
        axis1col_cnt++;
        if(axis1col_cnt > MAX_COL_COUNTS)
            stall_state |= 0x01;
    }
    else{
        axis1col_cnt=0;
        stall_state &= 0xfe;        //Clear bit 0 if no stall was detected on Axis 1
    }
    
    if(axis2.motI > STALL_I_2){        
        led3=LED_ON; //set the RED LED on
        axis2col_cnt++;
        if(axis2col_cnt > MAX_COL_COUNTS)
            stall_state |= 0x02;
    }
    else{
        axis2col_cnt=0;
        stall_state &= 0xfd;        //Clear bit 1 if no stall was detected on Axis 2
    }

    if(axis3.motI > STALL_I_3){        
        led3=LED_ON; //set the RED LED on
        axis3col_cnt++;
        if(axis3col_cnt > MAX_COL_COUNTS)        
            stall_state |= 0x04;
    }
    else{
        axis3col_cnt=0;
        stall_state &= 0xfb;        //Clear bit 2 if no stall was detected on Axis 3
    }

    if(axis4.motI > STALL_I_4){        
        led3=LED_ON; //set the RED LED on
        axis4col_cnt++;
        if(axis4col_cnt > MAX_COL_COUNTS)        
            stall_state |= 0x08;
    }
    else{
        axis4col_cnt=0;
        stall_state &= 0xf7;        //Clear bit 3 if no stall was detected on Axis 4
    }
    
    if(axis5.motI > STALL_I_5){        
        led3=LED_ON; //set the RED LED on
        axis5col_cnt++;
        if(axis5col_cnt > MAX_COL_COUNTS)         
            stall_state |= 0x10;
    }    
    else{
        axis5col_cnt=0;
        stall_state &= 0xef;        //Clear bit 4 if no stall was detected on Axis 5
    }
    
    if(GRIPPER_COL_DETECT){    
        if(axis6.motI > STALL_I_6){        
            led2=LED_ON; //set the YELLOW LED on
            axis6col_cnt++;
            if(axis6col_cnt > MAX_COL_COUNTS){
                stall_state |= 0x20;
            }            
            if(axis6col_cnt > MAX_COL6_COUNTS){
                led3=LED_ON; //set the RED LED on
                stall_state |= 0x20;
                pc.printf("%s\r\nGRIPPER STALLED FOR TOO LONG\r\n Shutting Down!!\r\n%.2f!!\r\n",RED_FONT, runTime.ms_total*.001);
                all_off();              //disable all of the axes (so motors dont cook forever)
            }
        }
        else{
            axis6col_cnt=0;
            stall_state &= 0xdf;        //Clear bit 5 if no stall was detected on Axis 6
        }
    }
        
    //check for stall condition and start timer if exceeds max
    if(stall_state != 0x00){
        if(sumAmpsTimeFlag == 0){
            if(DEBUG){
                pc.printf("%s\r\nSTART OVERCURRENT TIMER AT %.2f!!\r\n",RED_FONT, runTime.ms_total*.001);
                pc.printf("STALL STATE = 0x%X\r\n", stall_state);
                pc.printf("%s", DEFAULT_FONT);
            }
            sumAmpsTimeFlag = 1;        //turn on the sumAmpsTimeFlag
            sumAmpsTimer.start();
        }
        else{
            if(sumAmpsTimer.read() > MOT_SUM_TIME_MAX){
                sumAmpsTimer.stop();
                pc.printf("%sMAX SUM AMP TIMER EXCEEDED!!! SHUTING DOWN NOW AT %.2f!!\r\n",RED_FONT, runTime.ms_total*.001);
                pc.printf("STALL STATE = 0x%X\r\n", stall_state);
                pc.printf("%s", DEFAULT_FONT);
                enMotDrv = 1;   //disable all the motor driver IC's                
                panic_flash(20);
                sumAmpsTimeFlag = 0;        //turn off the sumAmpsTimeFlag
                NVIC_SystemReset();     //Restart the System
            }
        }  
    }
    else{
        sumAmpsTimeFlag = 0;
        sumAmpsTimer.stop();
        sumAmpsTimer.reset();
        led3 = LED_OFF;
    }
}

 
void print_all(void){
    pc.printf("axis,%7.1f,%7.1f,%7.1f,%7.1f,%7.1f,%7.1f  ", axis1.pos,axis2.pos,axis2.pos,axis3.pos,axis4.pos,axis5.pos);
    pc.printf("sw,%1d,%1d,%1d,%1d,%1d,%1d  ", limit1,limit2,limit3,limit4,limit5,limit6);
    pc.printf("I=%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f  ", axis1.motI,axis2.motI,axis3.motI,axis4.motI,axis5.motI,axis6.motI);
    pc.printf("0x%02X   \r\n", stall_state);    
}
    
int home_base(void){
    all_on();
    
    reset_stall_state();
    if(DEBUG)
        pc.printf("Homing Base\r\n");
    if(limit1 != 0){
        zero_axis(1);
        //move base CW (looking down) for short period first
        if(DEBUG)
            pc.printf("Moving CW, Limit 1 = %d\r\n", limit1);
        do{
            if(DEBUG)                        
                pc.printf("axis1=%.1f sw1=%1d I1=%.3f stall_state=0x%02X ax1dIdT=%.3f\r\n", axis1.pos, limit1, axis1.readCurrent(), stall_state, axis1.dIdT);
            axis1.set_point += 100;
            wait(homeDelayGlobal);
        }while((limit1 != 0) && (stall_state==0) && (axis1.pos < 2000));    
        if(DEBUG)
            pc.printf("axis1=%.1f sw1=%1d I1=%.3f stall_state=0x%02X ax1dIdT=%.3f\r\n", axis1.pos, limit1, axis1.readCurrent(), stall_state, axis1.dIdT);
        
        if(stall_state == 0x01){
            zero_axis(1);
            do{
                if(DEBUG)                        
                    pc.printf("axis1=%.1f sw1=%1d I1=%.3f stall_state=0x%02X ax1dIdT=%.3f\r\n", axis1.pos, limit1, axis1.readCurrent(), stall_state, axis1.dIdT);
                axis1.set_point -= 100;
                wait(homeDelayGlobal);
            }while((axis1.pos > -1000));               
            reset_stall_state();    //added 20180727
        }
        
        
        //move base CCW (looking down) until switch is pressed or base collision is detected
        if(DEBUG)
            pc.printf("Moving CCW, Limit 1 = %d\r\n", limit1);
        do{
            if(DEBUG)                        
                pc.printf("axis1=%.1f sw1=%1d I1=%.3f stall_state=0x%02X ax1dIdT=%.3f\r\n", axis1.pos, limit1, axis1.readCurrent(), stall_state, axis1.dIdT);
            axis1.set_point -= 100;
            wait(homeDelayGlobal);
        }while((limit1 != 0) && (stall_state==0));    
        if(DEBUG)
            pc.printf("axis1=%.1f sw1=%1d I1=%.3f stall_state=0x%02X ax1dIdT=%.3f\r\n", axis1.pos, limit1, axis1.readCurrent(), stall_state, axis1.dIdT);
        
        //did collision stop the base?
        if(stall_state != 0){
            zero_axis(1);
            
            reset_stall_state();
            //move base CW (looking down) until switch is pressed or base collision is detected
            if(DEBUG)
                pc.printf("Collision caused base stop, change direction until switch is pressed \r\n", limit1);
            
            do{                        
                if(DEBUG)
                    pc.printf("axis1=%.1f sw1=%1d I1=%.3f stall_state=0x%02X ax1dIdT=%.3f\r\n", axis1.pos, limit1, axis1.readCurrent(), stall_state, axis1.dIdT);
                axis1.set_point += 100;
                wait(homeDelayGlobal);
            }while((limit1 != 0) && (stall_state==0));    
            if(DEBUG)
                pc.printf("axis1=%.1f sw1=%1d I1=%.3f stall_state=0x%02X ax1dIdT=%.3f\r\n", axis1.pos, limit1, axis1.readCurrent(), stall_state, axis1.dIdT);
            
            if(stall_state != 0){
                if(DEBUG)
                    pc.printf("Collision detected, Returning error -1\r\n");
 //               return -1;
            }
                
            if(limit1 == 0){
                axis1.set_point += 10;
                wait(.02);
            }
        }
    }
          
   zero_axis(1);

    reset_stall_state();
    //move base CW (looking down) until switch is opened again
    if(DEBUG)
        pc.printf("Moving clockwise (looking down) until switch is released \r\n", limit1);
    do{                        
        if(DEBUG)
            pc.printf("axis1=%.1f sw1=%1d I1=%.3f stall_state=0x%02X ax1dIdT=%.3f\r\n", axis1.pos, limit1, axis1.readCurrent(), stall_state, axis1.dIdT);
        axis1.set_point += 10;
        wait(homeDelayGlobal);
    }while((limit1 != 1) && (stall_state==0));    
    
    if(stall_state != 0){
        if(DEBUG)
            pc.printf("Collision detected in error, exit with return -2\r\n");
        axis1.axisOff();
        return -2;
    }
    if(DEBUG)
        pc.printf("axis1=%.1f sw1=%1d I1=%.3f stall_state=0x%02X ax1dIdT=%.3f\r\n", axis1.pos, limit1, axis1.readCurrent(), stall_state, axis1.dIdT);
    zero_axis(1);
    
    axis1.stat=0;
    return 0;
}
   
int home_shoulder(void){
    all_on();
    reset_stall_state();  //re-zero stall state
    //fine tune shoulder, move up until switch closes ( == 0)
    if(limit2==1){
        if(DEBUG)
            pc.printf("\r\nMove Shoulder up until switch closes \r\n");
        do{ 
            SEW_up(100);
            wait(homeDelayGlobal);
            if(DEBUG)
                pc.printf("axis2=%.1f   sw2=%1d   I2=%.4f stall_state=0x%02X ax2dIdT=%.3f\r\n", axis2.pos, limit2, axis2.readCurrent(), stall_state, axis2.dIdT);
        }while((stall_state==0) && (limit2 != 0));
        if(DEBUG)
            pc.printf("axis2=%.1f   sw2=%1d   I2=%.4f stall_state=0x%02X ax2dIdT=%.3f\r\n", axis2.pos, limit2, axis2.readCurrent(), stall_state, axis2.dIdT);
        
        if(limit2 == 0){
            zero_axis(2);
            zero_axis(3);
            zero_axis(4);
            zero_axis(5);
        }
        if(stall_state != 0){
            if(DEBUG)
                pc.printf("\r\nCollision occured, stall state= 0x%02X, should have hit switch first!\r\nreturn Error -3\r\n", stall_state);
            return -3;
        }        
    }
    
    wait(.1);
    //W_up(200);
    //move shoulder down until switch opens again    
    if(DEBUG)
        pc.printf("\r\nMove Shoulder back down until switch opens again (slow)\r\n");
    do{     
        SEW_down(10);
        wait(homeDelayGlobal);
        if(DEBUG)
            pc.printf("axis2=%.1f   sw2=%1d   I2=%.4f stall_state=0x%02X ax2dIdT=%.3f\r\n", axis2.pos, limit2, axis2.readCurrent(), stall_state, axis2.dIdT);
    }while((stall_state==0) && (limit2 == 0));        
    if(DEBUG)
        pc.printf("\r\naxis2=%.1f   sw=%1d   I2=%.4f stall_state=0x%02X\r\n", axis2.pos, limit2, axis2.readCurrent(), stall_state);
    
    zero_axis(2);
    zero_axis(3);
    zero_axis(4);
    zero_axis(5);
    
    axis2.stat=0;
    return 0;
}
         
int home_elbow(void){
    all_on();
    reset_stall_state();
    zero_axis(3);        
    if(limit3 == 1){    //if limit switch is open
        //move elbow up until limit switch is closed or a collision occures
        if(DEBUG)
            pc.printf("\r\n\r\nLimit 3 = %d, move elbow and wrist up\r\n\r\n", limit3);
        EW_up(100);
        wait(homeDelayGlobal);
        do{
            EW_up(60);
            wait(homeDelayGlobal);
            if(DEBUG){
                pc.printf("axis3=%.1f axis4=%.1f axis5=%.1f sw3=%1d I3=%.4f stall_state=0x%02X ax3dIdT=%.3f ax4dIdT=%.3f ", axis3.pos, axis4.pos,axis5.pos,limit3, axis3.readCurrent(), stall_state, axis3.dIdT, axis4.dIdT);
                print_motI();
                pc.printf("\r\n");
            }                                
        }while((limit3 == 1) && (stall_state==0) && (axis3.pos > -8500));
        
        if(DEBUG){
            pc.printf("\r\n\r\naxis3=%.1f axis4=%.1f axis5=%.1f sw3=%1d I3=%.4f I4=%.4f I5=%.4f stall_state=0x%02X ax3dIdT=%.3f ax4dIdT=%.3f ax5dIdT=%.3f ", axis3.pos, axis4.pos,axis5.pos,limit3, axis3.readCurrent(),axis4.readCurrent(),axis5.readCurrent(), stall_state, axis3.dIdT, axis4.dIdT, axis5.dIdT);
            print_motI();
            pc.printf("\r\n");
        }            
        
        //already achieved home
        if((limit3 == 0) && (stall_state==0) && (axis3.pos > -8500)){
            //pc.printf("Early home achieved!!\r\n");        
            zero_axis(3);
            zero_axis(4);
            zero_axis(5);
            axis3.stat=0;
            return 0;
        }
        // or did a collision stop the upward motion, check motors 4 and 5 as well for wrist current collision
        if((stall_state != 0) && (limit3 == 1)){
            if(DEBUG)
                pc.printf("Collision Detected on Elbow home.  Moving down 200 steps");
            EW_down(200);
            wait(.03);
            
            //Maybe check if the WRIST is at the upper extreme here!!!  20170616
            home_pitch_wrist();   //20170616
            home_rotate_wrist();
            
            wait(.5);
            zero_axis(3);
            zero_axis(4);
            zero_axis(5);
            reset_stall_state();
            if(DEBUG){
                pc.printf("Collision stopped the upward movement. Move down.    ");            
                print_motI();
                pc.printf("\r\n  MAX I = ");
                print_motMaxI();
                pc.printf("\r\n");
            }                
                
            do{            
                EW_down(120);   //increased to 120 on 8-2-2017
                wait(homeDelayGlobal);
                if(DEBUG){
                    pc.printf("axis3=%.1f   sw=%1d   I3=%.4f stall_state=0x%02X ax3dIdT=%.3f   ", axis3.pos, limit3, axis3.readCurrent(), stall_state, axis3.dIdT);
                    print_motI();
                    pc.printf("\r\n");
                }
            }while((stall_state==0) && (limit3 == 1));
            if(DEBUG){
                pc.printf("axis3=%.1f   sw=%1d   I3=%.4f stall_state=0x%02X ax3dIdT=%.3f   ", axis3.pos, limit3, axis3.readCurrent(), stall_state, axis3.dIdT);
                print_motI();
                pc.printf("\r\n");
            }                
            
            if(stall_state!=0){
                if(DEBUG){
                    pc.printf("\r\nCollision detected, returning error -1  ");
                    print_motI();
                    pc.printf("\r\n   MAX = ");
                    print_motMaxI();
                    pc.printf("\r\n");
                }                
                return -1;    
            }
            
            if(limit3 == 0){
                //move down again until switch opens
                do{            
                    EW_down(60);
                    wait(homeDelayGlobal);
                    if(DEBUG){
                        pc.printf("axis3=%.1f   sw=%1d   I3=%.4f stall_state=0x%02X ax3dIdT=%.3f   ", axis3.pos, limit3, axis3.readCurrent(), stall_state, axis3.dIdT);
                        print_motI();
                        pc.printf("\r\n");
                    }
                }while((stall_state==0) && (limit3 == 0));
                if(DEBUG){
                    pc.printf("axis3=%.1f   sw=%1d   I3=%.4f stall_state=0x%02X ax3dIdT=%.3f   ", axis3.pos, limit3, axis3.readCurrent(), stall_state, axis3.dIdT);
                    print_motI();
                    pc.printf("\r\n");
                }                
                
                if(stall_state!=0){
                    if(DEBUG){
                        pc.printf("\r\nCollision detected, returning error -1  ");
                        print_motI();
                        pc.printf("\r\n   MAX = ");
                        print_motMaxI();
                        pc.printf("\r\n");
                    }                
                    return -1;    
                }
                
                //home reached 
                zero_axis(3);
                zero_axis(4);
                zero_axis(5);
                axis3.stat=0;
                return 0; 
            }           
            
        }        
        else{
            if((axis3.pos < -8500) && (limit3 == 1)){
                //Move down until switch closes
                do{            
                    EW_down(120);
                    wait(homeDelayGlobal);
                    if(DEBUG){
                        pc.printf("axis3=%.1f   sw=%1d   I3=%.4f stall_state=0x%02X ax3dIdT=%.3f   ", axis3.pos, limit3, axis3.readCurrent(), stall_state, axis3.dIdT);
                        //print_motI();
                        pc.printf("\r\n");
                    }
                }while((stall_state==0) && (limit3 == 1));
            }
            if(limit3==0){
                //Move down until switch opens 
                do{            
                    EW_down(50);
                    wait(homeDelayGlobal);
                    if(DEBUG){
                        pc.printf("axis3=%.1f   sw=%1d   I3=%.4f stall_state=0x%02X ax3dIdT=%.3f   ", axis3.pos, limit3, axis3.readCurrent(), stall_state, axis3.dIdT);
                        //print_motI();
                        pc.printf("\r\n");
                    }
                }while((stall_state==0) && (limit3 == 0));                
            }
            else{
                pc.printf("Problem with home!!!\r\n return -2\r\n");
                return -2;
            }
            
            if((limit3==1) && (stall_state==0)){
                //home reached 
                zero_axis(3);
                zero_axis(4);
                zero_axis(5);
                axis3.stat=0;                 
                return 0;    
            }
        }
                       
    }

    else{   // limit3 == 0 , was closed when we entered this function
        //Move down until switch opens
        do{            
            EW_down(10);
            wait(homeDelayGlobal);
            if(DEBUG){
                pc.printf("axis3=%.1f   sw=%1d   I3=%.4f stall_state=0x%02X ax3dIdT=%.3f   ", axis3.pos, limit3, axis3.readCurrent(), stall_state, axis3.dIdT);
                print_motI();
                pc.printf("\r\n");
            }
        }while((stall_state==0) && (limit3 == 0));
        
        if(stall_state!=0){
            pc.printf("Collision detected!!\r\n");
            pc.printf("axis3=%.1f   sw=%1d   I3=%.4f stall_state=0x%02X ax3dIdT=%.3f   ", axis3.pos, limit3, axis3.readCurrent(), stall_state, axis3.dIdT);
            zero_axis(3);
            zero_axis(4);
            zero_axis(5);
            pc.printf("Problem with home!!!\r\n return -3\r\n");
            return -3;
        }
        
        if(limit3 == 1){
            zero_axis(3);
            zero_axis(4);
            zero_axis(5);
            axis3.stat=0;    
        
            return 0;            
        }
        
        if(limit3 == 0){
            pc.printf("WTF!!\r\n");
            zero_axis(3);
            zero_axis(4);
            zero_axis(5);
            axis3.stat=0;                
            pc.printf("Problem with home!!!\r\n return -2\r\n");
            return -4;            
        }
    } //else (second half of home elbow)
    
    return 0;
}
    
int home_pitch_wrist(void){            
//pitch wrist 
    all_on();
    reset_stall_state();
    zero_axis(4);
    zero_axis(5);
    
    int wrist_flag=0;
    if(limit4 == 1){
        if(DEBUG)
            pc.printf("Limit switch not closed, move up until collision or 2500 counts or limit switch is pressed\r\n");
            
        reset_stall_state();
        //move up until collision or 1500 counts or limit switch is pressed
        do{
            W_up(50);  
            wait(homeDelayGlobal);
            //why was stall_state not 0 here!!!????
            if(DEBUG)
                pc.printf("axis4=%.1f axis5=%.1f  sw4=%1d  I4=%.4f I5=%.4f stall_state=0x%02X ax4dIdT=%.3f ax5dIdT=%.3f \r\n", axis4.pos, axis5.pos, limit4, axis4.readCurrent(), axis5.readCurrent(),stall_state, axis4.dIdT, axis5.dIdT);
        }while((stall_state==0) && (limit4 == 1) && (axis4.pos > -3000) && (axis5.pos < 3000));
        if(DEBUG)
            pc.printf("axis4=%.1f axis5=%.1f  sw4=%1d  I4=%.4f I5=%.4f stall_state=0x%02X ax4dIdT=%.3f ax5dIdT=%.3f \r\n", axis4.pos, axis5.pos, limit4, axis4.readCurrent(), axis5.readCurrent(),stall_state, axis4.dIdT, axis5.dIdT);
        
        
/*        if((axis4.pos < -3000) || (axis5.pos > 3000)){
            pc.printf("\r\naxis4=%.1f axis5=%.1f  sw=%1d  I4=%.4f I5=%.4f stall_state=0x%02X\r\n", axis4.pos, axis5.pos, limit4, axis4.readCurrent(), axis5.readCurrent(),stall_state);
            pc.printf("3000 count already, should be in the upper region and can go down until switch closes!\r\n");
        }
*/        
        if(stall_state!=0){
            if(DEBUG)
                pc.printf("\r\nmax wrist up reached\r\n", stall_state);  
            W_down(50);                      
            wait(homeDelayGlobal);
        }
        zero_axis(4);
        zero_axis(5);
        reset_stall_state();  //re-zero stall state
        
        if(DEBUG)
            pc.printf("Moving Wrist down until limit switch closes\r\n");
        do{ 
            W_down(40);
            wait(homeDelayGlobal);
            if(DEBUG)
                pc.printf("axis4=%.1f axis5=%.1f  sw4=%1d  I4=%.4f I5=%.4f stall_state=0x%02X ax4dIdT=%.3f ax5dIdT=%.3f \r\n", axis4.pos, axis5.pos, limit4, axis4.readCurrent(), axis5.readCurrent(),stall_state, axis4.dIdT, axis5.dIdT);
        }while((stall_state==0) && (limit4 == 1));
        if(DEBUG)
            pc.printf("axis4=%.1f axis5=%.1f  sw4=%1d  I4=%.4f I5=%.4f stall_state=0x%02X ax4dIdT=%.3f ax5dIdT=%.3f \r\n", axis4.pos, axis5.pos, limit4, axis4.readCurrent(), axis5.readCurrent(),stall_state, axis4.dIdT, axis5.dIdT);
        
        if(stall_state!=0){
            if(DEBUG)
                pc.printf("Collision detected before wrist switch press, return error -1 with stall state=0x%02X\r\n", stall_state);
            axis4.axisOff();            
            axis5.axisOff();
            return -1;    
        }
        
        if(DEBUG)
            pc.printf("Moving Wrist down until limit switch opens again\r\n");
        do{ 
            W_down(40);
            wait(homeDelayGlobal);
            if(DEBUG)
                pc.printf("axis4=%.1f axis5=%.1f  sw4=%1d  I4=%.4f I5=%.4f stall_state=0x%02X ax4dIdT=%.3f ax5dIdT=%.3f \r\n", axis4.pos, axis5.pos, limit4, axis4.readCurrent(), axis5.readCurrent(),stall_state, axis4.dIdT, axis5.dIdT);
        }while((stall_state==0) && (limit4 == 0));
        
        //was a collision while 
        if(stall_state!=0){
            if(DEBUG)
                pc.printf("Collision detected before wrist switch press, return error -2 with stall state=0x%02X\r\n", stall_state);
            axis4.axisOff();            
            axis5.axisOff();
            return -2;    
        }
        
        if(limit4 == 1){
            if(DEBUG)
                pc.printf("Wrist pitch homed! First If condition!\r\n\r\n");
            zero_axis(4);
            zero_axis(5);                
            wrist_flag=1;   
        }
    }

    else{ //limit4 == 0
        reset_stall_state();  //re-zero stall state
        
        do{ 
            W_down(30);
            wait(homeDelayGlobal);
            if(DEBUG)
                pc.printf("axis4=%.1f axis5=%.1f  sw4=%1d  I4=%.4f I5=%.4f stall_state=0x%02X ax4dIdT=%.3f ax5dIdT=%.3f \r\n", axis4.pos, axis5.pos, limit4, axis4.readCurrent(), axis5.readCurrent(),stall_state, axis4.dIdT, axis5.dIdT);
        }while((stall_state==0) && (limit4 == 0));

        if(stall_state!=0){
            pc.printf("stall detected while switch was supposed to be closed!!\r\n");
            pc.printf("return -5\r\n");
            return -5;            
        }
        
        if(limit4 == 1){            
            if(DEBUG)
                pc.printf("Wrist pitch homed! Second IF condition\r\n\r\n");            
            zero_axis(4);
            zero_axis(5);
                        
            wrist_flag=1;
        }        
    }    
    
    if(wrist_flag==1){
        W_down(120);
        //W_down(450);
        wait(.2);
        
        reset_stall_state();  //re-zero stall state
        
        do{ 
            W_up(10);
            wait(homeDelayGlobal);
            if(DEBUG)
                pc.printf("axis4=%.1f axis5=%.1f  sw4=%1d  I4=%.4f I5=%.4f stall_state=0x%02X ax4dIdT=%.3f ax5dIdT=%.3f \r\n", axis4.pos, axis5.pos, limit4, axis4.readCurrent(), axis5.readCurrent(),stall_state, axis4.dIdT, axis5.dIdT);
        }while((stall_state==0) && (limit4 == 1));

        if(stall_state!=0){
            pc.printf("stall detected while switch was supposed to be open moving up to close!!\r\n");
            pc.printf("return -6\r\n");
            return -6;            
        }
        
        if(limit4 == 0){            
            if(DEBUG)
                pc.printf("Wrist pitch homed!\r\n\r\n");            
            zero_axis(4);
            zero_axis(5);
            return 0; 
        }                   
    }

    return -1;
}

int home_rotate_wrist(void){
    zero_axis(4);
    zero_axis(5);
    all_on();
    reset_stall_state();
        
    if(limit5==1){
        if(DEBUG)
            pc.printf("Rotate CCW (looking at) until limit is pressed\r\n");
        do{
            W_roll(100);
            wait(homeDelayGlobal);
            if(DEBUG)
                pc.printf("axis4=%.1f axis5=%.1f  sw5=%1d  I4=%.4f I5=%.4f stall_state=0x%02X ax4dIdT=%.3f ax5dIdT=%.3f  \r\n", axis4.pos, axis5.pos, limit5, axis4.readCurrent(), axis5.readCurrent(),stall_state, axis4.dIdT, axis5.dIdT);
        }while((stall_state==0) && (limit5==1));
        if(DEBUG)
            pc.printf("axis4=%.1f axis5=%.1f  sw5=%1d  I4=%.4f I5=%.4f stall_state=0x%02X ax4dIdT=%.3f ax5dIdT=%.3f  \r\n", axis4.pos, axis5.pos, limit5, axis4.readCurrent(), axis5.readCurrent(),stall_state, axis4.dIdT, axis5.dIdT);
        
        if(stall_state != 0){
            if(DEBUG)
                pc.printf("Collision detected, abort rotation home, error -1 stall_state=0x%02X\r\n", stall_state);
            return -1;
        }
        
        if(limit5 == 0){
            do{
                W_roll(40);
                wait(homeDelayGlobal);
                if(DEBUG)
                    pc.printf("axis4=%.1f axis5=%.1f  sw5=%1d  I4=%.4f I5=%.4f stall_state=0x%02X ax4dIdT=%.3f ax5dIdT=%.3f  \r\n", axis4.pos, axis5.pos, limit5, axis4.readCurrent(), axis5.readCurrent(),stall_state, axis4.dIdT, axis5.dIdT);
            }while((stall_state==0) && (limit5==0));
                
            if(stall_state!=0){
                if(DEBUG){                
                    pc.printf("axis4=%.1f axis5=%.1f  sw5=%1d  I4=%.4f I5=%.4f stall_state=0x%02X ax4dIdT=%.3f ax5dIdT=%.3f  \r\n", axis4.pos, axis5.pos, limit5, axis4.readCurrent(), axis5.readCurrent(),stall_state, axis4.dIdT, axis5.dIdT);
                    pc.printf("Stall state reached, exiting with -2\r\n");
                }
                return -2;
            }
            
            if((stall_state ==0) && (limit5==1)){
                if(DEBUG)
                    pc.printf("Wrist Rotate Home success!! \r\n");
                //May need to adjust this
                W_roll(WRIST_SWITCH_OFFSET);
                wait(1.1);   //20190219 changed to .8 from 1.1
                zero_axis(4);
                zero_axis(5);                
                return 0;
            }
        }
    }
    else{   // limit5 == 0
        do{
            W_roll(10);
            wait(homeDelayGlobal);
            if(DEBUG)
                pc.printf("axis4=%.1f axis5=%.1f  sw5=%1d  I4=%.4f I5=%.4f stall_state=0x%02X ax4dIdT=%.3f ax5dIdT=%.3f  \r\n", axis4.pos, axis5.pos, limit5, axis4.readCurrent(), axis5.readCurrent(),stall_state, axis4.dIdT, axis5.dIdT);
        }while((stall_state==0) && (limit5==0));
            
        if(stall_state!=0){
            if(DEBUG)
                pc.printf("axis4=%.1f axis5=%.1f  sw5=%1d  I4=%.4f I5=%.4f stall_state=0x%02X ax4dIdT=%.3f ax5dIdT=%.3f  \r\n", axis4.pos, axis5.pos, limit5, axis4.readCurrent(), axis5.readCurrent(),stall_state, axis4.dIdT, axis5.dIdT);
            return -2;
        }
        
        if((stall_state ==0) && (limit5==1)){
            if(DEBUG)
                pc.printf("Wrist Rotate Home success!! \r\n");
            //May need to adjust this
            W_roll(WRIST_SWITCH_OFFSET);
            wait(1.2);      //20190219 changed to .8 from 1.2
            zero_axis(4);
            zero_axis(5);            
            return 0;
        }
    }    
    
    return 0;       //successful home of wrist rotate            
}

// TODO - 20190329 J. Bradshaw
// ADD GRIPPER TIMEOUT IN CASE CURRENT DETECT IS NOT CALIBRATED CORRECTLY

int home_gripper(void){
    //axis6.axisOn();
    
    GRIPPER_COL_DETECT=1;   //On for home
    all_on();
    reset_stall_state();
    //zero_axis(6);    
    G_open(200);
    wait(.4);
    do{
        G_open(100);
        wait(homeDelayGlobal * .65);
        if(DEBUG)
            pc.printf("position = %.1f  stall_state=0x%02X   I=%7.2f dIdT=%.4f\r\n", axis6.pos, stall_state, axis6.motI, axis6.dIdT);
    }while((stall_state & 0x20) != 0x20);//while(axis6.motI <= GRIPPER_STALL_I);   // > 1.0
    wait(.05);
    
    //20170725 mess with this to stop the closing gripper from stopping early!!
    G_close(300);   //was 200 20190315
    
    wait(.15);
    reset_stall_state();
    //close until current drops
    do{
        G_close(50);
        wait(homeDelayGlobal);
        if(DEBUG)
            pc.printf("position = %.1f  stall_state=0x%02X   I=%7.2f dIdT=%.4f\r\n", axis6.pos, stall_state, axis6.motI, axis6.dIdT);
    }while(axis6.motI >= .02);   //stall state ==0
    G_close(50);
    
    wait(.1);
    
    if(DEBUG)
        pc.printf("position = %.1f  stall_state=0x%02X   I=%7.2f dIdT=%.4f\r\n", axis6.pos, stall_state, axis6.motI, axis6.dIdT);
        
    zero_axis(6);    
    reset_stall_state();
    
    //close until less then -3200
    do{
        G_close(100);
        wait(homeDelayGlobal);
        if(DEBUG)
            pc.printf("position = %.1f  stall_state=0x%02X   I=%7.2f dIdT=%.4f\r\n", axis6.pos, stall_state, axis6.motI, axis6.dIdT);
    }while(axis6.pos >= -3200.0);   //close while disregarding current
        
    reset_stall_state();
    
    do{
        G_close(50);    //was 100
        wait(homeDelayGlobal * .65);
        if(DEBUG)
            pc.printf("position = %.1f  stall_state=0x%02X   I=%7.2f dIdT=%.4f\r\n", axis6.pos, stall_state, axis6.motI, axis6.dIdT);
    }while(axis6.motI <= GRIPPER_STALL_I);  //1.1
    wait(.05);
    reset_stall_state();
    //open until current drops
    do{
        G_open(50);
        wait(homeDelayGlobal);
        if(DEBUG)
            pc.printf("position = %.1f  stall_state=0x%02X   I=%7.2f dIdT=%.4f\r\n", axis6.pos, stall_state, axis6.motI, axis6.dIdT);
    }while(axis6.motI >= .02);   //stall state ==0
    G_open(GRIPPER_BACKOFF); //was 80 20190315, before that was 50
    wait(.05);
    reset_stall_state();
    
    if((abs(axis6.pos) < 4500)){
        zero_axis(6);
        wait(.1);
        if(DEBUG)
            pc.printf("\r\nFull throw not reached, return -1\r\n");
            
        axis6.axisOff();
        return -1;        
    }

    float high_counts = abs(axis6.pos);
    if(DEBUG)
        pc.printf("position = %.1f\r\n", high_counts);
    
    while((int)axis6.pos != 0){
        if(DEBUG)
            pc.printf("Zeroing axis 6\r\n");
        zero_axis(6);      
        wait(.02);
    }    
    reset_stall_state();
    axis6.p_lower = 0;
    axis6.p_higher = high_counts;
    if(DEBUG)
        pc.printf("Gripper max throw is %.1f\r\n", axis6.p_higher);        

    axis6.stat=0;   //Zero the axis 6 status as homed
    GRIPPER_COL_DETECT=0;   //Turn off gripper collision current detetc
    return 0;
}

int home_master(void){
    int error;
    reset_stall_state();    //reset the stall state
    if(DEBUG)
        pc.printf("Homing\r\n");
    zero_all();
    all_on();
 
//    clear_motMaxI();
    //first close gripper
    do{
        if(DEBUG){
            pc.printf("axis6=%.1f  I6=%.4f dIdt=%.4f stall_state=0x%02X  ", axis6.pos, axis6.readCurrent(), axis6.dIdT, stall_state);
            print_motI();
            pc.printf("\r\n");
        }
        G_close(100);
        wait(homeDelayGlobal);
    }while(axis6.motI <= GRIPPER_STALL_I);

    reset_stall_state();    //re-zero stall_state
    //then re-open the  gripper
    do{
        if(DEBUG){
            pc.printf("axis6=%.1f  I6=%.4f dIdt=%.4f stall_state=0x%02X  ", axis6.pos, axis6.readCurrent(), axis6.dIdT, stall_state);
            print_motI();
            pc.printf("\r\n");
        }
        G_open(100);
        wait(homeDelayGlobal);
    }while(axis6.motI >= .2);   //wait for the current to fall below
    zero_axis(6);
    reset_stall_state();    //re-zero stall_state
        
//    clear_motMaxI();
    if(DEBUG)
        pc.printf("Home Shoulder\r\n");
    error = home_shoulder();
    if(error==0){
        if(DEBUG)
            pc.printf("Home Shoulder success\r\n");
        axis2.stat = 0;
    }
    else{
        if(DEBUG)
            pc.printf("Home Shoulder fail, %d\r\n", error);
        axis2.stat = 1;
        print_motMaxI();
        return error;
    }   

//    clear_motMaxI();
    if(DEBUG)
        pc.printf("Home Elbow\r\n");    //also homes wrist if upper collision is detected
    error = home_elbow();
    if(error==0){
        if(DEBUG)
            pc.printf("Home Elbow success\r\n");
        axis3.stat = 0;
    }
    else{
        if(DEBUG)
            pc.printf("Home Elbow fail, %d\r\n", error);
        axis3.stat = 1;
        print_motMaxI();
        return error;
    }

    reset_stall_state();  //re-zero stall state            
    
//    clear_motMaxI();
    if(DEBUG)
        pc.printf("Home Wrist Pitch\r\n");
    error = home_pitch_wrist();
    if(error==0){
        if(DEBUG)
            pc.printf("Home Wrist Pitch success\r\n");
        //zero_axis(4); 
        //zero_axis(5);
        axis4.stat = 0;
        axis5.stat = 0;
    }
    else{
        if(DEBUG)
            pc.printf("Home Wrist Pitch fail, %d\r\n", error);
        axis4.stat = 1;
        axis5.stat = 1;
        print_motMaxI();
        return error;
    }    
    
    //zero_axis(4);
    //zero_axis(5);    
    reset_stall_state();  //re-zero stall state       
//    clear_motMaxI();
    
    if(DEBUG)
        pc.printf("Home Wrist Rotation\r\n");
    error = home_rotate_wrist();
    if(error==0){
        if(DEBUG)
            pc.printf("Home Wrist Rotation success\r\n");
        //zero_axis(4); 
        //zero_axis(5);
        axis4.stat = 0;
        axis5.stat = 0;
    }
    else{
        if(DEBUG)
            pc.printf("Home Wrist Rotation fail, %d\r\n", error);
        axis4.stat = 1;
        axis5.stat = 1;
        print_motMaxI();
        return error;
    }
    
    reset_stall_state();  //re-zero stall state            
    
//    clear_motMaxI();
    if(DEBUG)
        pc.printf("Home Wrist Pitch\r\n");
    error = home_pitch_wrist();
    if(error==0){
        if(DEBUG)
            pc.printf("Home Wrist Pitch success\r\n");
        //zero_axis(4); 
        //zero_axis(5);
        axis4.stat = 0;
        axis5.stat = 0;
    }
    else{
        if(DEBUG)
            pc.printf("Home Wrist Pitch fail, %d\r\n", error);
        axis4.stat = 1;
        axis5.stat = 1;
        print_motMaxI();
        return error;
    } 
        
//    clear_motMaxI();
    if(DEBUG)
        pc.printf("Home Base Rotation\r\n");    
    error = home_base();
    if(error==0){
        if(DEBUG)
            pc.printf("Home base success\r\n");
        zero_axis(1);
        axis1.stat = 0;
    }
    else{
        if(DEBUG)
            pc.printf("Home base fail, %d\r\n", error);
        axis1.stat =1;
        print_motMaxI();
        return error;
    }

//    clear_motMaxI();
    if(DEBUG)
        pc.printf("Home Wrist Pitch\r\n");
    error = home_pitch_wrist();
    if(error==0){
        if(DEBUG)
            pc.printf("Home Wrist Pitch success\r\n");
        //zero_axis(4); 
        //zero_axis(5);
        axis4.stat = 0;
        axis5.stat = 0;
    }
    else{
        if(DEBUG)
            pc.printf("Home Wrist fail, %d\r\n", error);
        axis4.stat = 1;
        axis5.stat = 1;
        print_motMaxI();
        return error;
    }    
    
    //zero_axis(4);
    //zero_axis(5);    
    reset_stall_state();  //re-zero stall state       
//    clear_motMaxI();
    
    if(DEBUG)
        pc.printf("Home Wrist Rotation\r\n");
    error = home_rotate_wrist();
    if(error==0){
        if(DEBUG)
            pc.printf("Home Wrist Rotation success\r\n");
        //zero_axis(4); 
        //zero_axis(5);
        axis4.stat = 0;
        axis5.stat = 0;
    }
    else{
        if(DEBUG)
            pc.printf("Home Wrist Rotation fail, %d\r\n", error);
        axis4.stat = 1;
        axis5.stat = 1;
        print_motMaxI();
        return error;
    }
    
    reset_stall_state();  //re-zero stall state     
//    clear_motMaxI();
    if(DEBUG)
        pc.printf("Home Gripper\r\n");
    error = home_gripper();
    if(error==0){
        if(DEBUG)
            pc.printf("Home gripper success\r\n");
        zero_axis(6);
        axis6.stat = 0;
    }
    else{
        if(DEBUG)
            pc.printf("Home gripper fail, %d\r\n", error);
        axis6.axisOff();
        axis6.stat = 1;
        print_motMaxI();
        return error;
    }
            
    if(DEBUG)
        pc.printf("\r\nEnd of Master Home\r\n");

    wait(.2);

//    clear_motMaxI();        
    //Write encoder values at zero position for all axes
    
//    pc.printf("\r\nAxis2 at %f\r\n", axis2.pos);
//    wait(.2);
    
    axis2.axisOff();                                             
    axis2.writeEncoderValue((long)SHOULDER_SWITCH_OFFSET_COUNTS);     //120.28 deg is 0 counts, switch closes at 0
    //axis2.pid->setInputLimits(-AXIS1_SPAN_POS_RAD, AXIS1_SPAN_POS_RAD);    
    axis2.set_point = (long)SHOULDER_SWITCH_OFFSET_COUNTS;
    axis2.pid->reset();
    //pc.printf("\r\ntempPos = %.4f\r\n", axis2.set_point);
    wait(.01);
    axis2.axisOn();                
                            
    //axis2.writeEncoderValue((long)SHOULDER_SWITCH_OFFSET_COUNTS);     //120.28 deg is 0 counts, switch closes at 0
      
//    pc.printf("\r\nAxis2 now at %f\r\n", axis2.pos);
//    wait(.2);
//    pc.printf("\r\nAxis3 at %f\r\n", axis3.pos);
//    wait(.2);   


    axis3.axisOff();                                         
    axis3.writeEncoderValue((long)ELBOW_SWITCH_OFFSET_COUNTS);     ////95.02 deg is 10735.2 counts
    axis3.set_point = (long)ELBOW_SWITCH_OFFSET_COUNTS;
    wait(.01);
    //pc.printf("\r\ntempPos = %.4f\r\n", axis3.set_point);
    axis3.pid->reset();
    axis3.axisOn();
    
    axis4.writeEncoderValue((long)(88.81 * MOT_4_5_CPD));        //pitch is -88.81 deg at home
    axis5.writeEncoderValue((long)(-88.81 * MOT_4_5_CPD));
    pc.printf("HOMED\r\n");
    return 0;
}

/*
void serialOutput(void){
    if((axis1.moveState==0) && (axis2.moveState==0) && (axis3.moveState==0) && (axis4.moveState==0) && (axis5.moveState==0) && (axis6.moveState==0)){
        //pc.printf("%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f   \r", (float)t.read(), (float)axis1.set_point, (float)axis1.pos, (float)axis2.set_point, (float)axis2.pos,
        //    (float)axis3.set_point, (float)axis3.pos, (float)axis4.set_point, (float)axis4.pos,(float)axis5.set_point, (float)axis5.pos, (float)axis6.set_point, (float)axis6.pos);                
            
//                pc.printf("T:%02d:%02d:%02d:%02d:%03d A1:%5.1f %5.1f  A2:%5.1f %5.1f  A3:%5.1f %5.1f  A4:%5.1f %5.1f  A5:%5.1f %5.1f  A6:%5.1f %5.1f  \r", runTime.day, runTime.hour, runTime.min, runTime.sec, runTime.ms, 
//                (float)axis1.set_point, (float)axis1.pos, (float)axis2.set_point, (float)axis2.pos, (float)axis3.set_point, (float)axis3.pos, (float)axis4.set_point, (float)axis4.pos,(float)axis5.set_point, 
//                (float)axis5.pos, (float)axis6.set_point, (float)axis6.pos); 
            
            //pc.printf("T%02d:%02d:%02d:%02d:%03d", runTime.day, runTime.hour, runTime.min, runTime.sec, runTime.ms);                
            
            pc.printf("T%.2f", runTime.ms_total*.001);
            pc.printf("P%.1f,%.1f,%.1f,%.1f,%.1f,%.1f", (float)axis1.pos,(float)axis2.pos,(float)axis3.pos,(float)axis4.pos,(float)axis5.pos,(float)axis6.pos);
            pc.printf("V%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", (float)axis1.vel,(float)axis2.vel,(float)axis3.vel,(float)axis4.vel,(float)axis5.vel,(float)axis6.vel);
            pc.printf("S%d,%d,%d,%d,%d,%d", axis1.stat, axis2.stat, axis3.stat, axis4.stat, axis5.stat, axis6.stat);
            pc.printf("           \r");           
                       
            if(streamFlag)
                pc.printf("\n");
            led2 = !led2;
    }    
}
*/

int CMD_read(char *str, int numchars, float timeout){        
    Timer t_Frame;
    int i=0;        
    int timeoutState=0;
    float timeoutmax=0;
    t_Frame.start();
    
    while((timeoutState != 3) && (i < numchars)){
        led2 = 1;
        switch(timeoutState){
            case 0:
                while(pc.readable()){ //if characters are in buffer, read them
                    str[i++] = pc.getc();
                    if(i == numchars){
                        timeoutState = 3;
                        break;   
                    }
                }
                //if no characters in buffer, initiate timeout
                timeoutState = 1;
                led2 = 0;
                break;
            case 1:
                timeoutmax = t_Frame.read() + timeout;    //current time plus timeout time
                timeoutState = 2;
//              pc.printf("Timeout initiated %f\r\n", timeout);
                led2 = 0;
                break;
            case 2:
                if(pc.readable()){ //check buffer while timeout is running
                    str[i++] = pc.getc();
                    if(i == numchars){
                        timeoutState = 3;
                    }
                    else{
                        timeoutState = 0;
                    }
                    led2 = 0;
                    break;
                }
                if(t_Frame.read() >= timeoutmax) //if timeout has elapsed, exit the while loop with state 3
                    timeoutState = 3;
                led2 = 0;
                break;
            default:
                timeoutState = 0;
        }//switch timeoutState                             
    }//while timeoutState != 2
    return i;   //return number of bytes read
}

void led_blink_sequence(void){
    led1=LED_OFF;
    led2=LED_OFF;
    led3=LED_OFF;
    
    led1=LED_ON;
    wait(.1);
    led1=LED_OFF;
    wait(.2);
    led2=LED_ON;
    wait(.1);
    led2=LED_OFF;
    wait(.2);
    led3=LED_ON;
    wait(.1);
    led3=LED_OFF;
    wait(.2);
}

void panic_flash(int times){
    led1=LED_OFF;
    led2=LED_OFF;
    led3=LED_OFF;
    
    for(int i=0;i < times;i++){
        led1=LED_ON;
        led2=LED_ON;
        led3=LED_ON;
        wait(.1);
        led1=LED_OFF;
        led2=LED_OFF;
        led3=LED_OFF;
        wait(.1);
    }
}

void test_all_motors(void){
    int error_state=0;
    float stepsCounts=400.0;
    //Test Motors for correct closed loop control
    zero_all();       //zero the axis
    
    all_on();     //turn on the axes       
    
    axis1.set_point = stepsCounts;  //move to 500 steps
    axis2.set_point = stepsCounts;  //move to 500 steps
    axis3.set_point = stepsCounts;  //move to 500 steps
    axis4.set_point = stepsCounts;  //move to 500 steps
    axis5.set_point = stepsCounts;  //move to 500 steps
    axis6.set_point = stepsCounts;  //move to 500 steps
    pc.printf("all axes +%.3f counts\r\n",stepsCounts);
    
    wait(.3);               //delay
    pc.printf("pos1=%.3f pos2=%.3f pos3=%.3f pos4=%.3f pos5=%.3f pos6=%.3f  \r\n", axis1.pos,axis2.pos,axis3.pos,axis4.pos,axis5.pos,axis6.pos);                
    pc.printf("I1=%.3f I1=%.3f I1=%.3f I1=%.3f I1=%.3f I1=%.3f\r\n", axis1.readCurrent(),axis2.readCurrent(),axis3.readCurrent(),axis4.readCurrent(),axis5.readCurrent(),axis6.readCurrent());
    all_off();;     //turn all the axes off
    if(axis1.pos < 100){
        pc.printf("Failed Mot 1 positive closed loop test. Is motor 1 backwards?\r\n");
        error_state |= 0x01;
    }
    if(axis2.pos < 100){
        pc.printf("Failed Mot 2 positive closed loop test. Is motor 2 backwards?\r\n");
        error_state |= 0x02;
    }
    if(axis3.pos < 100){
        pc.printf("Failed Mot 3 positive closed loop test. Is motor 3 backwards?\r\n");
        error_state |= 0x04;
    }
    if(axis4.pos < 100){
        pc.printf("Failed Mot 4 positive closed loop test. Is motor 4 backwards?\r\n");
        error_state |= 0x08;
    }
    if(axis5.pos < 100){
        pc.printf("Failed Mot 5 positive closed loop test. Is motor 5 backwards?\r\n");
        error_state |= 0x10;
    }
    if(axis6.pos < 100){
        pc.printf("Failed Mot 6 positive closed loop test. Is motor 6 backwards?\r\n");
        error_state |= 0x20;
    }                                                                
    wait(.2);
    
    //Negative movement test
    zero_all();       //zero the axis
    
    all_on();     //turn on the axes       
    
    axis1.set_point = -stepsCounts;  //move to 500 steps
    axis2.set_point = -stepsCounts;  //move to 500 steps
    axis3.set_point = -stepsCounts;  //move to 500 steps
    axis4.set_point = -stepsCounts;  //move to 500 steps
    axis5.set_point = -stepsCounts;  //move to 500 steps
    axis6.set_point = -stepsCounts;  //move to 500 steps
    pc.printf("all axes -%.3f counts\r\n", stepsCounts);
    wait(.3);               //delay
    pc.printf("pos1=%.3f pos2=%.3f pos3=%.3f pos4=%.3f pos5=%.3f pos6=%.3f  \r\n", axis1.pos,axis2.pos,axis3.pos,axis4.pos,axis5.pos,axis6.pos);                
    pc.printf("I1=%.3f I1=%.3f I1=%.3f I1=%.3f I1=%.3f I1=%.3f\r\n", axis1.readCurrent(),axis2.readCurrent(),axis3.readCurrent(),axis4.readCurrent(),axis5.readCurrent(),axis6.readCurrent());
    all_off();;     //turn all the axes off
    if(axis1.pos > -100){
        pc.printf("Failed Mot 1 negative closed loop test. Is motor 1 backwards?\r\n");
        if((error_state &= 0x01) == 0x01){  //mot 1 positive check failed as well
            pc.printf("Mot 1 has failed both checks!\r\n");
            wait(5);
        }                    
    }
    else{
        if((error_state &= 0x01) == 0x01){
            error_state &= 0xFE;    //clear bit 0
            pc.printf("Passed Mot 1 negative closed loop test. May have been stalled during positive test?\r\n");                        
        }                    
    }
    if(axis2.pos > -100){
        pc.printf("Failed Mot 2 negative closed loop test. Is motor 2 backwards?\r\n");
        if((error_state &= 0x02) == 0x02){  //mot 2 positive check failed as well
            pc.printf("Mot 2 has failed both checks!\r\n");
            wait(5);
        } 
    }
    else{
        if((error_state &= 0x02) == 0x02){
            error_state &= 0xFD;    //clear bit 1
            pc.printf("Passed Mot 2 negative closed loop test. May have been stalled during positive test?\r\n");                        
        }                    
    }                
    if(axis3.pos > -100){
        pc.printf("Failed Mot 3 negative closed loop test. Is motor 3 backwards?\r\n");
        if((error_state &= 0x04) == 0x04){  //mot 3 positive check failed as well
            pc.printf("Mot 3 has failed both checks!\r\n");
            wait(5);
        } 
    }
    else{
        if((error_state &= 0x04) == 0x04){
            error_state &= 0xFB;    //clear bit 2
            pc.printf("Passed Mot 3 negative closed loop test. May have been stalled during positive test?\r\n");                        
        }                    
    }                
    if(axis4.pos > -100){
        pc.printf("Failed Mot 4 negative closed loop test. Is motor 4 backwards?\r\n");
        if((error_state &= 0x08) == 0x08){  //mot 4 positive check failed as well
            pc.printf("Mot 4 has failed both checks!\r\n");
            wait(5);
        } 
    }
    else{
        if((error_state &= 0x08) == 0x08){
            error_state &= 0xF7;    //clear bit 3
            pc.printf("Passed Mot 4 negative closed loop test. May have been stalled during positive test?\r\n");                        
        }                    
    }                
    if(axis5.pos > -100){
        pc.printf("Failed Mot 5 negative closed loop test. Is motor 5 backwards?\r\n");
        if((error_state &= 0x10) == 0x10){  //mot 5 positive check failed as well
            pc.printf("Mot 5 has failed both checks!\r\n");
            wait(5);
        } 
    }
    else{
        if((error_state &= 0x10) == 0x10){
            error_state &= 0xEF;    //clear bit 4
            pc.printf("Passed Mot 5 negative closed loop test. May have been stalled during positive test?\r\n");                        
        }                    
    }                
    if(axis6.pos > -100){
        pc.printf("Failed Mot 6 negative closed loop test. Is motor 6 backwards?\r\n");
        if((error_state &= 0x20) == 0x20){  //mot 6 positive check failed as well
            pc.printf("Mot 6 has failed both checks!\r\n");
            wait(5);
        } 
    }
    else{
        if((error_state &= 0x20) == 0x20){
            error_state &= 0xDF;    //clear bit 5
            pc.printf("Passed Mot 6 negative closed loop test. May have been stalled during positive test?\r\n");                        
        }                    
    }                
    all_off();
    zero_all();       //zero the axis                                        
    wait(.2);
    
    if((error_state & 0xFF) != 0x00){
        pc.printf("Error_state = 0x%04X, exiting program!\r\n", error_state);   
    }
}

float read_Pk(char axis){
  float ret_val;
  switch(axis){
        case '1':
            ret_val = axis1.Pk;
            break;
        case '2':
            ret_val = axis2.Pk;
            break;
        case '3':
            ret_val = axis3.Pk;
            break;
        case '4':
            ret_val = axis4.Pk;
            break;
        case '5':
            ret_val = axis5.Pk;
            break;
        case '6':
            ret_val = axis6.Pk;
            break;
    }
    return ret_val;
}

float read_Ik(char axis){
  float ret_val;
  switch(axis){
        case '1':
            ret_val = axis1.Ik;
            break;
        case '2':
            ret_val = axis2.Ik;
            break;
        case '3':
            ret_val = axis3.Ik;
            break;
        case '4':
            ret_val = axis4.Ik;
            break;
        case '5':
            ret_val = axis5.Ik;
            break;
        case '6':
            ret_val = axis6.Ik;
            break;
    }
    return ret_val;
}


float read_Dk(char axis){
  float ret_val;
  switch(axis){
        case '1':
            ret_val = axis1.Dk;
            break;
        case '2':
            ret_val = axis2.Dk;
            break;
        case '3':
            ret_val = axis3.Dk;
            break;
        case '4':
            ret_val = axis4.Dk;
            break;
        case '5':
            ret_val = axis5.Dk;
            break;
        case '6':
            ret_val = axis6.Dk;
            break;
    }
    return ret_val;
}

//returns the OR of all axis status (1=axis error somewhere, 0=all go)
int check_stats(void){
    return axis1.stat|axis2.stat|axis3.stat|axis4.stat|axis5.stat|axis6.stat;   
}

void clear_motMaxI(void){
    default_eprom_motI();
    read_eprom_motI();
    axis1.mot_I_max = 0.0;
    axis2.mot_I_max = 0.0;
    axis3.mot_I_max = 0.0;
    axis4.mot_I_max = 0.0;
    axis5.mot_I_max = 0.0;
    axis6.mot_I_max = 0.0;
}

void print_motMaxI(void){
    pc.printf("Max I m1=%7.3f m2=%7.3f m3=%7.3f m4=%7.3f m5=%7.3f m6=%7.3f\r\n", axis1.mot_I_max,axis2.mot_I_max,
        axis3.mot_I_max,axis4.mot_I_max,axis5.mot_I_max,axis6.mot_I_max);
}

void print_motI(void){
    pc.printf("Mot I m1=%7.3f m2=%7.3f m3=%7.3f m4=%7.3f m5=%7.3f m6=%7.3f", axis1.motI,axis2.motI,
        axis3.motI,axis4.motI,axis5.motI,axis6.motI);
}

int homed(void){
    if((axis1.stat==0) && (axis2.stat==0) && (axis3.stat==0) && 
        (axis4.stat==0) && (axis5.stat==0) && (axis6.stat==0)){
        return 1;
    }
    else
        return 0;
}

int axisMoving(void){     
    int axisMoving=0;

    if(axis1.moveState!=0)
        axisMoving |= 0x01;
    if(axis2.moveState!=0)
        axisMoving |= 0x02;
    if(axis3.moveState!=0)
        axisMoving |= 0x04;
    if(axis4.moveState!=0)
        axisMoving |= 0x08;
    if(axis5.moveState!=0)
        axisMoving |= 0x10;
    if(axis6.moveState!=0)
        axisMoving |= 0x20;
    
    return axisMoving;
}
//update encoder positions with theta degree position commands
int theta2enc(float theta0, float theta1, float theta2, float theta3, float theta4){
    //float k[5] = {141.222, -112.978, 112.978, 27.8981, -27.8981};
    float k[5] = {MOT_1_CPD, -MOT_2_3_CPD, MOT_2_3_CPD, MOT_4_5_CPD, -MOT_4_5_CPD};
    float q[5];
    
    //make sure the Scorbot is homed first
    if(homed()){            
        theta3 = -theta3;        
        
        //If currently in radians, convert to degrees for state equations below
        if(dev_mode_Global == 'r'){
            theta0 *= 180.0/PI;
            theta1 *= 180.0/PI;
            theta2 *= 180.0/PI;
            theta3 *= 180.0/PI;
            theta4 *= 180.0/PI;
        }    
        
        q[0] = k[0] * theta0;
        q[1] = k[1] * (120.28 - theta1);
        q[2] = k[2] * (95.02 - (theta2+95.02)) - q[1];
        q[3] = k[3] * (theta3 - 88.81) + WRIST_ELBOW_OFFSET_RATIO*q[2] + k[4]*theta4;    
        q[4] = k[4] * theta4*2.0 - q[3];
        
        if(DEBUG)
            pc.printf("\r\nq0=%5.2f q1=%5.2f q2=%5.2f q3=%5.2f q4=%5.2f \r\n", q[0],q[1],q[2],q[3],q[4]);
        
        axis1.set_point = q[0];
        axis2.set_point = q[1];
        axis3.set_point = q[2];
        axis4.set_point = q[3];
        axis5.set_point = q[4];
        
        return 0;
    }
    else{
        pc.printf("NAK\r\n");        
        if(DEBUG)
            pc.printf("\r\nScorbot must be Homed first!!\r\n");
        return 1;
    }
}

//update encoder positions with theta degree position commands using trapazoidal profile movemeont
int theta2enc_dt(float dt, float theta0, float theta1, float theta2, float theta3, float theta4, float grip){
    //float k[5] = {141.222, -112.978, 112.978, 27.8981, -27.8981};
    float k[5] = {MOT_1_CPD, -MOT_2_3_CPD, MOT_2_3_CPD, MOT_4_5_CPD, -MOT_4_5_CPD};
    float q[5];
    
    //make sure the Scorbot is homed first
    if(homed()){                
        theta3 = -theta3;
        
        //If currently in radians, convert to degrees for state equations below
        if(dev_mode_Global == 'r'){
            theta0 *= 180.0/PI;
            theta1 *= 180.0/PI;
            theta2 *= 180.0/PI;
            theta3 *= 180.0/PI;
            theta4 *= 180.0/PI;
        }
        
        q[0] = k[0] * theta0;
        q[1] = k[1] * (120.28 - theta1);
        q[2] = k[2] * (95.02 - (theta2+95.02)) - q[1];
        q[3] = k[3] * (theta3 - 88.81) + WRIST_ELBOW_OFFSET_RATIO*q[2] + k[4]*theta4;    
        q[4] = k[4] * theta4*2.0 - q[3];        
        
        //if(DEBUG)
        //    pc.printf("\r\nq0=%5.2f q1=%5.2f q2=%5.2f q3=%5.2f q4=%5.2f \r\n", q[0],q[1],q[2],q[3],q[4]);
            
        axis1.moveTrapezoid(q[0], dt);
        axis2.moveTrapezoid(q[1], dt);
        axis3.moveTrapezoid(q[2], dt);
        axis4.moveTrapezoid(q[3], dt);
        axis5.moveTrapezoid(q[4], dt);
        axis6.moveTrapezoid(sbGripper.mm_to_counts(grip), dt);
            
        return 0;
    }
    else{
        pc.printf("NAK\r\n");
        if(DEBUG)
            pc.printf("\r\nScorbot must be Homed first!!\r\n");
        return 1;
    }
}

void enc2theta(float *B_theta, float *S_theta, float *E_theta, float *P_theta, float *R_theta){
    //float k[5] = {141.222, -112.978, 112.978, 27.8981, -27.8981};
    float k[5] = {MOT_1_CPD, -MOT_2_3_CPD, MOT_2_3_CPD, MOT_4_5_CPD, -MOT_4_5_CPD};
    float q[5];
    float b_theta, s_theta, e_theta, p_theta, r_theta;
    
    if(axis1.axisState)
        q[0] = axis1.pos;
    else
        q[0] = (float)axis1.readEncoderValue();
        
    if(axis2.axisState)
        q[1] = axis2.pos;
    else
        q[1] = (float)axis2.readEncoderValue();
        
    if(axis3.axisState)
        q[2] = axis3.pos;
    else
        q[2] = (float)axis3.readEncoderValue();
                
    if(axis4.axisState)
        q[3] = axis4.pos;
    else
        q[3] = (float)axis4.readEncoderValue();
        
    if(axis5.axisState)
        q[4] = axis5.pos;
    else
        q[4] = (float)axis5.readEncoderValue();                    
                    
    b_theta = q[0] / k[0];
    s_theta = -(q[1]/k[1])+120.28;
    e_theta = -(q[1]+q[2])/k[2];
    r_theta = (q[4]+q[3])/(k[4]*2.0);
    p_theta = -88.81+((WRIST_ELBOW_OFFSET_RATIO*q[2])/k[3])+((k[4]*r_theta)/k[3])-(q[3]/k[3]);    
            
    //convert to radians if in radian mode
    if(dev_mode_Global == 'r'){
        b_theta *= PI/180.0;
        s_theta *= PI/180.0;
        e_theta *= PI/180.0;
        p_theta *= PI/180.0;
        r_theta *= PI/180.0;
    }        
    
    *B_theta = b_theta;
    *S_theta = s_theta;
    *E_theta = e_theta;
    *P_theta = p_theta;
    *R_theta = r_theta;
}

void printControllerOutputs(void){
    //Declare variable for BSEPR and G (base, shoulder, elbow, wrist pitch, wrist roll, and gripper position
    float b,s,e,p,r,g;
                                                
    switch(dev_mode_Global){
        case 'c':
            pc.printf("T%.2f", runTime.ms_total*.001);
            pc.printf("PC%.1f,%.1f,%.1f,%.1f,%.1f,%.1f", (float)axis1.pos,(float)axis2.pos,(float)axis3.pos,(float)axis4.pos,(float)axis5.pos,(float)axis6.pos);
            pc.printf("V%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", (float)axis1.vel,(float)axis2.vel,(float)axis3.vel,(float)axis4.vel,(float)axis5.vel,(float)axis6.vel);
            pc.printf("S%d,%d,%d,%d,%d,%d", axis1.stat, axis2.stat, axis3.stat, axis4.stat, axis5.stat, axis6.stat);
            pc.printf("\r");
            if(streamFlag)
                pc.printf("\n");
            break;
        case 'd':
            pc.printf("T%.2f", runTime.ms_total*.001);
            
            enc2theta(&b,&s,&e,&p,&r);
            g = sbGripper.get_mm();
            
            pc.printf("PD%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", b, s, e, p, r, g);
            //pc.printf("V%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", (float)axis1.vel,(float)axis2.vel,(float)axis3.vel,(float)axis4.vel,(float)axis5.vel,(float)axis6.vel);
            pc.printf("S%02X", (int)check_stats() & 0x3F);
            pc.printf("M%02X", (int)axisMoving() & 0x3F);
            pc.printf("\r");
            if(streamFlag)
                pc.printf("\n");

            break;
        case 'r':
            pc.printf("T%.2f", runTime.ms_total*.001);
            
            enc2theta(&b,&s,&e,&p,&r);                    
            g = sbGripper.get_mm();
            
            pc.printf("\r\nPR%.4f,%.4f,%.4f,%.4f,%.4f,%.4f", b, s, e, p, r, g);
            pc.printf("\r\nV%.3f,%.3f,%.3f,%.3f,%.3f,%.3f", (float)axis1.vel,(float)axis2.vel,(float)axis3.vel,(float)axis4.vel,(float)axis5.vel,(float)axis6.vel);
            pc.printf("\r\nS%02X", (int)check_stats() & 0x3F);
            pc.printf("\r\nM%02X", (int)axisMoving() & 0x3F);
            pc.printf("\r\n");
            pc.printf("\r\n");
            if(streamFlag)
                pc.printf("\n");

        default:
        
            break;                
    } //    
}

//returns 0 if success, 1 for timeout, -1 for new move
int moveDone_or_qs(float dt){
    Timer tmout;
    tmout.start();
    while(axisMoving() != 0) {
        if(tmout.read() > dt){
            pc.printf("\r\n\r\n%sERR in moveDone_or_qs(): Timeout Occured at Line2679\r\n", RED_FONT);
            if(DEBUG)
                pc.printf("%s\r\nTimeout occured before finishing profile move\r\n%s", YELLOW_FONT, DEFAULT_FONT);
            // Added 20220609 - DOES THIS FIX THE EXERCISE FUNCTION !!!!!
            NVIC_SystemReset();     //Restart the System
            return -2;  //Indicate an error occured
        }//if(tmout.read() > (dt+2.0)){
                
        if(pc.readable()){
            char query = pc.getc();
            
            if(query == ' ')
                printControllerOutputs();
            if(query == 'q'){
                float b,s,e,p,r,g;                                    
                enc2theta(&b,&s,&e,&p,&r);
                g = sbGripper.get_mm();
                
                theta2enc(b,s,e,p,r);
                axis6.set_point = g;
                return 1;               // returns 1 if 'q' was pressed to quit
            }
            
            //interrupted by a new instantaneous move command
            if((query == '{') || (query == '}')){
                return -1;
            }
        } 
    }// theta2enc_dt is still running
    tmout.stop();
    return 0; 
}

void axisRampTestI(Axis &axisTest){
    float readIdelay=.02;
    float setPointTest=1000;
    float axisImeas=0.0;
    Timer   t1;                                                        

    overCurrentTick.detach();
    axisTest.axisOn();
    t1.start();
    axisTest.set_point += setPointTest;
    while(t1.read() < readIdelay){
        //print_motI();
        //both timestamps and updates mot max
        axisImeas = axisTest.readCurrent();
        if(DEBUG)
            pc.printf("t=%.3f %.4f \r\n", t1.read(), axisImeas);
        wait(.001);
    }
    axisTest.set_point -= setPointTest;
    t1.reset();
    while(t1.read() < readIdelay){
        //print_motI();
        axisImeas = axisTest.readCurrent();
        if(DEBUG)
            pc.printf("t=%.3f %.4f \r\n", t1.read(), axisImeas);
        wait(.001);
    }                    
    overCurrentTick.attach(&collisionCheck, CURRENT_CHECK_PERIOD);
    return;
}

void setPointRun(Axis &axisRun, float temp_setpoint){
    axisRun.axisOn();
    pc.printf("T=%.2f SP=%.3f co=%.3f pos=%.3f vel=%.3f acc=%.3f Imot=%.3f Imax=%.3f\r\n", t.read(), axisRun.set_point, axisRun.co, axisRun.pos, axisRun.vel, axisRun.acc, axisRun.motI, axisRun.mot_I_max); 
    axisRun.set_point = temp_setpoint;
    t.reset();
    while((axisRun.pos > (axisRun.set_point + SP_TOL)) || (axisRun.pos < (axisRun.set_point - SP_TOL))){
        axisRun.readCurrent();
        pc.printf("T=%.2f SP=%.3f co=%.3f pos=%.3f vel=%.3f acc=%.3f Imot=%.3f Imax=%.3f\r\n", t.read(), axisRun.set_point, axisRun.co, axisRun.pos, axisRun.vel, axisRun.acc, axisRun.motI, axisRun.mot_I_max); 
        wait(.009);
        if(t.read() > 10.0){
            pc.printf("Set point timeout!\r\n");
            break;
        }
        
        if(pc.readable()){      //if user types a 'q' or 'Q'
            char c = pc.getc();
            if(c == 'q' || c == 'Q') //quit after current movement
                break;
        }                            
    }
    pc.printf("T=%.2f SP=%.3f co=%.3f pos=%.3f vel=%.3f acc=%.3f Imot=%.3f Imax=%.3f\r\n", t.read(), axisRun.set_point, axisRun.co, axisRun.pos, axisRun.vel, axisRun.acc, axisRun.motI, axisRun.mot_I_max);
    return;
}

// this function has 63 bytes write limit
void writeEEPROM(int address, unsigned int eeaddress, char *data, int size)
{
    char i2cBuffer[size + 2];
    i2cBuffer[0] = (unsigned char)(eeaddress >> 8); // MSB
    i2cBuffer[1] = (unsigned char)(eeaddress & 0xFF); // LSB

    for (int i = 0; i < size; i++) {
        i2cBuffer[i + 2] = data[i];
    }

    int result = i2c.write(address, i2cBuffer, size + 2, false);
    wait_ms(6);
}

// this function has no read limit
void readEEPROM(int address, unsigned int eeaddress, char *data, int size)
{
    char i2cBuffer[2];
    i2cBuffer[0] = (unsigned char)(eeaddress >> 8); // MSB
    i2cBuffer[1] = (unsigned char)(eeaddress & 0xFF); // LSB

    // Reset eeprom pointer address
    int result = i2c.write(address, i2cBuffer, 2, false);
    wait_ms(6);

    // Read eeprom
    i2c.read(address, data, size);
    wait_ms(6);
}

void default_eprom_motI(void){
    char tempBuf[32];
    
    //Write the default value max current (conservative) to the I2C eeprom
    sprintf(tempBuf, "%f\0", STALL_I_1_DEFAULT);
    pointerAdddress = eprom_mot1maxI;
    writeEEPROM(EPROMaddress, pointerAdddress, tempBuf, BLOCK_SIZE);

    sprintf(tempBuf, "%f\0", STALL_I_2_DEFAULT);
    pointerAdddress = eprom_mot2maxI;
    writeEEPROM(EPROMaddress, pointerAdddress, tempBuf, BLOCK_SIZE);
    
    sprintf(tempBuf, "%f\0", STALL_I_3_DEFAULT);
    pointerAdddress = eprom_mot3maxI;
    writeEEPROM(EPROMaddress, pointerAdddress, tempBuf, BLOCK_SIZE);
    
    sprintf(tempBuf, "%f\0", STALL_I_4_DEFAULT);
    pointerAdddress = eprom_mot4maxI;
    writeEEPROM(EPROMaddress, pointerAdddress, tempBuf, BLOCK_SIZE);
    
    sprintf(tempBuf, "%f\0", STALL_I_5_DEFAULT);
    pointerAdddress = eprom_mot5maxI;
    writeEEPROM(EPROMaddress, pointerAdddress, tempBuf, BLOCK_SIZE);
    
    sprintf(tempBuf, "%f\0", STALL_I_6_DEFAULT);
    pointerAdddress = eprom_mot6maxI;
    writeEEPROM(EPROMaddress, pointerAdddress, tempBuf, BLOCK_SIZE);
}
    
void read_eprom_motI(void){
    char data_read[32];
    
    pc.printf("reading eprom for motor current values\r\n");
    readEEPROM(EPROMaddress, eprom_mot1maxI, data_read, BLOCK_SIZE);
    STALL_I_1 = atof(data_read);
    STALL_I_1 *= MAX_CURRENT_SCALE1;
    
    readEEPROM(EPROMaddress, eprom_mot2maxI, data_read, BLOCK_SIZE);
    STALL_I_2 = atof(data_read);
    STALL_I_2 *= MAX_CURRENT_SCALE2;
    
    readEEPROM(EPROMaddress, eprom_mot3maxI, data_read, BLOCK_SIZE);
    STALL_I_3 = atof(data_read);
    STALL_I_3 *= MAX_CURRENT_SCALE3;
    
    readEEPROM(EPROMaddress, eprom_mot4maxI, data_read, BLOCK_SIZE);
    STALL_I_4 = atof(data_read);
    STALL_I_4 *= MAX_CURRENT_SCALE4;
    
    readEEPROM(EPROMaddress, eprom_mot5maxI, data_read, BLOCK_SIZE);
    STALL_I_5 = atof(data_read);
    STALL_I_5 *= MAX_CURRENT_SCALE5;
    
    readEEPROM(EPROMaddress, eprom_mot6maxI, data_read, BLOCK_SIZE);
    STALL_I_6 = atof(data_read);
    STALL_I_6 *= MAX_CURRENT_SCALE6;
    
    pc.printf("Stall I : %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\r\n", STALL_I_1,STALL_I_2,STALL_I_3,STALL_I_4,STALL_I_5,STALL_I_6);
}

void exerciseAllAxes(void){
    pc.printf("\r\nPress q to quit Exercise\r\n");
    pc.printf("Received move test command\r\n"); 
    int qFlag=0;    
    float dt_x;
    int err=0;
    int state=0;
    while((err==0) && (qFlag==0)){
        switch(state){
            case 0:
                dt_x=5.1;
                err = theta2enc_dt(dt_x,0.0,1.57,-1.57,0.0,0.0,0.0);
                qFlag = moveDone_or_qs(dt_x+(dt_x*.2));//wait for move to complete or receive a 'q' or ' '
                state++;
            break;                            

            case 1:
                dt_x=1.1;
                err = theta2enc_dt(dt_x,0.0,1.57,-1.57,0.0,0.0,20.0);                        
                qFlag = moveDone_or_qs(dt_x+(dt_x*.2));//wait for move to complete or receive a 'q' or ' '
                state++;
            break;
            
            case 2:
                dt_x=4.1;
                err = theta2enc_dt(dt_x,0.0,1.57,-1.57,0.0,-1.57,40.0);                        
                qFlag = moveDone_or_qs(dt_x+(dt_x*.2));//wait for move to complete or receive a 'q' or ' '
                state++;
            break;
            
            case 3:
                dt_x=3.5;
                err = theta2enc_dt(dt_x,0.0,1.57,-1.57,0.0,0.0,0.0);                        
                qFlag = moveDone_or_qs(dt_x+(dt_x*.2));//wait for move to complete or receive a 'q' or ' '
                state++;
            break;
            
            
            case 4:
                dt_x=3.5;
                err = theta2enc_dt(dt_x,0.0,1.57,0.0,-1.57,0.0,0.0);                        
                qFlag = moveDone_or_qs(dt_x+(dt_x*.2));//wait for move to complete or receive a 'q' or ' '
                state++;
            break;
            
            case 5:
                dt_x=5.0;
                err = theta2enc_dt(dt_x,0.32,.78,0.0,-.78,1.57,0.0);                        
                qFlag = moveDone_or_qs(dt_x+(dt_x*.2));//wait for move to complete or receive a 'q' or ' '
                state++;
            break;
            
            case 6:
                dt_x=2.2;
                err = theta2enc_dt(dt_x,0.32,.78,0.0,-.78,1.57,50.0);
                qFlag = moveDone_or_qs(dt_x+(dt_x*.2));//wait for move to complete or receive a 'q' or ' '
                state++;
                break;
            
            case 7:
                dt_x=2.2;
                err = theta2enc_dt(dt_x,0.32,.78,0.0,-.78,1.57,0.0);
                qFlag = moveDone_or_qs(dt_x+(dt_x*.2));//wait for move to complete or receive a 'q' or ' '
                state++;
            break;
            
            case 8:
                dt_x=2.2;
                err = theta2enc_dt(dt_x,0.32,.78,0.0,-.78,1.57,50.0);
                qFlag = moveDone_or_qs(dt_x+(dt_x*.2));//wait for move to complete or receive a 'q' or ' '
                state++;
            break;
            
            case 9:
                dt_x=2.2;
                err = theta2enc_dt(dt_x,0.32,.78,0.0,-.78,1.57,0.0);
                qFlag = moveDone_or_qs(dt_x+(dt_x*.2));//wait for move to complete or receive a 'q' or ' '
                state = 0;  //back to the beginning
            break;
        }
        
        if(pc.readable()){
            char c = pc.getc();
            if(c == 'q'){
                float b,s,e,p,r,g;          //Read all the axes and gripper
                g = sbGripper.get_mm();
                enc2theta(&b,&s,&e,&p,&r);
                
                err = theta2enc_dt(dt_x,b,s,e,p,r,g);
                qFlag = moveDone_or_qs(dt_x+(dt_x*.2));//wait for move to complete or receive a 'q' or ' '
                
                err=1;
            }
        }
        
        printControllerOutputs();
        
        //test the error flag and exit if there was a problem
        if((qFlag != 0) || (err != 0)){
            pc.printf("%sERROR OCCURED qFlag=%d err=%d\r\nExiting.%s", RED_FONT, qFlag, err, DEFAULT_FONT);
            if(err == 1){
                pc.printf("%s q was pressed\r\n%s", GREEN_FONT, DEFAULT_FONT);
            }
            if(err == -1){
                pc.printf("%s timeout occured\r\n%s", YELLOW_FONT, DEFAULT_FONT);
            }
            if(qFlag == -1){
                pc.printf("%s timeout occured \r\n%s", YELLOW_FONT, DEFAULT_FONT);
            }            
            return;
        }

        led2 = !led2;                       
    }
    char tempBufnew[16];
    sprintf(tempBufnew, "MANUAL\0");
    writeEEPROM(EPROMaddress, eprom_mode, tempBufnew, 16);
    wait(.3);     
}

void motCurrentProfileUpdate(void){
    char tempBuf[32];
    
    pc.printf("Axis1\r\n");
    axisRampTestI(axis1);
    sprintf(tempBuf, "%f\0", axis1.mot_I_max);
    pointerAdddress = eprom_mot1maxI;
    writeEEPROM(EPROMaddress, pointerAdddress, tempBuf, BLOCK_SIZE);

    if(DEBUG)
        pc.printf("Data written %d: %s\r\n", pointerAdddress, tempBuf);
        
    wait(.1);
    
    pc.printf("Axis2\r\n");
    axisRampTestI(axis2);
    sprintf(tempBuf, "%f\0", axis2.mot_I_max);
    pointerAdddress = eprom_mot2maxI;
    writeEEPROM(EPROMaddress, pointerAdddress, tempBuf, BLOCK_SIZE);

    if(DEBUG)
        pc.printf("Data written %d: %s\r\n", pointerAdddress, tempBuf);
                                
    wait(.1);
    
    pc.printf("Axis3\r\n");
    axisRampTestI(axis3);
    sprintf(tempBuf, "%f\0", axis3.mot_I_max);
    pointerAdddress = eprom_mot3maxI;
    writeEEPROM(EPROMaddress, pointerAdddress, tempBuf, BLOCK_SIZE);

    if(DEBUG)
        pc.printf("Data written %d: %s\r\n", pointerAdddress, tempBuf);
        
    wait(.1);
    
    pc.printf("Axis4\r\n");
    axisRampTestI(axis4);
    sprintf(tempBuf, "%f\0", axis4.mot_I_max);
    pointerAdddress = eprom_mot4maxI;
    writeEEPROM(EPROMaddress, pointerAdddress, tempBuf, BLOCK_SIZE);

    if(DEBUG)
        pc.printf("Data written %d: %s\r\n", pointerAdddress, tempBuf);
        
    wait(.1);
    
    pc.printf("Axis5\r\n");
    axisRampTestI(axis5);
    sprintf(tempBuf, "%f\0", axis5.mot_I_max);
    pointerAdddress = eprom_mot5maxI;
    writeEEPROM(EPROMaddress, pointerAdddress, tempBuf, BLOCK_SIZE);

    if(DEBUG)
        pc.printf("Data written %d: %s\r\n", pointerAdddress, tempBuf);
        
    wait(.1);
    
    pc.printf("Axis6\r\n");
    axisRampTestI(axis6);
    sprintf(tempBuf, "%f\0", axis6.mot_I_max);
    pointerAdddress = eprom_mot6maxI;
    writeEEPROM(EPROMaddress, pointerAdddress, tempBuf, BLOCK_SIZE);

    if(DEBUG)
        pc.printf("Data written %d: %s\r\n", pointerAdddress, tempBuf);
        
    wait(.1);
    
    sprintf(tempBuf, "OK");
    pointerAdddress = eprom_status;
    writeEEPROM(EPROMaddress, pointerAdddress, tempBuf, BLOCK_SIZE);
    print_motMaxI();
    
    //now re-read the I2C memory 
    readEEPROM(EPROMaddress, eprom_status, data_read, BLOCK_SIZE);
    pc.printf("Read EPROM STATUS: %s\r\n", data_read);

    if(strcmp(data_read, "OK") == 0){
        read_eprom_motI();
    }
    else{
        pc.printf("EPROM read NOT OK!!");
    }
}

//Checks to see if any axes have exceeded their known boundaries after being homed
int checkBoundsRad(float *b,float *s,float *e,float *p,float *r,float *g){
    int error = 0;          //return 0 if no boundaries found
    if(homed()){            //make sure homed first
        //check the base position variable
        if(*b > 3.0){
            *b = 3.0;
            error |= 0x0001;
            if(DEBUG)
                pc.printf("base exceeded positive boundary!");            
        }
        if(*b < -3.0){
            *b = -3.0;
            error |= 0x0100;
            if(DEBUG)
                pc.printf("base exceeded negative boundary!");
        }
        //check shoulder position variable
        if(*s > 2.14){
            *s = 2.14;
            error |= 0x0002;
            if(DEBUG)
                pc.printf("shoulder exceeded positive boundary!");
        }
        if(*s < -.6){
            *s = -.6;
            error |= 0x0200;
            if(DEBUG)
                pc.printf("shoulder exceeded negative boundary!");
        }        

        //check gripper position
        if(*g < 0.0){
            *g = 0.0;
            error |= 0x2000;
            if(DEBUG)
                pc.printf("gripper exceeded negative boundary!");
        }
        if(*g > 75.0){      //Max is ~ 75mm
            *g = 75.0;
            error |= 0x2000;
            if(DEBUG)
                pc.printf("shoulder exceeded negative boundary!");
        }                        
    }
    else{
        pc.printf("\r\n Robot Arm not homed!\r\n");
    }
    return error;
}

//Structures for velocity using enc2theta() and theta2enc() for robot joint control 1-11-2019
Ticker  vel_Update;     //Ticker to run the velocity controller
Timer   vel_Timer;      //Timer for velocity command timeout

static float   vel_Timeout=0.0;
static float   vel_b=0.0, vel_s=0.0, vel_e=0.0, vel_p=0.0, vel_r=0.0, vel_g=0.0;
static float   velTot_b=0.0, velTot_s=0.0, velTot_e=0.0, velTot_p=0.0, velTot_r=0.0, velTot_g=0.0;
static float   velTot_s_e=0.0, velTot_s_p=0.0, velTot_e_p=0.0;
static float   vel_b_start=0.0, vel_s_start=0.0, vel_e_start=0.0, vel_p_start=0.0, vel_r_start=0.0, vel_g_start=0.0;
static float   vel_s_e_start=0.0, vel_s_p_start=0.0, vel_e_p_start=0.0; //used for differencing accumulator
//float   vel_UpdateRate;
#define VEL_UPDATE_RATE     (.02f)  //20 milliseconds
void velUpdateTick(void){
    float b,s,e,p,r,g;
                         
//    enc2theta(&b,&s,&e,&p,&r);
//    g = sbGripper.get_mm();
    
    //First check to see if the velocity timer has exceeded the timeout 
    if(vel_Timer.read() < vel_Timeout){
        /*
        //Is the base joint stalled, if so do not add any more velocity
        if((stall_state & 0x01) != 0x01){
            velTot_b += (vel_b * VEL_UPDATE_RATE);  //update with integrated velocity            
        }
        else{
            vel_b = 0.0;    //re-zero velocity
        }
        
        //Is the Shoulder joint stalled, if so do not add any more velocity
        if((stall_state & 0x02) != 0x02){
            velTot_s += (vel_s * VEL_UPDATE_RATE);      //update with integrated velocity
            
            velTot_s_e += (-vel_s * VEL_UPDATE_RATE);
            
            velTot_s_p += (-vel_s * VEL_UPDATE_RATE) * WRIST_ELBOW_RATIO;    
        }
        else{
            vel_s = 0.0;    //re-zero velocity
        }        
        
        //Is the Elbow joint stalled, if so do not add any more velocity
        if((stall_state & 0x04) != 0x04){
            velTot_e += (vel_e * VEL_UPDATE_RATE);  //update with integrated velocity                        

            velTot_e_p += (vel_e * VEL_UPDATE_RATE)* WRIST_ELBOW_RATIO;  //add elbow velocity to pitch            
        }
        else{
            vel_e = 0.0;    //re-zero velocity
        }    
        
        //Is the Pitch or roll joint stalled (either motor 4 or 5) do not add any more velocity
        if(!(stall_state & 0x18)){
            velTot_p += (vel_p * VEL_UPDATE_RATE);  //update with integrated velocity
            velTot_r += (vel_r * VEL_UPDATE_RATE);  //update with integrated velocity            
        }
        else{
            vel_p = 0.0;    //re-zero velocity
            vel_r = 0.0;    //re-zero velocity
        }
        
        //Is the Gripper joint stalled
        if((stall_state & 0x20) != 0x20){
            velTot_g += (vel_g * VEL_UPDATE_RATE);  //update with integrated velocity            
        }
        else{
            vel_g = 0.0;    //re-zero velocity
        }
        */
        //update all velocity relationships
        velTot_b += (vel_b * VEL_UPDATE_RATE);  //update with integrated velocity            
        velTot_s += (vel_s * VEL_UPDATE_RATE);      //update with integrated velocity            
        velTot_s_e += (-vel_s * VEL_UPDATE_RATE);            
        velTot_s_p += (-vel_s * VEL_UPDATE_RATE) * WRIST_ELBOW_RATIO;    
        velTot_e += (vel_e * VEL_UPDATE_RATE);  //update with integrated velocity                        
        velTot_e_p += (vel_e * VEL_UPDATE_RATE)* WRIST_ELBOW_RATIO;  //add elbow velocity to pitch            
        velTot_p += (vel_p * VEL_UPDATE_RATE);  //update with integrated velocity
        velTot_r += (vel_r * VEL_UPDATE_RATE);  //update with integrated velocity            
        velTot_g += (vel_g * VEL_UPDATE_RATE);  //update with integrated velocity                    
        
        //UPDATE FINAL POSITIONS WITH VELOCITY VARIABLES
        // starting point plus velocity totals is crucial, prevenst rounding errors
        b = vel_b_start + velTot_b;                           //Base is start plus velTot_b accumulator
        s = vel_s_start + velTot_s;                           //Shoulder is start plus velTot_s accumulators
        e = vel_e_start + velTot_e + velTot_s_e;              //Elbow is start plus velTot_e + velTot_s_e
        p = vel_p_start + velTot_p + velTot_s_p + velTot_e_p; //Pitch is start plus velTot_p + velTot_s_p + accumulators
        r = vel_r_start + velTot_r;                           //Pitch is start plus velTot_r
        g = vel_g_start + velTot_g;                           //Gripper is start plus velTot_g

/*
        //check the base position variable
        if(b > 2.8){
            b = 2.8;
            vel_Update.detach();    // disable the ticker
        }
        if(b < -3.0){
            b = -3.0;
            vel_Update.detach();    // disable the ticker            
        }
        //check shoulder position variable
        if(s > 2.14){
            s = 2.14;
            vel_Update.detach();    // disable the ticker
        }
        if(s < -.6){
            s = -.6;
            vel_Update.detach();    // disable the ticker
        }
        if(p > 1.9){
            p = 1.9;
            vel_Update.detach();    // disable the ticker
        }
        if(p < -1.9){
            p = -1.9;
            vel_Update.detach();    // disable the ticker
        }        
*/        
                
        //check to see if any axes have exceeded their bounds during velocity control
        //int error = checkBoundsRad(&b,&s,&e,&p,&r,&g);        
        //if(DEBUG)
        //    pc.printf("\r\nERROR = 0x%04X\r\n", error);
        
        //update the robot position
        theta2enc(b,s,e,p,r);
        sbGripper.set_mm(g);        
    }
    else{       //Timout has occured
        vel_Update.detach();    // disable the ticker
        
        //re-attach over current collision detection 2019-3-26 at Kutzers request
        overCurrentTick.attach(&collisionCheck, CURRENT_CHECK_PERIOD);
        vel_b = 0.0;            //zero all the velocity values
        vel_s = 0.0;
        vel_e = 0.0;
        vel_p = 0.0;
        vel_r = 0.0;
        vel_g = 0.0;
        
        //re-enable the gripper collision detection
        GRIPPER_COL_DETECT = 1;  
    }
}

void velocityCmd(float timeout,float vb,float vs,float ve,float vp,float vr,float vg){    
    //detach the velUpdateTick function to the Ticker if already running
    //vel_Update.detach();  //Removed to test 20191101 JB
    
    //Turn off the gripper collision detection
    GRIPPER_COL_DETECT = 0;
    
    //read the current position of all the axes
    enc2theta(&vel_b_start,&vel_s_start,&vel_e_start,&vel_p_start,&vel_r_start);
    vel_g_start = sbGripper.get_mm();        
    
    vel_b = vb; //update the velocity
    vel_s = vs; //update the velocity
    vel_e = ve; //update the velocity
    vel_p = vp; //update the velocity
    vel_r = vr; //update the velocity
    //velocity of gripper in mm/sec but still uses the velCmdRadSec variable
    vel_g = vg; //update the velocity
    
    //Zero the accumulators of the position
    velTot_b=0.0;
    
    velTot_s = 0.0;
    velTot_s_e = 0.0;
    velTot_s_p = 0.0;                       
    
    velTot_e = 0.0;
    velTot_e_p = 0.0;         
    
    velTot_p = 0.0;       
    
    velTot_r = 0.0;
    
    velTot_g = 0.0;
            
    // Set the global timeout variable and rest the timer and start it
    vel_Timeout = timeout;
    vel_Timer.reset();
    vel_Timer.start(); 
    
    //Attach the velUpdateTick function to the Ticker
    vel_Update.attach(&velUpdateTick, VEL_UPDATE_RATE);
    
    //detach over current collision detection 2019-3-26 at Kutzers request
    // during velocity commands
    overCurrentTick.detach();    
}        
//------------------- MAIN --------------------------------
int main()
{
    wait(.2);
    pulse.attach(&heartbeat, 2.0); // the address of the function to be attached (alive) and the interval (2 seconds)
 
    //NVIC_SetPriority(FLEX_INT0_IRQn, 1); // set mbed tickers to higher priority than other things
    NVIC_SetPriority(TIMER3_IRQn, 255);
 
    //pc.baud(921600);
    pc.baud(115200);
    i2c.frequency(400000);
        
    //pc.printf("%s\r\n%s\r\n", CLR_SCR, __FILE__); //clear screen and display the filename (this source file)    
    //pc.printf("vel flags = %d %d %d %d %d %d \r\n", vel_bFlag, vel_sFlag, vel_eFlag, vel_pFlag, vel_rFlag, vel_gFlag);
    //pc.printf("%s %s\n",__TIME__,__DATE__);
    //pc.printf("%s %s compiled on %s J. Bradshaw\r\n%s", YELLOW_FONT, VERSION_STRING, __DATE__, DEFAULT_FONT);
    
    // If a z is pressed within first 2 seconds of boot, update EEPROM and recal Currents
    int MRST_TimeoutCMD=0;
    while(MRST_TimeoutCMD<20){
        char c = pc.rxGetLastChar() & 0xFF;        
        if(c == 'z'){      // Pressed ESC to reset
            pc.printf("\r\n z was pressed, reset to defaults\r\n");
            char tempBufnew[16];
            sprintf(tempBufnew, "MANUAL\0");
            writeEEPROM(EPROMaddress, eprom_mode, tempBufnew, 16);
            wait(.3);
            while(pc.readable()){
                c =  pc.getc();
            }
            break;
        }
        wait(.1);
        MRST_TimeoutCMD++;
    }
    
    // !! -----------------------------------------------------------------------------------------
    // --------------------- WHEN EEPROM GETS CORRUPTED - UNCOMMENT THESE 3 LINES AND REPROGRAM ---
    //  THEN, COMMENT AGAIN AND REPROGRAM ... -----------------------------------------------------            
//    char tempBufnew[16];
//    sprintf(tempBufnew, "MANUAL\0");
//    writeEEPROM(EPROMaddress, eprom_mode, tempBufnew, 16);
//    wait(.3);
//    motCurrentProfileUpdate();
    
    pc.printf("\r\n%s\r\n", __FILE__); //clear screen and display the filename (this source file)
    pc.printf("%s compiled on %s J. Bradshaw\r\n", VERSION_STRING, __DATE__);
    //led_blink_sequence();        
    init_limitSwitches();   //get initial states of limit switches
    
    axis1.init(MOT_1_CPR);    //141.222 counts per degree * 360 degrees
    axis2.init(MOT_2_CPR);    //112.978 cpd * 360 degrees
    axis3.init(MOT_3_CPR);
    axis4.init(MOT_4_CPR);    //27.8981 cpd * 360 degrees
    axis5.init(MOT_5_CPR);
    axis6.init(MOT_6_CPR);       //about this many per throw but only used in position mode

    //NOTE: The way the integral component is calculated in the PID class used
    // the lower the Ik value, the faster the ramp up of the integration error 
    // value. Ik values below .5 will usually cause instability.

    //Tune gains for the base
    axis1.Pk = 280.0;   //210.0;
    axis1.Ik = 7.5;     //.4;
    axis1.Dk = 0.0; //.0003;
    axis1.updatePIDgains(axis1.Pk, axis1.Ik, axis1.Dk); //turns on controller
    
    //Tune gains for the shoulder
    axis2.Pk = 1080.0;   //450.0;
    axis2.Ik = 2.7;
    axis2.Dk = 0.0;
    axis2.updatePIDgains(axis2.Pk, axis2.Ik, axis2.Dk); //turns on controller

    //Tune gains for the elbow
    axis3.Pk = 610.0;   //450.0;
    axis3.Ik = 3.1;  //seems high
    axis3.Dk = 0.0;
    axis3.updatePIDgains(axis3.Pk, axis3.Ik, axis3.Dk); //turns on controller
    
    //Tune gains for the wrist
    axis4.Pk = 410.0;
    axis4.Ik = 15.0;    //995.0;
    axis4.updatePIDgains(axis4.Pk, axis4.Ik, 0.0); //turns on controller

    //Tune gains for the wrist
    axis5.Pk = 410.0;
    axis5.Ik = 15.0;    //995.0;
    axis5.updatePIDgains(axis5.Pk, axis5.Ik, 0.0); //turns on controller
  
    //Tune gains for the gripper
    axis6.Pk = 220.0;
    axis6.Ik = 30.0; //25.0;    //175.0;
    axis6.updatePIDgains(axis6.Pk, axis6.Ik, 0.0); //turns on controller
  
    //Turn on the collision detection ticker
    overCurrentTick.attach(&collisionCheck, CURRENT_CHECK_PERIOD);    
    enMotDrv=0; //Default is enabled
    
    axis1.motInvert = 1;
    axis2.motInvert = 1;
    axis3.motInvert = 1;
    axis4.motInvert = 1;
    axis5.motInvert = 1;
    axis6.motInvert = 1;
     
    t.start();  // Set up timer 
//    serialStream.attach(&serialOutput, .02);    
    
    readEEPROM(EPROMaddress, eprom_status, data_read, BLOCK_SIZE);
    pc.printf("Read EPROM STATUS: %s\r\n", data_read);
    
    if(strcmp(data_read, "OK") == 0){
        read_eprom_motI();
    }
    else{
        pc.printf("EPROM read NOT OK!!\r\n");
    }

    readEEPROM(EPROMaddress, eprom_mode, data_read, BLOCK_SIZE);
    //pc.printf("Read EPROM MODE: %s\r\n", data_read);
    
    //pc.printf("STRCMP EXERCISE = %d %s\r\n", strcmp(data_read, "EXERCISE"), data_read);
    
    if(strcmp(data_read, "EXERCISE") == 0){
        int err = home_master();
        if(err != 0){
            all_off();
            pc.printf("\r\n\r\nNAK\r\nHOME FAILED!!!\r\n");
            if(DEBUG)
                pc.printf("Master Home returned %d\r\n", err);                                                                
        }              
        else         
            pc.printf("OK\r\n");
        wait(.2);
        motCurrentProfileUpdate();
        wait(.2);
        motCurrentProfileUpdate();
        wait(.2);
        motCurrentProfileUpdate();
        wait(.2);
        err = home_master();
        
        if(err != 0){
            all_off();
            pc.printf("\r\n\r\nNAK\r\nHOME FAILED!!!\r\n");
            if(DEBUG)
                pc.printf("Master Home returned %d\r\n", err);                                                                
        }              
        else         
            pc.printf("OK\r\n");        
            
        char old_dev_mode_Global = dev_mode_Global;
        dev_mode_Global='d';
        wait(.1);
        float dt=8.0;
        theta2enc_dt(dt,0.0,120.28,-95.02,-88.81,0.00, 0.0);                    
        //wait for move to complete or receive a 'q' or ' '
        moveDone_or_qs(dt+(dt*.5));
            
        pc.printf("Home position OK\r\n");
        dev_mode_Global=old_dev_mode_Global;
        wait(2.0);
        exerciseAllAxes();
        
        char tempStr[16];
        sprintf(tempStr, "MANUAL\0");
        writeEEPROM(EPROMaddress, eprom_mode, tempStr, 16);
        pc.printf("Writing MANUAL to EEPROM\r\n");
        wait(.3);
    }
    if(strcmp(data_read, "MANUAL") == 0){
        pc.printf("Currently in Manual Operation!\r\n");
        pc.printf("Press ? for menu, Y to home\r\n");
    }            
  
    if(MRST_TimeoutCMD < 20){
        motCurrentProfileUpdate();
    }
        
    while(1){
        
        if(pc.readable()){
            int limitFlag = 1;
            char joint;            
            char bufferRead[10000];
            int MAX_ARRAY_SIZE=100;
            float position = 0.0;
            float time = 0.0;
            char *tok;
            float dt, theta0, theta1, theta2, theta3, theta4, grip_mm;
            int qFlag=0;
            float counts;
            char axis_sel;
            int chars;
            int err;
            for(int i=0;i<MAX_ARRAY_SIZE;i++)
                bufferRead[i]=0;            
                                
            char c = pc.getc();

            switch(c){                                                              
                case '?':    //get commands                    
                    pc.printf("\r\n%s Compiled on %s J. Bradshaw\r\n", VERSION_STRING, __DATE__);
                    wait(.05);
                    pc.printf("? - This menu of commands\r\n");
                    pc.printf("0 - Zero all encoder channels\r\n");
                    pc.printf("A - All: Enable/Disable All axes. Then 'O' for On and 'F' for Off\r\n");                
                    pc.printf("C - Current: Stream the Motor Currents\r\n");
                    pc.printf("E - Enable for all motor drivers.  Then '1' for On and '0' for Off\r\n");                
                    pc.printf("J - Excercise Joint degrees/radians function\r\n");
                    wait(.05);
                    pc.printf("W - Write value to axis\r\n");
                    pc.printf("T - Trapezoidal Profile Move command\r\n");
                    pc.printf("H - Home\r\n");
                    pc.printf("S - Set point in encoder counts\r\n");
                    pc.printf("Z - Zero Encoder channel\r\n");
                    pc.printf("V - Velocity command Vt,b,s,e,p,r,g\\r\r\n    t=timout,b,s,e,p,r=rad/sec,g=mm/sec\\r\\n \r\n");
                    wait(.05);
                    pc.printf("X - Excercise Robotic Arm\r\n");
                    pc.printf("O - Axis to turn On \r\n");
                    pc.printf("F - Axis to turn Off (Default)\r\n");
                    pc.printf("\r\n");
                    pc.printf("P - Set Proportional Gain on an Axis\r\n");
                    wait(.05);
                    pc.printf("I - Set Integral Gain on an Axis\r\n");
                    pc.printf("D - Set Derivative Gain on an Axis\r\n");                
                    pc.printf("\r\n");
                    pc.printf("B - Pitch Gripper\r\n");
                    pc.printf("N - Rotate Gripper\r\n");
                    pc.printf("G - Go Home (after previous home)\r\n");
                    pc.printf("L - Not used\r\n");
                    wait(.05);
                    pc.printf("K - Invert motor direction\r\n");
                    pc.printf("Y - Master Home function\r\n");
                    pc.printf("M - Change output mode (counts, degrees, radians)\r\n");
                                        
                    pc.printf("' ' - Space to query Axis Controller. Returns $T,B,S,E,P,R,Status,AxisMoving\\r\\n\r\n");
                    pc.printf("$ - Set buffer read timeout (Ex. $.02\\r)\r\n");
                    pc.printf("@ - Toggle DEBUG flag\r\n");
                    wait(.05);
                    pc.printf("# - Self Test with motor currents and encoder direction\r\n");
                    pc.printf("%% - \r\n");
                    pc.printf("^ - Stream Flag: Enable/Disable Stream. Then '1' for On and '0' for Off\r\n");
                    pc.printf("< - Stream Output Active Flag: Enable/Disable Active Stream. Then '1' for On and '0' for Off\r\n");
                    wait(.05);
                    pc.printf("> - Change the streaming time. Default is 20 milliseconds (.02)\r\n");                    
                    pc.printf("* - Manually increment/decrement an axis and other controls (ex. disable gripper collision detect)\r\n");
                    pc.printf("| - Fast Motor Max Current find\r\n");
                    pc.printf("\r\n");
                    wait(.05);
                    pc.printf("{ - B,S,E,P,R\\r Instant robot joint move\r\n returns OK\\r\\n when finished. BSEPR depends on mode, all floats\r\n");
                    pc.printf("} - B,S,E,P,R,Gmm\\r Instant robot joint move with gripper\r\n returns OK\\r\\n when finished. BSEPR depends on mode, all floats\r\n");
                    pc.printf("! - dt,B,S,E,P,R,gmm\\r Trapezoidal profile move\r\n returns OK\\r\\n when finished. BSEPR depends on mode, all floats\r\n");
                    pc.printf("\r\n");
                    pc.printf("[ - PosCMDdtxx\\r1,B,S,E,P,R\\r2,B,S,E,P,R\\r3,B,S,E,P,R\\r** \\r\r\n");
                    pc.printf("] - VelCMDdtxx\\r1,B,S,E,P,R\\r2,B,S,E,P,R\\r3,B,S,E,P,R\\r** \\r\r\n");
                    pc.printf(" q - will quit, l - will loop\r\n\r\n");    
                    
                    pc.printf("Press any key to continue.\r\n");
                    wait(.05);
                    
                    while(!pc.readable());
                break;
                
                case '@':
                    pc.printf("\r\nEnter axis to debug (1-6), or 'o' for all on, 'f' for all off\r\n");        
                    pc.scanf("%c",&c);
                    
                   switch(c){
                        case '1':
                            axis1.debug = !axis1.debug;
                            if(axis1.debug)
                                pc.printf("\r\nAxis 1 Debug On\r\n");
                            else
                                pc.printf("\r\nAxis 1 Debug Off\r\n");
                        break; 
                        
                        case '2':
                            axis2.debug = !axis2.debug;
                            if(axis2.debug)
                                pc.printf("\r\nAxis 2 Debug On\r\n");
                            else
                                pc.printf("\r\nAxis 2 Debug Off\r\n");              
                        break;   
                        
                        case '3':
                            axis3.debug = !axis3.debug;
                            if(axis3.debug)
                                pc.printf("\r\nAxis 3 Debug On\r\n");
                            else
                                pc.printf("\r\nAxis 3 Debug Off\r\n");
                        break;
                        
                        case '4':
                            axis4.debug = !axis4.debug;
                            if(axis4.debug)
                                pc.printf("\r\nAxis 4 Debug On\r\n");
                            else
                                pc.printf("\r\nAxis 4 Debug Off\r\n");          
                        break;   
                        
                        case '5':
                            axis5.debug = !axis5.debug;
                            if(axis5.debug)
                                pc.printf("\r\nAxis 5 Debug On\r\n");
                            else
                                pc.printf("\r\nAxis 5 Debug Off\r\n");
                        break;
    
                        case '6':
                            axis6.debug = !axis6.debug;
                            if(axis6.debug)
                                pc.printf("\r\nAxis 6 Debug On\r\n");
                            else
                                pc.printf("\r\nAxis 6 Debug Off\r\n");
                        break;
                        
                        case 'o':
                            DEBUG = 1;
                            axis1.debug = 1;
                            axis2.debug = 1;
                            axis3.debug = 1;
                            axis4.debug = 1;
                            axis5.debug = 1;
                            axis6.debug = 1;
                            pc.printf("\r\nAll Axes Debug On\r\n");
                        break;

                        case 'f':
                            DEBUG = 0;
                            axis1.debug = 0;
                            axis2.debug = 0;
                            axis3.debug = 0;
                            axis4.debug = 0;
                            axis5.debug = 0;
                            axis6.debug = 0;
                            pc.printf("\r\nAll Axes Debug Off\r\n");
                        break;
                    }                                         
                break;
                
                case '#':
                    test_all_motors();
                break;
                
                case 'Y':
                case 'y':
                    home_master();
                break;
                
                case '0':   //zero all encoders and channels
                    zero_all();
                break;
                
                // All: Enable/Disable ALL motors (On or Off)
                case 'A':
                case 'a':
                    pc.printf("\r\nAll: 'o' for all On, 'f' for all off\r\n");
                    
                    pc.scanf("%c",&c);                
                    if((c == 'O' || c == 'o')){
                        all_on();
                        enMotDrv=0; //signal inverted from fet, makes MC33926 EN high (Active high logic)
                    }
                    if((c == 'F' || c == 'f')){
                        all_off();        
                        enMotDrv=1; //signal inverted from fet, makes EN low
                    }
                break;            
                
                case 'B':
                case 'b':
                    //zero if either motor is not homed
                    if((axis4.stat!=0) || (axis5.stat!=0)){
                        axis4.axisOn();
                        axis5.axisOn();                
                    }
                    pc.printf("\r\nEnter wrist pitch counts\r\n");
                                               
                    float counts;
                    
                    pc.scanf("%f",&counts);                
                    axis4.set_point += counts;
                    axis5.set_point -= counts;
                    
                    pc.printf("%f\r\n",counts);
                    pc.printf("Press q to quit \r\n");
                    t.reset();
                    while((axis4.pos > (axis4.set_point + SP_TOL) || 
                           axis4.pos < (axis4.set_point - SP_TOL)) &&
                           (axis5.pos > (axis5.set_point + SP_TOL) ||
                           axis5.pos < (axis5.set_point - SP_TOL))){
                        pc.printf("T=%.2f SP4=%.3f pos4=%.3f SP5=%.3f pos5=%.3f \r\n", t.read(), axis4.set_point, axis4.pos, axis5.set_point, axis5.pos); 
                        wait(.009);
                        
                        if(pc.readable()){      //if user types a 'q' or 'Q'
                            char c = pc.getc();
                            if(c == 'q' || c == 'Q') //quit after current movement
                                break;
                        }
                    }                  
                break;
                
                //wrist rotate
                case 'N':
                case 'n':
                    //zero if either motor is not homed
                    if((axis4.stat!=0) || (axis5.stat!=0)){
                        axis4.axisOn();
                        axis5.axisOn();                
                    }
                    pc.printf("\r\nEnter wrist rotate counts\r\n");                                                    
                    
                    pc.scanf("%f",&counts);                
                    axis4.set_point += counts;
                    axis5.set_point += counts;
                    
                    pc.printf("%f\r\n",counts);
                    pc.printf("Press q to quit \r\n");
                    t.reset();
                    while((axis4.pos > (axis4.set_point + SP_TOL) || 
                           axis4.pos < (axis4.set_point - SP_TOL)) &&
                           (axis5.pos > (axis5.set_point + SP_TOL) ||
                           axis5.pos < (axis5.set_point - SP_TOL))){
                        pc.printf("T=%.2f SP=%.3f co=%.3f pos=%.3f vel=%.3f acc=%.3f       \r\n", t.read(), axis4.set_point, axis4.enc, axis5.set_point, axis5.enc); 
                        wait(.009);
                        
                        if(pc.readable()){      //if user types a 'q' or 'Q'
                            char c = pc.getc();
                            if(c == 'q' || c == 'Q') //quit after current movement
                                break;
                        }
                    }  
                break;    
                
                //Current Measurement: Stream the motor currents
                case 'C':
                case 'c':
                
                    pc.printf("\r\nPress 's' to stream, 'm' for max, 'c' to clear max, 'w' for write\r\n");
                    pc.scanf("%c",&c);
                    
                    if(c == 's'){
                        //turn OFF the collision check
                        overCurrentTick.detach(); 
                        int analogFlag = 0;
                        pc.printf("\r\nPress q to quit \r\n");
                        while(analogFlag == 0){
                            axis1.readCurrent();
                            axis2.readCurrent();
                            axis3.readCurrent();
                            axis4.readCurrent();
                            axis5.readCurrent();
                            axis6.readCurrent();
                            pc.printf("%.3f %.3f %.3f %.3f %.3f %.3f \r\n",axis1.motI,axis2.motI,axis3.motI,axis4.motI,axis5.motI,axis6.motI);
                            wait(.02);
                            if(pc.readable()){      //if user types a 'q' or 'Q'
                                char c = pc.getc();
                                if(c == 'q' || c == 'Q') //quit after current movement
                                    analogFlag = 1;
                            }
                        }
                        //turn collision check interrupt back ON
                        overCurrentTick.attach(&collisionCheck, CURRENT_CHECK_PERIOD); 
                    }
                    else if(c == 'm'){
                        print_motMaxI();
                    }
                    else if(c == 'c'){
                        clear_motMaxI();
                    }
                    else if(c == 'w'){                        
                        char tempBuf[32];
                        char data_read[32];
                        
                        pc.printf("\r\nCurrent Max I is:\r\n");
                        print_motMaxI();
                        pc.printf("Enter axis (1-6) to set stall current:\r\n");
                        pc.scanf("%c",&c);
                        pc.printf("Axis %c selected\r\n", c);
                        
                        float currentMax;
                        pc.printf("Enter current to write:\r\n");
                        pc.scanf("%f",&currentMax);
                        pc.printf("Stall current of %.3f selected\r\n", currentMax);
                        
                        
                        sprintf(tempBuf, "%f\0", currentMax);
                        
                        switch(c){
                            case '1':                        
                                pointerAdddress = eprom_mot1maxI;
                                writeEEPROM(EPROMaddress, pointerAdddress, tempBuf, BLOCK_SIZE);
                                wait(.01);
                                readEEPROM(EPROMaddress, eprom_mot1maxI, data_read, BLOCK_SIZE);
                                currentMax = atof(data_read);
                                pc.printf("Axis%c is now set to Stall Current of %.3f\r\n", c, currentMax);
                                currentMax *= MAX_CURRENT_SCALE1;
                                pc.printf("Will trip at %.3f\r\n", currentMax);
                                break;
                            case '2':                        
                                pointerAdddress = eprom_mot2maxI;
                                writeEEPROM(EPROMaddress, pointerAdddress, tempBuf, BLOCK_SIZE);
                                wait(.01);
                                readEEPROM(EPROMaddress, eprom_mot2maxI, data_read, BLOCK_SIZE);
                                currentMax = atof(data_read);
                                pc.printf("Axis%c is now set to Stall Current of %.3f\r\n", c, currentMax);
                                currentMax *= MAX_CURRENT_SCALE2;
                                pc.printf("Will trip at %.3f\r\n", currentMax);                                
                                break;
                            case '3':                        
                                pointerAdddress = eprom_mot3maxI;
                                writeEEPROM(EPROMaddress, pointerAdddress, tempBuf, BLOCK_SIZE);
                                wait(.01);
                                readEEPROM(EPROMaddress, eprom_mot3maxI, data_read, BLOCK_SIZE);
                                currentMax = atof(data_read);
                                pc.printf("Axis%c is now set to Stall Current of %.3f\r\n", c, currentMax);
                                currentMax *= MAX_CURRENT_SCALE3;
                                pc.printf("Will trip at %.3f\r\n", currentMax);                                
                                break;
                            case '4':                        
                                pointerAdddress = eprom_mot4maxI;
                                writeEEPROM(EPROMaddress, pointerAdddress, tempBuf, BLOCK_SIZE);
                                wait(.01);
                                readEEPROM(EPROMaddress, eprom_mot4maxI, data_read, BLOCK_SIZE);
                                currentMax = atof(data_read);
                                pc.printf("Axis%c is now set to Stall Current of %.3f\r\n", c, currentMax);
                                currentMax *= MAX_CURRENT_SCALE4;
                                pc.printf("Will trip at %.3f\r\n", currentMax);                                
                                break;
                            case '5':                        
                                pointerAdddress = eprom_mot5maxI;
                                writeEEPROM(EPROMaddress, pointerAdddress, tempBuf, BLOCK_SIZE);
                                wait(.01);
                                readEEPROM(EPROMaddress, eprom_mot5maxI, data_read, BLOCK_SIZE);
                                currentMax = atof(data_read);
                                pc.printf("Axis%c is now set to Stall Current of %.3f\r\n", c, currentMax);
                                currentMax *= MAX_CURRENT_SCALE5;
                                pc.printf("Will trip at %.3f\r\n", currentMax);                                
                                break;
                            case '6':                        
                                pointerAdddress = eprom_mot6maxI;
                                writeEEPROM(EPROMaddress, pointerAdddress, tempBuf, BLOCK_SIZE);
                                wait(.01);
                                readEEPROM(EPROMaddress, eprom_mot6maxI, data_read, BLOCK_SIZE);
                                currentMax = atof(data_read);
                                pc.printf("Axis%c is now set to Stall Current of %.3f\r\n", c, currentMax);
                                currentMax *= MAX_CURRENT_SCALE6;
                                pc.printf("Will trip at %.3f\r\n", currentMax);                                
                                break;
                            default:
                                pc.printf("\r\nIncorrect Axis selected. Aborting.\r\n");
                                wait(3);
                                break;
                        }
                    }
                    else{
                        pc.printf("Wrong command given!");
                        wait(1);
                    }                    
                break;
                
                //Limit: Limit Switch Monitor
                case 'L':
                case 'l':
                    int limitFlag = 1;
                    
                    while(limitFlag){                     
                        pc.printf("\r\n%1d %1d %1d %1d %1d %1d\r\n", limit1, limit2, limit3, limit4, limit5, limit6);
                        wait(.02);
                        
                        if(pc.readable()){      //if user types a 'q' or 'Q'
                            char c = pc.getc();
                            if(c == 'q' || c == 'Q') //quit after current movement
                                limitFlag = 0;
                        }
                    }
                break;    
    
                //W: write value to axis                  
                case 'W':
                case 'w':
                    float position = 0.0;
                    
                    pc.printf("\r\nEnter axis to change encoder value\r\n");        
                    pc.scanf("%c",&c);
                    
                    pc.printf("\r\n\r\nEnter new encoder value:");        
                    pc.scanf("%f",&position);
                    pc.printf("%f\r\n", position); 
                                              
                    switch(c){
                        case '1':
                            pc.printf("updating Robotic Axis 1 value to %.1f\r\n", position);   
                            axis1.writeEncoderValue((long)position);       
                        break; 
                        
                        case '2':
                            pc.printf("updating Robotic Axis 2 value to %.1f\r\n", position);   
                            axis2.writeEncoderValue((long)position);                                        
                        break;   
                        
                        case '3':
                            pc.printf("updating Robotic Axis 3 value to %.1f\r\n", position);   
                            axis3.writeEncoderValue((long)position);            
                        break;
                        
                        case '4':
                            pc.printf("updating Robotic Axis 4 value to %.1f\r\n", position);   
                            axis4.writeEncoderValue((long)position);              
                        break;   
                        
                        case '5':
                            pc.printf("updating Robotic Axis 5 value to %.1f\r\n", position);   
                            axis5.writeEncoderValue((long)position);             
                        break;
    
                        case '6':
                            pc.printf("updating Robotic Axis 6 value to %.1f\r\n", position);   
                            axis6.writeEncoderValue((long)position);               
                        break;
                    }                
                break;
                
                //Go Home (after previous home position
                case 'G':
                case 'g':
                    char old_dev_mode_Global = dev_mode_Global;
                    dev_mode_Global='d';
                    wait(.1);
                    dt=8.0;
                    theta2enc_dt(dt,0.0,120.28,-95.02,-88.81,0.00, 0.0);                    
                    //wait for move to complete or receive a 'q' or ' '
                    moveDone_or_qs(dt+(dt*.5));
                        
                    pc.printf("OK\r\n");
                    dev_mode_Global=old_dev_mode_Global;
                break;        
                                      
                //Exercise: Pre-programmed gate to excercise robot
                case 'X':
                case 'x':    //Exercise all axes     
                    char tempBufnew[16];
                    sprintf(tempBufnew, "EXERCISE\0");
                    writeEEPROM(EPROMaddress, eprom_mode, tempBufnew, 16);
                    
                    exerciseAllAxes();
                    
                    sprintf(tempBufnew, "MANUAL\0");
                    writeEEPROM(EPROMaddress, eprom_mode, tempBufnew, 16);                    
                break;
                
                //Trapazoid: move trapazoidal profile on Axis
                case 'T':
                case 't':    //move Trapazoid command
                
                    if(dev_mode_Global != 'c'){
                        pc.printf("\r\nMust be in counts mode to move a single axis\r\n");
                        break;
                    }
                    else{
                        pc.printf("\r\nEnter axis to move trapazoid\r\n");        
                        pc.scanf("%c",&c);
                        
                        pc.printf("\r\n\r\nEnter position:");
                        pc.scanf("%f",&position);
                        pc.printf("%f\r\n", position); 
                        
                        pc.printf("Enter time:");        
                        pc.scanf("%f",&time);
                        pc.printf("%f\r\n", time);
                                                  
                        switch(c){
                            case '1':
                                pc.printf("Moving Robotic Axis 1\r\n");        
                                axis1.moveTrapezoid(position, time);       
                            break; 
                            
                            case '2':
                                pc.printf("Moving Robotic Axis 2\r\n");        
                                axis2.moveTrapezoid(position, time);                                   
                            break;   
                            
                            case '3':
                                pc.printf("Moving Robotic Axis 3\r\n");        
                                axis3.moveTrapezoid(position, time);           
                            break;
                            
                            case '4':
                                pc.printf("Moving Robotic Axis 4\r\n");        
                                axis4.moveTrapezoid(position, time);           
                            break;   
                            
                            case '5':
                                pc.printf("Moving Robotic Axis 5\r\n");        
                                axis5.moveTrapezoid(position, time);           
                            break;
        
                            case '6':
                                pc.printf("Moving Robotic Axis 6\r\n");        
                                axis6.moveTrapezoid(position, time);           
                            break;
                            
                            case 'b':
                                pc.printf("Moving Base of Scorbot\r\n");        
                                //scorbotBase.axis.moveTrapezoid(position, time);
                                axis1.moveTrapezoid(position, time);       
                            break;
                        }
                    }
                break;   
                
                //Home: home command 
                case 'H':
                case 'h':
                    char test;
                    pc.printf("\r\nEnter axis to home (1-6), b-base, s-shoulder, e-elbow, p-wrist pitch, r-wrist rotate, g-gripper, \r\n\t a for all, d-adjust homing delay\r\n");        
                    pc.scanf("%c",&test);          
                    switch(test){
                        case '1':
                            pc.printf("Homing Robotic Axis 1\r\n");
                            axis1.axisOn();
                            axis1.center();
                        break;
                        
                        case '2':
                            pc.printf("Homing Robotic Axis 2\r\n");
                            axis2.axisOn();
                            axis2.center();         
                        break;   
                        
                        case '3':
                            pc.printf("Homing Robotic Axis 3\r\n");
                            axis3.axisOn();
                            axis3.center();
                        break;
                        
                        case '4':
                            pc.printf("Homing Robotic Axis 4\r\n");
                            axis4.axisOn();
                            axis4.center();
                        break;
                        
                        case '5':
                            pc.printf("Homing Robotic Axis 5\r\n");
                            axis5.axisOn();
                            axis5.center();
                        break;
                        
                        case '6':
                            pc.printf("Homing Robotic Axis 6\r\n");
                            axis6.axisOn();
                            axis6.center();
                        break;
      
                        case 'b':
                            pc.printf("Homing base\r\n");
                            if(home_base()==0){
                                pc.printf("Home base success\r\n");
                            }
                            else{
                                pc.printf("Home base failed!\r\n");
                            }
                        break;
                                          
                        case 'e':
                            pc.printf("Homing Elbow\r\n");
                            home_elbow();
                        break;
    
                        case 's':
                            pc.printf("Homing Shoulder\r\n");
                            home_shoulder();
                        break;
                        
                        case 'r':
                            pc.printf("Homing Wrist Rotate\r\n");
                            home_rotate_wrist();
                        break;
                        
                        case 'p':
                            pc.printf("Homing Wrist Pitch\r\n");
                            home_pitch_wrist();
                        break;
                        
                        case 'g':
                            pc.printf("Homing Gripper\r\n");
                            home_gripper();
                        break;
                                          
                        case 'a':
                            int err = home_master();
                            if(err != 0){
                                all_off();
                                pc.printf("\r\n\r\nNAK\r\nHOME FAILED!!!\r\n");
                                if(DEBUG)
                                    pc.printf("Master Home returned %d\r\n", err);                                                                
                            }              
                            else         
                                pc.printf("OK\r\n");
                        break;
                            
                        case 'd':
                            float newGlobalTime;
                            
                            pc.printf("\r\nEnter new global delay time for home functions. Currently %f\r\n", homeDelayGlobal);
                            pc.scanf("%f",&newGlobalTime);
                            pc.printf("homeDelayGlobal delay is now %.3f\r\n", newGlobalTime);
                            homeDelayGlobal=newGlobalTime;
                        break;
                        
                        default:
                            pc.printf("\r\nInvalid parameter entered\r\n");
                            break;
                    }
                    while(pc.readable())
                        char c = pc.getc();                            
                break;
                
                /*
                case 0x5b:  //F1 key
                    while(pc.readable()){
                        char c = pc.getc();
                    }
                    pc.printf("\r\nYou have pressed the sacred F1 key.  Please do not press this button again.\r\n");
                    while(!pc.readable());
                break;
                */
                
                case '|':                
                    motCurrentProfileUpdate();
                break;
                
                //Set Point:  Manually move to specific encoder position set point
                case 'S':
                case 's':
                    int axisSp;
                    pc.printf("\r\nEnter axis (1-6) to give set point:");
                    pc.scanf("%d",&axisSp);
                    pc.printf("Axis %d entered.\r\n", axisSp);
                    pc.printf("Enter value for set point axis %c\r\n", c);
                    float temp_setpoint;
                    pc.scanf("%f",&temp_setpoint);
                    pc.printf("Axis%d set point %.1f\r\n", axisSp, temp_setpoint);
                     
                    switch(axisSp){    
                        case 1:
                            setPointRun(axis1, temp_setpoint);  //execute position command
                            break;   
                        
                        case 2:
                            setPointRun(axis2, temp_setpoint);  //execute position command
                            break;            
                        
                        case 3:
                            setPointRun(axis3, temp_setpoint);
                            break;                             
    
                        case 4:
                            setPointRun(axis4, temp_setpoint);
                            break;
                        
                        case 5:
                            setPointRun(axis5, temp_setpoint);
                            break;                    
    
                        case 6:
                            setPointRun(axis6, temp_setpoint);
                            break;
                        
                        default:
                            pc.printf("Wrong Axis chosen!\r\n");
                            wait(2.0);
                            break;
                    }
                break;
                
                case 'V':
                case 'v':
                    float vb, vs, ve, vp, vr, vg;   //velocities in rad/sec of robot arm joints
                    float timeout = 3.0;             //default to 3 seconds for velocity command timeout
                    if(DEBUG)
                        pc.printf("\r\nEnter Timeout, velocity:b,s,e,p,r in Rad/sec,g in mmSec\r\n");
                        
                    int paramScanned = pc.scanf("%f,%f,%f,%f,%f,%f,%f",&timeout,&vb,&vs,&ve,&vp,&vr,&vg);
                    if(paramScanned == 7){
                        if(DEBUG)
                            pc.printf("\r\nT%.3f,Vb%.3f,s%.3f,e%.3f,p%.3f,r%.3f,g%.3f\r\n",timeout,vb,vs,ve,vp,vr,vg);
                        
                        //10 sec timout max
                        if(timeout > 10.0)
                            timeout = 10.0;
                        if(timeout < 0.0)
                            timeout = 0.0;
                                                        
                        //max/min rad/sec speed at 12V
                        //approximately .42 rad/sec max/min for base
                        if(vb > .4)
                            vb = .4;
                        if(vb < -.4)
                            vb = -.4;
                        
                        //shoulder rad/sec max/min    
                        if(vs > .5)
                            vs = .5;
                        if(vs < -.5)
                            vs = -.5;
                            
                        //elbow rad/sec max/min    
                        if(ve > .5)
                            ve = .5;
                        if(ve < -.5)
                            ve = -.5;
                            
                        //pitch rad/sec max/min    
                        if(vp > 1.8)
                            vp = 1.8;
                        if(vp < -1.8)
                            vp = -1.8;
                        
                        //roll rad/sec max/min    
                        if(vr > 1.8)
                            vr = 1.8;
                        if(vr < -1.8)
                            vr = -1.8;
                            
                        //gripper mm/sec max/min    
                        if(vg > 12.0)
                            vg = 12.0;
                        if(vg < -12.0)
                            vg = -12.0;
                                                        
                        velocityCmd(timeout,vb,vs,ve,vp,vr,vg);
                        pc.printf("OK\r\n");                                 
                    }
                    else{
                        pc.printf("\r\n Incorrect number of parameters scanned!!\r\n");
                        wait(3);
                    }             
                break;
                
                case 'P':
                case 'p':
                    float temp_Pk;
                    pc.printf("\r\nEnter axis to set Pk\r\n");
                    char axis_sel;
                    pc.scanf("%c",&axis_sel);
                                    
                    pc.printf("Current value is %7.2f\r\n", read_Pk(axis_sel));
                    pc.printf("Enter value for Axis%c Pk\r\n", axis_sel);
                    pc.scanf("%f",&temp_Pk);                
                    
                    switch(axis_sel){
                        case '1':
                            axis1.Pk = temp_Pk;
                            axis1.updatePIDgains(axis1.Pk, axis1.Ik, axis1.Dk); //turns on controller     
                        break;
                        
                        case '2':
                            axis2.Pk = temp_Pk;
                            axis2.updatePIDgains(axis2.Pk, axis2.Ik, axis2.Dk); //turns on controller     
                        break;
                        
                        case '3':
                            axis3.Pk = temp_Pk;
                            axis3.updatePIDgains(axis3.Pk, axis3.Ik, axis3.Dk); //turns on controller                             
                        break;
                        
                        case '4':
                            axis4.Pk = temp_Pk;
                            axis4.updatePIDgains(axis4.Pk, axis4.Ik, axis4.Dk); //turns on controller                             
                        break;
                        
                        case '5':
                            axis5.Pk = temp_Pk;
                            axis5.updatePIDgains(axis5.Pk, axis5.Ik, axis5.Dk); //turns on controller     
                        break;
                        
                        case '6':
                            axis6.Pk = temp_Pk;
                            axis6.updatePIDgains(axis6.Pk, axis6.Ik, axis6.Dk); //turns on controller                             
                        break;
                        
                        default:
                            pc.printf("\r\nInvalid axis selected\r\n");
                            break;                       
                    }
                    pc.printf("Axis%c Pk set to %f\r\nPress any key to exit\r\n", axis_sel, read_Pk(axis_sel));
                    
                    while(!pc.readable());
                break;
                
                case 'I':
                case 'i':
                    float temp_Ik;
                    pc.printf("\r\nEnter axis to set Ik\r\n");       
                    pc.scanf("%c",&axis_sel);
                    
                    pc.printf("Current value is %7.2f\r\n", read_Ik(axis_sel));
                    
                    pc.printf("Enter value for Axis%c Ik\r\n", axis_sel);
                    pc.scanf("%f",&temp_Ik);                
                    
                    switch(axis_sel){
                        case '1':
                            axis1.Ik = temp_Ik;
                            axis1.updatePIDgains(axis1.Pk, axis1.Ik, axis1.Dk); //turns on controller              
                        break;
                        
                        case '2':
                            axis2.Ik = temp_Ik;
                            axis2.updatePIDgains(axis2.Pk, axis2.Ik, axis2.Dk); //turns on controller
                        break;
                        
                        case '3':
                            axis3.Ik = temp_Ik;
                            axis3.pid->setTunings(axis3.Pk, axis3.Ik, axis3.Dk); //turns on controller
                        break;
                        
                        case '4':
                            axis4.Ik = temp_Ik;
                            axis4.updatePIDgains(axis4.Pk, axis4.Ik, axis4.Dk); //turns on controller                        
                        break;
                        
                        case '5':
                            axis5.Ik = temp_Ik;
                            axis5.updatePIDgains(axis5.Pk, axis5.Ik, axis5.Dk); //turns on controller                        
                        break;
                        
                        case '6':
                            axis6.Ik = temp_Ik;
                            axis6.updatePIDgains(axis6.Pk, axis6.Ik, axis6.Dk); //turns on controller
                        break;
                        
                        default:
                            pc.printf("\r\nInvalid axis selected\r\n");
                            break;                    
                    }
                    pc.printf("Axis%d Ik set to %f\r\nPress any key to exit\r\n", axis_sel, read_Ik(axis_sel));
                    while(!pc.readable());
                break;
                
                case 'D':
                case 'd':
                    float temp_Dk;
                    pc.printf("\r\nEnter axis to set Dk\r\n");                           
                    pc.scanf("%c",&axis_sel);            
                    
                    pc.printf("Current value is %7.2f\r\n", read_Dk(axis_sel));
                    
                    pc.printf("Enter value for Axis%c Dk\r\n", axis_sel);
                    pc.scanf("%f",&temp_Dk);                
                    
                    switch(axis_sel){
                        case '1':
                            axis1.Dk = temp_Dk;
                            axis1.updatePIDgains(axis1.Pk, axis1.Ik, axis1.Dk); //turns on controller
                        break;
                        
                        case '2':
                            axis2.Dk = temp_Dk;
                            axis2.updatePIDgains(axis2.Pk, axis2.Ik, axis2.Dk); //turns on controller
                        break;
                        
                        case '3':
                            axis3.Dk = temp_Dk;
                            axis3.updatePIDgains(axis3.Pk, axis3.Ik, axis3.Dk); //turns on controller
                        break;
                        
                        case '4':
                            axis4.Dk = temp_Dk;
                            axis4.updatePIDgains(axis4.Pk, axis4.Ik, axis4.Dk); //turns on controller
                        break;
                        
                        case '5':
                            axis5.Dk = temp_Dk;
                            axis5.updatePIDgains(axis5.Pk, axis5.Ik, axis5.Dk); //turns on controller
                        break;
                        
                        case '6':
                            axis6.Dk = temp_Dk;
                            axis6.updatePIDgains(axis6.Pk, axis6.Ik, axis6.Dk); //turns on controller
                        break;
                        
                        default:
                            pc.printf("\r\nInvalid axis selected\r\n");
                            break;
                    }
                    pc.printf("Axis%c Dk set to %f\r\nPress any key to exit\r\n", axis_sel, read_Dk(axis_sel));
                    while(!pc.readable());
                break;     
                
                //Zero: Zero specific axis
                case 'Z':
                case 'z':
                    pc.printf("\r\nEnter axis to Zero (1-6, or 'a' for all)\r\n");          
                    pc.scanf("%c",&c);          
                    switch(c){                                        
                        case '1':
                            axis1.zero();
                        break;
    
                        case '2':
                            axis2.zero();
                        break;
                        
                        case '3':
                            axis3.zero();
                        break;
                        
                        case '4':
                            axis4.zero();
                        break;
                        
                        case '5':
                            axis5.zero();
                        break;
                        
                        case '6':
                            axis6.zero();
                        break; 
                                                                                                                           
                        case 'a':   //for all
                            axis1.zero();
                            axis2.zero();
                            axis3.zero();
                            axis4.zero();
                            axis5.zero();                                                                                                                    
                            axis6.zero();
                        break; 
                    }                   
                break; 
                       
                case 'O':
                case 'o':
                    pc.printf("\r\nEnter axis to turn On (1-6, or 'a' for all)\r\n");     
                    pc.scanf("%c",&c);               
                    
                    switch(c){
                        case '1':
                            axis1.axisOn();
                        break;
    
                        case '2':
                            axis2.axisOn();
                        break;
    
                        case '3':
                            axis3.axisOn();
                        break;
                        
                        case '4':
                            axis4.axisOn();
                        break;
    
                        case '5':
                            axis5.axisOn();
                        break;
    
                        case '6':
                            axis6.axisOn();
                        break;                   
                        
                        case 'a':
                            axis1.axisOn();
                            axis2.axisOn();
                            axis3.axisOn();
                            axis4.axisOn();
                            axis5.axisOn();
                            axis6.axisOn();
                        break;                        
                    }
                break;
                
                case 'F':
                case 'f':
                    pc.printf("\r\nEnter axis to turn Off (1-6, or 'a' for all)\r\n");        
                    pc.scanf("%c",&c);               
                    
                    switch(c){
                        case '1':
                            axis1.axisOff();
                        break;
    
                        case '2':
                            axis2.axisOff();
                        break;
    
                        case '3':
                            axis3.axisOff();
                        break;
                        
                        case '4':
                            axis4.axisOff();
                        break;
    
                        case '5':
                            axis5.axisOff();
                        break;
    
                        case '6':
                            axis6.axisOff();
                        break;                  
                        
                        case 'a':
                            axis1.axisOff();
                            axis2.axisOff();
                            axis3.axisOff();
                            axis4.axisOff();
                            axis5.axisOff();
                            axis6.axisOff();
                        break;                          
                    }
                    pc.printf("Axis%c Off\r\n", c);
                break;    
                
                //Invert Axis Command
                case 'k':
                case 'K':
                    int inv_val;
                    pc.printf("\r\nEnter axis to Invert motor direction\r\n");        
                    pc.scanf("%c",&c);
                    
                    pc.printf("Enter value for inversion\r\n", c);
                    pc.scanf("%d",&inv_val); 
                    
                    switch(c){
                        case '1':
                            axis1.motInvert = inv_val;
                        break;
                        
                        case '2':
                            axis2.motInvert = inv_val;
                        break;
                        
                        case '3':
                            axis3.motInvert = inv_val;
                        break;
                        
                        case '4':
                            axis4.motInvert = inv_val;
                        break;
                        
                        case '5':
                            axis5.motInvert = inv_val;
                        break;
                        
                        case '6':
                            axis6.motInvert = inv_val;
                        break;
                    }
                    pc.printf("Axis%c motor direction inversion set to %d\r\n", c, axis1.motInvert);                                                        
                break;                
                
                // Toggle Stream flag (for display purposes)
                case '^':
                    pc.printf("\r\nEnter 1 to turn stream On, 0 to turn Off:\r\n");
                    pc.scanf("%c",&c);
                    
                    if(c == '1'){
                        streamFlag = 1;
                        pc.printf("Stream On\r\n");
                    }
                    if(c == '0'){
                        streamFlag = 0;
                        pc.printf("Stream Off\r\n");
                    }
                break;
                
                // Toggle Stream streamOutputActive Flag (for display purposes)
                case '<':
                    pc.printf("\r\nEnter 1 to turn stream Output Active On, 0 to turn Off (currently %d):\r\n", streamOutputActive_Flag);
                    pc.scanf("%c",&c);
                    
                    if(c == '1'){
                        streamOutputActive_Flag = 1;
                        pc.printf("Stream Output Active On\r\n");
                    }
                    if(c == '0'){
                        streamOutputActive_Flag = 0;
                        pc.printf("Stream Output Active Off\r\n");
                    }
                break;
                
                //Stream rate change
                case '>':
                    pc.printf("\r\nTime period between samples to stream (default is .02 seconds):\r\n");
                    pc.scanf("%f",&streamRate);
                    if(streamRate > 60.0)
                        streamRate=60.0;
                    if(streamRate < .001)
                        streamRate = .001;
                break;
                            
                // Toggle the human or PC flag
                case '$':
                    if(DEBUG)
                        pc.printf("\r\nEnter serial read timeout\r\n");
                    pc.scanf("%f",&serReadTimeout);
                    
                    if(serReadTimeout > 10.0)
                        serReadTimeout=10.0;
                    if(serReadTimeout < .001)
                        serReadTimeout=.001;
                break;
                
                //Excercise the Joint degree/radian functions  
                case 'J': 
                case 'j':                    
                    pc.printf("\r\nEnter joint to excercise: \r\n b-base\r\n s-shoulder\r\n e-elbow\r\n p-pitch\r\n r-roll\r\n q - quit\r\n");
                    pc.scanf("%c",&joint);
                    
                    switch(joint){
                        float degrees;
                        float radians;
                        char rad_deg;
                        
                        case 'b':
                            pc.printf("Base selected\r\n");
                            pc.printf("degrees or radians (d or r)?\r\n");
                            pc.scanf("%c", &rad_deg);
                            pc.printf("%c\r\n", rad_deg);
                            
                            if(rad_deg=='d'){                        
                                pc.printf("Enter degrees to turn base:\r\n");
                                pc.scanf("%f",&degrees);
                                pc.printf("Degrees entered:%.2f\r\nPress q to exit\r\n", degrees);
                                sbBase.set_deg(degrees);
                            }
                            if(rad_deg=='r'){                        
                                pc.printf("Enter radians to turn base:\r\n");
                                pc.scanf("%f",&radians);
                                pc.printf("Radians entered:%.2f\r\nPress q to exit\r\n", radians);
                                sbBase.set_rad(radians);
                            }
                            
                            while(limitFlag){
                                if(rad_deg=='d')
                                    pc.printf("base = %4.2f deg    \r\n", sbBase.get_deg());
                                else if(rad_deg=='r')
                                    pc.printf("base = %1.4f rad    \r\n", sbBase.get_rad());
                                
                                wait(.02);
                                
                                if(pc.readable()){      //if user types a 'q' or 'Q'
                                    char c = pc.getc();
                                    if(c == 'q' || c == 'Q') //quit after current movement
                                        limitFlag = 0;;
                                }
                            }                                                
                            break;
    
                        case 's':
                            pc.printf("Shoulder selected\r\n");
                            pc.printf("degrees or radians (d or r)?\r\n");
                            pc.scanf("%c", &rad_deg);
                            pc.printf("%c\r\n", rad_deg);
                            
                            if(rad_deg=='d'){                        
                                pc.printf("Enter degrees to turn shoulder (home=120.28 deg):\r\n");
                                pc.scanf("%f",&degrees);
                                pc.printf("%f\r\n", degrees);
                                
                                sbShoulder.set_deg(degrees);
                            }
                            if(rad_deg=='r'){                        
                                pc.printf("Enter radians to turn shoulder:\r\n");
                                pc.scanf("%f",&radians);
                                pc.printf("%f\r\n", radians);
                            }                
                        
                        while(limitFlag){                     
                            pc.printf("Shoulder = %.2f \r", sbShoulder.get_deg());
                            wait(.02);
                            
                            if(pc.readable()){      //if user types a 'q' or 'Q'
                                char c = pc.getc();
                                if(c == 'q' || c == 'Q') //quit after current movement
                                    limitFlag = 0;;
                            }
                        }                                                
                        break;
    
                        case 'e':
                            pc.printf("Elbow selected\r\n");
                            pc.printf("degrees or radians (d or r)?\r\n");
                            pc.scanf("%c", &rad_deg);
                            pc.printf("%c\r\n", rad_deg);
                            
                            if(rad_deg=='d'){                        
                                pc.printf("Enter degrees to turn Elbow (home=95.02 deg):\r\n");
                                pc.scanf("%f",&degrees);
                                pc.printf("%f\r\n", degrees);
                                
                                sbElbow.set_deg(degrees);
                            }
                            if(rad_deg=='r'){                        
                                pc.printf("Enter radians to turn Elbow:\r\n");
                                pc.scanf("%f",&radians);
                                pc.printf("%f\r\n", radians);
                            }  
                        
                        while(limitFlag){
                            pc.printf("Elbow = %.2f \r", sbElbow.get_deg());
                            wait(.02);
                            
                            if(pc.readable()){      //if user types a 'q' or 'Q'
                                char c = pc.getc();
                                if(c == 'q' || c == 'Q') //quit after current movement
                                    limitFlag = 0;;
                            }
                        }                                                
                        break;
                        
                        //select pitch
                        case 'p':
                            pc.printf("Pitch selected\r\n");
                            pc.printf("degrees or radians (d or r)?\r\n");
                            pc.scanf("%c", &rad_deg);
                            pc.printf("%c\r\n", rad_deg);
                            
                            if(rad_deg=='d'){                        
                                pc.printf("Enter degrees to pitch (home=-88.81 deg):");
                                pc.scanf("%f",&degrees);
                                pc.printf("%7.3f\r\n", degrees);
                                
                                sbPitch.set_deg(degrees);
                            }
                            if(rad_deg=='r'){                        
                                pc.printf("Enter radians to Pitch:\r\n");
                                pc.scanf("%f",&radians);
                            }  
                        
                        while(limitFlag){
                            pc.printf("Pitch = %.2f \r", sbPitch.get_deg());
                            wait(.02);
                            
                            if(pc.readable()){      //if user types a 'q' or 'Q'
                                char c = pc.getc();
                                if(c == 'q' || c == 'Q') //quit after current movement
                                    limitFlag = 0;;
                            }
                        }                                                
                        break;                    
    
                        //select roll                                        
                        case 'r':
                            pc.printf("Roll selected\r\n");
                            pc.printf("degrees or radians (d or r)?\r\n");
                            pc.scanf("%c", &rad_deg);
                            pc.printf("%c\r\n", rad_deg);
                            
                            if(rad_deg=='d'){                        
                                pc.printf("Enter degrees to roll (home=0.0 deg):\r\n");
                                pc.scanf("%f",&degrees);
                                pc.printf("%7.3f\r\n", degrees);
                                
                                sbRoll.set_deg(degrees);
                            }
                            if(rad_deg=='r'){                        
                                pc.printf("Enter radians to Roll:\r\n");
                                pc.scanf("%f",&radians);
                            }  
                        
                            while(limitFlag){
                                pc.printf("Roll = %.2f \r", sbRoll.get_deg());
                                wait(.02);
                                
                                if(pc.readable()){      //if user types a 'q' or 'Q'
                                    char c = pc.getc();
                                    if(c == 'q' || c == 'Q') //quit after current movement
                                        limitFlag = 0;;
                                }
                            }                                                
                            break;       
                                                              
                        default:
                            pc.printf("Incorrect joint entered.\r\n");
                            break;    
                            
                    }//switch
                break;
    
                //change mode
                case 'M':
                case 'm':
                    char mode_sel;
                    pc.printf("\r\nEnter Device Mode (c=counts, d=degrees, r=radians)?\r\n");
                    pc.scanf("%c", &mode_sel);
                    pc.printf("%c\r\n", mode_sel);
                    
                    switch(mode_sel){
                        case  'd':
                            if(check_stats() == 0){
                                dev_mode_Global = 'd';
                            }
                            else{
                                pc.printf("\r\nError changing to deg mode!\r\n");                                            
                            }
                            break;
                        case 'c':
                            dev_mode_Global = 'c';
                            break;
                            
                        case 'r':
                            if(check_stats() == 0){
                                dev_mode_Global = 'r';
                            }
                            else{
                                pc.printf("\r\nError changing to rad mode!\r\n");                                            
                            }
                            break;
                            
                        default:
                            dev_mode_Global = 'c';
                            break;
                    } //switch mode_sel                                  
                break; //change mode                                   
                
                //Excercise the conversion functions    
                case '%':
                    pc.printf("\r\nPress q to quit\r\n");                    
                    while(qFlag==0){
                        //;create random joint angles
                        float rand_time = float(rand() % 70)/10.0 + 2.0;
                        float rand_base = (rand() % 45) - 22.25;
                        float rand_shoulder = (rand() % 100) + 20.0;
                        float rand_elbow = (rand() % 85) - 32.25;
                        float rand_pitch = (rand() % 100) - 50.0;
                        float rand_roll = (rand() % 180) - 90.0;
                        float rand_grip = (rand() % 70);
                        
                        //convert to radians if in radian mode
                        if(dev_mode_Global == 'r'){
                            rand_base *= PI/180.0;
                            rand_shoulder *= PI/180.0;
                            rand_elbow *= PI/180.0;
                            rand_pitch *= PI/180.0;
                            rand_roll *= PI/180.0;
                        }                    
                        pc.printf("t=%7.3f b=%7.3f s=%7.3f e=%7.3f p=%7.3f r=%7.3f g=%7.3f\r\n", rand_time, rand_base,rand_shoulder,rand_elbow,rand_pitch, rand_roll, rand_grip);
                        theta2enc_dt(rand_time,rand_base,rand_shoulder,rand_elbow,rand_pitch, rand_roll,rand_grip);
                        
                            //WHILE LOOP TO WAIT UNTIL ALL TRAPAZOIDAL PROFILES ARE COMPLETE
                        while(axis1.moveState!=0 && axis2.moveState!=0 && axis3.moveState!=0 && axis4.moveState!=0
                             && axis5.moveState!=0 && axis6.moveState!=0 && (qFlag == 0)){
                                 
                            float base, shoulder, elbow, pitch, roll;                        
                            enc2theta(&base, &shoulder, &elbow, &pitch, &roll);
                            pc.printf("B=%7.3f S=%7.3f E=%7.3f P=%7.3f R=%7.3f G=%7.3fmm\r",
                                base, shoulder, elbow, pitch, roll, sbGripper.get_mm());
                            
                            wait(.02);
                            if(pc.readable()){      //if user types a 'q' or 'Q'
                                char c = pc.getc();
                                if(c == 'q' || c == 'Q') //quit after current movement
                                    qFlag = 1;         //set the flag to terminate the excercise
                                if(c == ' '){           //stop immediately
                                    axis1.moveState = 4;
                                    axis2.moveState = 4;
                                    axis3.moveState = 4;
                                    axis4.moveState = 4;
                                    axis5.moveState = 4;
                                    axis6.moveState = 4;
                                    qFlag = 1;         //set the flag to terminate the excercise
                                    break;
                                }
                            }
                            if(stall_state != 0){
                                qFlag = 1;
                            }
                        }                
                    }
                break;
                
                //*: Manually increment/decrement and axis
                case '*':
                    counts=0;
                    float base, shoulder, elbow, pitch, roll;
                    pc.printf("\r\nEnter robot joint to change (bseprg)\r\n");
                    pc.scanf("%c",&c);
                    pc.printf("%c\r\n", c);
                                              
                    pc.printf("Enter +/-/c/q (increment, decrement, quit)\r\n");
                    switch(c){                    
                        case 'b':
                            do{
                                c = pc.getc();
                                if(c == '+'){
                                    Base(70);
                                }
                                if(c == '-'){
                                    Base(-70);
                                }
                                if(c == 'c'){
                                    pc.printf("Enter counts to move base:");
                                    pc.scanf("%d",&counts);
                                    pc.printf("%d\r\n", counts);
                                                                   
                                    sbBase.set_counts(counts);
                                }
                                enc2theta(&base, &shoulder, &elbow, &pitch, &roll);
                                if(dev_mode_Global == 'r')
                                    pc.printf("Base: counts=%.1f rad=%.3f\r\n", axis1.set_point, base); 
                                else
                                    pc.printf("Base: counts=%.1f deg=%.2f\r\n", axis1.set_point, base); 
                            }while(c != 'q');
                        break; 
                        
                        case 's':                        
                            do{
                                c = pc.getc();
                                if(c == '+'){
                                    SEW_up(54);
                                }
                                if(c == '-'){
                                    SEW_down(54);
                                }
                                if(c == 'c'){
                                    pc.printf("Enter counts to move shoulder:");
                                    pc.scanf("%d",&counts);
                                    pc.printf("%d\r\n", counts);
                                                                   
                                    sbShoulder.set_counts(counts);
                                }                            
                                enc2theta(&base, &shoulder, &elbow, &pitch, &roll);
                                if(dev_mode_Global == 'r')
                                    pc.printf("Shoulder: counts=%.1f rad=%.3f\r\n", axis2.set_point, shoulder); 
                                else
                                    pc.printf("Shoulder: counts=%.1f deg=%.2f\r\n", axis2.set_point, shoulder); 
                            }while(c != 'q');
                        break;  
                        
                        case 'e':                        
                            do{
                                c = pc.getc();
                                if(c == '+'){
                                    EW_up(54);
                                }
                                if(c == '-'){
                                    EW_down(54);
                                }
                                if(c == 'c'){
                                    pc.printf("Enter counts to move elbow:");
                                    pc.scanf("%d",&counts);
                                    pc.printf("%d\r\n", counts);
                                                                   
                                    sbElbow.set_counts(counts);
                                }                            
                                enc2theta(&base, &shoulder, &elbow, &pitch, &roll);
                                if(dev_mode_Global == 'r')
                                    pc.printf("Elbow: counts=%.1f rad=%.3f\r\n", axis3.set_point, elbow); 
                                else
                                    pc.printf("Elbow: counts=%.1f deg=%.2f\r\n", axis3.set_point, elbow); 
                            }while(c != 'q');
                        break;
    
                        case 'p':                        
                            do{
                                c = pc.getc();
                                if(c == '+'){
                                    W_pitch(28);
                                }
                                if(c == '-'){
                                    W_pitch(-28);
                                }
                                if(c == 'c'){
                                    pc.printf("counts not an option for pitch yet\r\n");
                                }
                                enc2theta(&base, &shoulder, &elbow, &pitch, &roll);
                                if(dev_mode_Global == 'r')
                                    pc.printf("Pitch: countsAx4=%.1f countsAx5=%.1f rad=%.3f\r\n", axis4.set_point, axis5.set_point, pitch); 
                                else
                                    pc.printf("Pitch: countsAx4=%.1f countsAx5=%.1f deg=%.2f\r\n", axis4.set_point, axis5.set_point, pitch);
                            }while(c != 'q');
                        break;
    
                        case 'r':
                            do{
                                c = pc.getc();
                                if(c == '+'){
                                    W_roll(28);
                                }
                                if(c == '-'){
                                    W_roll(-28);
                                }
                                if(c == 'c'){
                                    pc.printf("counts not an option for pitch yet\r\n");
                                }                            
                                enc2theta(&base, &shoulder, &elbow, &pitch, &roll);
                                if(dev_mode_Global == 'r')
                                    pc.printf("Roll: countsAx4=%.1f countsAx5=%.1f rad=%.3f\r\n", axis4.set_point, axis5.set_point, roll);                             
                                else
                                    pc.printf("Roll: countsAx4=%.1f countsAx5=%.1f deg=%.2f\r\n", axis4.set_point, axis5.set_point, roll);
                            }while(c != 'q');
                        break;
                        
                        case 'g': 
                            pc.printf("+,-,c,s,e,d - Increment, decrement, counts, enable collision, disable collision detect\r\n");
                            do{
                                c = pc.getc();
                                if(c == '+'){
                                    if(axis6.pos < 5100)
                                        G_open(10);
                                    else
                                        pc.printf("5100 max open reached!!\r\n");
                                }
                                if(c == '-'){
                                                                    
                                    if(axis6.pos > 0)
                                        G_close(10);
                                    else
                                        pc.printf("0 max close reached!!\r\n");
                                }
                                if(c == 'c'){
                                    float temp_counts;
                                    pc.printf("Enter counts for gripper (between %.1f and %.1f)\r\n", axis6.p_lower, axis6.p_higher);
                                    pc.scanf("%d", &temp_counts);
                                    pc.printf("counts=%d\r\n", temp_counts);
                                    
                                    if((temp_counts < axis6.p_higher) && (temp_counts > axis6.p_lower)){
                                        axis6.set_point = temp_counts;
                                    }
                                    else{
                                        pc.printf("Counts out of range (max=%.1f, min=%.1f)\r\n", axis6.p_higher, axis6.p_lower);
                                    }
                                }
                                if(c == 's'){
                                    pc.printf("\r\nSet gripper GRIPPER_BACKOFF value in counts \r\n");
                                    pc.printf(" current value is: %.1f for HOME:", GRIPPER_BACKOFF);
                                    pc.scanf("%f", &GRIPPER_BACKOFF);
                                    pc.printf("\r\nEntered %.1f counts\r\n", GRIPPER_BACKOFF);
                                }
                                if(c == 'e'){
                                    GRIPPER_COL_DETECT=1;
                                    pc.printf("Collision detection on gripper Enabled GRIPPER_COL_DETECT=%d\r\n", GRIPPER_COL_DETECT);                                    
                                }
                                if(c == 'd'){
                                    GRIPPER_COL_DETECT=0;
                                    pc.printf("Collision detection on gripper Disbled GRIPPER_COL_DETECT=%d\r\n", GRIPPER_COL_DETECT);                                    
                                }
                                pc.printf("Gripper: counts=%.1f mm=%.3f inches=%.3f I=%7.3f\r\n", axis6.set_point, sbGripper.get_mm(), sbGripper.get_inches(), axis6.motI); 
                            }while(c != 'q');
                        break;
                        
                    }         
                break;        
                
                //instantaneous move - "{B,S,E,P,R\r"
                case '{':                    
                    int newInstMove = 1;    //set the flag indicating that a new instantaneous move has ocurred
                    
                    while(newInstMove){
                        newInstMove = 0;    //clear the flag
                        
                        //clear the read buffer
                        for(int i=0;i<100;i++)
                            bufferRead[i]=0;
                                              
                        //Use the read command wih a serReadTimeout  
                        chars = CMD_read(bufferRead, 50, serReadTimeout);
                        strcat(bufferRead, "\0");
                                
                        /*if(DEBUG){
                            int i=0;
                            do{                        
                                pc.printf("%c", bufferRead[i]);
                                i++;
                                wait(.005);
                            }while(bufferRead[i] != 0);                        
                        }*/
                            
                        if(DEBUG)
                            pc.printf("\r\nchars read = %d\r\n buffer=\r\n%s\r\n", chars, bufferRead);                                    
                                            
                        tok = strtok(bufferRead, "\r");
                        
                        err = sscanf(tok, "%f,%f,%f,%f,%f\r", &theta0, &theta1, &theta2, &theta3, &theta4);
                        
                        if(err == 5){     
                            theta2enc(theta0,theta1,theta2,theta3,theta4);
                        
                            //wait for move to complete or receive a 'q' or ' '
                            int errCode = moveDone_or_qs(5.0);    //max 5 second move (should be instantaneous
                            
                            //if we have received a new instantaneous move command before the last one finished
                            // exit out and take the new one
                            if(errCode == -1){
                                newInstMove = 1;    
                                break;
                            }
                            
                            //
                            if(DEBUG)
                                pc.printf("\r\nerrCode=%.3f\r\n", errCode);
                                
                            if(errCode == 1)
                                pc.printf("OK\r\n");
                        }
                        else{
                            pc.printf("Error parsing string\r\n");
                        }
                    }//newInstMove is set
                break;
                
                //instantaneous move with gripper - "}B,S,E,P,R,gmm\r"
                case '}':
                    // BSEPR - deltaTime, Base, Shoulder, Elbow, Pithc, Roll, gripper millimeters
                    if(homed()){                    
                        if(DEBUG)
                            pc.printf("\r\nEnter string of B,S,E,P,R,grip_mm\\r");
                        
                        for(int i=0;i<100;i++)
                            bufferRead[i]=0;
                        chars = CMD_read(bufferRead, 50, serReadTimeout);
                        strcat(bufferRead, "\0");                
                            
                        if(DEBUG){
                            int i=0;
                            do{                        
                                pc.printf("%c", bufferRead[i]);
                                i++;
                                wait(.005);
                            }while(bufferRead[i] != 0);                        
                        }
                            
                        if(DEBUG)
                            pc.printf("\r\nchars read = %d\r\n buffer=\r\n%s\r\n", chars, bufferRead);
                                                
                        char *tok;
                        tok = strtok(bufferRead, "\r");
                        
                        err = sscanf(tok, "%f,%f,%f,%f,%f,%f\r", &theta0, &theta1, &theta2, &theta3, &theta4, &grip_mm);                
                    
                        if(err == 6){                        
                            //if in radians, convert to degrees
                            if(dev_mode_Global == 'r'){
                                theta0 *= 180.0/PI;
                                theta1 *= 180.0/PI;
                                theta2 *= 180.0/PI;
                                theta3 *= 180.0/PI;
                                theta4 *= 180.0/PI;
                            }
                            //use degrees to test limits
                            if(theta0 > 150.0)
                                theta0 = 150.0;
                            if(theta0 < -150.0)
                                theta0 = -150.0;
            
                            if(theta1 > 120.28)
                                theta1 = 120.28;
                            if(theta1 < 0.0)
                                theta1 = 0.0;
            
                            if(theta2 > 95.02)
                                theta2 = 95.02;
                            if(theta2 < -90.0)
                                theta2 = -90.0;
                                
                            if(theta3 > 90.0)
                                theta3 = 90.0;
                            if(theta3 < -90.0)
                                theta3 = -90.0;
                                
                            if(grip_mm > 70.0)
                                grip_mm = 70.0;
                            if(grip_mm < 0.0)
                                grip_mm = 0.0;
                                
                            //if in radans mode, convert back to radians from degrees
                            if(dev_mode_Global == 'r'){
                                theta0 *= PI/180.0;
                                theta1 *= PI/180.0;
                                theta2 *= PI/180.0;
                                theta3 *= PI/180.0;
                                theta4 *= PI/180.0;
                            }                    
                                                                               
                            if(DEBUG)
                                pc.printf("B=%7.3f S=%7.3f E=%7.3f P=%7.3f R=%7.3f Grip=%7.3fmm\r",
                                    theta0,theta1,theta2,theta3,theta4,grip_mm);
                                
                            theta2enc(theta0,theta1,theta2,theta3,theta4);
                            axis6.set_point = sbGripper.mm_to_counts(grip_mm);
                            
                            //wait for move to complete or receive a 'q' or ' '
                            float timeDone = moveDone_or_qs(dt+(dt*.2));
                            
                            if(DEBUG)
                                pc.printf("OK\r\ndt=%.3f\r\n", timeDone);
                            else
                                pc.printf("OK\r\n");
                        }
                        else{
                            pc.printf("Error parsing string\r\n");
                            wait(3);
                        }
                    }
                    else{
                        pc.printf("Scorbot not homed!\r\n");
                        wait(3);
                    }
                break;                                                  
                
                //Trapezoidal profile move.  "!dt,B,S,E,P,R,Gmm\r"
                case '!':
                    // BSEPR - deltaTime, Base, Shoulder, Elbow, Pithc, Roll, gripper millimeters                                     
                    chars = CMD_read(bufferRead, 100, serReadTimeout);
                    strcat(bufferRead, "\0");
                    
                    if(DEBUG){
                        int i=0;
                        do{                        
                            pc.printf("%c", bufferRead[i]);
                            i++;
                            wait(.005);
                        }while(bufferRead[i] != 0);                   
                    }
                        
                    if(DEBUG)
                        pc.printf("\r\nchars read = %d\r\n buffer=\r\n%s\r\n", chars, bufferRead);
    
                    tok = strtok(bufferRead, "\r");
                    
                    err = sscanf(tok, "%f,%f,%f,%f,%f,%f,%f\r", &dt, &theta0, &theta1, &theta2, &theta3, &theta4, &grip_mm);
                        
                    if(DEBUG)
                        pc.printf("err=%d  dt=%f\r\n", err, dt);
                        
                    if(err == 7){                        
                        //if in radians, convert to degrees
                        if(dev_mode_Global == 'r'){
                            theta0 *= 180.0/PI;
                            theta1 *= 180.0/PI;
                            theta2 *= 180.0/PI;
                            theta3 *= 180.0/PI;
                            theta4 *= 180.0/PI;
                        }
                        //use degrees to test limits
                        if(theta0 > 150.0)
                            theta0 = 150.0;
                        if(theta0 < -150.0)
                            theta0 = -150.0;
        
                        if(theta1 > 120.28)
                            theta1 = 120.28;
                        if(theta1 < 0.0)
                            theta1 = 0.0;
        
                        if(theta2 > 95.02)
                            theta2 = 95.02;
                        if(theta2 < -90.0)
                            theta2 = -90.0;
                            
                        if(theta3 > 90.0)
                            theta3 = 90.0;
                        if(theta3 < -90.0)
                            theta3 = -90.0;
                            
                        if(grip_mm > 70.0)
                            grip_mm = 70.0;
                        if(grip_mm < 0.0)
                            grip_mm = 0.0;
                            
                        //if in radans mode, convert back to radians from degrees
                        if(dev_mode_Global == 'r'){
                            theta0 *= PI/180.0;
                            theta1 *= PI/180.0;
                            theta2 *= PI/180.0;
                            theta3 *= PI/180.0;
                            theta4 *= PI/180.0;
                        }                    
                                                                           
                        if(DEBUG){
                            pc.printf("dt=%.3f B=%7.3f S=%7.3f E=%7.3f P=%7.3f R=%7.3f Grip=%7.3fmm\r",
                                dt,theta0,theta1,theta2,theta3,theta4,grip_mm);
                        }
                            
                        theta2enc_dt(dt,theta0,theta1,theta2,theta3,theta4, grip_mm);
                        
                        //wait for move to complete or receive a 'q' or ' '
                        float timeDone = moveDone_or_qs(dt+(dt*.2));
                                                
                        if(DEBUG)
                            pc.printf("OK\r\ndt=%.3f\r\n", timeDone);
                        else
                            pc.printf("OK\r\n");
                    }//if err == 7
                    else{
                        pc.printf("Error parsing string\r\n");
                        wait(3);
                    }// else err != 7                    
                break; //! command
                
                //print the controller outputs
                case ' ':
                    printControllerOutputs();
                break;
                
                
                // Begin trajectory command sorting - Position Profile
                //Example in Radians Mode:
                //  "[PosCMDdt5.0\r1,0.0,1.57,-1.57,0.0,0.0\r
                //    2,0.0,1.57,0.0,0.0,0.0\r
                //    3,0.0,1.57,-1.57,0.0,0.0\r
                //    4,0.0,2.1,-1.68,-1.55,0.0,0.0\r
                //    **\r                
                case '[':
                    chars = CMD_read(bufferRead, 10000, serReadTimeout);
                    strcat(bufferRead, "\0");
                    
                    float dt = 0.0;            
                    float posCMD[6][500];
                    int numCMD=0;
                    int arrayCnt=0;
                    int MAX_ARRAY_SIZE=500;
                    
                    for(int i=0;i<6;i++)
                        posCMD[i][0] = 0.0;                                            
                    
                    if(DEBUG){
                        int i=0;
                        do{                        
                            pc.printf("%c", bufferRead[i]);
                            i++;
                            wait(.0001);
                        }while(bufferRead[i] != 0);                        
                    }
                        
                    //if(DEBUG)
                    //    pc.printf("\r\nchars read = %d\r\n buffer=\r\n%s\r\n", chars, bufferRead);
    
                    tok = strtok(bufferRead, "\r");
                    int err = sscanf(tok, "PosCMDdt%f\r", &dt);
    
                    if(DEBUG)
                        pc.printf("err=%d  dt=%f\r\n", err, dt);
                    
                    //if the proper PosCMD was sent with dt
                    if(err!=0){
                        int cmdSeqEnd=1;
                        err=7;
                        //float posCMD1,posCMD2,posCMD3,posCMD4,posCMD5,posCMD6;
                        while((err==7) && cmdSeqEnd){
                            tok = strtok(NULL, "\r");
                                            
                            err = sscanf(tok, "%d,%f,%f,%f,%f,%f,%f\r",&numCMD,&posCMD[0][arrayCnt],
                                &posCMD[1][arrayCnt],&posCMD[2][arrayCnt],&posCMD[3][arrayCnt],
                                &posCMD[4][arrayCnt],&posCMD[5][arrayCnt]); 
                            
                            if(err==7)
                                arrayCnt++;
                            
                            if(DEBUG)    
                                pc.printf("\r\nerr=%d %d,%f,%f,%f,%f,%f,%f\r\n",err,numCMD,posCMD[0][arrayCnt],
                                posCMD[1][arrayCnt],posCMD[2][arrayCnt],posCMD[3][arrayCnt],
                                posCMD[4][arrayCnt],posCMD[5][arrayCnt]);                        
                                                //capture position commands until "**" is received to terminate
                            if(sscanf(tok, "**\r") || (arrayCnt>=MAX_ARRAY_SIZE) || (err==0)){
                               pc.printf("\r\nEnd command Sequence\r\n");
                               cmdSeqEnd=0;
                               break;    
                            }
                                                
                         }//while   
    
                        //execute command position trajectory control
                        int stopFlag =1;
                        int loopFlag =0;
                        do{
                            for(int i=0;i<arrayCnt;i++){
                                if(DEBUG)
                                    pc.printf("\r\nsending %d,%f,%f,%f,%f,%f,%f\r\n",numCMD,posCMD[0][i],
                                    posCMD[1][i],posCMD[2][i],posCMD[3][i],
                                    posCMD[4][i],posCMD[5][i]);
                                
                                theta2enc_dt(dt,posCMD[0][i],posCMD[1][i],posCMD[2][i],posCMD[3][i],
                                    posCMD[4][i],posCMD[5][i]);
                                Timer tmout;
                                tmout.start();
                                while((axis1.moveState!=0 || axis2.moveState!=0 || axis3.moveState!=0 || axis4.moveState!=0
                                    || axis5.moveState!=0 || axis6.moveState!=0) && stopFlag){
                                    if(tmout.read() > (dt+2.0)){
                                        pc.printf("\r\nNAK line 5354\r\n");
                                        pc.printf("NAK - IS THIS CAUSING THE CRITICLE ERROR!!!!!!\r\n");
                                        pc.printf("NAK - IS THIS CAUSING THE CRITICLE ERROR!!!!!!\r\n");
                                        pc.printf("NAK - IS THIS CAUSING THE CRITICLE ERROR!!!!!!\r\n");
                                        
                                        if(DEBUG)
                                            pc.printf("Timeout occured before finishing profile move\r\n");
                                        break;
                                    }
                                    
                                    if(pc.readable()){      //if user types a 'q' or 'Q'
                                        char c = pc.getc();
                                        if(c == 'q' || c == 'Q'){ //quit after current movement
                                            //terminate the movement on all axes
                                            stop_all_movement();
                                            stopFlag = 0;               
                                            loopFlag = 0;                                                                  
                                            break;
                                        }
                                        if(c == 'l' || c == 'L'){
                                            loopFlag = 1;
                                            pc.printf("\r\nEntering loop mode.  \r\nPress q to quit or reset controller.\r\n\r\n");    
                                        }
                                        if(c == ' '){
                                            pc.printf("%d of %d ", i, arrayCnt);
                                            printControllerOutputs();
                                        }
                                    }
                                            
                                }
                                pc.printf("OK\r\n");
                                if(i==arrayCnt){
                                    if(DEBUG)
                                        pc.printf("\r\nFinal command sent\r\n");
                                    break;
                                }
                                if(stopFlag == 0){
                                    loopFlag = 0;
                                    break;    
                                }
                            }
                        }while(loopFlag);
                    }//proper PosCMD was sent with dt
                    if(DEBUG){
                        pc.printf("Press any key to continue.\r\n");
                        c = pc.getc();
                    }
                    //pc.scanf("%c",&c);    
                break;      //break for the PosCMDdtxx function
                                                
                                                
                // Begin trajectory command sorting - Velocity Profile
                //Example in Radians Mode:
                //  "]VelCMDdt5.0\r
                //    1,0.0,1.57,-1.57,0.0,0.0\r
                //    2,0.0,1.57,0.0,0.0,0.0\r
                //    3,0.0,1.57,-1.57,0.0,0.0\r
                //    4,0.0,2.1,-1.68,-1.55,0.0,0.0\r
                //    **\r
                case ']':
                    for(int i=0;i<10000;i++)
                        bufferRead[i] = 0;
                        
                    chars = CMD_read(bufferRead, 10000, serReadTimeout);
                    strcat(bufferRead, "\0");                                        
                             
                    float velCMD[6][500];
                    float dtCMD=0;
                    
                    for(int i=0;i<6;i++)
                        velCMD[i][0] = 0.0;                                            
                    
                    if(DEBUG){
                        int i=0;
                        do{                        
                            pc.printf("%c", bufferRead[i]);
                            i++;
                            wait(.0001);
                        }while(bufferRead[i] != 0);                        
                    }
                    
                    pc.printf("\r\nGot this far! 1\r\n\r\n");
                    
                        
                    //if(DEBUG)
                    //    pc.printf("\r\nchars read = %d\r\n buffer=\r\n%s\r\n", chars, bufferRead);
    
                    tok = strtok(bufferRead, "\r");
                    err = sscanf(tok, "VelCMDdt%f\r", &dt);
    
                    //if(DEBUG)
                        pc.printf("err=%d  dt=%f\r\n", err, dt);

                    pc.printf("\r\nGot this far! 2 \r\n\r\n");
/*                    
                    //if the proper VelCMD was sent with dt
                    if(err==1){
                        int cmdSeqEnd=1;
                        err=7;                                                                                                                        
                        
                        //float velCMD1,velCMD2,velCMD3,velCMD4,velCMD5,velCMD6;
                        while((err>=6) && cmdSeqEnd){
                            tok = strtok(NULL, "\r");
                                            
                            err = sscanf(tok, "%f,%f,%f,%f,%f,%f,%f\r",&dtCMD,&velCMD[0][arrayCnt],
                                &velCMD[1][arrayCnt],&velCMD[2][arrayCnt],&velCMD[3][arrayCnt],
                                &velCMD[4][arrayCnt],&velCMD[5][arrayCnt]); 
                            
                            if(err==7)
                                arrayCnt++;
                            else if(err <= 6){
                                velCMD[5][arrayCnt] = 0.0;      //no gripper velocity change
                                arrayCnt++;
                            }
                            else{
                                   pc.printf("\r\n ERROR PARSING JOINT VELOCITY COMMANDS\r\n\r\n");
                                   cmdSeqEnd=0;     //end the command sequence
                                   wait(3);                                   
                            }
                            //if(DEBUG)    
                                pc.printf("\r\nerr=%d %f,%f,%f,%f,%f,%f,%f\r\n",err,dtCMD,velCMD[0][arrayCnt],
                                velCMD[1][arrayCnt],velCMD[2][arrayCnt],velCMD[3][arrayCnt],
                                velCMD[4][arrayCnt],velCMD[5][arrayCnt]);                        
                                                //capture velocity commands until "**" is received to terminate
                            if(sscanf(tok, "**\r") || (arrayCnt>=MAX_ARRAY_SIZE) || (err==0)){
                               pc.printf("\r\nEnd command Sequence\r\n");
                               cmdSeqEnd=0;
                            }
                          
                         }//while((err>=6) && cmdSeqEnd)                         
                         
                    } //if(err==1)      //should receive only 1 valid dt in the "VelCMDdt" command
//   LEFT OFF HERE

                        //execute command velocity commands
                        int stopFlag =1;
                        int loopFlag =0;
                        float vb, vs, ve, vp, vr, vg;
                        float dtCMDold=0.0;
                        do{
                            for(int i=0;i<arrayCnt;i++){
                                if(DEBUG)
                                    pc.printf("\r\nsending %f,%f,%f,%f,%f,%f,%f\r\n",dtCMD,velCMD[0][i],
                                    velCMD[1][i],velCMD[2][i],velCMD[3][i],
                                    velCMD[4][i],velCMD[5][i]);
                                
                                vb = velCMD[0][i];
                                vs = velCMD[1][i];
                                ve = velCMD[2][i];
                                vp = velCMD[3][i];
                                vr = velCMD[4][i];
                                vg = velCMD[5][i];
                                
                                timeout = dtCMD - dtCMDold;
                                
                                //10 sec timout max
                                if(timeout > 10.0)
                                    timeout = 10.0;
                                if(timeout < 0.0)
                                    timeout = 0.0;
                                                        
                                //max/min rad/sec speed at 12V
                                //approximately .42 rad/sec max/min for base
                                if(vb > .4)
                                    vb = .4;
                                if(vb < -.4)
                                    vb = -.4;
                                
                                //shoulder rad/sec max/min    
                                if(vs > .5)
                                    vs = .5;
                                if(vs < -.5)
                                    vs = -.5;
                                    
                                //elbow rad/sec max/min    
                                if(ve > .5)
                                    ve = .5;
                                if(ve < -.5)
                                    ve = -.5;
                                    
                                //pitch rad/sec max/min    
                                if(vp > 1.8)
                                    vp = 1.8;
                                if(vp < -1.8)
                                    vp = -1.8;
                                
                                //roll rad/sec max/min    
                                if(vr > 1.8)
                                    vr = 1.8;
                                if(vr < -1.8)
                                    vr = -1.8;
                                    
                                //gripper mm/sec max/min    
                                if(vg > 12.0)
                                    vg = 12.0;
                                if(vg < -12.0)
                                    vg = -12.0;                                
                                
                                //if(runTime.ms_total
                                velocityCmd(timeout,vb,vs,ve,vp,vr,vg);
                                // adjust to fall through once dt has accurately elapsed
                                wait(dt);                                    
                                    
                                if(pc.readable()){      //if user types a 'q' or 'Q'
                                    char c = pc.getc();
                                    if(c == 'q' || c == 'Q'){ //quit after current movement
                                        //terminate the movement on all axes
                                        stop_all_movement();
                                        stopFlag = 0;               
                                        loopFlag = 0;                                                                  
                                        break;
                                    }
                                    if(c == 'l' || c == 'L'){
                                        loopFlag = 1;
                                        pc.printf("\r\nEntering loop mode.  \r\nPress q to quit or reset controller.\r\n\r\n");    
                                    }
                                    if(c == ' '){
                                        pc.printf("%d of %d ", i, arrayCnt);
                                        printControllerOutputs();
                                    }
                                }

                                if(i==arrayCnt){
                                    if(DEBUG)
                                        pc.printf("\r\nFinal command sent\r\n");
                                    break;
                                }
                                if(stopFlag == 0){
                                    loopFlag = 0;
                                    break;    
                                }
                            }
                        }while(loopFlag);
                                            
                                                
                    }//  if(err!=0)   - proper VelCMD was sent with dt

                    if(DEBUG){
                        pc.printf("Press any key to continue.\r\n");
                        c = pc.getc();
                    }
                                        
                    //pc.scanf("%c",&c);    
                */    
                break;      //break for the VelCMDdtxx function
                
                case '~':
                    float velCMDtest[6][2];
                    vb, vs, ve, vp, vr, vg;   //velocities in rad/sec of robot arm joints
                    float dt_vel = 2.0;             //default to 3 seconds for velocity command timeout
                                            
                    pc.printf("\r\nT%.3f,Vb%.3f,s%.3f,e%.3f,p%.3f,r%.3f,g%.3f\r\n",timeout,vb,vs,ve,vp,vr,vg);                    

                    vb=0.0; vs=0.0; ve=0.0; vp=0.0; vr=.1; vg=0.0;
                    velocityCmd(dt_vel*1.2,vb,vs,ve,vp,vr,vg);
                    wait(1.0);

                    vb=0.0; vs=0.0; ve=0.0; vp=0.0; vr=.2; vg=0.0;
                    velocityCmd(dt_vel*1.2,vb,vs,ve,vp,vr,vg);
                    wait(1.0);
                    
                    vb=0.0; vs=0.0; ve=0.0; vp=0.0; vr=.3; vg=0.0;
                    velocityCmd(dt_vel*1.2,vb,vs,ve,vp,vr,vg);
                    wait(1.0);
                    
                    vb=0.0; vs=0.0; ve=0.0; vp=0.0; vr=.4; vg=0.0;
                    velocityCmd(dt_vel*1.2,vb,vs,ve,vp,vr,vg);
                    wait(1.0);                    
                                                            
                    pc.printf("OK\r\n");
                break;                
            } //switch(c)    
        }//if pc.readable() -------------------------------------------------------------------------------------------------------
                                                              
        
        if(streamOutputActive_Flag){
            if((axis1.moveState==0) && (axis2.moveState==0) && (axis3.moveState==0) && (axis4.moveState==0) && (axis5.moveState==0) && (axis6.moveState==0)){
                //pc.printf("%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f   \r", (float)t.read(), (float)axis1.set_point, (float)axis1.pos, (float)axis2.set_point, (float)axis2.pos,
                //    (float)axis3.set_point, (float)axis3.pos, (float)axis4.set_point, (float)axis4.pos,(float)axis5.set_point, (float)axis5.pos, (float)axis6.set_point, (float)axis6.pos);                
                    
            //                pc.printf("T:%02d:%02d:%02d:%02d:%03d A1:%5.1f %5.1f  A2:%5.1f %5.1f  A3:%5.1f %5.1f  A4:%5.1f %5.1f  A5:%5.1f %5.1f  A6:%5.1f %5.1f  \r", runTime.day, runTime.hour, runTime.min, runTime.sec, runTime.ms, 
            //                (float)axis1.set_point, (float)axis1.pos, (float)axis2.set_point, (float)axis2.pos, (float)axis3.set_point, (float)axis3.pos, (float)axis4.set_point, (float)axis4.pos,(float)axis5.set_point, 
            //                (float)axis5.pos, (float)axis6.set_point, (float)axis6.pos); 
                    
                    //pc.printf("T%02d:%02d:%02d:%02d:%03d", runTime.day, runTime.hour, runTime.min, runTime.sec, runTime.ms);
                    
                printControllerOutputs();
                           
                if(streamFlag)
                    pc.printf("\n");
                led2 = !led2;
            }   
            wait(streamRate);
        }//stream output active
        
    }//while(1)                        
}//main
