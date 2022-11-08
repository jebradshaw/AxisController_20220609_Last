
#include "Axis.h"
#include "LS7366.h"
#include "MotCon.h"
#include "PID.h"

Axis::Axis(SPI& spi, PinName cs, PinName pwm, PinName dir, PinName analog, int* limit): _spi(spi), _cs(cs), _pwm(pwm), _dir(dir) , _analog(analog){
    this->_cs = 1;           // Initialize chip select as off (high)
    this->_pwm = 0.0;
    this->_dir = 0;
    this->co = 0.0;
    this->Tdelay = .01;
    this->Pk = 450.0; //120.0;      //rough gains, seem to work well but could use tuning   
    this->Ik = 105.0;  //75.0;
    this->Dk = 0.0;
    this->set_point = 0.0;
    this->set_point_last = 0.0;
    this->pos = 0.0;
    this->vel = 0.0;
    this->acc = 0.0;
    this->stat = -1;
    this->pos_cmd = 0.0;
    this->vel_cmd = 0.0;
    this->vel_avg_cmd = 0;
    this->acc_cmd = 0.0;
    this->vel_max = 2700.0 * Tdelay; //counts * Tdelay
    this->acc_max = 1200.0 * Tdelay; //counts/sec/sec  * Tdelay
    this->p_higher = 0.0;
    this->p_lower = 0.0;
    this->vel_accum = 0.0;
    this->moveTime = 0.0;
    this->enc = 0;
    this->moveStatus = 0;     //status flag to indicate state of profile movement
    this->moveState = 0;      //used for state machine in movement profiles
    this->debug = 0;   
    this->update.attach(this, &Axis::paramUpdate, Tdelay);
    this->axisState = 0;
    this->mot_I_lim = .35;    
    this->dIdT = 0.0;
    this->motI = 0.0;
    this->motI_last = 0.0;
    this->mot_I_max = 0.0;
    this->mot_I_max_last = 0.0;
    this->motInvert = 0;
    this->dataFormat = 'r'; //default is radians
//    this->ctsPerDeg = cpd;  //update counts per degree passed from constructor
    this->moveMode = 0;     //default is position mode
    
    this->pid = new PID(0.0,0.0,0.0,Tdelay); //Kc, Ti, Td, interval
    this->ls7366 = new LS7366(spi, cs);  //LS7366 encoder interface IC
    this->motcon = new MotCon(pwm, dir);
    this->ptr_limit = limit;
    
    //start at 0    
    this->ls7366->LS7366_reset_counter();
    this->ls7366->LS7366_quad_mode_x4();       
    this->ls7366->LS7366_write_DTR(0);
    
    this->set_point = 0.0;
    this->pid->setSetPoint(this->set_point);
    this->enc = this->ls7366->LS7366_read_counter();  //update class variable    
}

void Axis::init(float encCountsPerRev){ 
    //resets the controllers internals
    this->pid->reset();

    //Encoder counts limit
    this->pid->setInputLimits(-55000, 55000); 
    //Pwm output from 0.0 to 1.0
    this->pid->setOutputLimits(-1.0, 1.0);
    //If there's a bias.
    this->pid->setBias(0.0);
    this->pid->setMode(AUTO_MODE);
    
    this->pid->setInterval(this->Tdelay);
      
    //set the encoder counts per revolution/linear throw
    this->countsPerRev=encCountsPerRev;
          
    //start at 0    
    this->ls7366->LS7366_reset_counter();
    this->ls7366->LS7366_quad_mode_x4();       
    this->ls7366->LS7366_write_DTR(0);
    
    this->set_point = 0.0;
    this->pid->setSetPoint(this->set_point);
    this->enc = this->ls7366->LS7366_read_counter();  //update class variable
    
        //resets the controllers internals
    this->pid->reset();              
    //start at 0
    this->set_point = 0.0;
    this->pid->setSetPoint(0); 
                                                        
    this->pid->setTunings(this->Pk, this->Ik, this->Dk); //turns on controller    
}

void Axis::updatePIDgains(float P, float I, float D){
    this->Pk = P; //120.0;      //rough gains, seem to work well but could use tuning   
    this->Ik = I;  //75.0;
    this->Dk = D;
    this->pid->setTunings(this->Pk, this->Ik, this->Dk);
}

void Axis::paramUpdate(void){    
    //testOut = 1;    
    this->enc = this->ls7366->LS7366_read_counter();
//    this->pos = (float)this->enc; // * this->countsPerDeg * PI/180.0; //times counts/degree and convert to radians
  
    this->pos = (float)this->enc;
    this->vel = ((this->pos - this->pos_last) / this->countsPerRev) * 6.28319 * (1.0/this->Tdelay); //current vel in radians
    this->acc = (this->vel - this->vel_last) * (1.0/this->Tdelay);            

    this->pid->setSetPoint(this->set_point);    
    
    //Update the process variable.
    //Update the process variable.
    if(this->moveMode == 0)
        this->pid->setProcessValue(this->pos);
    if(this->moveMode == 1)
        this->pid->setProcessValue(this->vel);
    if(this->moveMode == 2)
        this->pid->setProcessValue(this->acc);
        
    //Set the new output.
    this->co = this->pid->compute();

    if(this->axisState){
        if(this->motInvert==0){
            this->motcon->mot_control(this->co); //send controller output to PWM motor control command    
        }
        else{
            this->motcon->mot_control(this->co, 1); //send controller output to PWM motor control command    
        }
    }
    else{
        this->co = 0.0;
        this->motcon->mot_control(0.0);     //turn off motor command
    }
        
    this->pos_last = this->pos;
    this->vel_last = this->vel;
    this->set_point_last = this->set_point;
}

void Axis::center(void){    
    while((*this->ptr_limit == 1) && (this->readCurrent() < mot_I_lim)){ //limit switch not pressed and mot current not exceeded
       this->set_point += 100;
       wait(.05);
       if(this->debug)
            printf("T=%.2f SP=%.3f co=%.3f pos=%.3f vel=%.3f acc=%.3f limit=%d motI=%.3f\r\n", t.read(), this->set_point, this->co, this->pos, this->vel, this->acc,*this->ptr_limit, this->_analog.read());
    }
    wait(.5);
    while((*this->ptr_limit == 0)){ //limit switch is pressed
       this->set_point -= 10;
       wait(.1);
       if(this->debug)
            printf("T=%.2f SP=%.3f co=%.3f pos=%.3f vel=%.3f acc=%.3f limit=%d motI=%.3f\r\n", t.read(), this->set_point, this->co, this->pos, this->vel, this->acc,*this->ptr_limit, this->_analog.read());
    }
    this->zero();   //zero channel        
    
//    this->set_point = -(totalCounts/2.0);
        
    if(this->debug)
        printf("HOME END:T=%.2f SP=%.3f co=%.3f pos=%.3f vel=%.3f acc=%.3f limit=%d motI=%.3f\r\n", t.read(), this->set_point, this->co, this->pos, this->vel, this->acc,*this->ptr_limit, this->_analog.read());
//    pc.printf("End Home\r\n\r\n");
}

void Axis::moveUpdate(void){               

/*    if(*this->ptr_limit == 0){
        this->moveState = 4;    //terminate the move
        printf("\r\nLimit reached on axis!\r\n");
    }
*/
//    if(this->debug)
//        printf("T=%.2f SP=%.3f co=%.3f pos=%.3f vel=%.3f acc=%.3f limit=%d motI=%.3f\r\n", t.read(), this->set_point, this->co, this->pos, this->vel, this->acc,*this->ptr_limit, this->_analog.read());

    switch(this->moveState){
        case 0:        
        break;
            
        //accelerate
        case 1:
            //testOut = 1;
            this->vel_accum += this->acc_cmd * this->Tdelay;    //add acceleration to the velocity accumulator
            if(this->vel_avg_cmd > 0.0){                        //check the sign of the movement
                if(this->vel_accum >= this->vel_cmd)            //if the accumulator reaches or exceeds the velocity command
                    this->vel_accum = this->vel_cmd;            // only add the velocity command to the accumulator
            }
            else{                                               //if the sign was negative
                if(this->vel_accum <= this->vel_cmd)
                    this->vel_accum = this->vel_cmd;
            }
            //testOut = 0;
                 
            this->set_point += this->vel_accum;
            //pc.printf("T=%.2f SP=%.3f co=%.3f enc=%d pos=%.3f vel=%.3f acc=%.3f vel_accum=%.2f accelCnt=%.2f     \r\n", t.read(), con0.set_point, con0.co, con0.enc, con0.pos, con0.vel, con0.acc, vel_accum, accelCnt); 
            if(this->debug)
                printf("acc_up,%.2f,%.3f,%.3f,%.1f,%.3f,%.3f,%.2f,%.2f\r\n", this->t.read(), this->pos_cmd, this->set_point, this->pos, this->vel, this->acc, this->vel_accum, this->acc_cmd); 

            if(this->t.read()>=(this->moveTime/3.0) || (abs(this->vel_accum) > abs(this->vel_cmd)))
                this->moveState = 2;
        break;
        
        //constant velocity
        case 2:        
            //testOut = 1;
            //this->vel_accum += this->vel_cmd * this->Tdelay;
            this->acc_cmd = 0.0;
            this->set_point += this->vel_cmd;
            //testOut = 0;
            //pc.printf("T=%.2f SP=%.3f co=%.3f enc=%d pos=%.3f vel=%.3f acc=%.3f vel_accum=%.2f accelCnt=%.2f     \r\n", t.read(), con0.set_point, con0.co, con0.enc, con0.pos, con0.vel, con0.acc, vel_accum, accelCnt); 
            if(this->debug)
                printf("vel_cn,%.2f,%.3f,%.3f,%.1f,%.3f,%.3f,%.2f,%.2f\r\n", this->t.read(), this->pos_cmd, this->set_point, this->pos, this->vel, this->acc, this->vel_accum, this->acc_cmd); 
            
            if(this->t.read()>=(2.0/3.0 * this->moveTime)){                
                //RECALCULATE DECELERATION BASED OFF OF CURRENT POSITION
                float displacement = this->pos_cmd - (float)this->set_point;
                //this->acc_cmd = -(powf(this->vel, 2.0))/(2.0 * final_displacement); //did not work
                //float movetime_div3 = this->moveTime / 3.0;
                float delta_t = this->moveTime - this->t.read();
            
                this->acc_cmd = (2.0*(displacement - (this->vel_accum * delta_t))/(delta_t * delta_t)) * this->Tdelay;                
                this->moveState = 3;
            }
            if((this->enc < this->pos_cmd + 5) && (this->enc > this->pos_cmd - 5)){
                this->moveState = 4;    //got there early
            }
        break;
     
        //decelerate
        case 3:                                   
            float test_acc = 0.0;
            float delta_t = this->moveTime - this->t.read();
            if(this->t.read() < (.95 * this->moveTime)){                
                //this->acc_cmd = (displacement * (this->moveTime - this->t.read())) / this->vel_accum;            
                //test_acc = (displacement / delta_t) * this->Tdelay;    
                this->vel_accum -= this->acc_cmd * this->Tdelay;
                this->set_point += this->vel_accum;
                
                //save for when else occurs
                this->test_acc = ((this->pos_cmd - this->set_point)) * delta_t * this->Tdelay;
            }
            else{       //Else we are too close to the end to recalculate and delta shrinks to 0
                //this->vel_accum = ((this->pos_cmd - (float)this->enc)) * (this->moveTime - this->t.read() * this->Tdelay;
                this->set_point += this->test_acc;
                //this->vel_accum -= this->acc_cmd * this->Tdelay;            
            }                                
        
            //this->set_point += this->vel_accum;             //ramp down velocity by acceleration       
            //pc.printf("T=%.2f SP=%.3f co=%.3f enc=%d pos=%.3f vel=%.3f acc=%.3f vel_accum=%.2f accelCnt=%.2f     \r\n", t.read(), con0.set_point, con0.co, con0.enc, con0.pos, con0.vel, con0.acc, vel_accum, accelCnt); 
            
            if(this->debug)
                printf("acc_dn,%.2f,%.3f,%.3f,%.1f,%.3f,%.3f,%.2f,%.2f\r\n", this->t.read(), this->pos_cmd, this->set_point, this->pos, this->vel, this->acc, this->vel_accum, this->acc_cmd); 
            
            /*
            if(this->vel_accum <= 0.0){
                //if(this->pos > this->pos_cmd + <= this->pos){
                    //finish with position command
                    this->set_point = this->pos_cmd;
                    if(this->debug)
                        printf("velocity accum at zero finished!");
                    this->moveState = 4;
                //}
            }
            */
            /*
            else{
                if(this->pos_cmd >= this->pos){
                    //finish with position command
                    //this->set_point = this->pos_cmd;  //May be causing jerk after multi-axis movements  J Bradshaw 20151124
                    this->moveState = 4;
                }
            }
            */
            if(this->t.read() >= this->moveTime){
                //finish with position command
                //this->set_point = this->pos_cmd;     //May be causing jerk after multi-axis movements  J Bradshaw 20151124
                if(this->debug)
                    printf("time finished!");
                this->moveState = 4;    
            }
            
            if((this->enc < this->pos_cmd + 2) && (this->enc > this->pos_cmd - 2)){
                this->moveState = 4;    //got there early
            }            
        break;
        
        case 4:   
            //this->set_point = this->pos_cmd;
            this->moveProfile.detach();   //turn off the trapazoidal update ticker
            this->t.stop();
            this->moveState = 0;     
        break;
    }//switch moveStatus
    return;
}

// position - encoder position to move to
// time - duration of the movement
void Axis::moveTrapezoid(float positionCmd, float time){
    this->pos_cmd = positionCmd;
    this->moveTime = time;
    float enc_distance = this->pos_cmd - (float)this->enc;// * 1.0/con0.countsPerDeg * 180.0/PI;
       
    this->vel_avg_cmd = enc_distance / time;    
    this->vel_cmd = 1.5 * this->vel_avg_cmd * this->Tdelay;
    this->acc_cmd = 4.5 * (enc_distance / (this->moveTime * this->moveTime)) * this->Tdelay;
    
    if(this->debug)
        printf("vel cmd = %.2f  acc_cmd = %.2f\r\n", vel_cmd, acc_cmd);
    //pc.printf("tx=%f encdist=%.3f vAvg=%.3f vMax=%.3f  Acc=%.3f  \r\n", this->moveTime, enc_distance,this->vel_avg_cmd,this->vel_cmd,this->acc_cmd);
    
    //establish encoder velocities and accelerations for position control per Tdelay
    this->vel_accum = 0.0;
    
//  this->set_point = this->pos;
    this->moveState = 1;
    this->t.reset();
    this->t.start();
    this->moveProfile.attach(this, &Axis::moveUpdate, this->Tdelay);
}

float Axis::readCurrent(void){    
    this->motI = (this->_analog.read() * 3.3) / .525;  //525mV per amp
    if(abs(this->motI) > this->mot_I_max){   //update the max motor current measured
        this->mot_I_max = this->motI;
    }
    this->dIdT = motI - motI_last;
    this->motI_last = motI;
    return this->motI;
}

void Axis::axisOff(void){
    this->co = 0;
    this->axisState = 0;
    this->update.detach();
}

void Axis::axisOn(void){    
    this->co = 0.0;
    this->pid->reset();
    //start at 0 if not already homed
    if(this->stat != 0){
        this->set_point = 0.0;
        this->pid->setSetPoint(0); 
    }
                                                        
    this->pid->setTunings(this->Pk, this->Ik, this->Dk); //turns on controller
    this->axisState = 1;
    this->update.attach(this, &Axis::paramUpdate, this->Tdelay);        
}

void Axis::zero(void){       

    this->ls7366->LS7366_reset_counter();
    this->ls7366->LS7366_quad_mode_x4();       
    this->ls7366->LS7366_write_DTR(0);
    
    this->enc = this->ls7366->LS7366_read_counter();
    
    this->pos = 0.0;
    this->pid->reset();     //added to avoid possible itegral windup effects on instant position change 20170616
    this->set_point = 0.0;
    this->pid->setSetPoint(0);    
    
}

void Axis::writeEncoderValue(long value){
    
    this->ls7366->LS7366_write_DTR(value);
        
    this->set_point = (float)value;
    this->pid->setSetPoint(this->set_point);        
}

float Axis::readEncoderValue(void){
    return this->ls7366->LS7366_read_counter();
}

//mode = 0 position, 1 - volocity, 2 - acceleration
void Axis::changeMoveMode(int mode){    
        
    if(mode == 0){
        this->Tdelay = .01;
        //Encoder counts limit
        this->moveMode = 0;
    }
    else if(mode == 1){
        this->Tdelay = .05;
        //this->pid->setInputLimits(-100.0, 100.0);
        this->moveMode = 1;        
    }
    else if(mode == 2){
        this->Tdelay = .05;
        //this->pid->setInputLimits(-100.0, 100.0);
        this->moveMode = 2;  
    }
    this->pid->setInterval(this->Tdelay);
    this->update.attach(this, &Axis::paramUpdate, this->Tdelay);
}