#include "Blimp.h"

//constructor
Fins::Fins(uint16_t loop_rate, uint16_t speed_hz){
    _loop_rate = loop_rate;
    _omega = 0.3; //MIR
}

//B,F,R,L = 1,2,3,4
void Fins::output(){ //assumes scaling -1 to 1 for each. Throttle is height control, hence neccessity for negative.
    // _time;
    
    //+ve roll is right, +ve pitch is forward, +ve yaw is right, +ve throttle is up
    //offset is -1 to 1
    //amplitude & omega is 0 to 1
    
    //MIR Must first remap values as per parameters to give correct values for servo.

    if (roll_out > 0) { //right
        _amp4 = roll_out;
        _amp3 = 0;
    }else if (roll_out < 0) { //left
        _amp3 = -roll_out;
        _amp4 = 0;
    }else{
        _amp3 = 0;
        _amp4 = 0;
        _offset3 = 0;
        _offset4 = 0;
    }

    if (pitch_out > 0) { //forwards
        _amp1 = pitch_out;
        _amp2 = 0;
    } else if (pitch_out < 0) { //backwards
        _amp1 = -pitch_out;
        _amp2 = 0;
    } else {
        _amp1 = 0;
        _amp2 = 0;
        _offset1 = 0;
        _offset2 = 0;
    }    
    
    _pos1 = _amp1*sinf(_omega*_time) + _maxAmp + _offset1; //finding current position for each servo from sine wave
    // fin1.write(pos1);                         //outputting  to servos
    _pos2 = _amp2*sinf(_omega*_time) + _maxAmp + _offset2;
    // fin2.write(pos2);
    _pos3 = _amp3*sinf(_omega*_time) + _maxAmp + _offset3;
    // fin3.write(pos3);
    _pos4 = _amp4*sinf(_omega*_time) + _maxAmp + _offset4;
    // fin4.write(pos4);
}

void Fins::output_min(){
    roll_out = 0;
    pitch_out = 0;
    throttle_out = 0;
    yaw_out = 0;
    Fins::output();
}

void AP_Motors::rc_write(uint8_t chan, uint16_t pwm) //how do I use this one?
{
    SRV_Channel::Aux_servo_function_t function = SRV_Channels::get_motor_function(chan);
    SRV_Channels::set_output_pwm(function, pwm);
}

void Fins::set_desired_spool_state(DesiredSpoolState spool)
{
    if (_armed || (spool == DesiredSpoolState::SHUT_DOWN)) {
        //Set DesiredSpoolState only if it is either armed or it wants to shut down.
        _spool_desired = spool;
    }
};