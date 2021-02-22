#include "Blimp.h"

//most of these will become parameters...
//put here instead of Fins.h so that it can be changed without having to recompile the entire vehicle

#define FREQ_HZ 3 //MIR later change to double omega when using height control (do a check for offset high & then increase omega)

#define FIN_SCALE_MAX 1000

void Fins::setup_fins(){
          //amp r   f   d     y,off r   f   d      y              right, front, down, yaw    amplitude, offset
    add_fin(0,  0, -1, 0.5,   0,    0,  0, -0.5,    0); //Back(?)
    add_fin(1,  0,  1, 0.5,   0,    0,  0, -0.5,    0); //Front(?)
    add_fin(2, -1,  0,   0, 0.5,    0,  0,    0,  0.5); //Right
    add_fin(3,  1,  0,   0, 0.5,    0,  0,    0, -0.5); //Left

    SRV_Channels::set_angle(SRV_Channel::k_motor1, FIN_SCALE_MAX);
    SRV_Channels::set_angle(SRV_Channel::k_motor2, FIN_SCALE_MAX);
    SRV_Channels::set_angle(SRV_Channel::k_motor3, FIN_SCALE_MAX);
    SRV_Channels::set_angle(SRV_Channel::k_motor4, FIN_SCALE_MAX);

    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "MIR: All fins have been added.");    
}

void Fins::add_fin(int8_t fin_num, float right_amp_fac, float front_amp_fac, float down_amp_fac, float yaw_amp_fac,
                                   float right_off_fac, float front_off_fac, float down_off_fac, float yaw_off_fac){

    // ensure valid fin number is provided
    if (fin_num >= 0 && fin_num < NUM_FINS) {

        // set amplitude factors
        _right_amp_factor[fin_num] = right_amp_fac;
        _front_amp_factor[fin_num] = front_amp_fac;
        _down_amp_factor[fin_num] = down_amp_fac;
        _yaw_amp_factor[fin_num] = yaw_amp_fac;

        // set offset factors
        _right_off_factor[fin_num] = right_off_fac;
        _front_off_factor[fin_num] = front_off_fac;
        _down_off_factor[fin_num] = down_off_fac;
        _yaw_off_factor[fin_num] = yaw_off_fac;  
    }
}

//B,F,R,L = 0,1,2,3
void Fins::output()
{
    if (!_armed) {
        // set everything to zero so fins stop moving
        right_out = 0;
        front_out = 0;
        down_out  = 0;
        yaw_out   = 0;
    }

    right_out /= RC_SCALE;
    front_out /= RC_SCALE;
    down_out  /= RC_SCALE;
    yaw_out   /= RC_SCALE;
    
    _time = AP_HAL::micros() * 1.0e-6;

    for (int8_t i=0; i<NUM_FINS; i++){ 
        _amp[i] =  max(0,_right_amp_factor[i]*right_out) + max(0,_front_amp_factor[i]*front_out) + 
            fabsf(_down_amp_factor[i]*down_out) + fabsf(_yaw_amp_factor[i]*yaw_out);
        _off[i] = _right_off_factor[i]*right_out + _front_off_factor[i]*front_out + 
            _down_off_factor[i]*down_out + _yaw_off_factor[i]*yaw_out;

        _num_added = 0;
        if (max(0,_right_amp_factor[i]*right_out) > 0.0f) {
            _num_added++;
        }
        if (max(0,_front_amp_factor[i]*front_out) > 0.0f) {
            _num_added++;
        }
        if (fabsf(_down_amp_factor[i]*down_out) > 0.0f) {
            _num_added++;
        }
        if (fabsf(_yaw_amp_factor[i]*yaw_out) > 0.0f) {
            _num_added++;
        }

        if (_num_added > 0) {
            _amp[i] = _amp[i]/_num_added;
            _off[i] = _off[i]/_num_added;
        }

        //double speed fins if offset at max... MIR
        // if (_amp[i] <= 0.5) {
            // _freq_mult = 1.0f/_off[i];
        // }
        
        // finding and outputting current position for each servo from sine wave 
        // blimp.g2.fins_freq_hz 
        _pos[i]= _amp[i]*sinf(FREQ_HZ * _time * 2 * M_PI) + _off[i]; //removed +MAX_AMP because output can do -ve numbers
        SRV_Channels::set_output_scaled(SRV_Channels::get_motor_function(i), _pos[i] * FIN_SCALE_MAX);
    }
}

void Fins::output_min(){
    right_out = 0;
    front_out = 0;
    down_out  = 0;
    yaw_out   = 0;
    Fins::output();
}

// MIR - Probably want to completely get rid of the desired spool state thing.
void Fins::set_desired_spool_state(DesiredSpoolState spool)
{
    if (_armed || (spool == DesiredSpoolState::SHUT_DOWN)) {
        // Set DesiredSpoolState only if it is either armed or it wants to shut down.
        _spool_desired = spool;
    }
};
