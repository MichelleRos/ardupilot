#include "Blimp.h"

//most of these will become parameters...
//put here instead of Fins.h so that it can be changed without having to recompile the entire vehicle
#define MIN_AMP 0
#define MAX_AMP 75
#define MAX_OFF MAX_AMP/2 //in degrees for now - will need to be adjusted to what the servo output function needs.
#define FREQ_HZ 0.1 //MIR later change to double omega when using height control (do a check for offset high & then increase omega)

//constructor
Fins::Fins(uint16_t loop_rate, uint16_t speed_hz){
    _loop_rate = loop_rate;
}

#define FIN_SCALE_MAX 1000

void Fins::setup_fins(){
          //amp r   f   d     y,off r   f   d      y
    add_fin(0,  0, -1, 0.5,   0,    0,  0, 0.5,    0);
    add_fin(1,  0,  1, 0.5,   0,    0,  0, 0.5,    0);
    add_fin(2, -1,  0,   0, 0.5,    0,  0,   0,  0.5);
    add_fin(3,  1,  0,   0, 0.5,    0,  0,   0, -0.5);

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

        //no check to see if this was successful
        // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "MIR: Fin added.");
    }
}

//B,F,R,L = 0,1,2,3
void Fins::output()
{
    //assumes scaling -1 to 1 for each
    // _time;
    //offset is -1 to 1
    //amplitude & omega is 0 to 1

    if (!_armed) {
        // set everything to zero so fins stop moving
        right_out = 0;
        front_out = 0;
        down_out = 0;
        yaw_out = 0;
    }

    down_out = -1; //MIR debug why fins don't move with down_out = -1
    
    _time = AP_HAL::micros() * 1.0e-6;

    for (int8_t i=0; i<NUM_FINS; i++){ 
        _amp[i] =  max(0,_right_amp_factor[i]*right_out) + max(0,_front_amp_factor[i]*front_out) + fabsf(_yaw_amp_factor[i]*yaw_out) + fabsf(_down_amp_factor[i]*down_out);
        _off[i] = _right_off_factor[i]*right_out + _front_off_factor[i]*front_out + _yaw_off_factor[i]*yaw_out + _down_off_factor[i]*down_out;
        // average over non-zero values
        //calculating amplitudes and offsets
        // _num_added = 0;
        // if (max(0,_right_amp_factor[i]*right_out) > 0.0f) {
        //     _amp[i] += max(0,_right_amp_factor[i]*right_out);
        //     _added[0] = 1;
        //     _num_added += 1;
        // } else _added[0] = 0;
        
        // if (max(0,_front_amp_factor[i]*front_out) > 0.0f) {
        //     _amp[i] += max(0,_front_amp_factor[i]*front_out);
        //     _added[1] = 1;
        //     _num_added += 1;
        // } else _added[1] = 0;
        
        // if (fabsf(_down_amp_factor[i]*down_out)   > 0.01f) { //small number because floating point.
        //     _amp[i] += fabsf(_down_amp_factor[i]*down_out);
        //     _added[2] = 1;
        //     _num_added += 1;
        // } else _added[2] = 0;
        
        // if (fabsf(_yaw_amp_factor[i]*yaw_out)     > 0.01f) {
        //     _amp[i] +=  fabsf(_yaw_amp_factor[i]*yaw_out);
        //     _added[3] = 1;
        //     _num_added += 1;
        // } else _added[3] = 0;            

        // if (_added[0]) _off[i] += _right_off_factor[i]*right_out;
        // if (_added[1]) _off[i] += _front_off_factor[i]*front_out;
        // if (_added[2]) _off[i] += _down_off_factor[i]*down_out;
        // if (_added[3]) _off[i] += _yaw_off_factor[i]*yaw_out;

        // if (_num_added > 0) _off[i] = _off[i]/_num_added;
        // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "MIR: Num added is %f", static_cast<double>(_num_added));
        // ::printf("FINS2 (%.1f %.1f %.1f)\n", (float)i, _amp[i], _off[i]);

        //scaling to amounts for servo
        // _amp[i] = max(0,mapf(_amp[i], -1, 1, MIN_AMP, MAX_AMP));
        // _off[i] = max(0,mapf(_off[i], -1, 1, -MAX_OFF, MAX_OFF));
        // not sure about this maths...
        // using max(0,x) function to ensure amplitudes don't go -ve

        // finding and outputting current position for each servo from sine wave  


        _pos[i]= _amp[i]*sinf(FREQ_HZ * _time * 2 * M_PI) + _off[i]; //removed +MAX_AMP because output can do -ve numbers
        SRV_Channels::set_output_scaled(SRV_Channels::get_motor_function(i), _pos[i] * FIN_SCALE_MAX);
    }

    // for (uint8_t i=0; i<NUM_FINS; i++) {
    //     const float rate_hz = 0.2 * (i+1);
    //     const float phase = AP_HAL::micros() * 1.0e-6;
    //     float fin = sinf(phase * rate_hz * 2 * M_PI);
    //     SRV_Channels::set_output_scaled(SRV_Channels::get_motor_function(i), fin * FIN_SCALE_MAX);
    // }
}

void Fins::output_min(){
    right_out = 0;
    front_out = 0;
    down_out = 0;
    yaw_out = 0;
    Fins::output();
}

void Fins::set_desired_spool_state(DesiredSpoolState spool)
{
    if (_armed || (spool == DesiredSpoolState::SHUT_DOWN)) {
        // Set DesiredSpoolState only if it is either armed or it wants to shut down.
        _spool_desired = spool;
    }
};
