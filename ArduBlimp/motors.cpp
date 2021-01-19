#include "Blimp.h"

// motors_output - send output to motors library which will adjust and send to ESCs and servos
void Blimp::motors_output()
{
    // output any servo channels
    SRV_Channels::calc_pwm();

    // cork now, so that all channel outputs happen at once
    SRV_Channels::cork();

    // update output on any aux channels, for manual passthru
    SRV_Channels::output_ch_all();

    // send output signals to motors
    motors->output();

    // push all channels
    SRV_Channels::push();
}