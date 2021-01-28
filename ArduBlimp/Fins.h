//This class converts horizontal acceleration commands to fin flapping commands.
#pragma once
#include <AP_Notify/AP_Notify.h>      // Notify library

#define FINS_SPEED_DEFAULT 10 //MIR what is this?
class Fins {      //mixer.h
public:
    friend class Blimp;
    Fins(void);

    enum motor_frame_class {
        MOTOR_FRAME_UNDEFINED = 0,
        MOTOR_FRAME_AIRFISH = 1,
    };
    enum motor_frame_type {
        MOTOR_FRAME_TYPE_AIRFISH = 1,
    };

    // Constructor
    Fins(uint16_t loop_rate, uint16_t speed_hz = FINS_SPEED_DEFAULT);

    // singleton support
    // static Fins    *get_singleton(void) { return _singleton; }

    // desired spool states
    // from AP_Motors_Class.h
    enum class DesiredSpoolState : uint8_t {
        SHUT_DOWN = 0,              // all motors should move to stop
        // GROUND_IDLE = 1,            // all motors should move to ground idle
        THROTTLE_UNLIMITED = 2,     // motors should move to being a state where throttle is unconstrained (e.g. by start up procedure)
    };

    // spool states
    enum class SpoolState : uint8_t {
        SHUT_DOWN = 0,                      // all motors stop
        // GROUND_IDLE = 1,                    // all motors at ground idle
        // SPOOLING_UP = 2,                       // increasing maximum throttle while stabilizing
        THROTTLE_UNLIMITED = 3,             // throttle is no longer constrained by start up procedure
        // SPOOLING_DOWN = 4,                     // decreasing maximum throttle while stabilizing
    };

    bool initialised_ok() const {
        return true;
    }

    void armed(bool arm) {
        if (arm != _armed){
            _armed = arm;
            AP_Notify::flags.armed = arm;
        }
        
    }
    bool armed() const { return _armed; }

protected:
    // internal variables
    uint16_t            _loop_rate;                 // rate in Hz at which output() function is called (normally 400hz)
    uint16_t            _speed_hz;                  // speed in hz to send updates to motors
    // float               _roll_in;                   // desired roll control from attitude controllers, -1 ~ +1
    // float               _roll_in_ff;                // desired roll feed forward control from attitude controllers, -1 ~ +1
    // float               _pitch_in;                  // desired pitch control from attitude controller, -1 ~ +1
    // float               _pitch_in_ff;               // desired pitch feed forward control from attitude controller, -1 ~ +1
    // float               _yaw_in;                    // desired yaw control from attitude controller, -1 ~ +1
    // float               _yaw_in_ff;                 // desired yaw feed forward control from attitude controller, -1 ~ +1
    float               _throttle_in;               // last throttle input from set_throttle caller
    float               _down_out;                  // throttle after mixing is complete
    // float               _forward_in;                // last forward input from set_forward caller
    // float               _lateral_in;                // last lateral input from set_lateral caller
    float               _throttle_avg_max;          // last throttle input from set_throttle_avg_max
    // LowPassFilterFloat  _throttle_filter;           // throttle input filter
    DesiredSpoolState   _spool_desired;             // desired spool state
    SpoolState          _spool_state;               // current spool mode

    float               _air_density_ratio;         //air density as a proportion of sea level density

    float               _time;                       //current timestep

    float              _amp1, _amp2, _amp3, _amp4;  //amplitudes
    float              _offset1, _offset2, _offset3, _offset4; //offsets
    float              _pos1, _pos2, _pos3, _pos4;  //servo positions
    float              _omega, _maxAmp; 

    bool _armed;             // 0 if disarmed, 1 if armed

// private:
public:
    float               right_out;                  //input right movement, negative for left, +1 to -1
    float               front_out;                  //input front/forwards movement, negative for backwards, +1 to -1
    float               yaw_out;                    //input yaw, +1 to -1
    float               down_out;                   //input height control, +1 to -1
    
    
    bool _interlock;         // 1 if the motor interlock is enabled (i.e. motors run), 0 if disabled (motors don't run)
    bool _initialised_ok;    // 1 if initialisation was successful

    // get_spool_state - get current spool state
    enum SpoolState  get_spool_state(void) const { return _spool_state; }

    void set_loop_rate(uint16_t loop_rate){ _loop_rate = loop_rate;}

    void output_min();
    
    float get_throttle_hover(){ return 0; } //MIR temp

    void set_desired_spool_state(DesiredSpoolState spool);

    void output();
    
    float get_throttle() {return 0.1f; } //MIR temp
    
    void rc_write(uint8_t chan, uint16_t pwm);

    // set_density_ratio - sets air density as a proportion of sea level density
    void  set_air_density_ratio(float ratio) { _air_density_ratio = ratio; }

};
