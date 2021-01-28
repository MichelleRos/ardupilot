#pragma once

#include "Blimp.h"
class Parameters;
class ParametersG2;

class GCS_Blimp;

class Mode {

public:

    // Auto Pilot Modes enumeration
    enum class Number : uint8_t {
        MANUAL =        0,  // manual control similar to Copter's stabilize mode
        LAND =          1,  // currently just stops moving
        // STABILIZE =     0,  // manual airframe angle with manual throttle
        // ACRO =          1,  // manual body-frame angular rate with manual throttle
        // ALT_HOLD =      2,  // manual airframe angle with automatic throttle
        // AUTO =          3,  // fully automatic waypoint control using mission commands
        // GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
        // LOITER =        5,  // automatic horizontal acceleration with automatic throttle
        // RTL =           6,  // automatic return to launching point
        // CIRCLE =        7,  // automatic circular flight with automatic throttle
        // LAND =          9,  // automatic landing with horizontal position control
        // DRIFT =        11,  // semi-automous position, yaw and throttle control
        // SPORT =        13,  // manual earth-frame angular rate control with manual throttle
        // FLIP =         14,  // automatically flip the vehicle on the roll axis
        // AUTOTUNE =     15,  // automatically tune the vehicle's roll and pitch gains
        // POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
        // BRAKE =        17,  // full-brake using inertial/GPS system, no pilot input
        // THROW =        18,  // throw to launch mode using inertial/GPS system, no pilot input
        // AVOID_ADSB =   19,  // automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
        // GUIDED_NOGPS = 20,  // guided mode but only accepts attitude and altitude
        // SMART_RTL =    21,  // SMART_RTL returns to home by retracing its steps
        // FLOWHOLD  =    22,  // FLOWHOLD holds position with optical flow without rangefinder
        // FOLLOW    =    23,  // follow attempts to follow another vehicle or ground station
        // ZIGZAG    =    24,  // ZIGZAG mode is able to fly in a zigzag manner with predefined point A and point B
        // SYSTEMID  =    25,  // System ID mode produces automated system identification signals in the controllers
        // AUTOROTATE =   26,  // Autonomous autorotation
    };

    // constructor
    Mode(void);

    // do not allow copying
    Mode(const Mode &other) = delete;
    Mode &operator=(const Mode&) = delete;

    // child classes should override these methods
    virtual bool init(bool ignore_checks) {
        return true;
    }
    virtual void run() = 0;
    virtual bool requires_GPS() const = 0;
    virtual bool has_manual_throttle() const = 0;
    virtual bool allows_arming(bool from_gcs) const = 0;
    virtual bool is_autopilot() const { return false; }
    virtual bool has_user_takeoff(bool must_navigate) const { return false; }
    virtual bool in_guided_mode() const { return false; }
    virtual bool logs_attitude() const { return false; }

    // return a string for this flightmode
    virtual const char *name() const = 0;
    virtual const char *name4() const = 0;

    bool do_user_takeoff(float takeoff_alt_cm, bool must_navigate);
    // virtual bool is_taking_off() const;
    // static void takeoff_stop() { takeoff.stop(); }

    virtual bool is_landing() const { return false; }

    // mode requires terrain to be present to be functional
    virtual bool requires_terrain_failsafe() const { return false; }

    // functions for reporting to GCS
    virtual bool get_wp(Location &loc) { return false; };
    virtual int32_t wp_bearing() const { return 0; }
    virtual uint32_t wp_distance() const { return 0; }
    virtual float crosstrack_error() const { return 0.0f;}

    void update_navigation();

    int32_t get_alt_above_ground_cm(void);

    // pilot input processing
    void get_pilot_desired_accelerations(float &right_out, float &front_out) const;
    float get_pilot_desired_yaw_rate(int16_t stick_angle);
    float get_pilot_desired_throttle() const;

    // returns climb target_rate reduced to avoid obstacles and
    // altitude fence
    float get_avoidance_adjusted_climbrate(float target_rate);

    // const Vector3f& get_desired_velocity() {
    //     // note that position control isn't used in every mode, so
    //     // this may return bogus data:
    //     return pos_control->get_desired_velocity();
    // }

protected:

    // navigation support functions
    virtual void run_autopilot() {}

    // helper functions
    bool is_disarmed_or_landed() const;
    void zero_throttle_and_relax_ac(bool spool_up = false);
    void zero_throttle_and_hold_attitude();
    void make_safe_spool_down();

    // functions to control landing
    // in modes that support landing
    void land_run_horizontal_control();
    void land_run_vertical_control(bool pause_descent = false);

    // return expected input throttle setting to hover:
    virtual float throttle_hover() const;

    // Alt_Hold based flight mode states used in Alt_Hold, Loiter, and Sport
    // enum AltHoldModeState {
    //     AltHold_MotorStopped,
    //     AltHold_Takeoff,
    //     AltHold_Landed_Ground_Idle,
    //     AltHold_Landed_Pre_Takeoff,
    //     AltHold_Flying
    // };
    // AltHoldModeState get_alt_hold_state(float target_climb_rate_cms);

    // convenience references to avoid code churn in conversion:
    Parameters &g;
    ParametersG2 &g2;
    // AC_WPNav *&wp_nav;
    // AC_Loiter *&loiter_nav;
    AP_InertialNav &inertial_nav;
    AP_AHRS &ahrs;
    // AC_AttitudeControl_t *&attitude_control;
    MOTOR_CLASS *&motors;
    // Fins *&motors;
    RC_Channel *&channel_right;
    RC_Channel *&channel_front;
    RC_Channel *&channel_down;
    RC_Channel *&channel_yaw;
    float &G_Dt;

    // note that we support two entirely different automatic takeoffs:

    // "user-takeoff", which is available in modes such as ALT_HOLD
    // (see has_user_takeoff method).  "user-takeoff" is a simple
    // reach-altitude-based-on-pilot-input-or-parameter routine.

    // "auto-takeoff" is used by both Guided and Auto, and is
    // basically waypoint navigation with pilot yaw permitted.

    // user-takeoff support; takeoff state is shared across all mode instances
    // class _TakeOff {
    // public:
    //     void start(float alt_cm);
    //     void stop();
    //     void get_climb_rates(float& pilot_climb_rate,
    //                          float& takeoff_climb_rate);
    //     bool triggered(float target_climb_rate) const;

    //     bool running() const { return _running; }
    // private:
    //     bool _running;
    //     float max_speed;
    //     float alt_delta;
    //     uint32_t start_ms;
    // };

    // static _TakeOff takeoff;

    // virtual bool do_user_takeoff_start(float takeoff_alt_cm);

    // method shared by both Guided and Auto for takeoff.  This is
    // waypoint navigation but the user can control the yaw.
    // void auto_takeoff_run();
    // void auto_takeoff_set_start_alt(void);

    // altitude above-ekf-origin below which auto takeoff does not control horizontal position
    // static bool auto_takeoff_no_nav_active;
    // static float auto_takeoff_no_nav_alt_cm;

public:
    // Navigation Yaw control
    class AutoYaw {

    public:

        // yaw(): main product of AutoYaw; the heading:
        float yaw();

        // mode(): current method of determining desired yaw:
        autopilot_yaw_mode mode() const { return (autopilot_yaw_mode)_mode; }
        void set_mode_to_default(bool rtl);
        void set_mode(autopilot_yaw_mode new_mode);
        autopilot_yaw_mode default_mode(bool rtl) const;

        // rate_cds(): desired yaw rate in centidegrees/second:
        float rate_cds() const;
        void set_rate(float new_rate_cds);

        // set_roi(...): set a "look at" location:
        void set_roi(const Location &roi_location);

        void set_fixed_yaw(float angle_deg,
                           float turn_rate_dps,
                           int8_t direction,
                           bool relative_angle);

    private:

        float look_ahead_yaw();
        float roi_yaw();

        // auto flight mode's yaw mode
        uint8_t _mode = AUTO_YAW_LOOK_AT_NEXT_WP;

        // Yaw will point at this location if mode is set to AUTO_YAW_ROI
        Vector3f roi;

        // bearing from current location to the ROI
        float _roi_yaw;

        // yaw used for YAW_FIXED yaw_mode
        int32_t _fixed_yaw;

        // Deg/s we should turn
        int16_t _fixed_yaw_slewrate;

        // heading when in yaw_look_ahead_yaw
        float _look_ahead_yaw;

        // turn rate (in cds) when auto_yaw_mode is set to AUTO_YAW_RATE
        float _rate_cds;

        // used to reduce update rate to 100hz:
        uint8_t roi_yaw_counter;

    };
    static AutoYaw auto_yaw;

    // pass-through functions to reduce code churn on conversion;
    // these are candidates for moving into the Mode base
    // class.
    float get_pilot_desired_climb_rate(float throttle_control);
    float get_non_takeoff_throttle(void);
    bool set_mode(Mode::Number mode, ModeReason reason);
    void set_land_complete(bool b);
    GCS_Blimp &gcs();
    void set_throttle_takeoff(void);
    uint16_t get_pilot_speed_dn(void);

    // end pass-through functions
};

class ModeManual : public Mode {

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return true; }
    bool allows_arming(bool from_gcs) const override { return true; };
    bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "MANUAL"; }
    const char *name4() const override { return "MANU"; }

private:

};

class ModeLand : public Mode {

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return true; }
    bool allows_arming(bool from_gcs) const override { return false; };
    bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "LAND"; }
    const char *name4() const override { return "LAND"; }

private:

};
