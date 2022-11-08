#pragma once

#include "Blimp.h"
class Parameters;
class ParametersG2;

class GCS_Blimp;

class Mode
{

public:

    // Auto Pilot Modes enumeration
    enum class Number : uint8_t {
        HOLD =          0,  // just stops moving
        MANUAL =        1,  // manual control
        VELOCITY =      2,  // velocity mode
        LOITER =        3,  // loiter mode (position hold)
        AUTO =          4,  // auto mode
        GUIDED =        5,  // guided
        RTL =           6,  // rtl
        SRCLOC =        7,  // source localisation
        PSO =           8,  // PSO - particle-swarm optimisation
    };

    // constructor
    Mode(void);

    // do not allow copying
    Mode(const Mode &other) = delete;
    Mode &operator=(const Mode&) = delete;

    // child classes should override these methods
    virtual bool init(bool ignore_checks)
    {
        return true;
    }
    virtual void run() = 0;
    virtual bool requires_GPS() const = 0;
    virtual bool has_manual_throttle() const = 0;
    virtual bool allows_arming(bool from_gcs) const = 0;
    virtual bool is_autopilot() const
    {
        return false;
    }
    virtual bool has_user_takeoff(bool must_navigate) const
    {
        return false;
    }
    virtual bool in_guided_mode() const
    {
        return false;
    }

    // return a string for this flightmode
    virtual const char *name() const = 0;
    virtual const char *name4() const = 0;

    virtual bool is_landing() const
    {
        return false;
    }

    // mode requires terrain to be present to be functional
    virtual bool requires_terrain_failsafe() const
    {
        return false;
    }

    // functions for reporting to GCS
    virtual bool get_wp(Location &loc)
    {
        return false;
    };
    virtual int32_t wp_bearing() const
    {
        return 0;
    }
    virtual uint32_t wp_distance() const
    {
        return 0;
    }
    virtual float crosstrack_error() const
    {
        return 0.0f;
    }

    void update_navigation();

    // pilot input processing
    void get_pilot_input(Vector3f &pilot, float &yaw);

protected:

    // navigation support functions
    virtual void run_autopilot() {}

    // helper functions
    bool is_disarmed_or_landed() const;

    // functions to control landing
    // in modes that support landing
    void land_run_horizontal_control();
    void land_run_vertical_control(bool pause_descent = false);

    // convenience references to avoid code churn in conversion:
    Parameters &g;
    ParametersG2 &g2;
    AP_InertialNav &inertial_nav;
    AP_AHRS &ahrs;
    Fins *&motors;
    Loiter *&loiter;
    RC_Channel *&channel_right;
    RC_Channel *&channel_front;
    RC_Channel *&channel_down;
    RC_Channel *&channel_yaw;
    float &G_Dt;

public:
    // Navigation Yaw control
    class AutoYaw
    {

    public:

        // yaw(): main product of AutoYaw; the heading:
        float yaw();

        // mode(): current method of determining desired yaw:
        autopilot_yaw_mode mode() const
        {
            return (autopilot_yaw_mode)_mode;
        }
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
    bool set_mode(Mode::Number mode, ModeReason reason);
    GCS_Blimp &gcs();

    // end pass-through functions
};

class ModeManual : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool requires_GPS() const override
    {
        return false;
    }
    bool has_manual_throttle() const override
    {
        return true;
    }
    bool allows_arming(bool from_gcs) const override
    {
        return true;
    };
    bool is_autopilot() const override
    {
        return false;
    }

protected:

    const char *name() const override
    {
        return "MANUAL";
    }
    const char *name4() const override
    {
        return "MANU";
    }

private:

};

class ModeVelocity : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool requires_GPS() const override
    {
        return true;
    }
    bool has_manual_throttle() const override
    {
        return false;
    }
    bool allows_arming(bool from_gcs) const override
    {
        return true;
    };
    bool is_autopilot() const override
    {
        return false;
        //TODO
    }

protected:

    const char *name() const override
    {
        return "VELOCITY";
    }
    const char *name4() const override
    {
        return "VELY";
    }

private:

};

class ModeLoiter : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual bool init(bool ignore_checks) override;
    virtual void run() override;

    bool requires_GPS() const override
    {
        return true;
    }
    bool has_manual_throttle() const override
    {
        return false;
    }
    bool allows_arming(bool from_gcs) const override
    {
        return true;
    };
    bool is_autopilot() const override
    {
        return false;
        //TODO
    }

protected:

    const char *name() const override
    {
        return "LOITER";
    }
    const char *name4() const override
    {
        return "LOIT";
    }

private:
    Vector3f target_pos;
    float target_yaw;
};

class ModeLand : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool requires_GPS() const override
    {
        return false;
    }
    bool has_manual_throttle() const override
    {
        return true;
    }
    bool allows_arming(bool from_gcs) const override
    {
        return false;
    };
    bool is_autopilot() const override
    {
        return false;
    }

protected:

    const char *name() const override
    {
        return "LAND";
    }
    const char *name4() const override
    {
        return "LAND";
    }

private:

};

class ModeAuto : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual bool init(bool ignore_checks) override;
    virtual void run() override;

    bool requires_GPS() const override
    {
        return true;
    }
    bool has_manual_throttle() const override
    {
        return false;
    }
    bool allows_arming(bool from_gcs) const override
    {
        return true;
    };
    bool is_autopilot() const override
    {
        return false;
        //TODO
    }

protected:

    const char *name() const override
    {
        return "AUTO";
    }
    const char *name4() const override
    {
        return "AUTO";
    }

private:
    Vector3f target_pos;
    float target_yaw;
    int step;

};

class ModeGuided : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual bool init(bool ignore_checks) override;
    virtual void run() override;
    void set_target(Location tar);

    bool requires_GPS() const override
    {
        return true;
    }
    bool has_manual_throttle() const override
    {
        return false;
    }
    bool allows_arming(bool from_gcs) const override
    {
        return true;
    };
    bool is_autopilot() const override
    {
        return false;
        //TODO
    }

protected:

    const char *name() const override
    {
        return "GUIDED";
    }
    const char *name4() const override
    {
        return "GUID";
    }

private:
    Vector3f target_pos;
    float target_yaw;
};

class ModeRTL : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual bool init(bool ignore_checks) override;
    virtual void run() override;

    bool requires_GPS() const override
    {
        return true;
    }
    bool has_manual_throttle() const override
    {
        return false;
    }
    bool allows_arming(bool from_gcs) const override
    {
        return true;
    };
    bool is_autopilot() const override
    {
        return false;
        //TODO
    }

protected:

    const char *name() const override
    {
        return "RTL";
    }
    const char *name4() const override
    {
        return "RTL";
    }
};

class ModeSrcloc : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual bool init(bool ignore_checks) override;
    virtual void run() override;

    bool requires_GPS() const override
    {
        return true;
    }
    bool has_manual_throttle() const override
    {
        return false;
    }
    bool allows_arming(bool from_gcs) const override
    {
        return true;
    };
    bool is_autopilot() const override
    {
        return false;
        //TODO
    }

protected:

    const char *name() const override
    {
        return "SRCLOC";
    }
    const char *name4() const override
    {
        return "SRCL";
    }

private:

    enum class CS : uint8_t {
        CASTING_RUN =   0,
        CASTING_START =  1,
        SURGING_START =  2,
        SURGING_RUN = 3,
    };

    enum class SLMode : uint8_t {
        CASTSURGEPOS =   1,
        CASTSURGEACCEL =  2,
        PUSHDRIFT =  3,
        LEVYWALK = 4,
        CASTSURGEVEL = 5,
    };

    CS cs;
    float beta;
    float cast_time;
    float lost_time;
    Vector3f target_pos;
    float target_yaw;
    bool right_mv;
    int16_t stage;
    float push;
    bool drift;
    bool fnd_pl;
    Vector2f out;

    float levydis(float x);
};


//-------------------------------------------------------------

class Par
{
public:
    float x;
    float y;
    uint32_t time_boot_ms_pos;
    float plu;
    uint32_t time_boot_ms_plu;

    constexpr Par()
        : x(0.0)
        , y(0.0)
        , time_boot_ms_pos(0)
        , plu(0.0)
        , time_boot_ms_plu(0) {}

    void zero(){
        x = y = plu = 0.0;
        time_boot_ms_plu = time_boot_ms_pos = 0;
    }
};

//Max number of blimps to be used. Means sysids of 1 to PAR_MAX being put into array indeces 0 to 4
#define PAR_MAX 10

class ModePSO : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual bool init(bool ignore_checks) override;
    virtual void run() override;
    void handle_msg(const mavlink_message_t &msg);
    float distance(int part1, int part2);

    bool requires_GPS() const override
    {
        return true;
    }
    bool has_manual_throttle() const override
    {
        return false;
    }
    bool allows_arming(bool from_gcs) const override
    {
        return true;
    };
    bool is_autopilot() const override
    {
        return false;
        //TODO
    }

protected:

    const char *name() const override
    {
        return "PSO";
    }
    const char *name4() const override
    {
        return "PSO";
    }

private:
    void send_debug_loc(const char *name, Location value);
    float sgn(float x);

    Vector3f target_vel;
    float target_yaw;

    //This should always contain the strength and location of the highest received plume strength for each blimp
    Par pbest[PAR_MAX];
    //This contains the most recent position mavlink message for each blimp
    // mavlink_global_position_int_t curarr[PAR_MAX];
    //index of the global best (within pbest)
    int gbest = 1;
    Par X[PAR_MAX];
    Vector2f V[PAR_MAX];
    uint32_t bests_sent;
    int max_seen = 0;
};