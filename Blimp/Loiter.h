#pragma once

class Vector4b
{
public:
    bool    x;
    bool    y;
    bool    z;
    bool    yaw;

    constexpr Vector4b()
        : x(0)
        , y(0)
        , z(0)
        , yaw(0) {}

    constexpr Vector4b(const bool x0, const bool y0, const bool z0, const bool yaw0)
        : x(x0)
        , y(y0)
        , z(z0)
        , yaw(yaw0) {}

    Vector4b operator &&(const Vector4b &v){
        Vector4b temp{x && v.x, y && v.y, z && v.z, yaw && v.yaw};
        return temp;
    }

    Vector4b operator ||(const Vector4b &v){
        Vector4b temp{x || v.x, y || v.y, z || v.z, yaw || v.yaw};
        return temp;
    }

};



class Loiter
{
public:
    friend class Blimp;
    friend class Fins;
    Vector4b limit_int;

    //constructor
    Loiter(uint16_t loop_rate){
        limit_int = {false,false,false,false};
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Created Loiter class");
    };

    void run(Vector3f target_pos, float target_yaw, Vector4b axes_disabled);
};
