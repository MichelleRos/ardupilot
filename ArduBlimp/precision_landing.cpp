//
// functions to support precision landing
//

#include "Blimp.h"

#if PRECISION_LANDING == ENABLED

void Blimp::init_precland()
{
    blimp.precland.init(400);
}

void Blimp::update_precland()
{
    int32_t height_above_ground_cm = current_loc.alt;

    // use range finder altitude if it is valid, otherwise use home alt
    if (rangefinder_alt_ok()) {
        height_above_ground_cm = rangefinder_state.alt_cm_glitch_protected;
    }

    precland.update(height_above_ground_cm, rangefinder_alt_ok());
}
#endif
