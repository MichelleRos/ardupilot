#include "Blimp.h"
/*
 * Init and run calls for rtl flight mode
 */

//Number of seconds of movement that the target position can be ahead of actual position.
#define POS_LAG 1

 bool ModeRTL::init(bool ignore_checks){
    return true;
 }

//Runs the main rtl controller
void ModeRTL::run()
{
    blimp.loiter->run({0,0,0}, 0, Vector4b{false,false,false,false});
}
