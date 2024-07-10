#include "Copter.h"

// land_init - initialise land controller
bool ModeQuicktune::init(bool ignore_checks)
{
    copter.quicktune->init();
    return true;
}

// land_run - runs the quicktune controller
// should be called at looprate
void ModeQuicktune::run()
{
    copter.quicktune->update();
}
