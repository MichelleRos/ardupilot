#include "Blimp.h"

#ifdef USERHOOK_INIT
void Blimp::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void Blimp::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Blimp::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Blimp::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Blimp::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Blimp::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Blimp::userhook_auxSwitch1(uint8_t ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Blimp::userhook_auxSwitch2(uint8_t ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Blimp::userhook_auxSwitch3(uint8_t ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
