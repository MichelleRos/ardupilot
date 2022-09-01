#!/bin/bash

mavproxy.py --master=mcast: --cmd='module load srcloc; map set showgpspos 0; map set showgps2pos 0; module unload ftp; slp 6; module load swarm; sl sethome -35.28025255 149.00584878'
