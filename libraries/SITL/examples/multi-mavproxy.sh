#!/bin/bash

mavproxy.py --console --map --master=mcast: --cmd='module load srcloc; set streamrate 1; set streamrate2 1; layout load; map set showgpspos 0; map set showgps2pos 0; module load swarm'
