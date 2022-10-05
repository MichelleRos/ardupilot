#!/bin/bash

mavproxy.py --console --map --master=mcast: --cmd='module load srcloc; set streamrate -1; set streamrate2 -1; layout load; map set showgpspos 0; map set showgps2pos 0; rc set override_hz 1; module load swarm; longb 511 33 1000000'
