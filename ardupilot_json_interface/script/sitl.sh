#!/bin/bash

export PARAM_FILE=$(rospack find ardupilot_json_interface)/params/gps_ek3_2source.param


echo $PARAM_FILE
# sim_vehicle.py -v APMrover2 -f gazebo-rover --wipe-eeprom -m --mav10 --map --console -I1 -D
# sim_vehicle.py -v APMrover2 -f JSON--map --console 
sim_vehicle.py -v APMrover2 -f gazebo-rover --wipe-eeprom --add-param-file=$PARAM_FILE -m --map --console -I1 
