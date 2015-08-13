#!/bin/bash

cd ~/sr_experimental_data
#/bin/bash ./package.sh
cd "/home/winkler/ros/catkin/src/semrec/scripts/bstools/Beliefstate Tools"
python ./condense.py ~/sr_experimental_data/packaging/
python ./create_training_data_new.py ~/sr_experimental_data/packaging/ training_data
/bin/bash ./runweka.sh | python wekaToDTree.py
cat o.json
echo
