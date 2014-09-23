#!/bin/bash

./condense.py $1
./create_training_data.py
./runweka.sh | ./wekaToDTree.py
cp o.json ~/dtree-model.json
cp out.json ~/task-model.json
