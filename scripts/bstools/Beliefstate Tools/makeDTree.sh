#!/bin/bash

echo "Condensing datasets.."
python ./condense.py $@
echo "Condensed into generalized model."

echo "Creating ARFF training data.."
for dir in $@; do
   echo " - $dir"
done

python ./create_training_data_new.py training_data $@
# python ./condense.py ~/sr_experimental_data/packaging/
# python ./create_training_data_new.py ~/sr_experimental_data/packaging/ training_data
# /bin/bash ./runweka.sh | python wekaToDTree.py
# cat o.json
# echo
