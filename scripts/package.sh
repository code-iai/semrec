#!/bin/bash

source ~/.bashrc
PLANLOGGER_DIR=${HOME}/sr_experimental_data/
cd ${PLANLOGGER_DIR}
rm -Rf packaging
mkdir -p packaging

mongoexport -d roslog -c logged_designators -o packaging/logged_designators.json
mongoexport -d roslog -c tf -o packaging/tf.json

cp -r current-experiment/* packaging
./makedot.sh

DATE_STRING=`date +"%Y-%m-%d-%H-%M-%S"`
mkdir -p results
cd packaging
tar -czf ../results/log-packaged-${DATE_STRING}.tar.gz *
cd -
