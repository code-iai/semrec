#!/bin/bash

PYTHONISH_OUTPUT=`./memorycondenser.py $1 data`

FIXED_OUTPUT="${PYTHONISH_OUTPUT//u\'/\'}"
FIXED_OUTPUT="${FIXED_OUTPUT//\'/\"}"

echo $FIXED_OUTPUT
