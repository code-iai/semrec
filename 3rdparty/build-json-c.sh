#!/bin/bash

if [ ! -e "$1/built_flag" ]; then
    cd $1
    sh autogen.sh
    ./configure
    make
    touch built_flag
    cd -
fi
