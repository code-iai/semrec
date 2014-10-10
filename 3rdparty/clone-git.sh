#!/bin/bash

if [ ! -e "$2/built_flag" ]; then
    if [ ! -d $2 ]; then
	git clone $1 $2
    else
	cd $2
	git pull
	cd -
    fi
fi
