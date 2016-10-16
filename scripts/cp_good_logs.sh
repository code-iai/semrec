#!/bin/bash


# Housekeeping
DIR_CURRENT=`pwd`
DIR_SCRIPT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DIR_BASE=$DIR_SCRIPT

# Output
DIR_RESULTS="$DIR_BASE/results"

# Input and intermediate
DIR_TEMP="$DIR_BASE/export_inspect_temp"
DIR_DESTINATION=$1
DIR_STORAGE_ORIGINALS="$DIR_BASE/storage"

# Check related data
REQUIRED_FILES="cram_log.owl tf.json logged_designators.json"


if [ -z "$DIR_DESTINATION" ]; then
    DIR_DESTINATION="$DIR_BASE/exported_results"
fi


mkdir -p $DIR_TEMP
mkdir -p $DIR_DESTINATION
mkdir -p $DIR_STORAGE_ORIGINALS


# Act
cd $DIR_RESULTS

for ARCHIVE_FILE in *.tar.gz; do
    echo "Checking ${ARCHIVE_FILE}"
    tar -xf $ARCHIVE_FILE -C $DIR_TEMP
    
    all_good=true
    
    for required_file in $REQUIRED_FILE; do
	if [ ! -f "$DIR_TEMP/$required_file" ]; then
	    all_good=false
	fi
    done
    
    rm -Rf ${DIR_TEMP}/*
    
    if [ "$all_good" = true ]; then
	cp $ARCHIVE_FILE $DIR_DESTINATION
	echo "OK"
    fi
    
    mv $ARCHIVE_FILE $DIR_STORAGE_ORIGINALS
done

rm -Rf "$DIR_TEMP"
echo "Done"
