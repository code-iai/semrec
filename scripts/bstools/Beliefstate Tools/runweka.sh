#!/bin/bash

WEKA_PATH="/usr/share/java/"
CP="$CLASSPATH:/usr/share/java/:$WEKA_PATH/mysql-connector-java-5.0.5-bin.jar:$WEKA_PATH/weka.jar"

java -cp $CP -Xmx1024m weka.classifiers.trees.J48 -t training_data.arff -i -k -c 1 -x 10
