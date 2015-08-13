#!/bin/bash

WEKA_PATH="/usr/share/java/"
CP="$CLASSPATH:/usr/share/java/:$WEKA_PATH/mysql-connector-java-5.0.5-bin.jar:$WEKA_PATH/weka.jar"

java -cp $CP -Xmx1024m weka.classifiers.trees.REPTree -t training_data_d.arff -i -k -c 4 -x 2
