#!/bin/bash

cd packaging
for f in *.dot; do
    filename=$(basename "$f")
    extension="${filename##*.}"
    filename="${filename%.*}"
    dot -Tpdf $f > ${filename}.pdf
done
cd -