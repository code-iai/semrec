#!/bin/bash

cd current-experiment
mkdir -p dot-svgs

for file in cram_log.dot.*; do
    dot -Tsvg $file > dot-svgs/$file.svg
done
