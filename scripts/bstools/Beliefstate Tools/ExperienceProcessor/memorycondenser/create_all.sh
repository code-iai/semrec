#!/bin/bash

for mode in "experiences" "condensed" "deduced"; do
    echo "-- Entering mode '${mode}'"
    
    ./memorycondenser.py ${mode} > temp_${mode}.dot
    dot -Tpdf temp_${mode}.dot > final_${mode}.pdf
    #rm temp_${mode}.dot
    
    echo "Created 'final_${mode}.pdf'"
    
    echo "-- Completed mode '${mode}'"
done

#./memorycondenser.py deduced > test_deduced
#.dot && dot -Tpdf test_deduced.dot > test_deduced.pdf && evince test_deduced.pdf
