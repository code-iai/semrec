#!/bin/bash

./memorycondenser.py | dot -Tpdf -o temp.pdf
evince temp.pdf
