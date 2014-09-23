#!/usr/bin/python

import sys
from DataExtractor import DataExtractor


deExtractor = DataExtractor()
deExtractor.extractData(sys.argv[1])
