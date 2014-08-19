#!/usr/bin/python

import sys
from LogAnalyzer import LogAnalyzer


laAnalyzer = LogAnalyzer()
laAnalyzer.analyzeLog(sys.argv[1])