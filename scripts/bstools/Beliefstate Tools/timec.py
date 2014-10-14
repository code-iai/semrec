#!/usr/bin/python

import sys
from TimeConfidence import TimeConfidence


sys.setrecursionlimit(100000)

tcConfidence = TimeConfidence()
tcConfidence.confidence(sys.argv[1])
