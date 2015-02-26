#!/usr/bin/python

import sys
from GraspExtractor import GraspExtractor


gpExtractor = GraspExtractor()
gpExtractor.extractGrasps(sys.argv[1])
