#!/usr/bin/python

import sys
from DataCondenser import DataCondenser


sys.setrecursionlimit(100000)

dcCondenser = DataCondenser()
dcCondenser.condenseData(sys.argv[1])
