#!/usr/bin/python

import sys
from DataKCondenser import DataKCondenser


sys.setrecursionlimit(100000)

dcCondenser = DataKCondenser()
dcCondenser.condenseData(sys.argv[1])
