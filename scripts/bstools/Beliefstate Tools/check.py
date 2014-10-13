#!/usr/bin/python

import sys
from LogReader import LogReader

sys.setrecursionlimit(100000)

rdrLog = LogReader()
log = rdrLog.loadLog(sys.argv[1])
dataOwl = log.getOwlData()


def findDepth(indivkey, lvl = 0):
    indiv = dataOwl["task-tree-individuals"][indivkey]
    if len(indiv.subActions()) > 0:
        for sa in indiv.subActions():
            findDepth(sa, lvl + 1)
    else:
        print lvl

findDepth(dataOwl["metadata"].subActions()[0])
