#!/usr/bin/python

from LogReader import LogReader
import sys


def getLength(strPath):
    rdrLog = LogReader()
    log = rdrLog.loadLog(sys.argv[1])
    
    owl = log.getOwlData()
    meta = owl["metadata"]
    tti = owl["task-tree-individuals"]
    
    root = tti[meta.subActions()[0]]
    
    return root.time()


length = 0
for strPath in sys.argv[1:]:
    length += getLength(strPath)

print length
