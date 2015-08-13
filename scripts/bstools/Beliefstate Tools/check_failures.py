#!/usr/bin/python

from LogReader import LogReader
import sys


rdrLog = LogReader()
log = rdrLog.loadLog(sys.argv[1])

owl = log.getOwlData()
tti = owl["task-tree-individuals"]

event_failures = []
caught_failures = []

for individual in tti:
    owlNode = tti[individual]
    
    failures = owlNode.failures()
    for failure in failures:
        if not failure in event_failures:
            event_failures.append(failure)
    
    caught = owlNode.caughtFailures()
    for c in caught:
        if not c in caught_failures:
            caught_failures.append(c)

print len(event_failures)
print len(caught_failures)
