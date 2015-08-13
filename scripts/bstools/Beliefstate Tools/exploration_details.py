#!/usr/bin/python

from LogReader import LogReader
import sys


rdrLog = LogReader()
log = rdrLog.loadLog(sys.argv[1])

owl = log.getOwlData()
tti = owl["task-tree-individuals"]

count_eff = 0
count_eff_fail = 0

def hasSubActionUIMAPerceiveFailed(owlIndiv):
    if owlIndiv.taskContext() == "UIMA-PERCEIVE":
        if len(owlIndiv.tagAttributeValues("knowrob:perceptionResult", "rdf:resource")) == 0:
            return True
    else:
        subactions = owlIndiv.subActions()
        
        for subaction in subactions:
            if hasSubActionUIMAPerceiveFailed(tti[subaction]):
                return True
    
    return False

for indiv in tti:
    owlIndiv = tti[indiv]
    
    if owlIndiv.taskContext() == "FIND-OBJECTS":
        count_eff += 1
        
        if hasSubActionUIMAPerceiveFailed(owlIndiv):
            count_eff_fail += 1

print "FIND-OBJECT:", count_eff
print "Failed:", count_eff_fail
