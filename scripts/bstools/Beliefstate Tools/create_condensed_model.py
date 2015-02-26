#!/usr/bin/python

from LogReader import LogReader

import math
import json
import sys

sys.setrecursionlimit(100000)


class DataCondenser:
    def __init__(self):
        self.rdrLog = LogReader()
        self.tti = {}
        
    def condenseData(self, strOutputFile, arrSourceDirectories):
        dicToplevelNodes = []
        
        for strSourceDirectory in arrSourceDirectories:
            logData = self.rdrLog.loadLog(strSourceDirectory)
            owlData = logData.getOwlData()
            
            self.tti = dict(self.tti.items() + owlData["task-tree-individuals"].items())
            meta = owlData["metadata"]
            
            if meta:
                dicToplevelNodes += meta.subActions()
        
        dicResult = {"Toplevel" : self.condenseNodes("", dicToplevelNodes)}
        
        with open(strOutputFile, "wb") as fOut:
            json.dump(dicResult, fOut)
    
    def condenseNodes(self, strParentNode, arrNodes, nLevel = 0):
        arrTypes = {}
        arrIndividuals = {}
        
        for strNode in arrNodes:
            owlNode = self.tti[strNode]
            nodeclass = owlNode.taskContext()
            
            if nLevel < 0:
                ident = "*"
            else:
                ident = nodeclass
            
            failures = owlNode.failures()
            failure = ""
            if len(failures) > 0:
                failure = self.tti[failures[0]].type()
            
            result = self.condenseNodes(strNode, owlNode.subActions(), nLevel + 1)
            if not ident in arrTypes:
                arrTypes[ident] = result
            else:
                arrTypes[ident] = self.unifyResults(arrTypes[ident], result)
            
            arrTypes[ident]["individuals"][strNode] = {"parameters" : owlNode.annotatedParameters(True),
                                                       "parent" : strParentNode,
                                                       "failure" : failure,
                                                       "class" : nodeclass}
        
        return {"subTypes" : arrTypes,
                "individuals" : {}}
    
    def unifyResults(self, res1, res2):
        resparams = {}
        if len(res1["individuals"]) > 0:
            resparams = res1["individuals"]
        
        if len(res2["individuals"]) > 0:
            resparams = dict(resparams.items() + res2["individuals"].items())
        
        unified = {"subTypes" : {},
                   "individuals" : resparams}
        
        for ressub1 in res1["subTypes"]:
            if ressub1 in res2["subTypes"]:
                unified["subTypes"][ressub1] = self.unifyResults(res1["subTypes"][ressub1],
                                                                 res2["subTypes"][ressub1])
            else:
                unified["subTypes"][ressub1] = res1["subTypes"][ressub1]
        
        for ressub2 in res2["subTypes"]:
            if not ressub2 in res1["subTypes"]:
                unified["subTypes"][ressub2] = res2["subTypes"][ressub2]
        
        return unified


if len(sys.argv) > 2:
    dcCondenser = DataCondenser()
    dcCondenser.condenseData(sys.argv[1], sys.argv[2:])
else:
    print "Usage:", sys.argv[0], "<output-filename> <source-dir-1> [<source-dir-2> [<source-dir-3> [...]]]"
