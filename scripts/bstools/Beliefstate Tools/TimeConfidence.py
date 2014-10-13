from LogReader import LogReader
import math
import json
import pickle

import numpy as np
import scipy as sp
import scipy.stats

def mean_confidence_interval(data, confidence=0.95):
    a = 1.0*np.array(data)
    n = len(a)
    m, se = np.mean(a), scipy.stats.sem(a)
    h = se * sp.stats.t._ppf((1+confidence)/2., n-1)
    return m, m-h, m+h

class TimeConfidence:
    def __init__(self):
        self.rdrLog = LogReader()
        
    def confidence(self, strPath):
        dataOwl = None
        
        log = self.rdrLog.loadLog(strPath)
        dataOwl = log.getOwlData()
        
        self.tti = dataOwl["task-tree-individuals"]
        owlMeta = dataOwl["metadata"]
        owlAnnot = dataOwl["annotation"]
        
        if owlMeta:
            toplevel_nodes = owlMeta.subActions()
        else:
            print "No meta data in file!"
        
        self.timeSpans = {}
        self.findTimeSpansPerTask(toplevel_nodes)
        
        for ctx in self.timeSpans:
            print ctx, mean_confidence_interval(self.timeSpans[ctx])
    
    def findTimeSpansPerTask(self, nodes):
        for node in nodes:
            owlNode = self.tti[node]
            
            ctx = owlNode.taskContext()
            if not ctx in self.timeSpans:
                self.timeSpans[ctx] = []
            
            self.timeSpans[ctx].append(owlNode.time())
            
            self.findTimeSpansPerTask(owlNode.subActions())
