from LogReader import LogReader
import pickle
import math


class DataExtractor:
    def __init__(self):
        self.rdrLog = LogReader()
        
    def extractData(self, strPath):
        log = self.rdrLog.loadLog(strPath)
        self.tti = log.getOwlData()["task-tree-individuals"]
        annot = None
        meta = None
        
        for key in self.tti:
            owlIndiv = self.tti[key]
            
            if owlIndiv.type() == "AnnotationInformation":
                annot = owlIndiv
                if annot and meta: break
            elif owlIndiv.type() == "RobotExperiment":
                meta = owlIndiv
                if annot and meta: break
        
        if annot and meta:
            params = annot.tagNodeValues("knowrob:annotatedParameterType")
            
            param_set = {}
            for param in params:
                param_set[param] = "?"
            
            strHeadline = "TASK-CONTEXT, RESULT"
            for param in params:
                strHeadline += ", " + param
            
            print strHeadline
            
            toplevelnodes = meta.tagAttributeValues("knowrob:subAction", "rdf:resource")
            
            for toplevelnode in toplevelnodes:
                node_key = toplevelnode.split("#")[1]
                self.printNode(node_key, param_set)
    
    def printNode(self, node_key, param_set):
        owlIndiv = self.tti[node_key]
        subactions = owlIndiv.tagAttributeValues("knowrob:subAction", "rdf:resource")
        
        for param in param_set:
            tnv = owlIndiv.tagNodeValues("knowrob:" + param)
            
            if len(tnv) > 0:
                param_set[param] = tnv[0]
        
        strLine = ""
        strLine += owlIndiv.tagNodeValues("knowrob:taskContext")[0]
        
        failures = owlIndiv.tagAttributeValues("knowrob:eventFailure", "rdf:resource")
        
        if len(failures) > 0:
            failure = failures[0].split("#")[1]
            owlFailure = self.tti[failure]
            
            strLine += ", " + owlFailure.type()
        else:
            strLine += ", Success"
        
        for param in param_set:
            strLine += ", " + param_set[param]
        
        for subaction in subactions:
            self.printNode(subaction.split("#")[1], param_set)
        
        print strLine
