#!/usr/bin/python

from LogReader import LogReader
import pickle
import sys

sys.setrecursionlimit(100000)


class DataSet:
    def __init__(self):
        self.dicFields = {}
    
    def setAttributeValue(self, strField, idValue):
        self.dicFields[strField] = idValue
    
    def getAttributeValue(self, strField):
        if strField in self.dicFields:
            return self.dicFields[strField]
        else:
            return "?"
    
    def attributes(self):
        arrAttributes = []
        for strAttribute in self.dicFields:
            arrAttributes.append(strAttribute)
        
        return arrAttributes
    
    def copy(self):
        dsCopy = DataSet()
        for strAttribute in self.dicFields:
            dsCopy.setAttributeValue(strAttribute, self.dicFields[strAttribute])
        
        return dsCopy


class TrainingData:
    def __init__(self):
        self.dicAttributeCollections = {}
        self.arrDataSets = []
        self.arrIgnoredAttributes = []
        self.strFirstAttribute = ""
    
    def selectFirstAttribute(self, strAttribute):
        self.strFirstAttribute = strAttribute
    
    def addIgnoredParameter(self, strIgnoredAttribute):
        self.arrIgnoredAttributes.append(strIgnoredAttribute)
    
    def isAttributeIgnored(self, strIgnoredAttribute):
        return strIgnoredAttribute in self.arrIgnoredAttributes
    
    def registerAttribute(self, strAttribute):
        if not strAttribute in self.dicAttributeCollections:
            self.dicAttributeCollections[strAttribute] = []
    
    def addValueToAttribute(self, strAttribute, idValue):
        self.registerAttribute(strAttribute)
        
        if not idValue in self.dicAttributeCollections[strAttribute]:
            self.dicAttributeCollections[strAttribute].append(idValue)
    
    def isAttributeNumeric(self, strAttribute):
        bIsNumeric = len(self.dicAttributeCollections[strAttribute]) > 0
        
        if strAttribute in self.dicAttributeCollections:
            for idValue in self.dicAttributeCollections[strAttribute]:
                try:
                    float(idValue)
                except ValueError:
                    bIsNumeric = False
                    break
        
        return bIsNumeric
    
    def setRelation(self, strRelation):
        self.strRelation = strRelation
    
    def addDataSet(self, dsDataSet):
        dicAttributes = dsDataSet.attributes()
        
        for strAttribute in dicAttributes:
            self.addValueToAttribute(strAttribute, dsDataSet.getAttributeValue(strAttribute))
        
        self.arrDataSets.append(dsDataSet)
    
    def assembleAttributeLine(self, strAttribute):
        strLine = ""
        
        if strAttribute in self.dicAttributeCollections:
            if not self.isAttributeIgnored(strAttribute):
                strLine += "@attribute " + strAttribute + " "
                
                if self.isAttributeNumeric(strAttribute):
                    strLine += "NUMERIC"
                else:
                    strNominalValues = ""
                    
                    for idValue in self.dicAttributeCollections[strAttribute]:
                        if strNominalValues != "":
                            strNominalValues += ", "
                        
                        strNominalValues += idValue
                    
                    strLine += "{" + strNominalValues + "}"
        
        return strLine
    
    def assembleAttributeLines(self):
        dicLines = []
        
        if self.strFirstAttribute != "":
            dicLines.append(self.assembleAttributeLine(self.strFirstAttribute))
        
        for strAttribute in self.dicAttributeCollections:
            if self.strFirstAttribute == "" or (self.strFirstAttribute != "" and self.strFirstAttribute != strAttribute):
                if not self.isAttributeIgnored(strAttribute):
                    dicLines.append(self.assembleAttributeLine(strAttribute))
        
        return dicLines
    
    def assembleDataSetLine(self, dsDataSet):
        strLine = ""
        
        if self.strFirstAttribute != "":
            strLine += dsDataSet.getAttributeValue(self.strFirstAttribute)
        
        for strAttribute in self.dicAttributeCollections:
            if self.strFirstAttribute == "" or (self.strFirstAttribute != "" and self.strFirstAttribute != strAttribute):
                if not self.isAttributeIgnored(strAttribute):
                    if strLine != "":
                        strLine += ", "
                    
                    strLine += dsDataSet.getAttributeValue(strAttribute)
        
        return strLine
    
    def assembleDataSetLines(self):
        dicLines = []
        
        for dsDataSet in self.arrDataSets:
            dicLines.append(self.assembleDataSetLine(dsDataSet))
        
        return dicLines
    
    def writeTrainingDataToFile(self, strFilename):
        with open(strFilename + ".arff", "wb") as fOut:
            fOut.write("@relation " + self.strRelation + "\n\n")
            
            dicAttributeLines = self.assembleAttributeLines()
            for strLine in dicAttributeLines:
                fOut.write(strLine + "\n")
            
            fOut.write("\n@data\n")
            
            dicDataSetLines = self.assembleDataSetLines()
            for strLine in dicDataSetLines:
                fOut.write(strLine + "\n")


class OwlToTrainingDataConverter:
    def __init__(self):
        self.tdTrainingData = TrainingData()
        self.rdrLog = LogReader()
        
        self.arrIgnoredTasks = []
        self.arrAnnotatedParameters = []
    
    def setTaskIgnored(self, strTask):
        if not strTask in self.arrIgnoredTasks:
            self.arrIgnoredTasks.append(strTask)
    
    def addTrackedParameter(self, strParameter):
        self.arrAnnotatedParameters.append(strParameter)
    
    def convertOwlToTrainingData(self, arrLogDirectories):
        self.addTrackedParameter("taskContext")
        
        self.setTaskIgnored(u"WITH-FAILURE-HANDLING")
        self.setTaskIgnored(u"WITH-DESIGNATORS")
        self.setTaskIgnored(u"TAG")
        self.setTaskIgnored(u"UIMA-PERCEIVE")
        self.setTaskIgnored(u"GOAL-MONITOR-ACTION")
        self.setTaskIgnored(u"GOAL-ACHIEVE")
        self.setTaskIgnored(u"GOAL-PERFORM")
        self.setTaskIgnored(u"GOAL-PERFORM-ON-PROCESS-MODULE")
        self.setTaskIgnored(u"PERFORM-ACTION-DESIGNATOR")
        self.setTaskIgnored(u"REPLACEABLE-FUNCTION-NAVIGATE")
        
        self.setTaskIgnored(u"AT-LOCATION")
        self.setTaskIgnored(u"VOLUNTARY-BODY-MOVEMENT-ARMS")
        self.setTaskIgnored(u"MOTION-PLANNING")
        self.setTaskIgnored(u"MOTION-EXECUTION")
        self.setTaskIgnored(u"PUTDOWN")
        self.setTaskIgnored(u"VOLUNTARY-BODY-MOVEMENT-HEAD")
        self.setTaskIgnored(u"OPEN-GRIPPER")
        self.setTaskIgnored(u"CLOSE-GRIPPER")
        
        self.tdTrainingData.registerAttribute(u"Result")
        self.tdTrainingData.selectFirstAttribute(u"Result")
        
        self.tdTrainingData.addIgnoredParameter("_time_created")
        self.tdTrainingData.setRelation("PlanExecution")
        
        for strLogDirectory in arrLogDirectories:
            self.logData = self.rdrLog.loadLog(strLogDirectory)
            self.owlData = self.logData.getOwlData()
            self.designatorData = self.logData.getDesignatorData()
            
            self.tti = self.owlData["task-tree-individuals"]
            self.di = self.owlData["designator-individuals"]
            self.meta = self.owlData["metadata"]
            self.annotation = self.owlData["annotation"]
            
            for strParameter in self.annotation.tagNodeValues("knowrob:annotatedParameterType"):
                self.addTrackedParameter(strParameter)
        
            self.walkTree(self.meta)
        
        self.tdTrainingData.writeTrainingDataToFile(sys.argv[1])
    
    def isTaskIgnored(self, strTask):
        return strTask in self.arrIgnoredTasks
    
    def walkTree(self, ndOriginNode, dsOrigin = DataSet()):
        for strParameter in self.arrAnnotatedParameters:
            arrParameters = ndOriginNode.tagNodeValues("knowrob:" + strParameter)
            
            if len(arrParameters) > 0:
                dsOrigin.setAttributeValue(strParameter, arrParameters[0])
        
        #dsOrigin.setAttributeValue(u"Duration", unicode(ndOriginNode.time()))
        
        arrSubActions = ndOriginNode.subActions()
        
        strSubResult = u"Success"
        for strSubAction in arrSubActions:
            strSubResultTemp = self.walkTree(self.tti[strSubAction], dsOrigin.copy())
            
            if strSubResultTemp != u"Success":
                strSubResult = strSubResultTemp
        
        arrFailures = ndOriginNode.failures()
        if len(arrFailures) == 0:
            dsOrigin.setAttributeValue(u"Result", strSubResult)
        else:
            desigFailure = self.tti[arrFailures[0]]
            dsOrigin.setAttributeValue(u"Result", desigFailure.type())
        
        if not self.isTaskIgnored(dsOrigin.getAttributeValue(u"taskContext")):
            self.tdTrainingData.addDataSet(dsOrigin)
        
        return dsOrigin.getAttributeValue(u"Result")


ottdcConverter = OwlToTrainingDataConverter()
ottdcConverter.convertOwlToTrainingData(sys.argv[2:])
