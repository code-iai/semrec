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
    
    def addIgnoredParameter(self, strIgnoredAttribute):
        self.arrIgnoredAttributes.append(strIgnoredAttribute)
    
    def isAttributeIgnored(self, strIgnoredAttribute):
        return strIgnoredAttribute in self.arrIgnoredAttributes
    
    def addValueToAttribute(self, strAttribute, idValue):
        if not strAttribute in self.dicAttributeCollections:
            self.dicAttributeCollections[strAttribute] = []
        
        if not idValue in self.dicAttributeCollections[strAttribute]:
            self.dicAttributeCollections[strAttribute].append(idValue)
    
    def isAttributeNumeric(self, strAttribute):
        bIsNumeric = True
        
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
        
        for strAttribute in self.dicAttributeCollections:
            if not self.isAttributeIgnored(strAttribute):
                dicLines.append(self.assembleAttributeLine(strAttribute))
        
        return dicLines
    
    def assembleDataSetLine(self, dsDataSet):
        strLine = ""
        
        for strAttribute in self.dicAttributeCollections:
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
        
        self.arrAnnotatedParameters = []
    
    def addTrackedParameter(self, strParameter):
        self.arrAnnotatedParameters.append(strParameter)
    
    def convertOwlToTrainingData(self, strLogDirectory):
        self.logData = self.rdrLog.loadLog(strLogDirectory)
        self.owlData = self.logData.getOwlData()
        self.designatorData = self.logData.getDesignatorData()
        
        self.tti = self.owlData["task-tree-individuals"]
        self.di = self.owlData["designator-individuals"]
        self.meta = self.owlData["metadata"]
        self.annotation = self.owlData["annotation"]
        
        for strParameter in self.annotation.tagNodeValues("knowrob:annotatedParameterType"):
            self.addTrackedParameter(strParameter)
        
        self.addTrackedParameter("taskContext")
        
        self.tdTrainingData.addIgnoredParameter("_time_created")
        self.tdTrainingData.setRelation("PlanExecution")
        self.walkTree(self.meta)
        self.tdTrainingData.writeTrainingDataToFile(sys.argv[2])
    
    def walkTree(self, ndOriginNode, dsOrigin = DataSet()):
        for strParameter in self.arrAnnotatedParameters:
            arrParameters = ndOriginNode.tagNodeValues("knowrob:" + strParameter)
            
            if len(arrParameters) > 0:
                dsOrigin.setAttributeValue(strParameter, arrParameters[0])
        
        arrFailures = ndOriginNode.failures()
        if len(arrFailures) == 0:
            dsOrigin.setAttributeValue("Result", u"Success")
        else:
            desigFailure = self.tti[arrFailures[0]]
            dsOrigin.setAttributeValue("Result", desigFailure.type())
        
        dsOrigin.setAttributeValue("Duration", unicode(ndOriginNode.time()))
        
        self.tdTrainingData.addDataSet(dsOrigin)
        arrSubActions = ndOriginNode.subActions()
        
        for strSubAction in arrSubActions:
            self.walkTree(self.tti[strSubAction], dsOrigin.copy())


ottdcConverter = OwlToTrainingDataConverter()
ottdcConverter.convertOwlToTrainingData(sys.argv[1])
