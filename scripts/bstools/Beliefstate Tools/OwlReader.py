from xml.dom.minidom import parse, parseString
import pickle
import os, sys
import time


class OwlIndividual:
    def __init__(self, domIndividual):
        self.domIndividual = domIndividual

    def name(self, bSplitNamespace = True):
        strName = self.domIndividual.getAttribute("rdf:about")
        if bSplitNamespace:
            return strName.split("#")[1]
        else:
            return strName

    def tagAttributeValues(self, strTagName, strAttributeName):
        arrAttributeValues = []
        domTags = self.domIndividual.getElementsByTagName(strTagName)

        for domTag in domTags:
            strAttributeValue = domTag.getAttribute(strAttributeName)

            if strAttributeValue:
                arrAttributeValues.append(strAttributeValue)

        return arrAttributeValues
    
    def tagNodeValues(self, strTagName):
        arrNodeValues = []
        domTags = self.domIndividual.getElementsByTagName(strTagName)
        
        for domTag in domTags:
            strNodeValue = domTag.firstChild.nodeValue
            arrNodeValues.append(strNodeValue)
        
        return arrNodeValues
    
    def type(self, bSplitNamespace = True):
        arrTypes = self.tagAttributeValues("rdf:type", "rdf:resource")
        if len(arrTypes) > 0:
            strType = arrTypes[0]

            if bSplitNamespace:
                return strType.split("#")[1]
            else:
                return strType
    
    def annotatedParameterTypes(self):
        return self.tagNodeValues("knowrob:annotatedParameterType")
    
    def annotatedParameters(self, bSingularParameters = False):
        arrParams = {}
        params = self.annotatedParameterTypes()
        
        for param in params:
            if bSingularParameters: 
                arrParams[param] = self.annotatedParameterValue(param)
            else:
                arrParams[param] = self.annotatedParameterValues(param)
        
        return arrParams
    
    def subActions(self):
        subs = self.tagAttributeValues("knowrob:subAction", "rdf:resource")
        arrSubs = []
        for sub in subs:
            arrSubs.append(sub.split("#")[1])
        
        return arrSub

    def annotatedParameterValues(self, strParam):
        return self.tagNodeValues("knowrob:" + strParam)
    
    def annotatedParameterValue(self, strParam):
        values = self.annotatedParameterValues(strParam)
        
        if len(values) > 0:
            return values[0]
        
        return None
    
    def taskContext(self):
        tnv = self.tagNodeValues("knowrob:taskContext")
        
        if tnv:
            return tnv[0]
        
        return None

    def goalContext(self):
        tnv = self.tagNodeValues("knowrob:goalContext")
        
        if tnv:
            return tnv[0]
        
        return None
    
    def failures(self):
        failures_pre = self.tagAttributeValues("knowrob:eventFailure", "rdf:resource")
        arrFailures = []
        for failure in failures_pre:
            arrFailures.append(failure.split("#")[1])
        
        return arrFailures
    
    def caughtFailures(self):
        caught_pre = self.tagAttributeValues("knowrob:caughtFailure", "rdf:resource")
        arrCaughtFailures = []
        for caught_failure in caught_pre:
            arrCaughtFailures.append(caught_failure.split("#")[1])
        
        return arrCaughtFailures
    
    def taskSuccess(self):
        success = self.tagNodeValues("knowrob:taskSuccess")[0]
        
        if success == "true":
            return True
        
        return False
    
    def timeSpan(self):
        timeStart = self.tagAttributeValues("knowrob:startTime", "rdf:resource")
        timeEnd = self.tagAttributeValues("knowrob:endTime", "rdf:resource")
        
        if len(timeStart) > 0 and len(timeEnd) > 0:
            return [timeStart[0].split("#")[1].split("_")[1], timeEnd[0].split("#")[1].split("_")[1]]

    def time(self):
        arrTimespan = self.timeSpan()

        if arrTimespan:
            if len(arrTimespan) == 2:
                return float(arrTimespan[1]) - float(arrTimespan[0])

        return 0

    def subActions(self, bSplitNamespace = True):
        arrReturn = []
        arrSubActions = self.tagAttributeValues("knowrob:subAction", "rdf:resource")

        for strSubAction in arrSubActions:
            if bSplitNamespace:
                arrReturn.append(strSubAction.split("#")[1])
            else:
                arrReturn.append(strSubAction)

        return arrReturn


class OwlReader:
    def __init__(self):
        pass
    
    def loadOwl(self, strFile):
        return self.crawlOwl(parse(strFile))
    
    def crawlOwl(self, domOwl):
        arrIndividuals = domOwl.getElementsByTagName("owl:NamedIndividual")
        
        arrOwlTaskTreeIndividuals = {}
        arrOwlDesignatorIndividuals = {}
        arrOwlAuxIndividuals = {}
        owlAnnotation = None
        owlMetaData = None
        
        for domIndividual in arrIndividuals:
            owlIndividual = OwlIndividual(domIndividual)
            
            if owlIndividual.type() == "CRAMDesignator":
                arrOwlDesignatorIndividuals[owlIndividual.name()] = owlIndividual
            elif owlIndividual.type() == "CameraImage" or owlIndividual.type() == "HumanScaleObject":
                arrOwlAuxIndividuals[owlIndividual.name()] = owlIndividual
            elif owlIndividual.type() == "AnnotationInformation":
                owlAnnotation = owlIndividual
            elif owlIndividual.type() == "RobotExperiment":
                owlMetaData = owlIndividual
            else:
                arrOwlTaskTreeIndividuals[owlIndividual.name()] = owlIndividual
        
        #dicTaskTree = self.createTaskTree(arrOwlTaskTreeIndividuals)
        
        return {#"task-tree": dicTaskTree,
                "task-tree-individuals": arrOwlTaskTreeIndividuals,
                "designator-individuals": arrOwlDesignatorIndividuals,
                "aux-individuals": arrOwlAuxIndividuals,
                "annotation": owlAnnotation,
                "metadata": owlMetaData}
    
    def createTaskTree(self, arrOwlTaskTreeIndividuals):
        arrToplevelIndividuals = {}

        for strIndividualName in arrOwlTaskTreeIndividuals:
            bFound = False

            for strIndividualNameCheckAgainst in arrOwlTaskTreeIndividuals:
                owlCheckAgainst = arrOwlTaskTreeIndividuals[strIndividualNameCheckAgainst]

                if not strIndividualName == strIndividualNameCheckAgainst:
                    arrSubActions = owlCheckAgainst.subActions()

                    if strIndividualName in arrSubActions:
                        bFound = True
                        break

            if not bFound:
                arrToplevelIndividuals[strIndividualName] = arrOwlTaskTreeIndividuals[strIndividualName]

        arrTaskTrees = {}
        for strToplevelIndividualName in arrToplevelIndividuals:
            arrTaskTrees[strToplevelIndividualName] = self.createSubTaskTree(arrOwlTaskTreeIndividuals, strToplevelIndividualName)

        return arrTaskTrees

    def createSubTaskTree(self, arrOwlTaskTreeIndividuals, strParent):
        arrTree = {"children": {}}

        if strParent in arrOwlTaskTreeIndividuals:
            owlParent = arrOwlTaskTreeIndividuals[strParent]
            arrTree["time"] = owlParent.time()

            for strSubAction in owlParent.subActions():
                arrTree["children"][strSubAction] = self.createSubTaskTree(arrOwlTaskTreeIndividuals, strSubAction)

        return arrTree
