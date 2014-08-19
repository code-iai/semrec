from xml.dom.minidom import parse, parseString


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

    def type(self, bSplitNamespace = True):
        arrTypes = self.tagAttributeValues("rdf:type", "rdf:resource")
        if len(arrTypes) > 0:
            strType = arrTypes[0]

            if bSplitNamespace:
                return strType.split("#")[1]
            else:
                return strType

    def timeSpan(self):
        timeStart = self.tagAttributeValues("knowrob:startTime", "rdf:resource")
        timeEnd = self.tagAttributeValues("knowrob:endTime", "rdf:resource")

        if len(timeStart) > 0 and len(timeEnd) > 0:
            return [timeStart[0].split("#")[1].split("_")[1], timeEnd[0].split("#")[1].split("_")[1]]

    def time(self):
        arrTimespan = self.timeSpan()

        if len(arrTimespan) == 2:
            return int(arrTimespan[1]) - int(arrTimespan[0])
        else:
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
        arrIndividuals = domOwl.getElementsByTagName("owl:namedIndividual")

        arrOwlTaskTreeIndividuals = {}
        arrOwlDesignatorIndividuals = {}
        arrOwlAuxIndividuals = {}
        for domIndividual in arrIndividuals:
            owlIndividual = OwlIndividual(domIndividual)

            if owlIndividual.type() == "CRAMDesignator":
                arrOwlDesignatorIndividuals[owlIndividual.name()] = owlIndividual
            elif owlIndividual.type() == "CameraImage" or owlIndividual.type() == "HumanScaleObject":
                arrOwlAuxIndividuals[owlIndividual.name()] = owlIndividual
            else:
                arrOwlTaskTreeIndividuals[owlIndividual.name()] = owlIndividual

        dicTaskTree = self.createTaskTree(arrOwlTaskTreeIndividuals)

        return {"task-tree": dicTaskTree,
                "task-tree-individuals": arrOwlTaskTreeIndividuals,
                "designator-individuals": arrOwlDesignatorIndividuals,
                "aux-individuals": arrOwlAuxIndividuals}

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