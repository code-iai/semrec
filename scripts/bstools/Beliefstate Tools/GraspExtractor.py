from LogReader import LogReader
import pickle
import math


class GraspExtractor:
    def __init__(self):
        self.rdrLog = LogReader()
    
    def getDesignatorSuccessor(self, strDesignatorID):
        desig = self.di[strDesignatorID]
        if desig:
            successors = desig.tagAttributeValues("knowrob:successorDesignator", "rdf:resource")
            if successors and len(successors) > 0:
                return successors[0].split("#")[1]
    
    def getNamedDesignator(self, dataDesignators, strName):
        for designator in dataDesignators:
            if designator["designator"]["_id"] == strName:
                return designator["designator"]
    
    def processPerform(self, owlPerform):
        desigsGraspDetails = owlPerform.tagAttributeValues("knowrob:graspDetails", "rdf:resource")
        
        if len(desigsGraspDetails) > 0:
            desigGraspDetails = self.getNamedDesignator(self.log.getDesignatorData(), desigsGraspDetails[0].split("#")[1])["GRASP"]
            
            dicGraspPose = desigGraspDetails["GRASP-POSE"]
            dicPregraspPose = desigGraspDetails["PREGRASP-POSE"]
            dicObjectPose = desigGraspDetails["OBJECT-POSE"]
            strObjectName = desigGraspDetails["OBJECT-NAME"]
            strSide = desigGraspDetails["ARM"]
            strEffort = desigGraspDetails["EFFORT"]
            strGraspType = desigGraspDetails["GRASP-TYPE"]["QUOTE"]
            
            print " -- Grasp action --"
            
            timeSpan = owlPerform.timeSpan()
            print "Time elapsed  :", (float(timeSpan[1]) - float(timeSpan[0])), "seconds"
            
            if owlPerform.taskSuccess():
                print "Success       : True"
            else:
                print "Success       : False"
            
            print "Side          :", strSide
            print "Grasp Type    :", strGraspType
            print "Object Name   :", strObjectName
            print "Object Pose   :"
            self.printPose(dicObjectPose)
            print "Grasp Pose    :"
            self.printPose(dicGraspPose)
            print "Pregrasp Pose :"
            self.printPose(dicPregraspPose)
            print
    
    def extractGrasps(self, strPath):
        # Load Log
        self.log = self.rdrLog.loadLog(strPath)
        self.tti = self.log.getOwlData()["task-tree-individuals"]
        self.di = self.log.getOwlData()["designator-individuals"]
        annot = self.log.getOwlData()["annotation"]
        meta = self.log.getOwlData()["metadata"]
        
        for key in self.tti:
            owlIndiv = self.tti[key]
            
            if owlIndiv.type() == "AnnotationInformation":
                annot = owlIndiv
                if annot and meta: break
            elif owlIndiv.type() == "RobotExperiment":
                meta = owlIndiv
                if annot and meta: break
        
        if annot and meta:
            for indiv in self.tti:
                if self.tti[indiv].taskContext() == "GRASP":
                    self.processPerform(self.tti[indiv])

    def printPose(self, pose):
        print "   Frame       :", pose["header"]["frame_id"] + "\n" + \
              "   Position    : x =", str(pose["pose"]["position"]["x"]) + "\n" + \
              "                 y =", str(pose["pose"]["position"]["y"]) + "\n" + \
              "                 z =", str(pose["pose"]["position"]["z"]) + "\n" + \
              "   Orientation : x =", str(pose["pose"]["orientation"]["x"]) + "\n" + \
              "                 y =", str(pose["pose"]["orientation"]["y"]) + "\n" + \
              "                 z =", str(pose["pose"]["orientation"]["z"]) + "\n" + \
              "                 w =", str(pose["pose"]["orientation"]["w"])
