#!/usr/bin/python

import sys
from PyQt4 import QtCore, QtGui

from OwlReader import OwlReader
from DesignatorReader import DesignatorReader
from Log import Log


from Visualization_ui import Ui_MainWindow


class ExperienceProcessor(QtGui.QMainWindow):
    def __init__(self, parent=None):
        self.rdrOwl = OwlReader()
        self.rdrDesig = DesignatorReader()
        
        self.arrExperiences = []
        self.dicEntities = {}
        
        self.app = QtGui.QApplication(sys.argv)
        QtGui.QWidget.__init__(self, parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        
        self.show()
        
        self.loadExperience("/home/winkler/ros/catkin/src/semrec/scripts/bstools/Beliefstate Tools/Datasets/ds4/cram_log.owl", "/home/winkler/ros/catkin/src/semrec/scripts/bstools/Beliefstate Tools/Datasets/ds4/logged_designators.json")
        
        sys.exit(self.app.exec_())
    
    def addExperience(self, expAdd):
        self.arrExperiences.append(expAdd)
    
    def loadExperience(self, strOwlFile, strDesignatorFile):
        logReturn = Log()
        
        logReturn.setOwlData(self.rdrOwl.loadOwl(strOwlFile))
        logReturn.setDesignatorData(self.rdrDesig.loadDesignators(strDesignatorFile))
        
        self.addExperience(logReturn)
    
    def update(self):
        self.updateEntities()
        self.renderCanvas()
    
    def updateEntities(self):
        pass
    
    def renderCanvas(self):
        for strName in self.dicEntities:
            self.renderEntity(dicEntities[strName])
    
    def renderEntity(self):
        pass
