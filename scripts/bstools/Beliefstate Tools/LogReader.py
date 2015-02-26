from OwlReader import OwlReader
from DesignatorReader import DesignatorReader
from Log import Log


class LogReader:
    def __init__(self):
        self.rdrOwl = OwlReader()
        self.rdrDesig = DesignatorReader()

    def loadLog(self, strPath):
        log = Log()

        log.setOwlData(self.rdrOwl.loadOwl(strPath + "/cram_log.owl"))
        log.setDesignatorData(self.rdrDesig.loadDesignators(strPath + "/logged_designators.json"))
        
        return log
