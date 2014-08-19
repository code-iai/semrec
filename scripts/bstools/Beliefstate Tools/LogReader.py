from OwlReader import OwlReader
from Log import Log


class LogReader:
    def __init__(self):
        self.rdrOwl = OwlReader()

    def loadLog(self, strPath):
        log = Log()

        log.setOwlData(self.rdrOwl.loadOwl(strPath + "/cram_log.owl"))

        return log