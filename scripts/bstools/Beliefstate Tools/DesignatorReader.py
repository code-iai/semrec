import json

class DesignatorReader:
    def __init__(self):
        pass
    
    def loadDesignators(self, strFile):
        entries = []
        with open(strFile) as f:
            for line in f:
                entries.append(json.loads(line))
        
        return entries
