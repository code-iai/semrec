#!/usr/bin/python

import sys
from memorycondenser.MemoryCondenser import MemoryCondenser


class MCController:
    def __init__(self):
        self.mcCondenser = MemoryCondenser()
    
    def loadMemory(self, strOwl, strDesig):
        self.mcCondenser.loadExperience(strOwl, strDesig)
    
    def start(self):
        count = self.mcCondenser.countExperiences()
        
        if count > 0:
            print_data = False
            
            if len(sys.argv) > 2:
                if sys.argv[2] == "data":
                    print_data = True
            
            if sys.argv[1] == "experiences":
                self.mcCondenser.printExperiences(dot=not print_data)
            elif sys.argv[1] == "condensed":
                self.mcCondenser.injectExperiences(data=print_data)
            elif sys.argv[1] == "deduced":
                self.mcCondenser.injectExperiences(deduced=True, data=print_data)
        else:
            print "No experiences loaded, nothing to condense"


if __name__ == "__main__":
    mcctrl = MCController()
    
    #mcctrl.loadMemory("/home/winkler/sr_experimental_data/exp-2015-07-31_13-17-25/cram_log.owl", "")
    #mcctrl.loadMemory("/home/winkler/sr_experimental_data/exp-2015-08-05_13-53-18/cram_log.owl", "")
    #mcctrl.loadMemory("/home/winkler/sr_experimental_data/exp-2015-08-05_18-41-58/cram_log.owl", "")
    mcctrl.loadMemory("/home/winkler/sr_experimental_data/exp-2015-08-05_18-43-28/cram_log.owl", "")
    #mcctrl.loadMemory("/home/winkler/sr_experimental_data/exp-2015-08-06_12-09-35/cram_log.owl", "")
    #mcctrl.loadMemory("/home/winkler/sr_experimental_data/exp-2015-08-06_12-13-09/cram_log.owl", "")

    
    mcctrl.start()
