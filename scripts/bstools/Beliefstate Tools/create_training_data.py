#!/usr/bin/python

import pickle
import sys

sys.setrecursionlimit(100000)
restypescnt = {}


def recurse(f, indiv_class, individual, level = 0):
    strLine = ""
    
    strLine += indiv_class
    strLine += ", " + str(level)
    
    for indiv in individual["individuals"]:
        strLineIndividual = strLine
        
        if individual["individuals"][indiv]["failure"] == "":
            strLineIndividual += ", Success"
        else:
            strLineIndividual += ", " + individual["individuals"][indiv]["failure"]
        
        params_found = False
        for param in model["parameters"]:
            if param in individual["individuals"][indiv]["parameters"]:
                params_found = True
                strLineIndividual += ", " + individual["individuals"][indiv]["parameters"][param]
            else:
                strLineIndividual += ", ?"
        
        strLineIndividual += "\n"
        
        if params_found:
            f.write(strLineIndividual)
    
    for subtype in individual["subTypes"]:
        recurse(f, subtype, individual["subTypes"][subtype], level + 1)


def collectTaskTypes(task):
    types = []
    
    for curtype in task:
        subtypes = collectTaskTypes(task[curtype]["subTypes"])
        
        for subtype in subtypes:
            if not subtype in types:
                types.append(subtype)
        
        if not curtype in types:
            types.append(curtype)
    
    return types

def collectResults(task):
    results = []
    
    for curtype in task:
        curresults = collectResults(task[curtype]["subTypes"])
        
        for indiv in task[curtype]["individuals"]:
            if task[curtype]["individuals"][indiv]["failure"] != "":
                fail = task[curtype]["individuals"][indiv]["failure"]
                if not fail in results:
                    results.append(fail)
                
                if not fail in restypescnt:
                    restypescnt[fail] = 0
            
                restypescnt[fail] += 1
        
        for curresult in curresults:
            if not curresult in results:
                results.append(curresult)
    
    return results

with open("generalized_model.pkl", "r") as f:
    model = pickle.load(f)

with open("training_data.arff", "wb") as f:
    f.write("@relation TaskInformation\n\n")
    
    dicTaskTypes = collectTaskTypes(model["model"])
    f.write("@attribute Task {")
    
    strline = "";
    for tasktype in dicTaskTypes:
        if strline != "":
            strline += ", "
        
        strline += tasktype
    
    strline += "}\n"
    f.write(strline)
    
    f.write("@attribute Level NUMERIC\n")

    dicResultTypes = collectResults(model["model"])
    dicResultTypes.append("Success");
    
    f.write("@attribute Result {")
    
    strline = "";
    for tasktype in dicResultTypes:
        if strline != "":
            strline += ", "
        
        strline += tasktype
    
    strline += "}\n"
    f.write(strline)
    
    for param in model["parameters"]:
        if param == "TAGNAME":
            f.write("@attribute " + param + " {PICK, PERCEIVE, NAVIGATE, GRASP, PLACE}\n")
        else:
            f.write("@attribute " + param + " NUMERIC\n")

    f.write("\n@data\n")
    
    indiv_class = model["model"].keys()[0]
    recurse(f, indiv_class, model["model"][indiv_class])
    
    print restypescnt
