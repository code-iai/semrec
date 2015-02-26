#!/usr/bin/python

import pickle
import sys

sys.setrecursionlimit(100000)
restypescnt = {}
paramvals = {}
ignore_params = ["_time_created"]


def collectTagNames(individual):
    print individual

def recurse(f, indiv_class, individual, level = 0, scout = False):
    strLine = ""
    
    strLine += indiv_class
    strLine += ", " + str(level)
    
    for indiv in individual["individuals"]:
        strLineIndividual = strLine
        
        if scout:
            for param in individual["individuals"][indiv]["parameters"]:
                if not param in ignore_params:
                    if not param in paramvals:
                        paramvals[param] = []
                    
                    if not individual["individuals"][indiv]["parameters"][param] in paramvals[param]:
                        paramvals[param].append(individual["individuals"][indiv]["parameters"][param])
        
        if individual["individuals"][indiv]["failure"] == "":
            strLineIndividual += ", Success"
        else:
            strLineIndividual += ", " + individual["individuals"][indiv]["failure"]
        
        params_found = False
        for param in model["parameters"]:
            if not param in ignore_params:
                if param in individual["individuals"][indiv]["parameters"]:
                    params_found = True
                    strLineIndividual += ", " + individual["individuals"][indiv]["parameters"][param]
                else:
                    strLineIndividual += ", ?"
        
        strLineIndividual += "\n"
        
        if not scout:
            if params_found:
                f.write(strLineIndividual)
    
    for subtype in individual["subTypes"]:
        recurse(f, subtype, individual["subTypes"][subtype], level + 1, scout)

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

def isParameterNumeric(param):
    is_numeric = True
    
    if param in paramvals:
        for paramval in paramvals[param]:
            if not paramval.isnumeric():
                is_numeric = False
                break
    
    return is_numeric

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

    indiv_class = model["model"].keys()[0]
    recurse(f, indiv_class, model["model"][indiv_class], 0, True)
    print paramvals
    
    strline = "";
    for tasktype in dicResultTypes:
        if strline != "":
            strline += ", "
        
        strline += tasktype
    
    strline += "}\n"
    f.write(strline)
    
    for param in paramvals:
        f.write("@attribute " + param + " ")
        
        if isParameterNumeric(param):
            f.write("NUMERIC")
        else:
            nominallist = ""
            for paramval in paramvals[param]:
                if nominallist != "":
                    nominallist += ", "
                
                nominallist += paramval
            f.write("{" + nominallist + "}")
        
        f.write("\n")
    
    f.write("\n@data\n")
    
    recurse(f, indiv_class, model["model"][indiv_class], 0, False)
    
    print restypescnt
