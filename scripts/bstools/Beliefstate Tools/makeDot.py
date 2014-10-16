#!/usr/bin/python

import sys
from LogReader import LogReader

lr = LogReader()

log = lr.loadLog(sys.argv[1])
od = log.getOwlData()
tti = od["task-tree-individuals"]

printable_types = ["CRAMAchieve"]
colors = ["#ddffff", "#ffddff", "#ffffdd", "#ddddff", "#ffdddd", "#ddffdd", "#dddddd", "#ffffff"]
color_assignments = {}
color_index = -1

def printIndividual(individual, parent_name):
    global color_index
    global color_assignments
    
    strdot = ""
    
    strlbl = individual.goalContext()
    if not strlbl:
        strlbl = individual.name()
    
    # if len(individual.failures()) > 0:
    #     color = "#ffdddd"
    # else:
    #     color = "#ddffdd"
    distinguisher = individual.goalContext()
    if distinguisher in color_assignments:
        color = color_assignments[distinguisher]
    else:
        color_index += 1
        if color_index >= len(colors):
            color_index = 0
        
        color_assignments[distinguisher] = colors[color_index]
        color = colors[color_index]
    
    strdot += "  " + individual.name() + " [shape=Mrecord, style=filled, fillcolor=\"" + color + "\" label=\"" + strlbl + "\"];\n"
    
    if parent_name:
        strdot += "  " + parent_name + " -> " + individual.name() + ";\n"
    
    strdot += "  \n";
    
    return strdot

def printTree(individual, parent_name = None):
    strdot = ""
    
    if individual.type() in printable_types:
        strdot += printIndividual(individual, parent_name)
        
        for subaction in individual.subActions():
            strdot += printTree(tti[subaction], individual.name())
    else:
        for subaction in individual.subActions():
            strdot += printTree(tti[subaction], parent_name)
    
    return strdot

strdot = "digraph plangraph {\n"
strdot += "  rankdir=LR\n";
strdot += "  node_toplevel [label=\"Toplevel\"];\n\n";

if od["metadata"]:
    toplevel_nodes = od["metadata"].subActions()
else:
    toplevel_nodes = []
    subactions = []
    for ti in tti:
        subactions += tti[ti].subActions()
    
    for ti in tti:
        if not ti in subactions:
            toplevel_nodes.append(ti)

for toplevel in toplevel_nodes:
    strdot += printTree(tti[toplevel], "node_toplevel")

strdot += "}"

print strdot
