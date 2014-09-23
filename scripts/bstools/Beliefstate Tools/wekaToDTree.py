#!/usr/bin/python

## Copyright (c) 2014, Jan Winkler <winkler@cs.uni-bremen.de>
## All rights reserved.
##
## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions are met:
##
## * Redistributions of source code must retain the above copyright
##   notice, this list of conditions and the following disclaimer.
## * Redistributions in binary form must reproduce the above copyright
##   notice, this list of conditions and the following disclaimer in the
##   documentation and/or other materials provided with the distribution.
## * Neither the name of the Institute for Artificial Intelligence/
##   Universitaet Bremen nor the names of its contributors may be used to 
##   endorse or promote products derived from this software without specific 
##   prior written permission.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
## AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
## IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
## ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
## LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
## CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
## SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
## INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
## CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
## ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
## POSSIBILITY OF SUCH DAMAGE.

import fileinput
import re
import json

lines = []
at_tree = False
line_skip = 0

for line_raw in fileinput.input():
    if line_skip > 0:
        line_skip -= 1
    else:
        line = line_raw.rstrip();
        
        if not at_tree:
            if line == "J48 pruned tree":
                at_tree = True
                line_skip = 2
        else:
            if line == "":
                break;
            
            lines.append(line)

data_lines = []

for line in lines:
    level = line.count("|")
    unlevelled_line = line[level * 4:]
    
    m = re.match("(?P<variable>[\w\-]+) (?P<operator>[\<\=\>\!]+) (?P<value>[0-9a-zA-Z\.\-_]+): (?P<result>[\S]+) \((?P<occurrences>[\S]+)\)", unlevelled_line)
    
    if not m: # this is not a result line, re-evaluate as normal line
        m = re.match("(?P<variable>[\w\-]+) (?P<operator>[\<\=\>\!]+) (?P<value>[0-9a-zA-Z\.\-_]+)", unlevelled_line)
    
    data = dict(m.groupdict().items() + {"level": level}.items())
    data_lines.append(data)


def recTB(data_lines, level = 0):
    children = []
    index = 0
    
    for data_line in data_lines:
        if data_line["level"] == level:
            # this one is on the current level - add it to the children
            children.append({"data": data_line, "children": []})
            index += 1
        elif data_line["level"] > level:
            # this is a child of the current level, recurse
            (intres, intindex) = recTB(data_lines[index:], level + 1)
            index += intindex
            
            for intr in intres:
                children[len(children) - 1]["children"].append(intr)
        else:
            # this is the end of our level, return.
            break;
    
    return (children, index)

def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

def formatDTree(branch_children):
    formatted = []
    
    for child in branch_children:
        append_data = {}
        append_data["relation"] = {}
        
        val = child["data"]["value"]
        
        if is_number(val):
            if "." in val:
                val = float(val)
            else:
                val = int(val)
        
        append_data["relation"][child["data"]["operator"]] = {"value": val,
                                                              "variable": child["data"]["variable"]}
        
        if "result" in child["data"]:
            append_data["true"] = [{"result": child["data"]["result"]}]
        else:
            append_data["true"] = formatDTree(child["children"])
        
        formatted.append(append_data)
    
    return formatted

with open("o.json", "wb") as f:
    (items, index) = recTB(data_lines)
    dtree = formatDTree(items)
    json.dump(dtree, f)
