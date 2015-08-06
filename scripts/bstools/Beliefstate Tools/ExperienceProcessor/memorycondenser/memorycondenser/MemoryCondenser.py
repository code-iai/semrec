#!/usr/bin/python

import sys
import json


from OwlReader import OwlReader
from DesignatorReader import DesignatorReader
from Log import Log


class MemoryCondenser:
    def __init__(self, parent=None):
        self.rdrOwl = OwlReader()
        self.rdrDesig = DesignatorReader()
        
        self.arrExperiences = []
    
    def countExperiences(self):
        return len(self.arrExperiences)
    
    def addExperience(self, expAdd):
        self.arrExperiences.append(expAdd)
    
    def loadExperience(self, strOwlFile, strDesignatorFile):
        logReturn = Log()
        
        logReturn.setOwlData(self.rdrOwl.loadOwl(strOwlFile))
        
        if strDesignatorFile != "":
            logReturn.setDesignatorData(self.rdrDesig.loadDesignators(strDesignatorFile))
        
        self.addExperience(logReturn)
    
    def condenseData(self, dataOwl):
        result = None
        
        self.tti = dataOwl["task-tree-individuals"]
        owlMeta = dataOwl["metadata"]
        owlAnnot = dataOwl["annotation"]
        
        if owlMeta:
            result = {"Toplevel" : self.condenseNodes("", owlMeta.subActions())};
        else:
            print "No meta data in file!"
        
        return result
    
    def condenseNodes(self, strParentNode, arrNodes, nLevel = 0):
        arrTypes = {}
        arrIndividuals = {}
         
        for strNode in arrNodes:
            owlNode = self.tti[strNode]
            ident = owlNode.taskContext()
            
            failures = owlNode.failures()
            failure = ""
            if len(failures) > 0:
                failure = self.tti[failures[0]].type()
            
            result = self.condenseNodes(strNode, owlNode.subActions(), nLevel + 1)
            if not ident in arrTypes:
                arrTypes[ident] = result
            else:
                arrTypes[ident] = self.unifyResults(arrTypes[ident], result)
            
            arrTypes[ident]["individuals"][strNode] = {"parameters" : owlNode.annotatedParameters(True),
                                                       "parent" : strParentNode,
                                                       "failure" : failure}
        
        return {"subTypes" : arrTypes,
                "individuals" : {}}
    
    def unifyResults(self, res1, res2):
        resparams = {}
        if len(res1["individuals"]) > 0:
            resparams = res1["individuals"]
        
        if len(res2["individuals"]) > 0:
            resparams = dict(resparams.items() + res2["individuals"].items())
        
        unified = {"subTypes" : {},
                   "individuals" : resparams}
        
        for ressub1 in res1["subTypes"]:
            if ressub1 in res2["subTypes"]:
                unified["subTypes"][ressub1] = self.unifyResults(res1["subTypes"][ressub1],
                                                                 res2["subTypes"][ressub1])
            else:
                unified["subTypes"][ressub1] = res1["subTypes"][ressub1]
        
        for ressub2 in res2["subTypes"]:
            if not ressub2 in res1["subTypes"]:
                unified["subTypes"][ressub2] = res2["subTypes"][ressub2]
        
        return unified
    
    def condense(self):
        arrStartNodes = []
        
        self.tti = {}
        
        for experience in self.arrExperiences:
            owlData = experience.getOwlData()
            metaData = owlData["metadata"]
            arrStartNodes += metaData.subActions()
            
            self.tti.update(owlData["task-tree-individuals"])
        
        self.processed_nodes = []
        tree = self.condenseNodesByContext(arrStartNodes)
        parameters = {}
        
        for node in self.processed_nodes:
            params = self.tti[node].annotatedParameters(bSingularParameters = True)
            
            if len(params) > 0:
                parameters[node] = {}
                for p in params:
                    if not p == "_time_created":
                        parameters[node][p.lstrip("parameter-")] = params[p]
        
        result = {"tree": tree,
                  "parameters": parameters}
        
        #print self.findPaths(result["tree"], "REPLACEABLE-FUNCTION-ACTION-4")
        print result
    
    def sortComparatorActionTime(self, action1, action2):
        if action1.timeSpan[0] > action2.timeSpan[0]:
            return 1
        elif action1.timeSpan[0] == action2.timeSpan[0]:
            return 0
        else:
            return -1
    
    def sortActionsByTime(self, actions):
        actions.sort(self.sortComparatorActionTime)
        
        return actions
    
    def condenseNodesByContext(self, nodes):
        # Separate nodes by their taskContext
        dicContexts = {}
        
        for node in nodes:
            self.processed_nodes.append(node)
            
            owlNode = self.tti[node]
            
            if not owlNode.taskContext() in dicContexts:
                dicContexts[owlNode.taskContext()] = {"nodes": [],
                                                      "terminal-state": []}
            
            dicContexts[owlNode.taskContext()]["nodes"].append(node)
        
        for context in dicContexts:
            all_children = []
            
            for node in dicContexts[context]["nodes"]:
                sub_actions = self.sortActionsByTime(self.tti[node].subActions())
                
                if len(sub_actions) > 0:
                    all_children += sub_actions
                else:
                    dicContexts[context]["terminal-state"].append(node)
            
            dicContexts[context]["children"] = self.condenseNodesByContext(all_children)
        
        return dicContexts
    
    def findPaths(self, condensed, target_context):
        arr_final_path_nodes = self.findFinalPathNodes(condensed, target_context)
        
        
        
        return arr_final_path_nodes
    
    def expandTreeSequence(self, condensed):
        pass
    
    def findFinalPathNodes(self, condensed, target_context):
        arr_final_path_nodes = []
        
        for context in condensed:
            if context == target_context:
                arr_final_path_nodes += condensed[context]["nodes"]
            else:
                arr_final_path_nodes += self.findPaths(condensed[context]["children"], target_context)
        
        return arr_final_path_nodes
    
    def generalizeExperiences(self):
        self.generalizedExperience = {}
        self.tti = {}
        
        arrStartNodes = []
        
        for experience in self.arrExperiences:
            owlData = experience.getOwlData()
            metaData = owlData["metadata"]
            arrStartNodes += metaData.subActions()
            
            self.tti.update(owlData["task-tree-individuals"])
        
        for node in arrStartNodes:
            self.injectActionIntoGeneralizedExperience(node, self.generalizedExperience)
        
        print self.generalizedExperience
    
    def injectActionIntoGeneralizedExperience(self, action, target_branch):
        target_branch["a"] = 5
    
    def dotNode(self, node, first):
        dot = ""
        tti = self.t_tti[node]
        former_subnode = ""
        
        dot += "  {rank=same;"
        for subnode in tti.subActions():
            dot += " " +subnode
        dot += "}\n"
        
        for subnode in tti.subActions():
            dot += "  " + subnode + " [shape=box, label=\"" + self.t_tti[subnode].taskContext() + "\"]\n"
            #dot += "  edge ";
            
            if first == True:
                first = False
                dot += "edge [dir=both, arrowhead=normal, arrowtail=none]"
                dot += "\n  " + node + " -> " + subnode + "\n"
            else:
                pass#dot += "[dir=both, arrowhead=diamond, arrowtail=ediamond]"
            
            
            if not former_subnode == "":
                dot += "  edge [arrowhead=empty, arrowtail=none]\n"
                dot += "  " + former_subnode + " -> " + subnode + "\n"
            
            dot += self.dotNode(subnode, True)
            
            former_subnode = subnode
        
        if len(tti.subActions()) == 0 and tti.nextAction() == None:
            #dot += "  terminal_state_" + node + " [shape=doublecircle, label=\"\"]\n"
            #dot += "  edge [arrowhead=empty, arrowtail=none]\n"
            #dot += "  " + node + " -> terminal_state_" + node + "\n"
            pass
        
        return dot
    
    def printDotExperiences(self):
        for experience in self.arrExperiences:
            self.printDotExperience(experience)
    
    def printDotExperience(self, experience):
        owlData = experience.getOwlData()
        metaData = owlData["metadata"]
        start_nodes = metaData.subActions()
        self.t_tti = owlData["task-tree-individuals"]
        
        dot = "digraph plangraph {\n"
        dot += "  label=\"Original Experiences\"\n"
        dot += "  labeljust=center\n"
        dot += "  labelloc=top\n"
        
        for node in start_nodes:
            dot += "  " + node + " [shape=box, label=\"" + self.t_tti[node].taskContext() + "\"]\n\n"
            
            dot += self.dotNode(node, True)
        
        dot += "}\n"
        
        print dot
    
    def compareSubActions(self, subaction1, subaction2):
        if subaction1 == subaction2:
            return 0
        
        next_action = subaction1
        
        while next_action != None:
            next_action = self.tti[subaction1].nextAction()
            
            if next_action == subaction2:
                return 1
        
        return -1
    
    def sortSubActions(self, subactions):
        subactions.sort(self.compareSubActions)
        
        return subactions
    
    def sortSubActionsList(self, subactions_list):
        sorted_list = []
        
        for subactions in subactions_list:
            sorted_list.append(self.sortSubActions(subactions))
        
        return sorted_list
    
    def generalizeNodes(self, nodes):
        sequences = []
        
        for node in nodes:
            sequences.append(self.tti[node].subActions())
        
        return sequences
    
    def workOnExperiences(self):
        start_nodes = []
        self.tti = {}
        
        for experience in self.arrExperiences:
            owlData = experience.getOwlData()
            metaData = owlData["metadata"]
            
            start_nodes += metaData.subActions()
            self.tti.update(owlData["task-tree-individuals"])
        
        #start_subnodes = []
        #for node in start_nodes:
        #    start_subnodes.append(self.tti[node].subActions())
        
        print self.generalizeNodes(start_nodes)
    
    def injectExperiences(self, deduced = False, data = False):
        self.arrInjected = {}
        self.tti = {}
        
        for experience in self.arrExperiences:
            owlData = experience.getOwlData()
            metaData = owlData["metadata"]
            self.tti.update(owlData["task-tree-individuals"])
            
            for node in metaData.subActions():
                self.injectExperienceNode(node, self.arrInjected)
        
        for experience in self.arrExperiences:
            owlData = experience.getOwlData()
            metaData = owlData["metadata"]
            self.tti.update(owlData["task-tree-individuals"])
            
            for node in metaData.subActions():
                self.checkForOptionalInjectedNodes(self.tti[node].taskContext(), self.arrInjected)
        
        if deduced:
            self.printDeduced(dot = not data)
        else:
            self.printInjected(dot = not data)
    
    def injectExperienceNode(self, node, frame, rootlevel = False):
        ctx = self.tti[node].taskContext()
        
        if not ctx in frame:
            frame[ctx] = {"children": {}, "next-actions" : [], "terminal-state": "false", "start-state": "false", "optional": "false", "instances": 0}
        
        frame[ctx]["instances"] = frame[ctx]["instances"] + 1
        
        sub_nodes = self.tti[node].subActions()
        
        for sub in sub_nodes:
            self.injectExperienceNode(sub, frame[ctx]["children"])
        
        if self.tti[node].nextAction():
            nextCtx = self.tti[self.tti[node].nextAction()].taskContext()
            
            if not nextCtx in frame[ctx]["next-actions"] and not rootlevel:
                frame[ctx]["next-actions"].append(nextCtx)
        else:
            if len(self.tti[node].subActions()) == 0:
                frame[ctx]["terminal-state"] = "true"
        
        if frame[ctx]["start-state"] == "false":
            if self.tti[node].previousAction() == None:
                frame[ctx]["start-state"] = "true"
    
    def checkForOptionalInjectedNodes(self, ctx, frame, parent_instances = -1):
        if frame[ctx]["instances"] < parent_instances:
            frame[ctx]["optional"] = "true"
        
        for child in frame[ctx]["children"]:
            if frame[ctx]["children"][child]["start-state"] == "true":
                self.checkForOptionalInjectedNodes(child, frame[ctx]["children"], frame[ctx]["instances"])
        
        for next_action in frame[ctx]["next-actions"]:
            if next_action in frame and not next_action == ctx:
                self.checkForOptionalInjectedNodes(next_action, frame, frame[ctx]["instances"])
    
    def printDeduced(self, dot = False):
        # TODO: Extend this to use all top-level nodes in case they
        # are different
        deduced = self.expandPathways(self.arrInjected.keys()[0], self.arrInjected)
        
        if dot:
            self.printDotDeduced(deduced)
        else:
            print deduced
    
    def expandPathways(self, ctx, nodes):
        expanded_pathways = []
        
        current_node = [{"node": ctx, "instances": nodes[ctx]["instances"]}]
        children = self.getStartNodes(nodes[ctx]["children"])
        
        had_non_optional_children = False
        for child in children:
            if not children[child]["optional"] == "true":
                had_non_optional_children = True
            
            child_pathways = self.expandPathways(child, nodes[ctx]["children"])
            
            for child_pathway in child_pathways:
                expanded_pathways.append(current_node + child_pathway)
        
        if not had_non_optional_children:
            expanded_pathways.append(current_node)
        
        next_actions = nodes[ctx]["next-actions"]
        final_pathways = []
        
        had_non_optional_next_actions = False
        for next_action in next_actions:
            if next_action != ctx:
                if not nodes[next_action]["optional"] == "true":
                    had_non_optional_next_actions = True
                
                expanded_next_pathways = self.expandPathways(next_action, nodes)
                
                for expanded_next_pathway in expanded_next_pathways:
                    for expanded_pathway in expanded_pathways:
                        final_pathways = final_pathways + [expanded_pathway + expanded_next_pathway]
        
        if not had_non_optional_next_actions:
            final_pathways = final_pathways + expanded_pathways
            # NO! This overwrites the next actions!
        
        return final_pathways
    
    def getStartNodes(self, nodes):
        start_nodes = {}
        
        for node in nodes:
            if nodes[node]["start-state"] == "true":
                start_nodes[node] = nodes[node]
        
        return start_nodes
    
    def deducePathways(self, nodes):
        pathways = []
        
        start_nodes = self.getStartNodes(nodes)
        had_non_optional_start_nodes = False
        
        for node in start_nodes:
            if not start_nodes[node]["optional"] == "true":
                had_non_optional_start_nodes = True
            
            children_pathways = self.findNewChildPathways(node, start_nodes)
            # NOTE: This is the next actions handling bit, and it
            # doesn't scale to more than one next action yet. To fix
            # this, it needs to recurse into the list of those (as
            # each action can have multiple next actions, effectively
            # spreading into a tree).
            
            na_pw = self.deduceNextActionPathways(node, nodes, children_pathways)
            
            pathways = pathways + na_pw
        
        if not had_non_optional_start_nodes and len(nodes) > 0:
            pathways.append([])
        
        return pathways
    
    def findNewChildPathways(self, node, nodes):
        new_pathways = []
        sub_pathways = []
        
        if len(nodes[node]["children"]) > 0:
            sub_pathways = self.deducePathways(nodes[node]["children"])
            #print "AND WE GOT THESE SUB PATHWAYS:"
            #for pathway in sub_pathways:
            #    for item in pathway:
            #        print "  " + item["node"]
                
            #    print
        
        if len(sub_pathways) > 0:
            for sub_pathway in sub_pathways:
                new_pathway = [{"node": node, "instances": nodes[node]["instances"]}] + sub_pathway
                new_pathways.append(new_pathway)
        else:
            new_pathways = [[{"node": node, "instances": nodes[node]["instances"]}]]
        
        return new_pathways
    
    def deduceNextActionPathways(self, node, nodes, new_pathways):
        next_actions = nodes[node]["next-actions"]
        extended_pathways = []
        had_valid_next_actions = False
        had_non_optional_next_action = False
        
        if len(next_actions) > 0:
            for next_action in next_actions:
                if next_action != node:
                    had_valid_next_actions = True
                    
                    if not nodes[next_action]["optional"] == "true":
                        had_non_optional_next_action = True
                    
                    extended_pathways_post = []
                    for new_pathway in new_pathways:
                        extended_pathways_pre = [new_pathway + [{"node": next_action, "instances": nodes[next_action]["instances"]}]]
                        next_pathways = self.deducePathways(nodes[next_action]["children"])
                        
                        if len(next_pathways) > 0:
                            for extended_pathway in extended_pathways_pre:
                                for next_pathway in next_pathways:
                                    extended_pathways_post = extended_pathways_post + [extended_pathway + next_pathway]
                        else:
                            extended_pathways_post = extended_pathways_pre
                    
                    # go on into the next next actions here
                    next_next_pathways = self.deduceNextActionPathways(next_action, nodes, extended_pathways_post);
                    
                    # print "-- NNP --" 
                    # for next_next_pathway in next_next_pathways:
                    #     for item in next_next_pathway:
                    #         print "  " + item["node"]
                        
                    #     print
                    
                    extended_pathways = extended_pathways + next_next_pathways# + extended_pathways_post
        
        if len(extended_pathways) == 0:
            extended_pathways = new_pathways
        
        if had_valid_next_actions:
            if not had_non_optional_next_action:
                extended_pathways = extended_pathways + new_pathways
        
        return extended_pathways
    
    def printInjected(self, dot = False):
        if dot:
            self.printInjectedDot()
        else:
            print self.arrInjected
    
    def printInjectedChildren(self, children, parent = None):
        dot = ""
        edge_pointers = {}
        ids = {}
        
        parent_id = parent
        if not parent:
            parent_id = "root"
        
        for child in children:
            child_id = "node_" + child.replace("-", "_") + "_" + str(self.counterdot)
            ids[child] = child_id
            self.counterdot = self.counterdot + 1
            
            dot += "  " + child_id + " [shape=box, label=\"" + child + " (" + str(children[child]["instances"]) + ")\"]\n"
            dot += self.printInjectedChildren(children[child]["children"], child_id)
            
            if parent:
                if children[child]["start-state"] == "true":
                    if children[child]["optional"] == "true":
                        dot += "  edge [style=solid, arrowhead=normal, arrowtail=none, label=\"optional\"]\n"
                    else:
                        dot += "  edge [style=solid, arrowhead=normal, arrowtail=none, label=\"\"]\n"
                else:
                    if children[child]["optional"] == "true":
                        dot += "  edge [style=dashed, arrowhead=none, arrowtail=none, label=\"\"]\n"
                    else:
                        dot += "  edge [style=dashed, arrowhead=none, arrowtail=none, label=\"\"]\n"
                
                dot += "  " + parent + " -> " + child_id + "\n"
            
            # if children[child]["terminal-state"] == "true":
            #     dot += "  node_terminal_state_" + str(self.counterdot) + " [shape=doublecircle, label=\"\"]\n"
            #     dot += "  edge [arrowhead=none, arrowtail=none, label=\"\"]\n"
            #     dot += "  " + child_id + " -> node_terminal_state_" + str(self.counterdot) + "\n"
            #     self.counterdot = self.counterdot + 1
            
            for na in children[child]["next-actions"]:
                if parent:
                    if not na in edge_pointers:
                        edge_pointers[na] = []
                    
                    edge_pointers[na].append(child_id)
        
        for child in children:
            child_id = ids[child]
            
            if child in edge_pointers:
                for target in edge_pointers[child]:
                    dot += "  {rank=same; " + child_id + " " + target + "}\n"
                    dot += "  edge [style=solid, arrowhead=empty, arrowtail=none, label=\"\"]\n"
                    dot += "  " + target + " -> " + child_id + "\n"
        
        return dot
    
    def printInjectedDot(self):
        self.counterdot = 0
        self.edge_pointers = {}
        
        dot = "digraph condensed {\n"
        dot += "  label=\"Condensed Experience Graph\"\n"
        dot += "  labeljust=center\n"
        dot += "  labelloc=top\n"
        dot += self.printInjectedChildren(self.arrInjected)
        dot += "}\n"
        
        print dot
    
    def expScore(self, exp):
        acc_score = 0
        
        for item in exp:
            instances = item["instances"]
            acc_score = acc_score + float(instances) / float(len(exp)) #float(len(self.arrExperiences))
        
        return acc_score
    
    def expScoreCmp(self, exp1, exp2):
        score1 = self.expScore(exp1)
        score2 = self.expScore(exp2)
        
        if score1 < score2:
            return 1
        elif score1 > score2:
            return -1
        else:
            return 0
    
    def printDotDeduced(self, deduced):
        counter = 0
        subgraphcounter = 0
        all_first = True
        
        dot = "digraph deduced {\n"
        dot += "  label=\"Deduced Possible Action Paths\"\n"
        dot += "  labeljust=center\n"
        dot += "  labelloc=top\n"
        
        highest_score = 0
        for line in deduced:
            acc_score = self.expScore(line)
            
            if acc_score > highest_score:
                highest_score = acc_score
        
        deduced.sort(self.expScoreCmp)
        
        for line in deduced:
            if all_first:
                all_first = False
            else:
                dot += "  \n"
            
            dot += "  subgraph cluster_" + str(subgraphcounter) + " {\n"
            dot += "    pencolor=transparent;\n"
            dot += "    \n"
            subgraphcounter = subgraphcounter + 1
            
            first = True
            for item in line:
                instances = item["instances"]
                node = item["node"]
                
                acc_score = acc_score + instances
                
                if not first:
                    dot += "    node_" + str(counter - 1) + " -> node_" + str(counter) + "\n"
                else:
                    first = False
                
                dot += "    node_" + str(counter) + " [shape=box, label=\"" + node + " (" + str(instances) + ")\"]\n"
                counter = counter + 1
            
            dot += "    \n"
            dot += "    label=\"Score: " + str(round(float(self.expScore(line)) / float(highest_score), 2)) + "\";\n"
            dot += "    labeljust=center;\n"
            dot += "    labelloc=top;\n"
            dot += "  }\n"
            
        dot += "}\n"
        
        print dot
