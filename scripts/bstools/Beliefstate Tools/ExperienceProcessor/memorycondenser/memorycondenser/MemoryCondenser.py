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
    
    def printExperiences(self, dot):
        for experience in self.arrExperiences:
            if dot:
                self.printDotExperience(experience)
            else:
                self.printRawExperience(experience)
    
    def printRawExperience(self, experience):
        owlData = experience.getOwlData()
        metaData = owlData["metadata"]
        start_nodes = metaData.subActions()
        self.t_tti = owlData["task-tree-individuals"]
        
        for node in start_nodes:
            self.printRawExperienceNode(node)
    
    def printRawExperienceNode(self, node, level = 0):
        indent = "   " * level
        owl = self.t_tti[node]
        
        parameters = owl.annotatedParameters()
        param_str = "("
        first = True
        
        for parameter in parameters:
            if not parameter == "_time_created":
                if first == True:
                    first = False
                else:
                    param_str = param_str + ", "
                
                key_str = parameter[10:] if parameter[:10] == "parameter-" else parameter
                param_str = param_str + key_str + "=" + parameters[parameter][0]
        
        param_str = param_str + ")"
        
        print indent + owl.taskContext() + " " + param_str
        
        if len(owl.subActions()) > 0:
            for node in owl.subActions():
                self.printRawExperienceNode(node, level + 1)
    
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
        
        print self.generalizeNodes(start_nodes)
    
    def injectExperiences(self, deduced = False, data = False):
        self.arrInjected = {}
        self.tti = {}
        
        root_action_count = 0
        
        for experience in self.arrExperiences:
            owlData = experience.getOwlData()
            metaData = owlData["metadata"]
            self.tti.update(owlData["task-tree-individuals"])
            
            for node in metaData.subActions():
                self.injectExperienceNode(node, self.arrInjected)
                root_action_count = root_action_count + 1
        
        for experience in self.arrExperiences:
            owlData = experience.getOwlData()
            metaData = owlData["metadata"]
            self.tti.update(owlData["task-tree-individuals"])
            
            for node in metaData.subActions():
                self.checkForTerminalStateOccurrences(self.tti[node].taskContext(), self.arrInjected)
            
            for node in metaData.subActions():
                self.checkForOptionalInjectedNodes(self.tti[node].taskContext(), self.arrInjected)
        
        if deduced:
            self.printDeduced(dot = not data, root_action_count = root_action_count)
        else:
            self.printInjected(dot = not data)
    
    def injectExperienceNode(self, node, frame, rootlevel = False):
        ctx = self.tti[node].taskContext()
        
        params = self.tti[node].annotatedParameters()
        params_fixed = {}
        
        for param in params:
            if not param == "_time_created":
                key_str = param[10:] if param[:10] == "parameter-" else param
                params_fixed[key_str] = params[param][0]
        
        if not ctx in frame:
            frame[ctx] = {"children": {}, "next-actions" : {}, "terminal-state": "false", "start-state": "false", "optional": "false", "instances": 0, "invocations": [params_fixed]}
        else:
            frame[ctx]["invocations"].append(params_fixed)
        
        frame[ctx]["instances"] = frame[ctx]["instances"] + 1
        
        sub_nodes = self.tti[node].subActions()
        
        for sub in sub_nodes:
            self.injectExperienceNode(sub, frame[ctx]["children"])
        
        next_node = self.tti[node].nextAction()
        
        if next_node:
            current_ctx = ctx
            while next_node:
                nextCtx = self.tti[next_node].taskContext()
                
                if not current_ctx in frame:
                    frame[current_ctx] = {"children": {}, "next-actions" : {}, "terminal-state": "false", "start-state": "false", "optional": "false", "instances": 0, "invocations": []}
                
                if not nextCtx in frame[current_ctx]["next-actions"] and not rootlevel:
                    if not nextCtx == current_ctx:
                        if not nextCtx in frame[current_ctx]["next-actions"]:
                            frame[current_ctx]["next-actions"][nextCtx] = []
                        
                        params = self.tti[next_node].annotatedParameters()
                        params_fixed = {}
                        
                        for param in params:
                            if not param == "_time_created":
                                key_str = param[10:] if param[:10] == "parameter-" else param
                                params_fixed[key_str] = params[param][0]
                        
                        frame[current_ctx]["next-actions"][nextCtx].append(params_fixed)
                    
                next_node = self.tti[next_node].nextAction()
                current_ctx = nextCtx
        else:
            if len(self.tti[node].subActions()) == 0:
                frame[ctx]["terminal-state"] = "true"
        
        if frame[ctx]["start-state"] == "false":
            if self.tti[node].previousAction() == None:
                frame[ctx]["start-state"] = "true"
    
    def checkForOptionalInjectedNodes(self, ctx, frame, parent_instances = -1, came_from = None):
        if not "check-optional" in frame[ctx]:
            frame[ctx]["check-optional"] = "done"
            
            came_from_terminates = False
            came_from_valid = True
            if came_from:
                if came_from == ctx:
                    came_from_valid = False
                
                if frame[came_from]["terminal-state"] == "true":
                    came_from_terminates = True
            
            if came_from_valid == True:
                if frame[ctx]["instances"] < parent_instances or came_from_terminates:
                    frame[ctx]["optional"] = "true"
            
            for child in frame[ctx]["children"]:
                if frame[ctx]["children"][child]["start-state"] == "true":
                    self.checkForOptionalInjectedNodes(child, frame[ctx]["children"], frame[ctx]["instances"])
            
            for next_action in frame[ctx]["next-actions"]:
                if next_action in frame and not next_action == ctx:
                    self.checkForOptionalInjectedNodes(next_action, frame, frame[ctx]["instances"], ctx)
    
    def checkForTerminalStateOccurrences(self, ctx, frame):
        if not "check-terminal" in frame[ctx]:
            frame[ctx]["check-terminal"] = "done"
            
            child_instances = 0
            next_instances = 0
            
            if frame[ctx]["terminal-state"] == "true":
                for child in frame[ctx]["children"]:
                    if frame[ctx]["children"][child]["start-state"] == "true":
                        child_instances = child_instances + frame[ctx]["children"][child]["instances"]
                
                for next_action in frame[ctx]["next-actions"]:
                    if next_action in frame and not next_action == ctx:
                        next_instances = next_instances + frame[next_action]["instances"]
                
                terminal_instances = frame[ctx]["instances"] - (child_instances + next_instances)
                
                if terminal_instances > 0:
                    frame[ctx]["terminal-instances"] = terminal_instances
                else:
                    frame[ctx]["terminal-instances"] = 0
            else:
                frame[ctx]["terminal-state"] = "false"
                frame[ctx]["terminal-instances"] = 0
            
            for child in frame[ctx]["children"]:
                self.checkForTerminalStateOccurrences(child, frame[ctx]["children"])
            
            for next_action in frame[ctx]["next-actions"]:
                if next_action in frame and not next_action == ctx:
                    self.checkForTerminalStateOccurrences(next_action, frame)
    
    def printDeduced(self, dot = False, root_action_count = 1):
        # TODO: Extend this to use all top-level nodes in case they
        # are different
        self.global_ctx_counter = 0
        deduced = self.expandPathways(self.arrInjected.keys()[0], self.arrInjected, root_action_count)
        
        fixed_deduced = []
        for d in deduced:
            fixed_deduced.append(d[2:])
        
        with open("deduced_experiences.json", "w") as f:
            json.dump(fixed_deduced, f)
        
        if dot:
            self.printDotDeduced(deduced)
        else:
            print deduced
    
    def expandPathways(self, ctx, nodes, root_action_count, trace = []):
        expanded_pathways = []
        
        if not "uid" in nodes[ctx]:
            nodes[ctx]["uid"] = self.global_ctx_counter
            self.global_ctx_counter = self.global_ctx_counter + 1
        
        if not nodes[ctx]["uid"] in trace:
            current_node = [{"node": ctx, "instances": nodes[ctx]["instances"], "rel-occ" : (float(nodes[ctx]["instances"]) / float(root_action_count)), "rel-term" : (float(nodes[ctx]["terminal-instances"]) / float(nodes[ctx]["instances"])), "invocations": nodes[ctx]["invocations"]}]
            children = self.getStartNodes(nodes[ctx]["children"])
            
            had_non_optional_children = False
            for child in children:
                if not children[child]["optional"] == "true":
                    had_non_optional_children = True
                    
                child_pathways = self.expandPathways(child, nodes[ctx]["children"], nodes[ctx]["instances"], trace + [nodes[ctx]["uid"]])
                
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
                        
                    expanded_next_pathways = self.expandPathways(next_action, nodes, nodes[ctx]["instances"], trace + [nodes[ctx]["uid"]])
                    
                    for expanded_next_pathway in expanded_next_pathways:
                        for expanded_pathway in expanded_pathways:
                            final_pathways = final_pathways + [expanded_pathway + expanded_next_pathway]
                            
            if not had_non_optional_next_actions:
                final_pathways = final_pathways + expanded_pathways
                
            return final_pathways
        else:
            return []
    
    def getStartNodes(self, nodes):
        start_nodes = {}
        
        for node in nodes:
            if nodes[node]["start-state"] == "true":
                start_nodes[node] = nodes[node]
        
        return start_nodes
    
    def printInjected(self, dot = False):
        if dot:
            self.printInjectedDot()
        else:
            print self.arrInjected
    
    def printInjectedChildren(self, children, parent = None):
        dot = ""
        edge_pointers = {}
        next_action_parameters = {}
        ids = {}
        optionals = {}
        
        parent_id = parent
        if not parent:
            parent_id = "root"
        
        for child in children:
            child_id = "node_" + child.replace("-", "_") + "_" + str(self.counterdot)
            ids[child] = child_id
            
            self.counterdot = self.counterdot + 1
            
            label = child
            if label[:21] == "REPLACEABLE-FUNCTION-":
                label = label[21:]
            
            dot += "  " + child_id + " [shape=box, label=\"" + label + " (" + str(children[child]["instances"]) + ")\"]\n"
            
            if children[child]["terminal-state"] == "true":
                if children[child]["terminal-instances"] > 0:
                    dot += "    ts_" + str(self.counterdot) + " [shape=doublecircle, label=\"" + str(children[child]["terminal-instances"]) + "\"]\n"
                    dot += "    edge [style=dashed, arrowhead=normal, arrowtail=none, label=\"terminal\"]\n"
                    dot += "    " + child_id + " -> " + "ts_" + str(self.counterdot) + "\n"
            
            dot += self.printInjectedChildren(children[child]["children"], child_id)
            
            if parent:
                if children[child]["start-state"] == "true":
                    if children[child]["optional"] == "true":
                        dot += "  edge [style=solid, arrowhead=normal, arrowtail=none, label=\"optional\"]\n"#      this->drawTextureScaled(srdRenderer, m_tmManager->texture("normal"), 200, 200);

                    else:
                        dot += "  edge [style=solid, arrowhead=normal, arrowtail=none, label=\"\"]\n"
                else:
                    if children[child]["optional"] == "true":
                        dot += "  edge [style=dashed, arrowhead=none, arrowtail=none, label=\"\"]\n"
                    else:
                        dot += "  edge [style=dashed, arrowhead=none, arrowtail=none, label=\"\"]\n"
                
                dot += "  " + parent + " -> " + child_id + "\n"
            
            for na in children[child]["next-actions"]:
                if parent:
                    if not na in edge_pointers:
                        edge_pointers[na] = []
                        next_action_parameters[na] = {}
                    
                    if not child_id in next_action_parameters[na]:
                        next_action_parameters[na][child_id] = []
                    
                    edge_pointers[na].append(child_id)
                    #next_action_parameters[na][child_id].append(children[child]["next-actions"][na])
        
        #print "!"
        #print next_action_parameters
        for child in children:
            child_id = ids[child]
            
            if child in edge_pointers:
                for target in edge_pointers[child]:
                    param_str = ""
                    # for param_sets in next_action_parameters[child][target]:
                    #     for param_set in param_sets:
                    #         first_p = True
                    #         for p in param_set:
                    #             if first_p:
                    #                 first_p = False
                    #             else:
                    #                 param_str = param_str + ", "
                                
                    #             param_str = param_str + p + " = " + param_set[p]
                            
                    #         param_str = param_str + "\\n"
                    #if next_action_parameters[
                    
                    if children[child]["optional"] == "true":
                        param_str = "optional"
                    else:
                        param_str = ""
                    
                    dot += "  {rank=same; " + child_id + " " + target + "}\n"
                    dot += "  edge [style=solid, arrowhead=empty, arrowtail=none, label=\"" + param_str + "\"]\n"
                    dot += "  " + target + " -> " + child_id + "\n"
        
        return dot
    
    def printInjectedDot(self):
        self.counterdot = 0
        self.edge_pointers = {}
        
        dot = "digraph condensed {\n"
        dot += "  graph []\n"#ranksep=0.5#nodesep=0.5#pad=0.5
        dot += "  label=\"Condensed Experience Graph\"\n"
        dot += "  labeljust=center\n"
        dot += "  labelloc=top\n"
        dot += self.printInjectedChildren(self.arrInjected)
        dot += "}\n"
        
        print dot
    
    def expScore(self, exp):
        acc_score = 1.0
        
        for item in exp:
            instances = item["instances"]
            rel_occ = item["rel-occ"]
            acc_score = acc_score * rel_occ
        
        last_item = exp[len(exp) - 1]
        acc_score = acc_score * last_item["rel-term"]
        
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
            dot += "  \n"
            
            dot += "  subgraph cluster_" + str(subgraphcounter) + " {\n"
            dot += "    pencolor=transparent;\n"
            dot += "    \n"
            subgraphcounter = subgraphcounter + 1
            
            first = True
            acc_score = 1.0
            
            for item in line:
                instances = item["instances"]
                node = item["node"]
                rel_occ = item["rel-occ"]
                
                # Correct node label
                if node[:21] == "REPLACEABLE-FUNCTION-":
                    node = node[21:]
                
                acc_score = acc_score * rel_occ
                
                if not first:
                    dot += "    node_" + str(counter - 1) + " -> node_" + str(counter) + "\n"
                else:
                    first = False
                
                dot += "    node_" + str(counter) + " [shape=box, label=\"" + node + " (" + str(round(rel_occ, 2)) + ")\"]\n"
                counter = counter + 1
            
            last_item = line[len(line) - 1]
            
            dot += "    ts_" + str(counter - 1) + " [shape=doublecircle, label=\"" + str(round(last_item["rel-term"], 2)) + "\"]\n"
            dot += "    edge [style=dashed, arrowhead=normal, arrowtail=none, label=\"\"]\n"
            dot += "    node_" + str(counter - 1) + " -> " + "ts_" + str(counter - 1) + "\n"
            
            acc_score = acc_score * last_item["rel-term"]
            
            dot += "    \n"
            dot += "    label=\"Score: " + str(round(acc_score, 2)) + "\";\n"
            dot += "    labeljust=center;\n"
            dot += "    labelloc=top;\n"
            dot += "  }\n"
            
        dot += "}\n"
        
        print dot
