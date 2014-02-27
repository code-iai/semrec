#!/usr/bin/python

import sys
import json
from xml.dom.minidom import parse, parseString

params_ok = True

if len(sys.argv) < 3 or sys.argv[1] == "" or sys.argv[2] == "":
    print "Usage: " + sys.argv[0] + " <designators.json> <experiment.owl>"
    params_ok = False

if params_ok:
    file_logged_designators = sys.argv[1]
    file_owl = sys.argv[2]
    # Load the logged designator indices
    logged_designators = []
    logged_details = []
    with open(file_logged_designators) as f:
        for line in iter(lambda: f.readline(), ''):
            data = json.loads(line)
            logged_designators.append(data['designator']['_id'])
            logged_details.append(data['designator'])
            
    # Load the OWL designators
    dom_owl = parse(file_owl)
    individuals = dom_owl.getElementsByTagName("owl:namedIndividual")
    owl_designators = []
    for individual in individuals:
        individual_type = individual.getElementsByTagName("rdf:type")
        if individual_type.length == 1:
            individual_type = individual_type[0]
            indiv_type_resource = individual_type.getAttribute("rdf:resource")
            if indiv_type_resource.split("#")[1] == "CRAMDesignator":
                indiv_name = individual.getAttribute("rdf:about")
                owl_designators.append(indiv_name.split("#")[1])
    
    # Evaluate
    logged_but_not_owl = []
    owl_but_not_logged = []
    logged_duplicates = []
    owl_duplicates = []
    
    logged_inspected = []
    owl_inspected = []
    
    for desig in logged_designators:
        seen = 0
        for desig2 in owl_designators:
            if desig2 == desig:
                seen = 1
                break
    
        if seen == 0:
            logged_but_not_owl.append(desig)
    
        seen = 0
        for desig2 in logged_inspected:
            if desig2 == desig:
                seen = 1
                break
    
        if seen == 1:
            logged_duplicates.append(desig)
        else:
            logged_inspected.append(desig)

    for desig in owl_designators:
        seen = 0
        for desig2 in logged_designators:
            if desig2 == desig:
                seen = 1
                break
    
        if seen == 0:
            owl_but_not_logged.append(desig)
    
        seen = 0
        for desig2 in owl_inspected:
            if desig2 == desig:
                seen = 1
                break
    
        if seen == 1:
            owl_duplicates.append(desig)
        else:
            owl_inspected.append(desig)

    # Print results
    if len(logged_but_not_owl) > 0:
        print "Designators logged but not present in OWL:"
        for desig in logged_but_not_owl:
            print " - " + desig

    if len(owl_but_not_logged) > 0:
        print "Designators present in OWL but not logged:"
        for desig in owl_but_not_logged:
            print " - " + desig

    # Print statistics
    print "\nStatistics:"
    print " - Total logged: " + str(len(logged_designators))
    print " - Total in OWL: " + str(len(owl_designators))
    print " - Logged, but not in OWL: " + str(len(logged_but_not_owl))
    print " - In OWL, but not logged: " + str(len(owl_but_not_logged))
    print " - Duplicates in logged: " + str(len(logged_duplicates))
    print " - Duplicates in OWL: " + str(len(owl_duplicates))
    
    if len(logged_but_not_owl) == 0 and len(owl_but_not_logged) == 0:
        print
        print "Results consistent."
