#!/usr/bin/env python

import os

import argparse
import sys

import data.data
from chinesepostman import eularian, network

def setup_args():
    """ Setup argparse to take graph name argument. """
    parser = argparse.ArgumentParser(description='Find an Eularian Cicruit.')
    parser.add_argument('graph', nargs='?', help='Name of graph to load')
    args = parser.parse_args()
    return args.graph



#def main_adriano(graph_name, list_active):
#def CPP(edges_all, list_active, start_node):
def CPP(edges, start_node):
    """ Make it so. """


    """
    print edges_all
    print list_active
    print start_node

    #Exclude some edges
    edges = []
    for k in range(len(edges_all)):
        if ((k + 1) in list_active):
            edges.append(edges_all[k])
    """



    original_graph = network.Graph(edges)

    #print('{} edges'.format(len(original_graph)))
    if not original_graph.is_eularian:
        #print('Converting to Eularian path...')
        graph = eularian.make_eularian(original_graph)
        #print('Conversion complete')
        #print('\tAdded {} edges'.format(len(graph) - len(original_graph)))
        #print('\tTotal cost is {}'.format(graph.total_cost))
    else:
        graph = original_graph


    #print('Attempting to solve Eularian Circuit...')
    route, attempts = eularian.eularian_path(graph, start=1)

    #print '\nOriginal Route:'
    #print route
    #print '\n'

    #Stretch to rewrite the route in such a way the first node is the start node
    route.pop()
    ok = False
    count = 0
    while(not ok and count < len(route)):
        count = count + 1
        if (route[0] != start_node):
            move_node = route.pop(0)
            route.append(move_node)
        else:
            ok = True
    route.append(start_node)





    if not route:
        print('\tGave up after {} attempts.'.format(attempts))
    else:
        #print('\tSolved in {} attempts'.format(attempts, route))
        #print('Solution: ({} edges)'.format(len(route) - 1))
        #print('\t{}'.format(route))
	    return(route)

    return(-1)

