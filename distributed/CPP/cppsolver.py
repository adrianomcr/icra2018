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

    # start_node -> node in which the robot start
    # edges -> list of tuples in the form (from, to, cost)

    #print edges

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

        #print 'grafo inicial NAO euleriano'
    else:
        graph = original_graph

    """
    if graph.is_eularian:
        print 'grafo transformado euleriano'
    else:
        print 'grafo transformado NAO euleriano'
    """

    """
    print 'graph'
    print graph
    """

    #print('Attempting to solve Eularian Circuit...')
    #route, attempts = eularian.eularian_path(graph, start=1)
    #route, attempts = eularian.eularian_path(graph, start=None)

    """
    list_plot = []
    for k in range(len(edges)):
        tupla = edges[k]
        list_plot.append(tupla[0])
        list_plot.append(tupla[1])
    list_plot = list(set(list_plot))
    print 'list_plot', list_plot
    print 'start_node', start_node
    """
    route, attempts = eularian.eularian_path(graph, start=start_node)

    """
    print '\nOriginal Route:'
    print route
    print '\n'
    """


    if not route:
        print('\tGave up after {} attempts.'.format(attempts))
    else:
        #print('\tSolved in {} attempts'.format(attempts, route))
        #print('Solution: ({} edges)'.format(len(route) - 1))
        #print('\t{}'.format(route))

        # Stretch to rewrite the route in such a way the first node is the start node
        route.pop()
        ok = False
        count = 0
        while (not ok and count < len(route)):
            count = count + 1
            if (route[0] != start_node):
                move_node = route.pop(0)
                route.append(move_node)
            else:
                ok = True
        route.append(start_node)

        return(route)

    return(-1)

