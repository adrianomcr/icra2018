# in this function a graph is the input,
# after solving MILP of the graph, we apply MST and CPP
# the out put is a structure of the routes
#from distributed.MST.MST_test import read_graph
from MST_test import *
from main import *



# reading the graph
read_graph()

#-------------------------------------
# MILP Solver on input graph


#-------------------------------------
# MST
execute_MST()

#-------------------------------------
# CPP
main()