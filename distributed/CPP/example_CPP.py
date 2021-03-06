#!/usr/bin/env python


import cppsolver

#Graph - list of tuples in the form (from, to, cost)
list_edges_36 = [(1, 2, 1.1314),(2, 3, 7.3361),(3, 4, 2.3756),(4, 5, 0.9334),(4, 6, 0.9505),(3, 7, 3.6404),(7, 8, 2.2691),(8, 9, 1.3403),(9, 10, 2.9768),(10, 11, 1.7253),(10, 12, 0.8733),(12, 13, 1.1133),(13, 14, 1.0200),(14, 2, 4.2955),(14, 15, 4.5619),(15, 16, 3.3659),(16, 17, 2.0418),(17, 18, 1.4585),(18, 13, 2.2421),(18, 19, 1.3011),(17, 20, 2.3103),(20, 21, 1.0009),(21, 22, 1.6789),(22, 23, 2.2889),(23, 24, 1.6400),(24, 25, 2.1576),(25, 26, 1.0359),(25, 27, 1.0596),(24, 28, 4.4059),(28, 29, 3.5484),(29, 21, 1.6860),(29, 16, 2.6831),(28, 30, 1.4142),(23, 7, 0.0849),(22, 31, 1.1597),(20, 32, 0.8202),(15, 33, 1.1781),(12, 34, 1.8668),(9, 35, 1.4142)]


#List of edges above that are going to be used
list_active = range(1,41)



import time
t = time.time()
x = cppsolver.CPP(list_edges_36, 10)
print 'elapsed = ', time.time()-t, 's'


print '\nHere is the solution: '
print x