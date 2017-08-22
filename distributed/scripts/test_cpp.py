#!/usr/bin/env python
import rospy
import os
import sys
import rospkg
rp = rospkg.RosPack()
path = rp.get_path('distributed')
path = path + '/CPP'
sys.path.append(path)

import test_adriano

print 'Function imported'
#list_active = range(1,37)
list_active = range(1,20)
print 'List of active edges:', list_active
route = test_adriano.main_adriano('list_edges_36',list_active)
print 'Here is the route solution:'
print route