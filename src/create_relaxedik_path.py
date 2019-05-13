#! /usr/bin/env python

import rospkg
import os

path_to_src = os.path.dirname(__file__)

rospack = rospkg.RosPack()
rospack.list()
try:
    p = rospack.get_path('relaxed_ik')
except:
    print 'relaxed_ik package not found!  Please install relaxed_ik package first, and return to stampede after this.' \
          ' Link to code: https://github.com/uwgraphics/relaxed_ik'

fp = path_to_src + '/Stampede/Config/relaxedik_path'

f = open(fp, 'w')
f.write(p)
f.close()

