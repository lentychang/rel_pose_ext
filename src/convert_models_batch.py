#!/usr/bin/python3
# -*- coding: <utf-8> -*-

from rel_pose_ext.dataIO import stp2pcd
import sys

if __name__ == '__main__':
    filePrefixName = "lf064-0"
    subName = ".stp"
    if len(sys.argv) > 1:
        Dir = sys.argv[1]
    else:
        Dir = "/root/catkin_ws/src/rel_pose_ext/src/test"
    print(f"Directory of Step models to be converted: \n{Dir}")
    
    for i in range(1, 7):
        name = filePrefixName + str(i) + subName
        print(name)
        stp2pcd(name, Dir)
