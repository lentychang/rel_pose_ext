#!/usr/bin/python
# -*- coding: <utf-8> -*-

from dataIO import stp2ply
import ipdb

if __name__ == '__main__':
    filePrefixName = "lf064-0"
    subName = ".stp"
    Dir = "/home/lenty/Dropbox/IWT/src/rel_pose_ext/models"
    ipdb.set_trace(context=10)
    for i in range(1, 7):
        name = filePrefixName + str(i) + subName
        print(name)
        stp2ply(name, Dir)
