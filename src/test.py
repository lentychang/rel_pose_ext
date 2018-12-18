#!/usr/bin/python3
# -*- coding: <utf-8> -*-

import sys
import logging
# print("testErr", file=sys.stderr)
from rel_pose_ext.dataIO import Display
import os

logging.basicConfig(filename='example.log', filemode='w', level=logging.DEBUG)


def main():
    print(os.getcwd())
    test = Display()
#     for i in range(0, 10):
#         logging.info("info: {0}".format(i))
#         logging.debug("debug: {0}".format(i))
#         logging.warn("warn: {0}".format(i))


if __name__ == "__main__":
    main()


