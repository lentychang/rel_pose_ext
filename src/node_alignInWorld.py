#!/usr/bin/env python3
# -*- coding: <utf-8> -*-

# export PYTHONPATH="/root/catkin_ws/devel/lib/python2.7/dist-packages:/opt/ros/kinetic/lib/python2.7/dist-packages"
# from topo2 import RecognizeTopo
from dataIO import read_stp_solid_withTf, Display
import rospy
import sys
from rel_pose_ext.msg import objectLocalization
from geometry_msgs.msg import TransformStamped
import ipdb
from plane_detection import autoPlaneAlign
from hole_detection import getNHoleCouldBeMatched

recognizeModelTopic = "/detectedObjs_preliminary"
tf2Topic = "/tf2_kinect2_world"
modelDir = "/root/catkin_ws/src/rel_pose_ext/models"


def position2list(position):
    return [position.x, position.y, position.z]


def orientation2list(orientation):
    return [orientation.x, orientation.y, orientation.z, orientation.w]


class Align():
    def __init__(self):
        rospy.init_node('modelAlign')
        self.rate = rospy.Rate(1)
        self.objLocMsg = objectLocalization()
        self.tf2 = TransformStamped()
        self.solids = []
        # self.frame = Display() 

    def __register_subcriber(self):
        self.localFrameSubscriber = rospy.Subscriber(recognizeModelTopic, objectLocalization, self.__objLocSubCb)
        self.tf2Subscriber = rospy.Subscriber(tf2Topic, TransformStamped, self.__tf2SubCb)

    def __unregister_subcriber(self):
        self.localFrameSubscriber.unregister()
        self.tf2Subscriber.unregister()

    def update_subscription(self):
        self.__register_subcriber()
        i = 0
        while i < 10:
            i += 1
            rospy.sleep(0.1)
        self.__unregister_subcriber()

    def __objLocSubCb(self, msg):
        self.objLocMsg = msg

    def __tf2SubCb(self, msg):
        self.tf2 = msg

    def getSolids(self):
        for i in range(0, len(self.objLocMsg.modelList)):
            modelname = self.objLocMsg.modelList[i]
            translation = position2list(self.objLocMsg.pose[i].position)
            quaternion = orientation2list(self.objLocMsg.pose[i].orientation)
            self.solids.append(read_stp_solid_withTf(modelDir=modelDir, stpFilename=modelname, vecXYZlist=translation, quaternionXYZWlist=quaternion, unitIsMM=False))

    def showModels(self):
        self.frame = Display(self.solids[0], run_display=True)
        for i in range(1, len(self.solids)):
            self.frame.add_shape(self.solids[i])
        self.frame.open()

    def init_alignZ(self):
        autoPlaneAlign(solid_add=self.solids[0], solid_base=None, negletParallelPln=False)

    def align(self):
        maxPossibleMatchedHolePairs = getNHoleCouldBeMatched(self.solids[0], self.solids[1])
        for i in range(0, len(self.objLocMsg.modelList) - 1):
            if maxPossibleMatchedHolePairs < 2:
                autoPlaneAlign(solid_add=self.solids[i+1], solid_base=self.solids[i], negletParallelPln=False)
                pass
            else:
                pass


def __testSubscriber():
    tmp = Align()
    while not rospy.is_shutdown():
        print(tmp.objLocMsg)
        print(tmp.tf2)
        tmp.rate.sleep()


def __testSubOnce():
    tmp = Align()
    tmp.update_subscription()


def __test_readModelsFromTopic():
    tmp = Align()
    tmp.update_subscription()
    tmp.getSolids()
    tmp.showModels()
    ipdb.set_trace()


def __test():
    pass 


if __name__ == "__main__":
    # __testSubscriber()
    # __testSubOnce()
    __test_readModelsFromTopic()
