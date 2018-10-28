#!/usr/bin/env python3
# -*- coding: <utf-8> -*-

# export PYTHONPATH="/root/catkin_ws/devel/lib/python2.7/dist-packages:/opt/ros/kinetic/lib/python2.7/dist-packages"
# from topo2 import RecognizeTopo
from dataIO import read_stp_solid_withTf, Display
import rospy
import sys
from thesis_visualization_msgs.msg import objectLocalization
from geometry_msgs.msg import TransformStamped
import ipdb
from plane_detection import autoPlaneAlign
from hole_detection import getNHoleCouldBeMatched, autoHoleAlign
import copy
import threading

def position2list(position):
    return [position.x, position.y, position.z]


def orientation2list(orientation):
    return [orientation.x, orientation.y, orientation.z, orientation.w]


class Align():
    def __init__(self):
        rospy.init_node('aligned_model_publisher')
        self.__recognizeModelTopic = sys.argv[1]
        self.__tf2Topic = sys.argv[2]
        self.__modelDir = sys.argv[3]
        self.__pubTopic = sys.argv[4]


        self.rate = rospy.Rate(1)
        self.objLocMsgIn = objectLocalization()
        self.objLocMsgOut = objectLocalization()
        self.tf2 = TransformStamped()
        self.solids = []
        self.publisher =rospy.Publisher(self.__pubTopic, objectLocalization, queue_size=1)
        self.__enablePub = True
        # self.frame = Display()

    def __register_subcriber(self):
        self.localFrameSubscriber = rospy.Subscriber(self.__recognizeModelTopic, objectLocalization, self.__objLocSubCb)
        self.tf2Subscriber = rospy.Subscriber(self.__tf2Topic, TransformStamped, self.__tf2SubCb)


    def __unregister_subcriber(self):
        self.localFrameSubscriber.unregister()
        self.tf2Subscriber.unregister()

    def activate_subscriberOnce(self):
        self.__register_subcriber()
        i = 0
        while i < 10:
            i += 1
            rospy.sleep(0.1)
        self.__unregister_subcriber()
        # rospy.logdebug("subscribe once")

    def __objLocSubCb(self, msg):
        self.objLocMsgIn = msg

    def __tf2SubCb(self, msg2):
        self.tf2 = msg2

    def getSolids(self):
        self.solids = []
        for i in range(0, len(self.objLocMsgIn.modelList)):
            modelname = self.objLocMsgIn.modelList[i]
            translation = position2list(self.objLocMsgIn.pose[i].position)
            quaternion = orientation2list(self.objLocMsgIn.pose[i].orientation)
            self.solids.append(read_stp_solid_withTf(modelDir=self.__modelDir, stpFilename=modelname, vecXYZlist=translation, quaternionXYZWlist=quaternion, unitIsMM=False))

    def showModels(self, init=False):
        if init:
            self.frame = Display(self.solids[0], run_display=True)
            for i in range(1, len(self.solids)):
                self.frame.add_shape(self.solids[i])
        self.frame.open()


    def align(self, i):
        autoPlaneAlign(solid_add=self.solids[i + 1], solid_base=self.solids[i], negletParallelPln=False)
        autoHoleAlign(solid_add=self.solids[i + 1], solid_base=self.solids[i])
    
    def alignAll(self):
        # first align to Z axis
        autoPlaneAlign(solid_add=self.solids[0], solid_base=None, negletParallelPln=False)
        for i in range(0, len(self.objLocMsgIn.modelList) - 1):
            self.align(i)
    
    def preparePubMsg(self):
        self.objLocMsgOut = copy.deepcopy(self.objLocMsgIn)
        # rospy.logdebug("length of modelList %d" % (len((self.objLocMsgIn.modelList))))
        # rospy.logdebug("length of solids %d" % (len((self.solids))))        
        for i in range(0, len(self.solids)):
            #rospy.logdebug("i: %d" % (i))
            self.objLocMsgOut.headers[i].stamp= rospy.Time.now()
            trsf = self.solids[i].Location().Transformation()
            q = trsf.GetRotation()
            t = trsf.TranslationPart()
            self.objLocMsgOut.pose[i].position.x = t.X()
            self.objLocMsgOut.pose[i].position.y = t.Y()
            self.objLocMsgOut.pose[i].position.z = t.Z()
            self.objLocMsgOut.pose[i].orientation.x = q.X()
            self.objLocMsgOut.pose[i].orientation.y = q.Y()
            self.objLocMsgOut.pose[i].orientation.z = q.Z()
            self.objLocMsgOut.pose[i].orientation.w = q.W()
            # rospy.logdebug("TranslationXYZ: (%f, %f, %f)\nquaternionXYZW: (%f, %f, %f, %f)\n" % (t.X(), t.Y(), t.Z(), q.X(), q.Y(), q.Z(), q.W()))

    
    def pub_start(self):
        self.__enablePub = True
        self.__register_subcriber()
        for i in range(0,3):
            self.rate.sleep()
        
        while (not rospy.is_shutdown()) and self.__enablePub:
            self.getSolids()
            self.alignAll()
            self.preparePubMsg()
            self.publisher.publish(self.objLocMsgOut)
            self.rate.sleep()
            rospy.logdebug("\n\npublish Once!\n\n")
        self.__unregister_subcriber()
    
   
    def testSubscriber(self):
        self.__register_subcriber()
        while not rospy.is_shutdown():
            rospy.logdebug(self.objLocMsgIn)
            rospy.logdebug(self.tf2)
            self.rate.sleep()
        self.__unregister_subcriber()


def __testSubscriber():
    tmp = Align()
    tmp.testSubscriber()
    

def __test_activate_subscriberOnce():
    tmp = Align()
    tmp.activate_subscriberOnce()
    # rospy.logdebug("ModelName subscribed: %s" % (tmp.objLocMsgIn.modelList))
    # rospy.logdebug("tf2 subscribed: %s" % (tmp.tf2.header))


def __test_readModelsFromTopic():
    tmp = Align()
    tmp.activate_subscriberOnce()
    tmp.getSolids()
    tmp.showModels(init=True)
    ipdb.set_trace()


def __test_align():
    tmp = Align()
    tmp.activate_subscriberOnce()
    tmp.getSolids()
    tmp.showModels(init=True)
    tmp.init_alignZ()
    tmp.showModels()
    for i in range(0, len(tmp.objLocMsgIn.modelList) - 1):
        tmp.align(i)
    tmp.showModels(init=False)


def __test_pub_start():
    tmp = Align()
    tmp.pub_start()


if __name__ == "__main__":
    # __testSubscriber()
    # __test_activate_subscriberOnce()
    # __test_readModelsFromTopic()
    #__test_align()
    __test_pub_start()
