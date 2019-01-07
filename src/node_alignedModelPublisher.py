#!/usr/bin/env python3
# -*- coding: <utf-8> -*-

import copy
import sys

import ipdb
import rospy
from geometry_msgs.msg import Pose, TransformStamped
from std_msgs.msg import Header
from thesis_visualization_msgs.msg import objectLocalization

# export PYTHONPATH="/root/catkin_ws/devel/lib/python2.7/dist-packages:/opt/ros/kinetic/lib/python2.7/dist-packages"
# from topo2 import RecognizeTopo
from rel_pose_ext.dataIO import Display, read_stp_solid_withTf
from rel_pose_ext.hole_detection import autoHoleAlign
from rel_pose_ext.plane_detection import autoPlaneAlign


def position2list(position):
    return [position.x, position.y, position.z]


def orientation2list(orientation):
    return [orientation.x, orientation.y, orientation.z, orientation.w]


def list2position(xyz):
    assert len(xyz) == 3, "length of inputList should be 3 in order to fit x,y,z "
    position = Pose().position
    position.x = xyz[0]
    position.y = xyz[1]
    position.z = xyz[2]
    return position


def list2orientation(xyzw):
    assert len(xyzw) == 4, "length of inputList should be 4 in order to fit x,y,z,w "

    orientation = Pose().orientation
    orientation.x = xyzw[0]
    orientation.y = xyzw[1]
    orientation.z = xyzw[2]
    orientation.w = xyzw[3]
    return orientation


def list2pose(xyzxyzw):
    assert len(xyzxyzw) == 7, "length of inputList should be 7 in order to fit x,y,z,x,y,z,w"
    pose = Pose()
    pose.position = list2position(xyz=xyzxyzw[:3])
    pose.orientation = list2orientation(xyzw=xyzxyzw[3:])
    return pose


class Align():
    def __init__(self):
        rospy.init_node('aligned_model_publisher')
        if len(sys.argv) >= 5:
            self.__subTopic_detectedObj_beforeAligned = sys.argv[1]
            self.__tf2Topic = sys.argv[2]
            self.__modelDir = sys.argv[3]
            self.__pubTopic_detectObjs_afterAligned = sys.argv[4]
        else:
            rospy.logwarn("Node parameters are not given, using default value")
            self.__subTopic_detectedObj_beforeAligned = "/detectedObjs_beforeAligned"
            self.__tf2Topic = "/tf2_kinect2_world"
            self.__modelDir = "/root/exchange/tempData/models"
            self.__pubTopic_detectObjs_afterAligned = "/detectedObjs_afterAligned"

        self.rate = rospy.Rate(1)
        self.objLocMsgIn = objectLocalization()
        self.objLocMsgOut = objectLocalization()
        self.tf2 = TransformStamped()
        self.solids = []
        self.publisher = rospy.Publisher(self.__pubTopic_detectObjs_afterAligned, objectLocalization, queue_size=1)
        self.__enablePub = True
        self.__fakeModelsInBin = []
        self.__fakePosesInBin = []
        self.__initFakeMsg()
        self.__display_initialized = False

    def __register_subcriber(self):
        self.localFrameSubscriber = rospy.Subscriber(self.__subTopic_detectedObj_beforeAligned, objectLocalization, self.__objLocSubCb)
        # now the topic detectedObj_beforeAligned is with frame world, so below tf2 subscriber is not needed
        # [WARN] if the topic of is not in world, the a transformation to world must be applied
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

    def __objLocSubCb(self, msg):
        self.objLocMsgIn = msg

    def __tf2SubCb(self, msg2):
        self.tf2 = msg2

    def __initFakeMsg(self):
        self.__fakeModelsInBin = ["lf06401_01", "lf06402_01", "lf06402_02", "lf06402_03", "lf06402_04",
                                  "lf06403_01", "lf06404_01", "lf06405_01"]  # , "lf06401_02"]
        # xyz xyzw
        self.__fakePosesInBin = [[0.0724816187616, 0.124606657388, 1.03745215655, 0.0920451500368, -0.0323602349858, -0.0019425099839, 0.995226975218],
                                 [0.0141289626921, 0.110071527685, 0.98505878063, 0.0552568089657, 0.0584880362144, 0.0983473966193, 0.991893958173],
                                 [0.100577081296, 0.137620339663, 1.02420057587, 0.113009451207, -0.011916964575, 0.129227680582, 0.98508225873],
                                 [0.0969918090138, 0.173674935621, 1.03840119455, 0.0913792005976, 0.0398021742578, 0.0829041181838, 0.991560656647],
                                 [0.0556155663549, 0.169867486008, 0.990480218566, 0.0383023575875, 0.0201185237378, -0.0254117589308, 0.99874041518],
                                 [0.0570710771168, 0.0826196809518, 1.05383140667, -0.000540758057947, -0.0143166556673, -0.0713254607238, 0.997350199079],
                                 [0.0969141154042, 0.150417286296, 1.00112320288, -0.0865704879567, 0.0938614468037, -0.0120670678191, 0.991740876083],
                                 [0.07518621408, 0.212001413843, 1.04784130973, -0.0144559164571, 0.134463670046, -0.00685760157496, 0.990789342503]]

    # fakeSubscriber
    def fakeSubscriberSubOnce(self):
        def append_one_solid():
            if len(self.__fakeModelsInBin) != 0 or len(self.solids) != len(self.objLocMsgIn.modelList):
                i = len(self.solids)
                modelname = self.objLocMsgIn.modelList[i].split("_")[0]
                # reform modelname lf06401--> lf064-01
                modelname = modelname[:-2] + "-" + modelname[-2:]
                translation = position2list(self.objLocMsgIn.pose[i].position)
                quaternion = orientation2list(self.objLocMsgIn.pose[i].orientation)
                self.solids.append(read_stp_solid_withTf(modelDir=self.__modelDir, stpFilename=modelname, vecXYZlist=translation, quaternionXYZWlist=quaternion, unitIsMM=False))
        if len(self.__fakeModelsInBin) >= 1:
            self.objLocMsgIn.headers.append(Header())
            self.objLocMsgIn.modelList.append(self.__fakeModelsInBin.pop(0))
            self.objLocMsgIn.pose.append(list2pose(xyzxyzw=self.__fakePosesInBin.pop(0)))
        else:
            rospy.logwarn("##### No model in Bin... #####\n")
        append_one_solid()

    def getAllSolids(self):
        n_modelList_in = len(self.objLocMsgIn.modelList)

        if n_modelList_in == 0:
            rospy.loginfo("No model detected")

        for i in range(0, n_modelList_in):
            modelname = self.objLocMsgIn.modelList[i].split("_")[0]
            # reform modelname lf06401--> lf064-01
            modelname = modelname[:-2] + "-" + modelname[-2:]
            translation = position2list(self.objLocMsgIn.pose[i].position)
            quaternion = orientation2list(self.objLocMsgIn.pose[i].orientation)
            self.solids.append(read_stp_solid_withTf(modelDir=self.__modelDir, stpFilename=modelname, vecXYZlist=translation, quaternionXYZWlist=quaternion, unitIsMM=False))

    def __update_solid2frame(self):
        nSolids = len(self.solids)
        nShapesInFrame = len(self.frame.shape_list)
        assert nSolids >= nShapesInFrame, "n of Solids should be more than n of shapes in frame"
        if nSolids > nShapesInFrame:
            addList = self.solids[nShapesInFrame:]
            for i in addList:
                self.frame.add_shape(i)

    def showModels(self, run_display=True):
        # not initialized
        if not self.__display_initialized:
            if len(self.solids) >= 1:
                self.frame = Display(self.solids[0], run_display=run_display)
                self.__display_initialized = True
                self.__update_solid2frame()
            else:
                rospy.logwarn("display not yet initialized, because of no model")
        # is initialized
        else:
            if run_display:
                self.__update_solid2frame()
                self.frame.open()

    def align(self, baseIdx, addIdx):
        autoPlaneAlign(solid_add=self.solids[addIdx], solid_base=self.solids[baseIdx], negletParallelPln=False)
        autoHoleAlign(solid_add=self.solids[addIdx], solid_base=self.solids[baseIdx])

    def alignHoles(self, baseIdx, addIdx):
        autoHoleAlign(solid_add=self.solids[addIdx], solid_base=self.solids[baseIdx])

    def init_alignZ(self, align_nth_solid=0):
        autoPlaneAlign(solid_add=self.solids[align_nth_solid], solid_base=None, negletParallelPln=False, xyplane_z=1.2, match_planes=False)

    def alignAll(self):
        # first align to Z axis
        n_solids = len(self.solids)
        if n_solids > 0:
            for i in range(0, n_solids):
                rospy.logdebug("#### Align {0} ####".format(self.objLocMsgIn.modelList[i]))
                rospy.logdebug("Align with XY plane")
                self.init_alignZ(align_nth_solid=i)
                if i == 0:
                    continue
                else:
                    rospy.logdebug("Align to the nearest hole")
                self.alignHoles(baseIdx=0, addIdx=i)
                self.showModels(run_display=False)
                rospy.logdebug("Alignment finished!\n")
                # self.showModels(init=False)

        else:
            rospy.logwarn("No Model detected, check topic: %s", self.__subTopic_detectedObj_beforeAligned)

    def preparePubMsg(self, pubUnitIsMeter=True):
        self.objLocMsgOut = copy.deepcopy(self.objLocMsgIn)
        # rospy.logdebug("length of modelList %d" % (len((self.objLocMsgIn.modelList))))
        # rospy.logdebug("length of solids %d" % (len((self.solids))))
        for i in range(0, len(self.solids)):
            # rospy.logdebug("i: %d" % (i))
            self.objLocMsgOut.headers[i].stamp = rospy.Time.now()
            trsf = self.solids[i].Location().Transformation()
            q = trsf.GetRotation()
            t = trsf.TranslationPart()
            self.objLocMsgOut.pose[i].position.x = t.X()
            self.objLocMsgOut.pose[i].position.y = t.Y()
            self.objLocMsgOut.pose[i].position.z = t.Z()
            if pubUnitIsMeter:
                self.objLocMsgOut.pose[i].position.x /= 1000.0
                self.objLocMsgOut.pose[i].position.y /= 1000.0
                self.objLocMsgOut.pose[i].position.z /= 1000.0

            self.objLocMsgOut.pose[i].orientation.x = q.X()
            self.objLocMsgOut.pose[i].orientation.y = q.Y()
            self.objLocMsgOut.pose[i].orientation.z = q.Z()
            self.objLocMsgOut.pose[i].orientation.w = q.W()
            # rospy.logdebug("TranslationXYZ: (%f, %f, %f)\nquaternionXYZW: (%f, %f, %f, %f)\n" % (t.X(), t.Y(), t.Z(), q.X(), q.Y(), q.Z(), q.W()))

    def pub_start(self):
        self.__enablePub = True
        self.__register_subcriber()

        for i in range(0, 3):
            self.rate.sleep()

        while (not rospy.is_shutdown()) and self.__enablePub:
            self.getAllSolids()
            self.alignAll()
            rospy.loginfo("All model alignment finished")
            rospy.loginfo("Models are: {0}".format(self.objLocMsgIn.modelList))
            self.preparePubMsg(pubUnitIsMeter=True)
            self.publisher.publish(self.objLocMsgOut)
            self.rate.sleep()
            rospy.logdebug("\npublish Once!\n")
            self.solids = []
        self.__unregister_subcriber()

    def test_fake_pub_start(self):
        while not rospy.is_shutdown():
            self.fakeSubscriberSubOnce()
            self.alignAll()
            rospy.loginfo("All model alignment finished")
            rospy.loginfo("Models are: {0}".format(self.objLocMsgIn.modelList))
            self.preparePubMsg(pubUnitIsMeter=True)
            self.publisher.publish(self.objLocMsgOut)
            self.rate.sleep()
            rospy.logdebug("\npublish Once!\n")

    def testSubscriber(self):
        self.__register_subcriber()
        for i in range(0, 3):
            rospy.loginfo("objLocMsgIn:\n{0}".format(self.objLocMsgIn))
            rospy.loginfo("tf2 Subscriber:\n{0}".format(self.tf2))
            self.rate.sleep()
        self.__unregister_subcriber()


def test():
    def __testSubscriber():
        print("\n\n### TEST: __testSubscriber ###")
        tmp = Align()
        tmp.testSubscriber()

    def __test_activate_subscriberOnce():
        print("\n\n### TEST: __test_activate_subscriberOnce ###")
        tmp = Align()
        tmp.activate_subscriberOnce()
        print("ModelName subscribed: %s" % (tmp.objLocMsgIn.modelList))
        print("tf2 subscribed: %s" % (tmp.tf2.header))

    def __test_readModelsFromTopic():
        print("\n\n### TEST: __test_readModelsFromTopic ###")
        tmp = Align()
        tmp.activate_subscriberOnce()
        if len(tmp.solids) != 0:
            tmp.getAllSolids()
            tmp.showModels()
        else:
            rospy.logerr("Test failed: No model from subscriber, not able to test readModels")

    def __test_align_with_subscriber():
        print("\n\n### TEST: __test_align_with_subscriber ###")
        tmp = Align()
        tmp.activate_subscriberOnce()
        if len(tmp.solids) != 0:
            tmp.getAllSolids()
            tmp.showModels()
            tmp.init_alignZ()
            tmp.showModels()
            for i in range(0, len(tmp.objLocMsgIn.modelList) - 1):
                tmp.align(baseIdx=i, addIdx=i + 1)
            tmp.showModels(init=False)
        else:
            rospy.logerr("Test failed: No model from subscriber, not able to test alignment")

    def __test_align_with_fakeSubscriber():
        print("\n\n### TEST: __test_align_with_fakeSubscriber ###")
        tmp = Align()
        tmp.fakeSubscriberSubOnce()
        tmp.showModels(run_display=False)
        tmp.init_alignZ()
        tmp.showModels(run_display=False)

        # align every part with first part
        for i in range(1, 8):
            tmp.fakeSubscriberSubOnce()
            rospy.logdebug("#### Align {0} ####".format(tmp.objLocMsgIn.modelList[i]))
            rospy.logdebug("Align with XY plane")
            tmp.init_alignZ(align_nth_solid=i)
            rospy.logdebug("Align to the nearest hole")
            tmp.alignHoles(baseIdx=0, addIdx=i)
            tmp.showModels(run_display=False)

        rospy.logdebug("Alignment finished!")
        tmp.showModels()

    def __test_fake_pub():
        tmp = Align()
        tmp.test_fake_pub_start()

    __testSubscriber()
    __test_activate_subscriberOnce()
    __test_readModelsFromTopic()
    __test_align_with_subscriber()
    __test_align_with_fakeSubscriber()
    __test_fake_pub()


def main():
    alignModel = Align()
    alignModel.pub_start()


if __name__ == "__main__":
    # test()
    main()
