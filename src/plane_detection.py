#!/usr/bin/python
# -*- coding: <utf-8> -*-

import os.path
from math import degrees, radians

import ipdb
import numpy as np
from OCC.AIS import ais_ProjectPointOnPlane
from OCC.BRepAdaptor import BRepAdaptor_Surface
from OCC.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire
from OCC.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCC.GC import GC_MakeSegment
from OCC.BRepGProp import brepgprop_VolumeProperties
from OCC.gp import gp_Ax1, gp_Ax3, gp_Dir, gp_Pln, gp_Pnt, gp_Trsf, gp_Vec
from OCC.GProp import GProp_GProps
from OCC.TopLoc import TopLoc_Location

from dataIO import Display, read_step_file, solid_comp, write_step_file
from topo2 import RecognizeTopo

import rospy


def group_planes_by_axis(shape):
    """[summary]
    Extract all planes from shape, and group them according to their normal
    vector.

    Arguments:
        shape {TopoDS_Shape} -- [description]

    Returns:
        {dict} -- key: normal vector as string)
                  value: list of TopoDS_Shape(Plane)
    """
    if shape is None:
        rospy.logwarn("Input shape is None")
        pln_dict = None
    else:
        planeList = RecognizeTopo(shape).planes()
        pln_dict = {}
        for pln in planeList:
            gp_pln = BRepAdaptor_Surface(pln).Plane()
            normal = gp_pln.Axis().Direction()
            key = '%.6f,%.6f,%.6f' % (round(normal.X(), 6), round(normal.Y(), 6), round(normal.Z(), 6))
            key = tuple([float(i) for i in key.split(',')])
            if key not in pln_dict.keys():
                pln_dict[key] = [pln]
            else:
                pln_dict[key].append(pln)
    return pln_dict


def mv2CMass(shp, pnt):
    # ipdb.set_trace()
    # ais_shp2 = frame.display.DisplayShape(shp2, update=True)
    # the point want to be origin express in local  # the local Z axis expressed in local system expressed in global coordinates

    shp2Trsf = gp_Trsf()
    vec = gp_Vec(pnt, gp_Pnt(0, 0, 0))
    shp2Trsf.SetTranslation(vec)
    shp2Toploc = TopLoc_Location(shp2Trsf)
    shp.Move(shp2Toploc)
    return shp


'''
# Rotate main axis into global Z axis
def turn2Z(shp, mainDir):
    ax3 = gp_Ax3(gp_Pnt(0, 0, 0), mainDir)
    rospy.logdebug('Coordinates:%.3f, %.3f, %.3f' % (ax3.XDirection().X(), ax3.XDirection().Y(), ax3.XDirection().Z()))
    shp2Trsf = gp_Trsf()
    shp2Trsf.SetTransformation(ax3)
    shp2Toploc = TopLoc_Location(shp2Trsf)
    shp.Move(shp2Toploc)
    return shp
'''


def centerOfMass(solid):
    prop = GProp_GProps()
    brepgprop_VolumeProperties(solid, prop)
    return prop.CentreOfMass()


def find_closest_normal_pair(solid_add, solid_base=None, negelet_parallelPair=False, ang_tol=0.5):
    """[summary]
    # [ToDo] sort by angle difference
    Return the direction pairs with least difference of orientation of normal.
    If solid_base is not given, then take the default XY, YZ, ZX planes


    Arguments:
        solid_add {topoDS_shape} -- the solid to be assembled on solid_base
        solid_base {topoDS_shape} -- the solid which the solid_add assembled on

    Keyword Arguments:
        ang_tol [deg] {float} -- the angle between the normal of planes within ang_tol [deg] will be considered as the same pair. (default: {0.5})

    Returns:
        {dictionary} -- 'minVal': {float} [deg] minimal angle between normals of planes
                        'minDirPair': {tuple of float} direction of normal
    """
    # should consider Z axis first?
    if solid_base is None:
        normal_base_withPlanes = None
    else:
        normal_base_withPlanes = group_planes_by_axis(solid_base)
    normal_add_withPlanes = group_planes_by_axis(solid_add)

    # if normal_base_withPlanes is None:
    #     plnXY = gp_Pln(gp_Ax3(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)))
    #     plnYZ = gp_Pln(gp_Ax3(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(1.0, 0.0, 0.0)))
    #     plnZX = gp_Pln(gp_Ax3(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 1.0, 0.0)))
    #     normal_base_withPlanes = {(0, 0, 1): [BRepBuilderAPI_MakeFace(plnXY).Face()],
    #                               (1, 0, 0): [BRepBuilderAPI_MakeFace(plnYZ).Face()],
    #                               (0, 1, 0): [BRepBuilderAPI_MakeFace(plnZX).Face()]}
    if normal_base_withPlanes is None:
        plnXY = gp_Pln(gp_Ax3(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)))
        normal_base_withPlanes = {(0, 0, 1): [BRepBuilderAPI_MakeFace(plnXY).Face()]}
    normals_base = normal_base_withPlanes.keys()
    normals_add = normal_add_withPlanes.keys()
    minPair = {'minVal': 90.0,
               'minGpDirPair': [],
               'minAxisKeyPair': []}

    for normal_base in normals_base:
        dir_base = gp_Dir(normal_base[0], normal_base[1], normal_base[2])
        for normal_add in normals_add:
            dir_add = gp_Dir(normal_add[0], normal_add[1], normal_add[2])
            ang = degrees(dir_base.Angle(dir_add))
            complementary_angle = 180.0 - ang
            min_ang = min(ang, complementary_angle)
            if negelet_parallelPair and min_ang <= ang_tol / 10:
                rospy.logdebug('neglet parallel planes')
                continue

            if abs(min_ang - minPair['minVal']) <= ang_tol:
                minPair['minGpDirPair'].append([dir_base, dir_add])
                minPair['minAxisKeyPair'].append([normal_base, normal_add])

            else:
                if minPair['minVal'] - min_ang >= ang_tol:
                    minPair['minGpDirPair'] = [[dir_base, dir_add]]
                    minPair['minAxisKeyPair'] = [[normal_base, normal_add]]
                    minPair['minVal'] = min_ang
    return minPair


def align_planes_byNormal(shp_add, normalDir_base, normalDir_add, rotateAng):
    """[summary]

    Arguments:
        shp_add {topoDS_Shape} -- [description]
        normalDir_base {gp_Dir} -- [description]
        normalDir_add {gp_Dir} -- [description]
        rotateAng [deg]{float} -- [description]
    """
    if not normalDir_base.IsParallel(normalDir_add, radians(0.01)):
        rotateAxDir = normalDir_base.Crossed(normalDir_add)
        rotateAx1 = gp_Ax1(centerOfMass(shp_add), rotateAxDir)
        ax3 = gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1))
        ax3 = ax3.Rotated(rotateAx1, radians(rotateAng))
        shp2Trsf = gp_Trsf()
        shp2Trsf.SetTransformation(ax3)
        shp2Toploc = TopLoc_Location(shp2Trsf)
        shp_add.Move(shp2Toploc)
    else:
        rospy.logdebug("Planes are already parallel to each other")


def gen_boxSolidAsTable(width=1000, depth=1000, height=1000):
    pntList = [gp_Pnt(width / 2., depth / 2., 0),
               gp_Pnt(-width / 2., depth / 2., 0),
               gp_Pnt(-width / 2., -depth / 2., 0),
               gp_Pnt(width / 2., -depth / 2., 0)]
    edgList = []
    for i in range(0, len(pntList) - 1):
        edgList.append(BRepBuilderAPI_MakeEdge(pntList[i], pntList[i + 1]))
    edgList.append(BRepBuilderAPI_MakeEdge(pntList[3], pntList[0]))

    wire = BRepBuilderAPI_MakeWire()
    for i in edgList:
        wire.Add(i.Edge())
    wireProfile = wire.Wire()
    faceProfile = BRepBuilderAPI_MakeFace(wireProfile).Shape()
    aPrismVec = gp_Vec(0, 0, height)

    return BRepPrimAPI_MakePrism(faceProfile, aPrismVec).Shape()


def get_closest_parallel_planePair(solid_add, solid_base=None, init_min_dist=10.0):
    """
    Arguments:
        solid_add {TopoDS_Solid} -- The solid going to be added

    Keyword Arguments:
        solid_base {TopoDS_Solid} -- By default, solid_base will be xy-plane (default: {None})
        init_min_dist {float} -- [description] (default: {10.0})

    Returns:
        {dict} -- keyValues: miniDist, topoPair, geomPair, mvVec
    """
    min_dist = init_min_dist
    ang_list = find_closest_normal_pair(solid_add, solid_base)
    if solid_base is None:
        solid_base = gen_boxSolidAsTable()
    axisGrp1 = group_planes_by_axis(solid_base)
    axisGrp2 = group_planes_by_axis(solid_add)
    ipdb.set_trace()
    axisPair = ang_list['minAxisKeyPair']
    for axKeyPair in axisPair:
        for topoPln1 in axisGrp1[axKeyPair[0]]:
            surf1 = BRepAdaptor_Surface(topoPln1, True)
            pln1 = surf1.Plane()
            # [ToDo] A better distance evaluation, how to select a correct point which is inside the wire or centerofmass of plane
            plnpnt1 = pln1.Location()
            for topoPln2 in axisGrp2[axKeyPair[1]]:
                surf2 = BRepAdaptor_Surface(topoPln2, True)
                pln2 = surf2.Plane()
                # rospy.logdebug('Distance:', pln2.Distance(plnpnt1))
                dist = pln2.Distance(plnpnt1)
                if dist < min_dist:
                    min_dist = dist
                    minDistTopoPair = [topoPln1, topoPln2]
                    minDistGeomPair = [pln1, pln2]
    p = gp_Pnt(0, 0, 0)
    p1 = ais_ProjectPointOnPlane(p, minDistGeomPair[0])
    p2 = ais_ProjectPointOnPlane(p, minDistGeomPair[1])
    # in order to remove small digits
    projPln = minDistGeomPair[0]
    mvVec = gp_Vec(p2, p1)
    projPlnNormal = gp_Vec(projPln.Axis().Direction())
    mag = np.sign(mvVec.Dot(projPlnNormal)) * mvVec.Magnitude()
    mvVec = projPlnNormal.Normalized().Multiplied(mag)
    return {'minDist': dist, 'topoPair': minDistTopoPair, 'geomPair': minDistGeomPair, 'mvVec': mvVec}


def align_closest_planes(shp, mvVec):
    shp2Trsf = gp_Trsf()
    shp2Trsf.SetTranslation(mvVec)
    shp2Toploc = TopLoc_Location(shp2Trsf)
    shp.Move(shp2Toploc)


def autoPlaneAlign(solid_add, solid_base=None, negletParallelPln=False):
    """ A function to align solids with it's parallel plane pairs
    If solid_base is not given, by default, the added solid will be align to xy-plane

    Arguments:
        solid_add {TopoDS_Solid} -- This solid will be moved in order to algin parallel plane pair

    Keyword Arguments:
        solid_base {TopoDS_Solid} -- This solid will not be moved. If not given, by default, it will be xy-plane (default: {None})
        negletParallelPln {bool} -- Set it False to cacel alignmet, if there's already parallel planes (default: {False})
    """
    # find normal of closest plane pairs
    ang_list = find_closest_normal_pair(solid_add=solid_add, solid_base=solid_base, negelet_parallelPair=negletParallelPln)

    # Get rotation axis
    # [ToDo] the angle value related to rotation direction is not define clearly
    # [ToDo] Add condition: if after rotation the angle between two axis is bigger then reverse one axis
    # [ToDo] use While loop to rotate until angle smaller than given angle tolerance
    # need to weight by the area of surface

    # here we only align the first pair
    align_planes_byNormal(solid_add, ang_list['minGpDirPair'][0][0], ang_list['minGpDirPair'][0][1], rotateAng=ang_list['minVal'])

    # Plane distance detection
    # [ToDo] a better algorithm to choose the correct plane is needed

    minDistPair = get_closest_parallel_planePair(solid_add, solid_base)

    # Move plane to be coincident
    # [ToDo] vector direction need to be determined...
    # [ToDo] while loop to move until distance smaller than given distance tolerance
    align_closest_planes(solid_add, minDistPair['mvVec'])


def __test_autoPlaneAlign():
    fileList = ['lf064-01.stp', 'lf064-0102_1.stp', 'holes_match_morehole_default.stp']
    shapeFromModel = read_step_file(os.path.join('..', 'models', fileList[-1]))
    shp_topo = RecognizeTopo(shapeFromModel)
    solid1 = shp_topo.solids[0]
    solid2 = shp_topo.solids[1]

    frame = Display(solid1, run_display=True)
    frame.add_shape(solid2)
    frame.open()
    ipdb.set_trace(context=10)
    autoPlaneAlign(solid_base=solid1, solid_add=solid2, negletParallelPln=True)
    autoPlaneAlign(solid_base=solid1, solid_add=solid2, negletParallelPln=True)
    write_step_file(solid_comp([solid1, solid2]), 'write_step_file_test.stp')
    frame.open()


if __name__ == '__main__':
    __test_autoPlaneAlign()

    ipdb.set_trace(context=10)
