

from dataIO import read_step_file, Display, write_step_file, solid_comp
from topo2 import RecognizeTopo
import os.path
import logging
from OCC.BRepAdaptor import BRepAdaptor_Surface
import ipdb
from OCC.gp import gp_Ax1, gp_Pnt, gp_Dir, gp_Trsf, gp_Ax3, gp_Pln, gp_Vec
from math import degrees, radians
from OCC.BRepBuilderAPI import BRepBuilderAPI_MakeFace
from OCC.TopLoc import TopLoc_Location
from OCC.GProp import GProp_GProps
from OCC.BRepGProp import brepgprop_VolumeProperties
from OCC.AIS import ais_ProjectPointOnPlane
import numpy as np


def group_planes_by_axis(shape):
    """[summary]
    Extract all planes from shape, and group them according to their normal
    vector.

    Arguments:
        shape {TopoDS_Shape} -- [description]

    Returns:
        dictionary -- key: normal vector as string)
                      value: list of TopoDS_Shape(Plane)
    """
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
    print('Coordinates:%.3f, %.3f, %.3f' % (ax3.XDirection().X(), ax3.XDirection().Y(), ax3.XDirection().Z()))
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


def find_closest_normal_pair(solid_add, solid_base=None, ang_tol=0.5):
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
    normal_base_withPlanes = group_planes_by_axis(solid1)
    normal_add_withPlanes = group_planes_by_axis(solid2)

    if normal_base_withPlanes is None:
        plnXY = gp_Pln(gp_Ax3(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)))
        plnYZ = gp_Pln(gp_Ax3(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(1.0, 0.0, 0.0)))
        plnZX = gp_Pln(gp_Ax3(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 1.0, 0.0)))
        normal_base_withPlanes = {(0, 0, 1): [BRepBuilderAPI_MakeFace(plnXY).Face()],
                                  (1, 0, 0): [BRepBuilderAPI_MakeFace(plnYZ).Face()],
                                  (0, 1, 0): [BRepBuilderAPI_MakeFace(plnZX).Face()]}
    normals_base = normal_base_withPlanes.keys()
    normals_add = normal_add_withPlanes.keys()
    minPair = {'minVal': 90.0}

    for normal_base in normals_base:
        dir_base = gp_Dir(normal_base[0], normal_base[1], normal_base[2])
        for normal_add in normals_add:
            dir_add = gp_Dir(normal_add[0], normal_add[1], normal_add[2])
            ang = degrees(dir_base.Angle(dir_add))
            complementary_angle = 180.0 - ang
            min_ang = min(ang, complementary_angle)
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
    rotateAxDir = normalDir_base.Crossed(normalDir_add)
    rotateAx1 = gp_Ax1(pnt2, rotateAxDir)
    ax3 = gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1))
    ax3 = ax3.Rotated(rotateAx1, radians(rotateAng))
    shp2Trsf = gp_Trsf()
    shp2Trsf.SetTransformation(ax3)
    shp2Toploc = TopLoc_Location(shp2Trsf)
    shp_add.Move(shp2Toploc)


def get_closest_parallel_planePair(solid_base, solid_add, init_min_dist=10.0):
    min_dist = init_min_dist
    ang_list = find_closest_normal_pair(solid_add, solid_base)
    axisGrp1 = group_planes_by_axis(solid1)
    axisGrp2 = group_planes_by_axis(solid2)
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
                print('Distance:', pln2.Distance(plnpnt1))
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


if __name__ == '__main__':
    logging.basicConfig(filename="logging.txt", filemode='w',
                        level=logging.warning)
    fileList = ['lf064-01.stp', 'cylinder_group_test.stp', 'cylinder_cut.stp',
                'cylinder_cut2.stp', 'face_recognition_sample_part.stp',
                'cylinder_with_side_hole.stp', 'cylinder_with_side_slot.stp',
                'cylinder_with_slot.stp', 'cylinders.stp', 'compound_solid_face-no-contact.stp',
                'lf064-02.stp', 'lf064-0102_1.stp', 'holes_match.stp']
    shapeFromModel = read_step_file(os.path.join('..', 'models', fileList[9]))
    shp_topo = RecognizeTopo(shapeFromModel)
    solid1 = shp_topo.solids[0]
    solid2 = shp_topo.solids[1]

    pnt1 = centerOfMass(solid1)
    pnt2 = centerOfMass(solid2)

    frame = Display(solid1, run_display=True)
    frame.add_shape(solid2)
    frame.open()
    ipdb.set_trace(context=10)
    # find normal of closest plane pairs
    ang_list = find_closest_normal_pair(solid_add=solid2, solid_base=solid1)

    # Get rotation axis
    # [ToDo] the angle value related to rotation direction is not define clearly
    # [ToDo] Add condition: if after rotation the angle between two axis is bigger then reverse one axis
    # [ToDo] use While loop to rotate until angle smaller than given angle tolerance
    # need to weight by the area of surface

    # here we only align the first pair
    align_planes_byNormal(solid2, ang_list['minGpDirPair'][0][0], ang_list['minGpDirPair'][0][1], rotateAng=ang_list['minVal'])

    # Plane distance detection
    # [ToDo] a better algorithm to choose the correct plane is needed

    minDistPair = get_closest_parallel_planePair(solid2, solid1)

    # Move plane to be coincident
    # [ToDo] vector direction need to be determined...
    # [ToDo] while loop to move until distance smaller than given distance tolerance
    align_closest_planes(solid2, minDistPair['mvVec'])

    # temp = solid_comp([solid1, solid2])
    # write_step_file(solid_comp([solid1, solid2]), 'lf064-0102_1.stp')
    frame.open()

    ipdb.set_trace(context=10)
