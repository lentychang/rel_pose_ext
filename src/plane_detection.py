

from dataIO import read_step_file, Display
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
        key = '%.6f,%.6f,%.6f' % (normal.X(), normal.Y(), normal.Z())
        if key not in pln_dict.keys():
            pln_dict[key] = [pln]
        else:
            pln_dict[key].append(pln)
    return pln_dict


def list2gpVec(ls):
    if len(ls) != 3:
        print('Error, the length of the input list should be 3, [float x, float y, float z]')
    return gp_Vec(ls[0], ls[1], ls[2])


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


def find_closest_normal_pair(normal_add, normal_base=None, tolerance=0.5):
    """[summary]
    Return the plane pairs with least difference of orientation of normal.
    If normal_base is not given, then take the default XY, YZ, ZX planes

    Arguments:
        normal_add {dictionary} -- {'normalOfPlane':plane}
                                     normalOfPlane{string}
                                     plane{list of TopoDS_Shape}
        normal_base {dictionary} -- same as normal_base

    Keyword Arguments:
        tolerance {float} -- content: float    vec = [float(i) for i in ang_list['minPair'][0][1].split(',')]
                             the angle within this tolerance [deg] will be consider as the same set. (default: {0.5})

    Returns:
        {dictionary} -- content: {string:TopoDS_Shape}
    """
    # should consider Z axis first?
    if normal_base is None:
        plnXY = gp_Pln(gp_Ax3(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)))
        plnYZ = gp_Pln(gp_Ax3(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(1.0, 0.0, 0.0)))
        plnZX = gp_Pln(gp_Ax3(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 1.0, 0.0)))
        normal_base = {'0.000,0.000,1.000': [BRepBuilderAPI_MakeFace(plnXY).Face()],
                       '1.000,0.000,0.000': [BRepBuilderAPI_MakeFace(plnYZ).Face()],
                       '0.000,1.000,0.000': [BRepBuilderAPI_MakeFace(plnZX).Face()]}
    keys_base = normal_base.keys()
    keys_add = normal_add.keys()
    tol = tolerance  # [deg]
    minPair = {'minVal': 90.0, 'minPair': []}

    for key_base in keys_base:
        surf_base = BRepAdaptor_Surface(normal_base[key_base][0], True)
        ax1_normal_base = surf_base.Plane().Axis()
        for key_add in keys_add:
            surf_add = BRepAdaptor_Surface(normal_add[key_add][0], True)
            ax1_normal_add = surf_add.Plane().Axis()
            ang = degrees(ax1_normal_base.Angle(ax1_normal_add))
            complementary_angle = 180.0 - ang
            min_ang = min(ang, complementary_angle)
            if abs(min_ang - minPair['minVal']) <= tol:
                minPair['minPair'].append([key_base, key_add])
            else:
                if minPair['minVal'] - min_ang >= tol:
                    minPair['minPair'] = [[key_base, key_add]]
                    minPair['minVal'] = min_ang
    return minPair


if __name__ == '__main__':
    logging.basicConfig(filename="logging.txt", filemode='w',
                        level=logging.warning)
    fileList = ['lf064-01.stp', 'cylinder_group_test.stp', 'cylinder_cut.stp',
                'cylinder_cut2.stp', 'face_recognition_sample_part.stp',
                'cylinder_with_side_hole.stp', 'cylinder_with_side_slot.stp',
                'cylinder_with_slot.stp', 'cylinders.stp', 'compound_solid_face-no-contact.stp',
                'lf064-02.stp']
    shapeFromModel = read_step_file(os.path.join('..', 'models', fileList[9]))
    shp_topo = RecognizeTopo(shapeFromModel)
    solid1 = shp_topo.solids[0]
    solid2 = shp_topo.solids[1]

    pnt1 = centerOfMass(solid1)
    pnt2 = centerOfMass(solid2)

    normal_group1 = group_planes_by_axis(solid1)
    normal_group2 = group_planes_by_axis(solid2)   
    ang_list = find_closest_normal_pair(normal_group2, normal_base=normal_group1)

    frame = Display(solid1, run_display=True)
    frame.add_shape(solid2)
    # Get rotation axis
    # [ToDo] the angle value related to rotation direction is not define clearly
    # [ToDo] adapt a, b with the vector got from ang_list
    # [ToDo] Add condition: if after rotation the angle between two axis is bigger then reverse one axis
    # [ToDo] use While loop to rotate until angle smaller than given angle tolerance
    a = list2gpVec([0., 1., 0.])
    b = list2gpVec([0.146, 0.985, -0.088])
    c = a.Crossed(b)
    rev_ax = gp_Dir(c)
    ax1 = gp_Ax1(pnt2, rev_ax)
    ax3 = gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1))
    ax3 = ax3.Rotated(ax1, radians(float(ang_list['minVal'])))
    shp2Trsf = gp_Trsf()
    shp2Trsf.SetTransformation(ax3)
    shp2Toploc = TopLoc_Location(shp2Trsf)
    solid2.Move(shp2Toploc)

    # Plane distance detection
    # [ToDo] a better algorithm to choose the correct plane is needed
    normal_group1 = group_planes_by_axis(solid1)
    normal_group2 = group_planes_by_axis(solid2)

    min_dist = 10.0
    ang_list = find_closest_normal_pair(normal_group2, normal_base=normal_group1)
    # [ToDo] A better distance evaluation, how to select a correct point which is inside the wire
    for topoPln1 in normal_group1[ang_list['minPair'][0][0]]:
        surf1 = BRepAdaptor_Surface(topoPln1, True)
        pln1 = surf1.Plane()
        plnpnt1 = pln1.Location()

        for topoPln2 in normal_group2[ang_list['minPair'][0][1]]:
            surf2 = BRepAdaptor_Surface(topoPln2, True)
            pln2 = surf2.Plane()
            print('Distance:', pln2.Distance(plnpnt1))
            min_dist = min(pln2.Distance(plnpnt1), min_dist)

    # Move plane to be coincident
    # [ToDo] vector direction need to be determined...
    # [ToDo] while loop to move until distance smaller than given distance tolerance
    ax3 = gp_Ax3()
    shp2Trsf = gp_Trsf()    
    dirList = [float(i) for i in ang_list['minPair'][0][0].split(',')]    
    mvVec = gp_Vec(dirList[0], dirList[1], dirList[2]).Normalized().Multiplied(-1.0 * min_dist)
    shp2Trsf.SetTranslation(mvVec)
    shp2Toploc = TopLoc_Location(shp2Trsf)
    solid2.Move(shp2Toploc)

    ipdb.set_trace()

    # Another mether to align the normal of plane: too complicated
    '''
    # extract normal vector into a list
    toVec = [float(i) for i in ang_list['minPair'][0][0].split(',')]
    xVec = [1.0, 0.0, 0.0]
    fromVec = [float(i) for i in ang_list['minPair'][0][1].split(',')]
    fromDir = gp_Dir(fromVec[0], fromVec[1], fromVec[2])
    toDir = gp_Dir(toVec[0], toVec[1], toVec[2])
    toDir = gp_Dir(0.0, 1.0, 0.0)
    # solid 1 turn to Z
    turn2Z(solid1, toDir)
    # solid 2 move to center of Mass
    mv2CMass(solid2, pnt2)
    # solid2 turn to Z
    turn2Z(solid2, toDir)
    ax3 = gp_Ax3(gp_Pnt(0, 0, 0), toDir)
    shp2Trsf = gp_Trsf()
    shp2Trsf.SetTransformation(ax3)
    pnt2 = pnt2.Transformed(shp2Trsf)
    # align to Z
    toDir2 = gp_Dir(-0.088, 0.146, 0.985)
    turn2Z(solid2, toDir2)
    mv2CMass(solid2, pnt2.Mirrored(gp_Pnt(0,0,0)))
    '''
    ipdb.set_trace(context=10)
    frame.open()

    ipdb.set_trace(context=10)


