

from dataIO import read_step_file, Display
from topo2 import RecognizeTopo
import os.path
import logging
from OCC.BRepAdaptor import BRepAdaptor_Surface, BRepAdaptor_Curve
import ipdb
from OCC.gp import gp_Ax1, gp_Pnt, gp_Dir, gp_Trsf, gp_Ax3, gp_Pln
from math import degrees
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


# Todo
'''
def shp_trsf():
    shp2 = read_step_file(os.path.join('..', 'models', fileList[0]))
    # ipdb.set_trace()
    frame.update_shape(shp2)
    # ais_shp2 = frame.display.DisplayShape(shp2, update=True)
    # the point want to be origin express in local  # the local Z axis expressed in local system expressed in global coordinates 
    ax3 = gp_Ax3(gp_Pnt(0., 0., -60.), gp_Dir(0.7071067811865, 0., -0.7071067811865))
    shp2Trsf = gp_Trsf()
    shp2Trsf.SetTransformation(ax3)
    shp2Toploc = TopLoc_Location(shp2Trsf)
    shp2.Move(shp2Toploc)
    frame.update_shape(shp2)

    #frame.display.Context.SetLocation(ais_shp2, shp2Toploc)
    #frame.display.Context.UpdateCurrentViewer()
    #print(shp2.HashCode(100000000))
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
        tolerance {float} -- content: float
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

    ipdb.set_trace(context=10)
    ang_list = find_closest_normal_pair(normal_group2)
    showShapes = shapeFromModel
    frame = Display(solid2, run_display=True)
    ipdb.set_trace(context=10)


