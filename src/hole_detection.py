#!/usr/bin/python
# -*- coding: <utf-8> -*-

from dataIO import read_step_file, Display
from topo2 import RecognizeTopo
import os.path
import logging
from OCC.BRepAdaptor import BRepAdaptor_Surface
import ipdb
from math import radians
from core_topology_traverse import Topo


def group_coaxial_cylinders(cylinders, tol_ang=0.5, tol_lin=0.1):
    """According to cylinders' axis, categorize a list of cylinders into a dictionary by using its axis as a key.

    Arguments:
        cylinders {[TopoDS_Face,...]} -- list of TopoDS_Face
        tol_ang {float} -- [Unit - degree] the angle between these two axis below this value will be recognize as two parallel axis
        tol_lin

    Returns:
        {'string': [TopoDS_Face,...]} -- returns a dictionary, key is string of axis and location vector, value is list of TopoDS_Shape
    """
    logging.debug('Entering group_coaxial')   
    tol_rad = radians(tol_ang)
    skipList = []
    cyl_ax_grp = {}
    for i in range(0, len(cylinders)):
        if i in skipList:
            continue

        cyl1 = BRepAdaptor_Surface(cylinders[i], True).Cylinder()
        axis1 = cyl1.Axis()
        location1 = cyl1.Location()
        key = 'Axis=%.3f,%.3f,%.3f,Loc=%.3f,%.3f,%.3f' % (axis1.Direction().X(),
                                                          axis1.Direction().Y(),
                                                          axis1.Direction().Z(),
                                                          location1.X(),
                                                          location1.Y(),
                                                          location1.Z())
        
        if key not in cyl_ax_grp:
            cyl_ax_grp[key] = [cylinders[i]]
        else:
            logging.warning('Error !!! Please check the logic again !')
        
        for j in range(i + 1, len(cylinders)):
            logging.debug('i = %d, j = %d' % (i, j))

            if j in skipList:
                logging.debug('skip !!')
                continue
            cyl2 = BRepAdaptor_Surface(cylinders[j]).Cylinder()
            axis2 = cyl2.Axis()            
            if axis1.IsCoaxial(axis2, tol_rad, tol_lin):
                logging.debug('Coaxial !!')
                cyl_ax_grp[key].append(cylinders[j])
                skipList.append(j)
    return cyl_ax_grp


def cylinder_group_radius(cyl_ax_grp, tol=0.1):
    logging.debug('Entering group_radius')

    keyList = list(cyl_ax_grp.keys())  
    cyl_rad_grp = {}
    for k in keyList:
        cylinders = cyl_ax_grp[k]
        skipList = []
        for i in range(0, len(cylinders)):
            if i in skipList:
                continue
            cyl1 = BRepAdaptor_Surface(cylinders[i], True).Cylinder()
            radius1 = cyl1.Radius()
            key = '%s,r=%.3f' % (k, radius1)

            if key not in cyl_rad_grp:
                cyl_rad_grp[key] = [cylinders[i]]
            else:
                logging.warning('Error !!! Please check the logic again !')

            for j in range(i + 1, len(cylinders)):
                logging.debug('i = %d, j = %d' % (i, j))
                if j in skipList:
                    continue
                cyl2 = BRepAdaptor_Surface(cylinders[j], True).Cylinder()
                radius2 = cyl2.Radius()
                if abs(radius1 - radius2) <= tol:
                    logging.debug('Same radius found')
                    cyl_rad_grp[key].append(cylinders[j])
                    skipList.append(j)
    return cyl_rad_grp


def find_full_cylinder(cylinder_group):
    keys = list(cylinder_group.keys())
    full_cyl_list = {}
    for key in keys:
        for i in range(0, len(cylinder_group[key])):
            counter = 0
            vtxList1 = list(Topo(cylinder_group[key][i]).vertices())
            vtxList2 = []
            for j in range(i + 1, len(cylinder_group[key])):
                vtxList2 = list(Topo(cylinder_group[key][j]).vertices())
                for k in vtxList1:
                    for l in vtxList2:
                        if k.IsSame(l):
                            counter = counter + 1

            # After comparision, if both cylinder has the same vertices, they stand for the same full cylinder
            # Or if there are only two vertices in cylinder surface, it is a full cylinder
            if (counter == len(vtxList1) and counter == len(vtxList2)) or len(vtxList1) == 2:
                if key in full_cyl_list:
                    full_cyl_list[key].append(cylinder_group[key][i])
                else:
                    full_cyl_list[key] = [cylinder_group[key][i]]
    return full_cyl_list


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
    solid1cylinders = RecognizeTopo(solid1).cylinders()
    ipdb.set_trace(context=10)
    coaxial_cylinders = group_coaxial_cylinders(solid1cylinders)
    cylinder_axis_r_group = cylinder_group_radius(coaxial_cylinders)
    full_cylinder_group = find_full_cylinder(cylinder_axis_r_group)
    
    ipdb.set_trace(context=10)

    frame = Display(solid1, run_display=True)
    frame.add_shape(solid2)
    frame.open()
