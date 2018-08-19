#!/usr/bin/python
# -*- coding: <utf-8> -*-

from dataIO import read_step_file, Display
from topo2 import RecognizeTopo
import os.path
import logging
from OCC.BRepAdaptor import BRepAdaptor_Surface
from OCC.GeomAbs import GeomAbs_Cylinder
import ipdb
from math import radians, degrees
from core_topology_traverse import Topo
from OCC.AIS import ais_ProjectPointOnPlane
from OCC.gp import gp_Pnt, gp_Vec
from OCC.gp import gp_Trsf
from OCC.TopLoc import TopLoc_Location


class Hole():
    def __init__(self, topods_shp, proj_pln=None):
        self.shp = topods_shp
        adaptor = BRepAdaptor_Surface(self.shp)
        if adaptor.GetType() == GeomAbs_Cylinder:
            self.cyl = adaptor.Cylinder()
            self.axis = self.cyl.Axis()
            self.direction = self.axis.Direction()
            self.location = self.cyl.Location()
            self.localCoor = self.cyl.Position()
            self.neighborIsSet = False 
        else:
            logging.warning('Wrong shape type, input should be cylinder')
        if proj_pln is not None:
            self.setProjPln(proj_pln)

    def setProjPln(self, gp_pln):
        self.projPln = gp_pln
        self.projLocation = ais_ProjectPointOnPlane(self.location, self.projPln)

    def setNeighborHoles(self, listOfHoles):
        localListofHoles = listOfHoles.copy()
        if self in listOfHoles:
            localListofHoles.remove(self)
        self.neighbors = localListofHoles
        self.neighborIsSet = True
        self.neighbor_feature = self.__get_neighbor_feature()

    def __get_neighbor_feature(self):
        neighborsFeatureList = []
        for neighborHole in self.neighbors:
            v1 = gp_Vec(self.projLocation, neighborHole.projLocation)
            r1 = v1.Magnitude()
            # th1 = v1.AngleWithRef(self.projPln.Position().XDirection(), self.projPln.Axis())
            if r1 != 0.0:
                th1 = degrees(v1.AngleWithRef(gp_Vec(self.projPln.XAxis().Direction()), gp_Vec(self.projPln.Axis().Direction())))
                neighborsFeatureList.append({'Hole': neighborHole, 'Magnitude': r1, 'angle': th1})
            else:
                neighborsFeatureList.append({'Hole': neighborHole, 'Magnitude': r1, 'angle': 0.0})
        return neighborsFeatureList

    def get_common_neighnor_candidate(self, ahole, tol=1e-2):
        newListA = []
        newListB = []
        for neighborA in self.neighbor_feature:
            r1 = neighborA['Magnitude']
            for neighborB in ahole.neighbor_feature:
                r2 = neighborB['Magnitude']
                if abs(r2 - r1) < tol:
                    if neighborA not in newListA:
                        newListA.append(neighborA)
                    if neighborB not in newListB:
                        newListB.append(neighborB)
        diffListA = []
        diffListB = []
        for i in range(0, len(newListA)):
            for j in range(i + 1, len(newListA)):
                deltR = round(newListA[i]['Magnitude'] - newListA[j]['Magnitude'], 3)
                deltTh = round(newListA[i]['angle'] - newListA[j]['angle'], 1)
                innerPair = [i, j]
                diffListA.append({'innerPair': innerPair, 'dR': deltR, 'dTh': deltTh})

        for i in range(0, len(newListB)):
            for j in range(i + 1, len(newListB)):
                deltR = round(newListB[i]['Magnitude'] - newListB[j]['Magnitude'], 3)
                deltTh = round(newListB[i]['angle'] - newListB[j]['angle'], 1)
                innerPair = [i, j]
                diffListB.append({'innerPair': innerPair, 'dR': deltR, 'dTh': deltTh})

        print(diffListA)
        print(diffListB)







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


def cylinder_pnt2pln(cyl, pln):
 
    ais_ProjectPointOnPlane(cyl.Location(), pln)


def select_cyl_by_axisDir(full_cylinders, gpDir, ang_tol=0.5):
    """[summary]

    Arguments:
        full_cylinders {dictionary} -- {'string': List of TopoDS_Shape; GeomType in item of list:Cylinder}
        gpDir {gp_Dir} -- target axis direction
        ang_tol {float} -- angular tolerance expressed in Degrees. The angle between cylinder's axis and target axis dirction in this tol, will be consider parallel

    Returns:
        selected_cyl_List {list} -- List of TopoDS_Shape(cylinders)
    """
    selected_cyl_List = []
    for cylList in full_cylinders.values():
        for cyl in cylList:
            axis = BRepAdaptor_Surface(cyl, True).Cylinder().Axis().Direction()
            if gpDir.IsParallel(axis, radians(0.5)):
                selected_cyl_List.append(cyl)
    return selected_cyl_List


def filter_cylinder_by_position(cylList, projPln, dist_tol=0.3):
    initPnt = ais_ProjectPointOnPlane(BRepAdaptor_Surface(cylList[0], True).Cylinder().Location(), projPln)
    projPntList = [initPnt]
    for cyl in cylList:
        newPnt = ais_ProjectPointOnPlane(BRepAdaptor_Surface(cyl, True).Cylinder().Location(), projPln)
        isNewPnt = True
        for pnt in projPntList:
            if newPnt.Distance(pnt) < dist_tol:
                isNewPnt = False
        if isNewPnt:
            projPntList.append(newPnt)
    return projPntList


def get_shortest_vec(pntList1, pntList2):
    minVec = gp_Vec(pntList1[0], pntList2[0])
    for pnt1 in pntList1:
        for pnt2 in pntList2:
            newVec = gp_Vec(pnt1, pnt2)
            if newVec.Magnitude() < minVec.Magnitude():
                minVec = newVec

    return minVec


def get_neighbor_vector(hole_cyl, cylList, projPln):
    vecList = []
    pnt_hole = BRepAdaptor_Surface(hole_cyl, True).Cylinder().Location()
    pnt_start = ais_ProjectPointOnPlane(pnt_hole, projPln)
    for cyl in cylList:
        pnt_cyl = BRepAdaptor_Surface(cyl, True).Cylinder().Location()
        pnt_end = ais_ProjectPointOnPlane(pnt_cyl, projPln)
        vecList.append(gp_Vec(pnt_start, pnt_end))
    return vecList


if __name__ == '__main__':
    logging.basicConfig(filename="logging.txt", filemode='w',
                        level=logging.warning)
    fileList = ['lf064-01.stp', 'cylinder_group_test.stp', 'cylinder_cut.stp',
                'cylinder_cut2.stp', 'face_recognition_sample_part.stp',
                'cylinder_with_side_hole.stp', 'cylinder_with_side_slot.stp',
                'cylinder_with_slot.stp', 'cylinders.stp', 'compound_solid_face-no-contact.stp',
                'lf064-02.stp', 'lf064-0102_1.stp', 'holes_match_3.stp']
    shapeFromModel = read_step_file(os.path.join('..', 'models', fileList[12]))
    # ipdb.set_trace(context=10)
    shp_topo = RecognizeTopo(shapeFromModel)

    solid1 = shp_topo.solids[0]
    solid2 = shp_topo.solids[1]

    frame = Display(solid1, run_display=True)
    frame.add_shape(solid2)
    frame.open()

    cylinders1 = RecognizeTopo(solid1).cylinders()
    cylinders2 = RecognizeTopo(solid2).cylinders()

    coaxial_cylinders1 = group_coaxial_cylinders(cylinders1)
    cylinder_axis_r_group1 = cylinder_group_radius(coaxial_cylinders1)
    full_cylinder_group1 = find_full_cylinder(cylinder_axis_r_group1)

    coaxial_cylinders2 = group_coaxial_cylinders(cylinders2)
    cylinder_axis_r_group2 = cylinder_group_radius(coaxial_cylinders2)
    full_cylinder_group2 = find_full_cylinder(cylinder_axis_r_group2)
    # find cylinders with axis perpendicular to plane or select cylinder by axis direction?
    # find coaxial between solid1 and solid2

    
    # match single hole
    projPln = BRepAdaptor_Surface(RecognizeTopo(solid1).planes()[5], True).Plane()
    normal = projPln.Axis().Direction()

    sel_list1 = select_cyl_by_axisDir(full_cylinder_group1, normal)
    sel_list2 = select_cyl_by_axisDir(full_cylinder_group2, normal, ang_tol=0.5)
    ipdb.set_trace(context=10)

    holeList1 = []
    for i in sel_list1:
        aHole = Hole(i)
        aHole.setProjPln(projPln)
        holeList1.append(aHole)
    for aHole in holeList1:
        aHole.setNeighborHoles(holeList1)

    holeList2 = []
    for i in sel_list2:
        aHole = Hole(i)
        aHole.setProjPln(projPln)
        holeList2.append(aHole)
    for aHole in holeList2:
        aHole.setNeighborHoles(holeList2)

    hole1 = holeList1[2]
    hole2 = holeList2[1]
    ipdb.set_trace(context=10)
    hole1.get_common_neighnor_candidate(hole2)



    abc = get_neighbor_vector(sel_list1[0], sel_list1, projPln=projPln)
    abcList = {}
    for i in abc:
        print('Vec:(%.6f, %.6f, %.6f), Magnitude: %.6f' % (i.X(), i.Y(), i.Z(), round(i.Magnitude(), 4)))
        if i.Magnitude() in abcList:
            abcList[round(i.Magnitude(), 4)].append(i)
        else:
            abcList[round(i.Magnitude(), 4)] = [i]

    defg = get_neighbor_vector(sel_list2[0], sel_list2, projPln=projPln)
    defgList = {}
    for i in defg:
        print('Vec:(%.6f, %.6f, %.6f), Magnitude: %.6f' % (i.X(), i.Y(), i.Z(), round(i.Magnitude(), 4)))
        if i.Magnitude() in defgList:
            defgList[round(i.Magnitude(), 4)].append(i)
        else:
            defgList[round(i.Magnitude(), 4)] = [i]

    commonKeys = []
    for i in abcList.keys():
        if i in defgList:
            commonKeys.append(i)
    ipdb.set_trace(context=10)
    commonVecAngleList = []
    for i in commonKeys:
        for j in abcList[i]:
            for k in defgList[i]:
                if j.Magnitude() != 0 and k.Magnitude != 0:
                    commonVecAngleList.append(degrees(j.Angle(k)))


    for i in range(0, len(commonKeys)):
        
        for j in abcList[commonKeys[i]]:


            for k in defgList[i]:
                if j.Magnitude() != 0 and k.Magnitude != 0:
                    commonVecAngleList.append(degrees(j.Angle(k)))


    ipdb.set_trace(context=10)
    '''
    # match single hole
    filterList1 = filter_cylinder_by_position(sel_list1, projPln)
    filterList2 = filter_cylinder_by_position(sel_list2, projPln)

    mvVec = get_shortest_vec(filterList2, filterList1)
    trsf = gp_Trsf()
    trsf.SetTranslation(mvVec)
    solid2.Move(TopLoc_Location(trsf))
    '''
    # get cylinders with given axis for solid1 and solid2
    # project them all to plane
    # find the closest pair
    # if radius is close then ok

    # for more holes matching
    # align first nearest hole by translation and the match second nearest hole by rotation around first holes

    ipdb.set_trace(context=10)


