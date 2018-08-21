#!/usr/bin/python
# -*- coding: <utf-8> -*-

from dataIO import read_step_file, Display, extractDictByVal
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


def create_holeList(topds_shp_list, proj_pln=None):
    holeList = []
    for i in topds_shp_list:
        aHole = Hole(i)
        if proj_pln is not None:
            aHole.setProjPln(projPln)
        holeList.append(aHole)
    return holeList


def getNeighborRelTable(holeList, projPlane, roundingDigit_dist=2, roundingDigit_ang=1):
    distTable = {}
    angTable = {}
    for holeA in holeList:
        for holeB in holeList:
            if holeA == holeB:
                continue
            vec2neighbor = gp_Vec(holeA.projLocation, holeB.projLocation)
            # [ToDo] change the type of key of distTable into set{} which negelet order
            distTable[(holeA, holeB)] = round(vec2neighbor.Magnitude(), roundingDigit_dist)
            angTable[(holeA, holeB)] = round(degrees(vec2neighbor.AngleWithRef(gp_Vec(projPlane.XAxis().Direction()), gp_Vec(projPlane.Axis().Direction()))), roundingDigit_ang)
    return {'distTable': distTable, 'angTable': angTable}


def __group_coaxial_cylinders(cylinders, tol_ang=0.5, tol_lin=0.1):
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


def __cylinder_group_radius(cyl_ax_grp, tol=0.1):
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


def find_full_cylinder(cylinders):
    coaxial_cylinders = __group_coaxial_cylinders(cylinders)
    cylinder_group = __cylinder_group_radius(coaxial_cylinders)

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


def find_n_max(inputList):
    ls = list(inputList)
    ls.sort()
    n = len(ls)
    for i in range(max(ls), 0, -1):
        print(i)
        if i in ls:
            val = n - ls.index(i)
            if val >= i + 1:
                maxPossibleVal = i
                break
        else:
            # if in not in ls, then use val computed from previous computation
            if val >= i + 1:
                maxPossibleVal = i
                break
    n_max = maxPossibleVal + 1
    return n_max


def get_holes_pairs(list1, list2, projPlane):
    holeList1 = create_holeList(list1, projPlane)
    holeList2 = create_holeList(list2, projPlane)

    # tolerance should be considered
    relTable1 = getNeighborRelTable(holeList1, projPlane)
    distTable1 = relTable1['distTable'] 
    angTable1 = relTable1['angTable']
    relTable2 = getNeighborRelTable(holeList2, projPlane)
    distTable2 = relTable2['distTable'] 
    angTable2 = relTable2['angTable']

    # intersection, get the common distance
    dist_set1 = set(list(distTable1.values()))
    dist_set2 = set(list(distTable2.values()))
    commonDist = dist_set1.intersection(dist_set2)

    # table with common distance
    commonDistTable1 = extractDictByVal(distTable1, commonDist)
    commonDistTable2 = extractDictByVal(distTable2, commonDist)

    # turn keys into set type inorder to count number of each point/hole
    holePairs1 = list(commonDistTable1.keys())
    holePairsConcat1 = [i for t in holePairs1 for i in t]
    holeSet1 = set(holePairsConcat1)
    holePairs2 = list(commonDistTable2.keys())
    holePairsConcat2 = [i for t in holePairs2 for i in t]
    holeSet2 = set(holePairsConcat2)
    holeCountList1, holeCountList2 = [], []
    for i in holeSet1:
        holeCountList1.append(holePairsConcat1.count(i))
    for i in holeSet2:
        holeCountList2.append(holePairsConcat2.count(i))

    # find minimal possible maximal n
    nMax = min(find_n_max(holeCountList2), find_n_max(holeCountList2))
    # [Attention, if distTable's key change into set type, threshold shoulb be (n-1)]
    threshold = 2 * (nMax - 1)
    # if connection number of the point is smaller than threshold, delete it.
    # [Attention] algoritm works only 80% cases. sometimes solution N is much smaller than N_max
    # please see list_gen.py
    rmHoles1, rmHoles2 = [], []
    newHolePairs1, newHolePairs2 = holePairs1, holePairs2
    for i in holeSet1:
        if holePairsConcat1.count(i) < threshold:
            rmHoles1.append(i)  
    for i in holeSet2:
        if holePairsConcat2.count(i) < threshold:
            rmHoles2.append(i)
    if len(rmHoles1) != 0:
        newHolePairs1 = [i for i in holePairs1 for k in rmHoles1 if k not in i]
    if len(rmHoles2) != 0:
        newHolePairs2 = [i for i in holePairs2 for k in rmHoles2 if k in i]

    # get holes
    newHoleSet12 = set([k for i in newHolePairs1 for k in i])
    newHoleSet21 = set([k for i in newHolePairs2 for k in i])

    # grouping hole pairs by first hole
    groupHole1 = {}
    for aHole in newHoleSet12:
        selectedHolePairs = [i for i in newHolePairs1 if i[0] == aHole]
        groupHole1[aHole] = selectedHolePairs

    # create feature table
    featureTable1 = {}
    for aHole in newHoleSet12:
        # extract [r, th] table
        valList = []
        for holePair in groupHole1[aHole]:
            r = commonDistTable1[holePair]
            th = angTable1[holePair]
            neighborHole = list(holePair)
            neighborHole = neighborHole[1]
            valList.append([r, th])
        # compute gradient table for [r, th]
        dValList = []
        for i in range(0, len(valList)):
            for j in range(i + 1, len(valList)):
                dR = valList[i][0] - valList[j][0]
                dTh = valList[i][1] - valList[j][1]
                # rounding is done when create distTable, but some lower digit don't know why appear
                dValList.append((round(dR, 2), round(dTh, 1)))
        featureTable1[aHole] = dValList

    groupHole2 = {}
    for aHole in newHoleSet21:
        selectedHolePairs = [i for i in newHolePairs2 if i[0] == aHole]
        groupHole2[aHole] = selectedHolePairs

    featureTable2 = {}
    for aHole in newHoleSet21:
        valList = []
        for holePair in groupHole2[aHole]:
            r = commonDistTable2[holePair]
            th = angTable2[holePair]
            neighborHole = list(holePair)
            neighborHole = neighborHole[1]
            valList.append([r, th])
        dValList = []
        for i in range(0, len(valList)):
            for j in range(i + 1, len(valList)):
                dR = valList[i][0] - valList[j][0]
                dTh = valList[i][1] - valList[j][1]
                dValList.append((round(dR, 2), round(dTh, 1)))
        featureTable2[aHole] = dValList

    holeMatchingPair = []
    for i in featureTable1.keys():
        for j in featureTable2.keys():
            m = set(featureTable1[i])
            n = set(featureTable2[j])
            if m == n:
                holeMatchingPair.append([i, j])
    return holeMatchingPair


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

    full_cylinder_group1 = find_full_cylinder(cylinders1)
    full_cylinder_group2 = find_full_cylinder(cylinders2)
    # find cylinders with axis perpendicular to plane or select cylinder by axis direction?
    # find coaxial between solid1 and solid2
    
    # match single hole
    projPln = BRepAdaptor_Surface(RecognizeTopo(solid1).planes()[5], True).Plane()
    normal = projPln.Axis().Direction()

    sel_list1 = select_cyl_by_axisDir(full_cylinder_group1, normal)
    sel_list2 = select_cyl_by_axisDir(full_cylinder_group2, normal, ang_tol=0.5)
    ipdb.set_trace(context=10)

    get_holes_pairs(sel_list1, sel_list2, projPln)


    '''
    # match single hole
    filterList1 = filter_cylinder_by_position(sel_list1, projPln)
    filterList2 = filter_cylinder_by_position(sel_list2, projPln)

    mvVec = get_shortest_vec(filterList2, filterList1)
    trsf = gp_Trsf()
    trsf.SetTranslation(mvVec)
    solid2.Move(TopLoc_Location(trsf))
    '''

    ipdb.set_trace(context=10)


