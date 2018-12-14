#!/usr/bin/python
# -*- coding: <utf-8> -*-

import logging
import os.path
from math import degrees, radians

import ipdb
import numpy as np
import rospy
from OCC.AIS import ais_ProjectPointOnPlane
from OCC.BRepAdaptor import BRepAdaptor_Surface
from OCC.GeomAbs import GeomAbs_Cylinder
from OCC.gp import gp_Dir, gp_Mat, gp_Pln, gp_Pnt, gp_Quaternion, gp_Trsf, gp_Vec, gp_XYZ
from OCC.TopLoc import TopLoc_Location

from core_topology_traverse import Topo
from dataIO import (Display, deg2PlusMinusPi, extractDictByVal, read_step_file,
                    tuple2gpDir)
from topo2 import RecognizeTopo


class Hole():
    def __init__(self, topods_shp, proj_pln=None):
        self.shp = topods_shp
        adaptor = BRepAdaptor_Surface(self.shp)
        if adaptor.GetType() == GeomAbs_Cylinder:
            self.cyl = adaptor.Cylinder()
            self.axis = self.cyl.Axis()
            self.direction = self.axis.Direction()
            self.location = self.cyl.Location()
            self.neighborIsSet = False
        else:
            logging.warning('Wrong shape type, input should be cylinder')
        if proj_pln is not None:
            self.setProjPln(proj_pln)

    def setProjPln(self, gp_pln):
        self.projPln = gp_pln
        self.projLocation = ais_ProjectPointOnPlane(self.location, self.projPln)


def test_print_projPnt(holeList):
    n = 1
    for i in holeList:
        pnt = i.projLocation
        rospy.logdebug('hole %d: (%.3f, %.3f, %.3f)' % (n, pnt.X(), pnt.Y(), pnt.Z()))
        n += 1


def test_print_matchedPntPairXYZ(matchedPairList):
    for matchedPair in matchedPairList:
        loc1 = matchedPair[0].cyl.Location()
        loc2 = matchedPair[1].cyl.Location()
        rospy.logdebug('Base: (%.3f, %.3f, %.3f) , %s' % (loc1.X(), loc1.Y(), loc1.Z(), matchedPair[0].shp.HashCode(1000000)))
        rospy.logdebug('Add : (%.3f, %.3f, %.3f) , %s' % (loc2.X(), loc2.Y(), loc2.Z(), matchedPair[1].shp.HashCode(1000000)))


def create_holeList(topds_shp_list, proj_pln=None):
    holeList = []
    for i in topds_shp_list:
        aHole = Hole(i)
        if proj_pln is not None:
            aHole.setProjPln(proj_pln)
        holeList.append(aHole)
    return holeList


def getNeighborRelTable(holeList, projPlane, roundingDigit_dist=6, roundingDigit_ang=4):
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


def group_cylinders_byAxisDir(cylinders, anglTolDeg=5, roundDigit=6, groupParallelAx=True):
    logging.debug('Entering group_cylinders_byAxisDir')
    cyl_ax = {}
    for cylinder in cylinders:
        cyl = BRepAdaptor_Surface(cylinder, True).Cylinder()
        axis = cyl.Axis()
        key = (round(axis.Direction().X(), roundDigit),
               round(axis.Direction().Y(), roundDigit),
               round(axis.Direction().Z(), roundDigit))
        if key in cyl_ax.keys():
            cyl_ax[key].append(cylinder)
        else:
            cyl_ax[key] = [cylinder]
    for key in cyl_ax.keys():
        listOflist = list(find_full_cylinder(cyl_ax[key]).values())
        cyl_ax[key] = [k for i in listOflist for k in i]

    if groupParallelAx:
        axisKeyList = list(cyl_ax.keys())
        combineList = []
        j = 0
        while len(axisKeyList) >= 1:
            ax1 = axisKeyList.pop(0)
            grp = [ax1]
            dir1 = gp_Dir(ax1[0], ax1[1], ax1[2])
            while j < len(axisKeyList):
                ax2 = axisKeyList[j]
                dir2 = gp_Dir(ax2[0], ax2[1], ax2[2])
                j += 1
                if dir1.IsParallel(dir2, radians(anglTolDeg)):
                    grp.append(ax2)
                    j -= 1
                    axisKeyList.pop(j)
            combineList.append(grp)
            j = 0
        cyl_grpByAx = {}
        for i in combineList:
            cyl_grpByAx[i[0]] = []
            for j in i:
                cyl_grpByAx[i[0]] += cyl_ax[j]
    else:
        cyl_grpByAx = cyl_ax

    return cyl_grpByAx


def __group_coaxial_cylinders(cylinders, tol_ang=0.5, tol_lin=0.1, roundDigit=6):
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
        axisDir1 = (round(axis1.Direction().X(), roundDigit),
                    round(axis1.Direction().Y(), roundDigit),
                    round(axis1.Direction().Z(), roundDigit))
        cylLoc1 = (round(location1.X(), roundDigit),
                   round(location1.Y(), roundDigit),
                   round(location1.Z(), roundDigit))
        key = (axisDir1, cylLoc1)

        if key not in cyl_ax_grp.keys():
            cyl_ax_grp[key] = [cylinders[i]]
        else:
            logging.warning('Error !!! Please check the logic again !')

        for j in range(i + 1, len(cylinders)):
            # logging.debug('i = %d, j = %d' % (i, j))

            if j in skipList:
                # logging.debug('skip !!')
                continue
            cyl2 = BRepAdaptor_Surface(cylinders[j]).Cylinder()
            axis2 = cyl2.Axis()
            if axis1.IsCoaxial(axis2, tol_rad, tol_lin) or axis1.IsCoaxial(axis2.Reversed(), tol_rad, tol_lin):
                # logging.debug('Coaxial !!')
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
                # logging.debug('i = %d, j = %d' % (i, j))
                if j in skipList:
                    continue
                cyl2 = BRepAdaptor_Surface(cylinders[j], True).Cylinder()
                radius2 = cyl2.Radius()
                if abs(radius1 - radius2) <= tol:
                    # logging.debug('Same radius found')
                    cyl_rad_grp[key].append(cylinders[j])
                    skipList.append(j)
    return cyl_rad_grp


def find_full_cylinder(cylinders):
    # coaxial_cylinders = __group_coaxial_cylinders(cylinders)
    cylinder_group = __group_coaxial_cylinders(cylinders)
    # cylinder_group = __cylinder_group_radius(coaxial_cylinders)

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


def filter_cylinder_by_position(cylList_from_solidAdd, projPln, dist_tol=0.3):
    """Project all points of cylinders' axis into the projPln, and return the projected point on the projPln

    Arguments:
        cylList_from_solidAdd {[type]} -- [description]
        projPln {[type]} -- [description]

    Keyword Arguments:
        dist_tol {float} -- [description] (default: {0.3})

    Returns:
        [type] -- [description]
    """
    # project the first cylinder to get the point. Which is needed for the later distance calculation
    initPnt = ais_ProjectPointOnPlane(BRepAdaptor_Surface(cylList_from_solidAdd[0], True).Cylinder().Location(), projPln)
    projPntList = [initPnt]
    for cyl in cylList_from_solidAdd:
        newPnt = ais_ProjectPointOnPlane(BRepAdaptor_Surface(cyl, True).Cylinder().Location(), projPln)
        isNewPnt = True
        # check whehter the projected point overlaps other points
        for pnt in projPntList:
            if newPnt.Distance(pnt) < dist_tol:
                isNewPnt = False
        if isNewPnt:
            projPntList.append(newPnt)
    return projPntList


def get_shortest_vec(pntList_add, pntList_base):
    minVec = gp_Vec(pntList_add[0], pntList_base[0])
    for pnt_base in pntList_base:
        for pnt_add in pntList_add:
            newVec = gp_Vec(pnt_add, pnt_base)
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


def find_n_max(countList1, countList2):
    """[summary]
    compute the possible maximal (N)umber of matching points from one model.
    In practical the function should be applied on holeCountLists of two models. And then choose the minimal one.
    Arguments:
        countList1 {list} -- each element reprecent the number of connections to a point.
                             The direction of connections is not considered.
                             Ex, PointA->point B and from point B-> pointA are consider as the same connection.
                             The connection number of point A is 1. Also connection number of point B is 1.
        countList2 {list} -- same as above

    Returns:
        maxPossibleN {int} -- N means the number of matching points
    """
    def find_n_max_sub(inputList):
        ls = list(inputList)
        ls = [int(i / 2) for i in ls]
        ls.sort()
        n = len(ls)
        for i in range(max(ls), 0, -1):
            if i in ls:
                count = n - ls.index(i)
                if count >= i + 1:
                    nMax = i + 1
                    break
            else:
                # if in not in ls, then use val computed from previous computation
                if count >= i + 1:
                    nMax = i + 1
                    break
        return nMax
    maxPossibleN = min(find_n_max_sub(countList1), find_n_max_sub(countList2))
    return maxPossibleN


def get_holes_pairs(list1, list2, projPlane, roundDigit_lin=6, roundDigit_ang=6):
    holeList1 = create_holeList(list1, projPlane)
    # test_print_projPnt(holeList1)
    holeList2 = create_holeList(list2, projPlane)
    # test_print_projPnt(holeList2)

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
    if len(commonDist) == 0:
        rospy.logwarn('There''s no common hole pairs !!!')
        return []

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
    nMax = find_n_max(holeCountList1, holeCountList2)
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
        temp = []
        for i in holePairs1:
            if i[0] not in rmHoles1 and i[1] not in rmHoles1:
                temp.append(i)
        newHolePairs1 = temp
    if len(rmHoles2) != 0:
        temp = []
        for i in holePairs2:
            if i[0] not in rmHoles2 and i[1] not in rmHoles2:
                temp.append(i)
        newHolePairs2 = temp

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
                dTh = deg2PlusMinusPi(valList[i][1] - valList[j][1])
                if dR < 0:
                    dR = -dR
                    dTh = -dTh
                # rounding is done when create distTable, but some lower digit don't know why appear
                dValList.append((round(dR, roundDigit_lin), round(dTh, roundDigit_ang)))
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
                dTh = deg2PlusMinusPi(valList[i][1] - valList[j][1])
                if dR < 0:
                    dR = -dR
                    dTh = -dTh
                dValList.append((round(dR, roundDigit_lin), round(dTh, roundDigit_ang)))
        featureTable2[aHole] = dValList

    holeMatchingPair = []
    for i in featureTable1.keys():
        for j in featureTable2.keys():
            m = set(featureTable1[i])
            n = set(featureTable2[j])
            if m == n:
                holeMatchingPair.append([i, j])
    if len(holeMatchingPair) == 0:
        featureTable2_mirrored = featureTable2.copy()
        for i in featureTable2_mirrored:
            i[1] = -i[1]
        for i in featureTable1.keys():
            for j in featureTable2_mirrored.keys():
                m = set(featureTable1[i])
                n = set(featureTable2_mirrored[j])
            if m == n:
                holeMatchingPair.append([i, j])
        if len(holeMatchingPair) != 0:
            rospy.logwarn('One of the model need to be mirrored/ or rotate with axis on the plane with its normal parallel to cylinder axis !!!')
        rospy.logwarn('There''s no common hole pairs !!!')
        return []
    # test_print_matchedPntPairXYZ(holeMatchingPair)
    rospy.logdebug('Found Matching Pairs: %d\n' % len(holeMatchingPair))
    return holeMatchingPair


def centerOfMass_pnts(pnts):
    xSum, ySum, zSum = 0, 0, 0
    for pnt in pnts:
        xSum += pnt.X()
        ySum += pnt.Y()
        zSum += pnt.Z()
        # rospy.logdebug('%.2f, %.2f, %.2f' % (pnt.X(), pnt.Y(), pnt.Z()))
    n = len(pnts)
    centerOfMass = gp_Pnt(xSum / n, ySum / n, zSum / n)
    return centerOfMass


def gpVec2npMat(gpVec):
    return np.matrix([[gpVec.X()], [gpVec.Y()], [gpVec.Z()]])


def align_mutiHoles(shp, matched_hole_pairs):
    # matching points
    pntsA = [i[0].projLocation for i in matched_hole_pairs]
    pntsB = [i[1].projLocation for i in matched_hole_pairs]
    normal = gp_Vec(matched_hole_pairs[0][1].direction)

    pntsA_noDuplicates = list(set(pntsA))
    pntsB_noDuplicates = list(set(pntsB))
    if len(pntsA_noDuplicates) != len(pntsA) or len(pntsB_noDuplicates) != len(pntsB):
        print("[WARN] There are duplicated pairs(i.e. [A,B], [B, A]) in the given matched_hole_pairs")
    assert len(pntsA_noDuplicates) == len(pntsB_noDuplicates), "[Error] data are not pairwise"
    assert len(pntsA_noDuplicates) >= 2 or len(pntsB_noDuplicates) >= 2, "[FATEL] input hole pairs should be more than 2"

    # Create the 3rd point on the project plane in the perpandicular diretion from the 1st to the 2nd point
    # [TODO] More test
    # Now 3rd point was added in the middle, so there will be sysmetry problem?
    if (len(pntsA_noDuplicates) == 2 and len(pntsB_noDuplicates) == 2):
        print("[WARN] only 2 hole pairs found, add a dummpy pair to constrain rotating along the normal project plane")
        # distance between two points, which is used as a scale
        dist = pntsA_noDuplicates[0].Distance(pntsA_noDuplicates[1]) / 2.0
        vecA = gp_Vec(pntsA_noDuplicates[0], pntsA_noDuplicates[1])
        # Translation vector from orignal mid-points to the 3rd point(dummy points)
        translateVecA = vecA.Crossed(normal).Normalized().Scaled(dist)
        dummyPntA = centerOfMass_pnts(pntsA_noDuplicates).Translated(translateVecA)
        pntsA_noDuplicates.append(dummyPntA)

        vecB = gp_Vec(pntsB_noDuplicates[0], pntsB_noDuplicates[1])
        translateVecB = vecB.Crossed(normal).Normalized().Scaled(dist)
        dummyPntB = centerOfMass_pnts(pntsB_noDuplicates).Translated(translateVecB)
        pntsB_noDuplicates.append(dummyPntB)

    # center of Mass
    cenA = centerOfMass_pnts(pntsA_noDuplicates)
    cenB = centerOfMass_pnts(pntsB_noDuplicates)

    # translation vector from centerOfMass to origin
    mvVecA2O = gp_Vec(cenA, gp_Pnt(0, 0, 0))
    mvVecB2O = gp_Vec(cenB, gp_Pnt(0, 0, 0))

    # translate all points, with rigid movement to move solid's center of Mass to origin
    newPntsA = [i.Translated(mvVecA2O) for i in pntsA_noDuplicates]
    newPntsB = [i.Translated(mvVecB2O) for i in pntsB_noDuplicates]

    if newPntsB[1].X() * newPntsA[1].X() < 0:
        ttmp = newPntsB[0]
        newPntsB[0] = newPntsB[1]
        newPntsB[1] = ttmp

    # matching 2 points set by SVD, transform from B to A
    npMatA = [gpVec2npMat((gp_Vec(gp_Pnt(0, 0, 0), i))) for i in newPntsA]
    npMatB = [gpVec2npMat(gp_Vec(gp_Pnt(0, 0, 0), i)) for i in newPntsB]
    varMat = np.asmatrix(np.zeros((3, 3)))
    for i in range(0, len(npMatA)):
        varMat += npMatB[i] * npMatA[i].transpose()

    linAlg = np.linalg
    U, s, Vh = linAlg.svd(varMat, full_matrices=True)
    R = Vh.transpose() * U.transpose()

    # if R has determinant -1, then R is a rotation plus a reflection
    if linAlg.det(R) < 0:
        # [ToDo] Don't know why third column or row should multiply -1
        reverseMat = np.matrix([(1, 0, 0),
                                (0, 1, 0),
                                (0, 0, -1)])
        rospy.loginfo('det(R) is < 0, change the sign of last column of Vh')
        R = reverseMat * (Vh.transpose()) * U.transpose()

    # q = quaternion_from_matrix(R)
    R = np.array(R)
    RgpMat = gp_Mat(R[0][0], R[0][1], R[0][2],
                    R[1][0], R[1][1], R[1][2],
                    R[2][0], R[2][1], R[2][2])
    # RgpMat = gp_Mat(gp_XYZ(R[0][0], R[1][0], R[2][0]), gp_XYZ(R[0][1], R[1][1], R[2][1]), gp_XYZ(R[0][2], R[1][2], R[2][2]))
    q = gp_Quaternion(RgpMat)
    # gp_Extrinsic_XYZ = 2
    # q.GetEulerAngles(2)
    # qq = gp_Quaternion()
    # qq.SetVectorAndAngle(gp_Vec(gp_Dir(gp_XYZ(0,0,1))), q.GetEulerAngles(2)[2])

    trsf = gp_Trsf()
    trsf.SetTranslation(mvVecB2O)
    toploc = TopLoc_Location(trsf)
    shp.Move(toploc)

    trsf = gp_Trsf()
    trsf.SetRotation(q)
    toploc = TopLoc_Location(trsf)
    shp.Move(toploc)

    trsf = gp_Trsf()
    trsf.SetTranslation(mvVecA2O.Reversed())
    toploc = TopLoc_Location(trsf)
    shp.Move(toploc)


def match_singleHole(solid_add, solid_base):

    cyls_sel_base, cyls_sel_add, projPln = selectCylinderFromClosestPln(solid_add, solid_base)
    pntList_base = []
    for i in cyls_sel_base:
        loc = BRepAdaptor_Surface(i).Cylinder().Location()
        pnt = gp_Pnt(loc.X(), loc.Y(), loc.Z())
        pntList_base.append(pnt)

    pntList_add = []
    for i in cyls_sel_add:
        loc = BRepAdaptor_Surface(i).Cylinder().Location()
        pnt = gp_Pnt(loc.X(), loc.Y(), loc.Z())
        pntList_add.append(pnt)

    pntList_base_proj = [ais_ProjectPointOnPlane(pnt, projPln) for pnt in pntList_base]
    pntList_add_proj = [ais_ProjectPointOnPlane(pnt, projPln) for pnt in pntList_add]

    mvVec = get_shortest_vec(pntList_add_proj, pntList_base_proj)
    trsf = gp_Trsf()
    trsf.SetTranslation(mvVec)
    solid_add.Move(TopLoc_Location(trsf))


def match_multiHoles(solid_add, solid_base):
    cyls_sel_base, cyls_sel_add, projPln = selectCylinderFromClosestPln(solid_add, solid_base)

    # [ToDo] select cylinders from plane
    # matched_hole_pairs = get_holes_pairs(sel_list1, sel_list2, projPln)
    matched_hole_pairs = get_holes_pairs(cyls_sel_base, cyls_sel_add, projPln)
    if len(matched_hole_pairs) != 0:
        align_mutiHoles(solid_add, matched_hole_pairs)


def autoHoleAlign(solid_add, solid_base):
    """Automatic align holes

    Arguments:
        solid_add {TopoDS_Shape} -- [description]
        solid_base {TopoDS_Shape} -- [description]

    Returns:
        {string} -- "succeed" means there are more than 2 holes can be used on alignment, which restrict 5 DOF on added solid
                    "half"    means there is only 1 hole can be used on alignment, which restrict 4 DOF on added solid
                    "failed"  means there is no hole can be used on alignment, which restrict 0 DOF
    """
    holeAlignResult = "failed"
    holePair = getNHoleCouldBeMatched(solid_add, solid_base, angTolDeg=5.0)
    if holePair > 2:
        print("Multihole match")
        match_multiHoles(solid_add, solid_base)
        holeAlignResult = "succeed"
    elif holePair == 2:
        print("2_hole match")
        match_multiHoles(solid_add, solid_base)
        holeAlignResult = "succeed"
    elif holePair == 1:
        print("singel hole match")
        match_singleHole(solid_add, solid_base)
        holeAlignResult = "half"
    elif holePair == 0:
        rospy.logwarn("No hole pair found")
        holeAlignResult = "failed"
    rospy.logdebug("Result of Hole alignment: %s" % (holeAlignResult))
    return holeAlignResult


def getParallelDirPair(dirList_base, dirList_add, angTolDeg=5.0):
    commonList = []
    j = 0
    while len(dirList_base) > 0:
        i = dirList_base.pop(0)
        dir1 = gp_Dir(i[0], i[1], i[2])

        while j < len(dirList_add):
            dir2 = gp_Dir(dirList_add[j][0], dirList_add[j][1], dirList_add[j][2])
            j += 1
            if dir1.IsParallel(dir2, radians(angTolDeg)):
                j -= 1
                commonList.append((i, dirList_add[j]))
                dirList_add.pop(j)
                pass
        j = 0
    return commonList


def getNHoleCouldBeMatched(solid_add, solid_base, angTolDeg=5.0):
    """Get number of holes could be matched. This number is used as a flag for determining which hole matching algorithm is used.

    Arguments:
        solid_add {TopoDS_Solid} -- Both solides will not be moved
        solid_base {TopoDS_Solid} -- [description]

    Returns:
        {int} -- Maximal number of holes could be matched
    """

    cylinders_base = RecognizeTopo(solid_base).cylinders()
    cylinders_add = RecognizeTopo(solid_add).cylinders()
    holeAxGrp_base = group_cylinders_byAxisDir(cylinders_base, groupParallelAx=True)
    holeAxGrp_add = group_cylinders_byAxisDir(cylinders_add, groupParallelAx=True)

    holeAxList_base = list(holeAxGrp_base.keys())
    holeAxList_add = list(holeAxGrp_add.keys())

    commonList = getParallelDirPair(holeAxList_base, holeAxList_add, angTolDeg=angTolDeg)

    tfValue = []
    for i in commonList:
        n_incoaxial_full_cylinder_base = len(__group_coaxial_cylinders(holeAxGrp_base[i[0]]))
        n_incoaxial_full_cylinder_add = len(__group_coaxial_cylinders(holeAxGrp_add[i[1]]))
        tfValue.append(min(n_incoaxial_full_cylinder_base, n_incoaxial_full_cylinder_add))

    if tfValue == []:
        print("[WARN] No hole alignment possible")
        return 0
    else:
        return max(tfValue)


def group_cyl_byPln(solid, distTol=0.5, angTolDeg=5.0):
    cylinders = RecognizeTopo(solid).cylinders()
    cyl_dirGrp = group_cylinders_byAxisDir(cylinders, anglTolDeg=angTolDeg, groupParallelAx=True)
    cyl_dirGrpKeys = list(cyl_dirGrp.keys())

    # group cylinders by plane
    CylinderGrpsFromDiffPln = {}
    j = 0
    for dirKey in cyl_dirGrpKeys:
        CylinderGrpsFromDiffPln[dirKey] = []
        parallel_cyl_grps = cyl_dirGrp[dirKey].copy()
        grp_cylinderOnSamePln = {}
        while len(parallel_cyl_grps) >= 1:
            # take out the first element for creating plane, and search the other cylinders on the same plane
            firstCylinder = parallel_cyl_grps.pop(0)
            # fitting a geomety surface to TopoDS_Surface
            brepCylinder = BRepAdaptor_Surface(firstCylinder).Cylinder()
            dir1 = brepCylinder.Axis().Direction()
            # location of cylinder is usually the location of local coordinate system, which is on the axis of cylinder
            loc1 = brepCylinder.Location()
            # create gp_Pln
            pln = gp_Pln(loc1, dir1)
            grp_cylinderOnSamePln[pln] = [firstCylinder]
            # Search the cylinders on the same pln, if yes, extract from parallel_cyl_grps
            # loop all elements in parallel_cyl_grps
            while j < len(parallel_cyl_grps):
                brepCylinder2 = BRepAdaptor_Surface(parallel_cyl_grps[j]).Cylinder()
                loc2 = brepCylinder2.Location()
                if pln.Distance(gp_Pnt(loc2.X(), loc2.Y(), loc2.Z())) < distTol:
                    grp_cylinderOnSamePln[pln].append(parallel_cyl_grps[j])
                    parallel_cyl_grps.pop(j)
                else:
                    j += 1
            j = 0
        CylinderGrpsFromDiffPln[dirKey] = grp_cylinderOnSamePln
    return CylinderGrpsFromDiffPln


def selectCylinderFromClosestPln(solid_add, solid_base, roundingDigit=6):
    # selecting closest plane as the projectPln

    cylGrp_byPln_base = group_cyl_byPln(solid_base)
    cylGrp_byPln_add = group_cyl_byPln(solid_add)

    parallelDirPairList = getParallelDirPair(list(cylGrp_byPln_base.keys()), list(cylGrp_byPln_add.keys()))

    plnPairWithDist = {}
    for parallelDirPair in parallelDirPairList:

        plnList_base = list(cylGrp_byPln_base[parallelDirPair[0]].keys())
        plnList_add = list(cylGrp_byPln_add[parallelDirPair[1]].keys())
        for pln_base in plnList_base:
            for pln_add in plnList_add:
                dist = round(pln_base.Distance(pln_add.Location()), roundingDigit)
                if dist not in plnPairWithDist.keys():
                    # here we use point on pln_add instead of pln_add to prevent error from slightly unparallel plane
                    # for a better estimation of distance, a point nearby center of shape must be used
                    plnPairWithDist[dist] = [(parallelDirPair, (pln_base, pln_add))]
                else:
                    plnPairWithDist[dist].append((parallelDirPair, (pln_base, pln_add)))
    distList = list(plnPairWithDist.keys())
    distList.sort()
    closestPlnPairs = plnPairWithDist[distList[0]]

    if len(closestPlnPairs) > 1:
        logging.warning("There are more than one closest plane pair, the pair with the direction close to Z axis is selected")
        zAx = gp_Dir(0, 0, 1)
        min_ang = radians(180)
        for i in range(0, len(closestPlnPairs)):
            ang = zAx.Angle(tuple2gpDir(closestPlnPairs[i][0][0]))
            ang = min(ang, radians(180) - ang)
            if min_ang > ang:
                min_ang = ang
                closestPlnPair = closestPlnPairs[i]
    else:
        closestPlnPair = closestPlnPairs[0]

    projPln = closestPlnPair[1][1]
    sel_cyls_base = cylGrp_byPln_base[closestPlnPair[0][0]][closestPlnPair[1][0]]
    sel_cyls_add = cylGrp_byPln_add[closestPlnPair[0][1]][closestPlnPair[1][1]]
    return [sel_cyls_base, sel_cyls_add, projPln]


def __test_multiHoleMatching():
    fileList = ['lf064-01.stp', 'lf064-0102_1.stp', 'holes_match_default_2.stp']
    shapeFromModel = read_step_file(os.path.join('..', 'models', fileList[-1]))
    shp_topo = RecognizeTopo(shapeFromModel)

    solid_base = shp_topo.solids[0]
    solid_add = shp_topo.solids[1]

    match_multiHoles(solid_add, solid_base)
    # explorer = BRepClass3d_SolidExplorer(solid1)

    frame = Display(solid_base, run_display=True)
    frame.add_shape(solid_add)
    frame.open()


def __test_singleHoleMatching():
    fileList = ['lf064-01.stp', 'lf064-0102_1.stp', 'holes_match_default.stp']
    shapeFromModel = read_step_file(os.path.join('..', 'models', fileList[1]))
    shp_topo = RecognizeTopo(shapeFromModel)

    solid_base = shp_topo.solids[0]
    solid_add = shp_topo.solids[1]

    match_singleHole(solid_add, solid_base)

    frame = Display(solid_base, run_display=True)
    frame.add_shape(solid_add)
    frame.open()


if __name__ == '__main__':
    # __test_singleHoleMatching()
    __test_multiHoleMatching()

    ipdb.set_trace(context=10)
