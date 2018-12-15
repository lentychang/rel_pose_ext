#!/usr/bin/python3
# -*- coding: <utf-8> -*-

import logging
import os
import os.path
import sys

import ipdb
import numpy as np
import pyassimp
import rospy
from OCC.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface
from OCC.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Display.SimpleGui import init_display
from OCC.GeomAbs import (GeomAbs_BezierCurve, GeomAbs_BezierSurface,
                         GeomAbs_BSplineCurve, GeomAbs_BSplineSurface,
                         GeomAbs_Circle, GeomAbs_Cone, GeomAbs_Cylinder,
                         GeomAbs_Ellipse, GeomAbs_Hyperbola, GeomAbs_Line,
                         GeomAbs_OffsetSurface, GeomAbs_OtherCurve,
                         GeomAbs_OtherSurface, GeomAbs_Parabola, GeomAbs_Plane,
                         GeomAbs_Sphere, GeomAbs_SurfaceOfExtrusion,
                         GeomAbs_SurfaceOfRevolution, GeomAbs_Torus)
from OCC.gp import gp_Dir, gp_Quaternion, gp_Trsf, gp_Vec
from OCC.IFSelect import IFSelect_RetDone
from OCC.Interface import Interface_Static_SetCVal
from OCC.STEPControl import (STEPControl_AsIs, STEPControl_Reader,
                             STEPControl_Writer)
from OCC.StlAPI import StlAPI_Writer
from OCC.TopLoc import TopLoc_Location
from OCC.TopoDS import TopoDS_Builder, TopoDS_CompSolid
from open3d import read_point_cloud, write_point_cloud

from core_topology_traverse import Topo
from topo2 import RecognizeTopo


def stp2pcd(stpName, modelDir):
    baseName = stpName.split('.')[0]
    stlName = baseName + '.stl'
    plyName = baseName + '.ply'
    pcdName = baseName + '.pcd'
    stp2stl(filename=stpName, fileIODir=modelDir, linDeflection=0.1, solidOnly=True)
    stl2ply(filename=stlName, fileIODir=modelDir)
    print("Load a ply point cloud, print it, and render it")
    pcd = read_point_cloud(os.path.join(modelDir, plyName))
    # print(pcd)
    # print(np.asarray(pcd.points))
    # draw_geometries([pcd])
    os.remove(os.path.join(modelDir, stlName))
    os.remove(os.path.join(modelDir, plyName))
    write_point_cloud(os.path.join(modelDir, pcdName), pcd)


def stp2ply(stpName, modelDir):
    baseName = stpName.split('.')[0]
    stlName = baseName + '_mm.stl'
    # plyName = baseName + '_mm.ply'
    stp2stl(filename=stpName, fileIODir=modelDir, linDeflection=0.1, solidOnly=True)
    stl2ply(filename=stlName, fileIODir=modelDir)
    os.remove(os.path.join(modelDir, stlName))


def stp2stl(filename, fileIODir, linDeflection=0.1, angDeflection=0.1, solidOnly=True):
    # make sure the path exists otherwise OCE get confused
    assert os.path.isdir(fileIODir)

    nameBase = filename.split('.')[0]

    stpName = os.path.abspath(os.path.join(fileIODir, nameBase + '.stp'))
    modelShp = read_step_file(stpName)
    if solidOnly:
        solids = list(Topo(modelShp).solids())
        modelShp = solids[0]
    mesh = BRepMesh_IncrementalMesh(modelShp, linDeflection)
    mesh.Perform()
    assert mesh.IsDone()

    # set the directory where to output the
    stlName = os.path.abspath(os.path.join(fileIODir, nameBase + '_mm.stl'))

    stl_exporter = StlAPI_Writer()
    stl_exporter.SetASCIIMode(True)  # change to False if you need binary export
    stl_exporter.Write(modelShp, stlName)
    # make sure the program was created
    assert os.path.isfile(stlName)


def stl2ply(filename, fileIODir):
    stlName = os.path.abspath(os.path.join(fileIODir, filename))
    scene = pyassimp.load(stlName, file_type='stl', processing=pyassimp.postprocess.aiProcess_Triangulate)
    plyName = stlName.split('.')[0] + '.ply'
    pyassimp.export(scene, plyName, file_type='ply')


def deg2PlusMinusPi(deg):
    deg = deg % 360.0
    if abs(deg) > 180.0:
        deg += -1 * np.sign(deg) * 360.0
    return deg


def tuple2gpDir(tupl):
    if len(tupl) != 3:
        print('Error! length must be 3 in order to map to xyz')
    return gp_Dir(tupl[0], tupl[1], tupl[2])


def extractDictByVal(dictionary, valueList):
    # copy a new dictionary to prevent destroy the original one
    new_dict = dict(dictionary)
    for key, val in dictionary.items():
        if val not in valueList:
            del new_dict[key]
    return new_dict


def read_step_file(filename):
    """ read the STEP file and returns a compound
    """
    step_reader = STEPControl_Reader()
    rospy.loginfo("### Read Step File ###")
    status = step_reader.ReadFile(filename)
    if status == IFSelect_RetDone:  # check status
        # failsonly = True
        # step_reader.PrintCheckLoad(failsonly, IFSelect_ItemsByEntity)
        # step_reader.PrintCheckTransfer(failsonly, IFSelect_ItemsByEntity)
        step_reader.TransferRoot(1)
        a_shape = step_reader.Shape(1)
    else:
        rospy.logdebug('Current Path:', os.getcwd())
        rospy.logerr("Error: can't read file.")
        sys.exit(0)
    return a_shape


def read_stp_as_solid(filename):
    shape = read_step_file(filename)
    solid = RecognizeTopo(shape).solids[0]

    return solid


def read_stp_solid_withTf(modelDir, stpFilename, vecXYZlist, quaternionXYZWlist, unitIsMM=False):

    solid = read_stp_as_solid(os.path.join(modelDir, stpFilename + ".stp"))
    trsf = gp_Trsf()
    q = gp_Quaternion()
    if not unitIsMM:
        vecXYZlist = [i * 1000.0 for i in vecXYZlist]
    q.Set(quaternionXYZWlist[0], quaternionXYZWlist[1], quaternionXYZWlist[2], quaternionXYZWlist[3])
    trsf.SetTransformation(q, gp_Vec(vecXYZlist[0], vecXYZlist[1], vecXYZlist[2]))
    toploc = TopLoc_Location(trsf)
    solid.Move(toploc)
    return solid


def getTfFromSolid(solid, outputUnitIsMM=False):
    gpVec = solid.Location().Transformation().TranslationPart()
    gpQuaternion = solid.Location().Transformation().GetRotation()
    vecList = [gpVec.X(), gpVec.Y(), gpVec.Z()]
    qList = [gpQuaternion.X(), gpQuaternion.Y(), gpQuaternion.Z(), gpQuaternion.W()]
    if not outputUnitIsMM:
        vecList = [i / 1000.0 for i in vecList]

    return [vecList, qList]


def write_step_file(shape, filename, step_ver="AP214"):
    step_writer = STEPControl_Writer()
    Interface_Static_SetCVal("write.step.schema", step_ver)
    step_writer.Transfer(shape, STEPControl_AsIs)
    status = step_writer.Write(filename)

    assert(status == IFSelect_RetDone)


def solid_comp(solidList):
    aRes = TopoDS_CompSolid()
    builder = TopoDS_Builder()
    builder.MakeCompSolid(aRes)
    # transfer shapes and write file
    for shp in solidList:
        builder.Add(aRes, shp)
    return aRes


class Display():
    def __init__(self, shp=None, run_display=True, *args, **kwargs):
        self.shape_list = []
        if shp is not None:
            self.shape_list.append(shp)
            self.shape_selected = shp

        self.selectMode = 'Edge'
        self.callbackIsRegistered = False
        self.open(run_display=run_display)

    def open(self, run_display=True):
        self.callbackIsRegistered = False
        if run_display:
            self.display, self.start_display, self.add_menu, self.add_function_to_menu = init_display()
            self.__show_all()
            self.add_menu('Selection Mode')
            self.add_menu('Show')
            self.add_function_to_menu('Selection Mode', self.edge_select_mode)
            self.add_function_to_menu('Selection Mode', self.face_select_mode)
            self.add_function_to_menu('Show', self.selected_shape_info)
            self.start_display()

    def __show_all(self):
        if len(self.shape_list) >= 1:
            self.display.DisplayShape(self.shape_list[0], update=False)
            for i in range(1, len(self.shape_list)):
                self.display.DisplayShape(self.shape_list[i], update=True)
        else:
            print("[WARN] No shapes given, please use .add_shape to add the shape to be display")

    def add_shape(self, shp):
        self.shape_list.append(shp)
        # self.display.DisplayShape(shp, update=True)

    def remove_shape(self, shp):
        if shp in self.shape_list:
            self.shape_list.remove(shp)
            self.display.DisplayShape(self.shape_list[0], update=False)
            for i in range(1, len(self.shape_list)):
                self.display.DisplayShape(self.shape_list[i], update=True)
        else:
            logging.warning('The shape given is not in shown in the display')

    # Todo
    # def get_geom_info(shape):

    def selected_shape_info(self):
        if self.selectMode == 'Face':
            print(self.shape_selected)
            surf = BRepAdaptor_Surface(self.shape_selected, True)
            if surf.GetType() == GeomAbs_Plane:
                gp_pln = surf.Plane()
                normal = gp_pln.Axis().Direction()
                print('plane normal: (%.3f, %.3f, %.3f)' % (normal.X(), normal.Y(), normal.Z()))
            elif surf.GetType() == GeomAbs_Cylinder:
                gp_cyl = surf.Cylinder()
                axis = gp_cyl.Axis().Direction()
                location = gp_cyl.Location()
                print('cylinder axis direction: (%.3f, %.3f, %.3f)' % (axis.X(), axis.Y(), axis.Z()))
                print('cylinder axis location: (%.3f, %.3f, %.3f)' % (location.X(), location.Y(), location.Z()))
            else:
                typeList = ['Plane', 'Cylinder', 'Cone', 'Sphere', 'Torus', 'BezierSurface', 'BSplineSurface', 'SurfaceOfRevolution', 'SurfaceOfExtrusion', 'OffsetSurface', 'OtherSurface']
                print('This surface type "%s" is not implemented !!' % typeList[surf.GetType()])

        elif self.selectMode == 'Edge':
            print(self.shape_selected)
            edge = BRepAdaptor_Curve(self.shape_selected)
            curveType = edge.GetType()
            if curveType == GeomAbs_Line:
                gp_lin = edge.Line()
                direction = gp_lin.Direction()
                print('Line direction: (%.3f, %.3f, %.3f)' % (direction.X(), direction.Y(), direction.Z()))
            elif curveType == GeomAbs_Circle:
                gp_circ = edge.Circle()
                center = gp_circ.Location()
                print('Center of circle: (%.3f, %.3f, %.3f)' % (center.X(), center.Y(), center.Z()))
                print('Radius of circle:', gp_circ.Radius())

            else:
                typeList = ['Line', 'Circle', 'Ellipse', 'Parabola', 'BezierCurve', 'BSplineCurve', 'OffsetCurve or OtherCurve?', 'OtherCurve']
                print('This edge type is not implemented !!')
                print('This surface type "%s" is not implemented !!' % typeList[surf.GetType()])

    def edge_select_mode(self):
        print('Edge select mode activated')
        if self.callbackIsRegistered is True and self.selectMode == 'Face':
            self.display.unregister_callback(self.click_face)
            self.callbackIsRegistered = False
        self.selectMode = 'Edge'
        if self.callbackIsRegistered is False:
            self.display.register_select_callback(self.click_edge)
        self.display.SetSelectionModeEdge()
        self.__show_all()
        self.callbackIsRegistered = True

    def click_edge(self, edge_click, *kwargs):
        """ This is the function called every time
        an edge is clicked in the 3d view
        """
        # shp = A list of TopoDS_Shape; type=Face, if click a place without model, it is null
        # kwargs xy coordinate in 2D where mouse is clicked
        print("\nClicked - edge select mode !!")
        print('===============================================')
        for edge in edge_click:  # this should be a TopoDS_Face TODO check it is
            print("Edge selected: ", edge.HashCode(1000000))  # TopoDS_Shape
            shp = Topo(edge)
            self.shape_selected = list(shp.edges())[0]
            self.selected_shape_info()

    def face_select_mode(self):
        print('Face select mode activated')
        if self.callbackIsRegistered is True and self.selectMode == 'Edge':
            self.display.unregister_callback(self.click_edge)
            self.callbackIsRegistered = False
        self.selectMode = 'Face'
        if self.callbackIsRegistered is False:
            self.display.register_select_callback(self.click_face)
        self.display.SetSelectionModeFace()
        self.__show_all()
        self.callbackIsRegistered = True

    def click_face(self, face_click, *kwargs):
        """ This is the function called every time
        a face is clicked in the 3d view
        """
        # shp = A list of TopoDS_Shape; type=Face, if click a place without model, it is null
        # kwargs xy coordinate in 2D where mouse is clicked

        print("\nClicked - face select Mode!!")
        print('===============================================')

        for face in face_click:  # this should be a TopoDS_Face TODO check it is
            print("Face selected: ", face.HashCode(1000000))  # TopoDS_Shape
            shp = Topo(face)
            self.shape_selected = list(shp.faces())[0]
            self.selected_shape_info()


def test():
    logging.basicConfig(filename="logging.txt", filemode='w',
                        level=logging.warning)
    fileList = ['lf064-01.stp']
    shapeFromModel = read_step_file(os.path.join('..', 'models', fileList[0]))
    showShapes = shapeFromModel

    frame = Display(showShapes, run_display=True)
    print('\n\ntype "frame.open()" to reopen the frame !!!')


def __test_stp2ply():
    a = sys.argv[1]
    b = sys.argv[2]
    print("filename:", a)
    b = os.path.abspath(b)
    print("modelDir:", b)

    stp2ply(a, b)


def __test_read_stp_solid_withTf():
    solid1 = read_stp_solid_withTf("lf064-03", [0.01, 0.01, 1], [0, 0, 1, 0], unitIsMM=False)
    solid2 = read_stp_as_solid(os.path.join("/root/catkin_ws/src/rel_pose_ext/models", "lf064-02" + ".stp"))

    frame = Display(solid1, run_display=True)
    frame.add_shape(solid2)

    aa = getTfFromSolid(solid1, outputUnitIsMM=False)
    for i in aa:
        print(i)
    frame.open()


if __name__ == "__main__":
    # __test_stp2ply()
    __test_read_stp_solid_withTf()

    # ipdb.set_trace()
