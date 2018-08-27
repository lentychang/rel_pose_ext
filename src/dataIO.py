#!/usr/bin/python
# -*- coding: <utf-8> -*-

from OCC.STEPControl import STEPControl_Reader, STEPControl_Writer, STEPControl_AsIs
from OCC.Interface import Interface_Static_SetCVal
from OCC.IFSelect import IFSelect_RetDone, IFSelect_ItemsByEntity
import os
import os.path
import sys
import logging
import numpy as np
import ipdb
from OCC.BRepAdaptor import BRepAdaptor_Surface, BRepAdaptor_Curve
from OCC.gp import gp_Dir
from OCC.GeomAbs import GeomAbs_Plane, GeomAbs_Cylinder, GeomAbs_Cone, GeomAbs_Sphere,\
                        GeomAbs_Torus, GeomAbs_BezierSurface, GeomAbs_BSplineSurface,\
                        GeomAbs_SurfaceOfRevolution, GeomAbs_SurfaceOfExtrusion,\
                        GeomAbs_OffsetSurface, GeomAbs_OtherSurface,\
                        GeomAbs_Line, GeomAbs_Circle, GeomAbs_Ellipse,\
                        GeomAbs_Hyperbola, GeomAbs_Parabola, GeomAbs_BezierCurve,\
                        GeomAbs_BSplineCurve, GeomAbs_OtherCurve
from core_topology_traverse import Topo
from OCC.TopoDS import TopoDS_Builder, TopoDS_CompSolid
from OCC.Display.SimpleGui import init_display


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
    status = step_reader.ReadFile(filename)

    if status == IFSelect_RetDone:  # check status
        failsonly = False
        step_reader.PrintCheckLoad(failsonly, IFSelect_ItemsByEntity)
        step_reader.PrintCheckTransfer(failsonly, IFSelect_ItemsByEntity)
        step_reader.TransferRoot(1)
        a_shape = step_reader.Shape(1)
    else:
        print('Current Path:', os.getcwd())
        print("Error: can't read file.")
        sys.exit(0)
    return a_shape


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
    def __init__(self, shp, run_display=True, *args, **kwargs):
        self.shape = shp
        self.shape_list = [shp]
        self.shape_selected = shp
        self.selectMode = 'Edge'
        self.callbackIsRegistered = False
        self.open(run_display=run_display)

    def open(self, shape=None, run_display=True):
        # Todo: display shape from shape_list
        self.callbackIsRegistered = False
        if run_display:
            if shape is not None:
                self.shape = shape
            self.display, self.start_display, self.add_menu, self.add_function_to_menu = init_display()
            self.__show_all()
            self.add_menu('Selection Mode')
            self.add_menu('Show')
            self.add_function_to_menu('Selection Mode', self.edge_select_mode)
            self.add_function_to_menu('Selection Mode', self.face_select_mode)
            self.add_function_to_menu('Show', self.selected_shape_info)
            self.start_display()

    def __show_all(self):
        self.display.DisplayShape(self.shape_list[0], update=False)
        for i in range(1, len(self.shape_list)):
            self.display.DisplayShape(self.shape_list[i], update=True)

    def add_shape(self, shp):
        self.shape_list.append(shp)
        self.display.DisplayShape(shp, update=True)

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


if __name__ == '__main__':
    logging.basicConfig(filename="logging.txt", filemode='w',
                        level=logging.warning)
    fileList = ['lf064-01.stp', 'cylinder_group_test.stp', 'cylinder_cut.stp',
                'cylinder_cut2.stp', 'face_recognition_sample_part.stp',
                'cylinder_with_side_hole.stp', 'cylinder_with_side_slot.stp',
                'cylinder_with_slot.stp', 'cylinders.stp']
    shapeFromModel = read_step_file(os.path.join('..', 'models', fileList[0]))
    showShapes = shapeFromModel

    frame = Display(showShapes, run_display=True)
    print('\n\ntype "frame.open()" to reopen the frame !!!')
    ipdb.set_trace()

