
from OCC.GeomAbs import GeomAbs_Line, GeomAbs_Circle, GeomAbs_Ellipse, \
                        GeomAbs_Hyperbola, GeomAbs_Parabola, \
                        GeomAbs_BezierCurve, GeomAbs_BSplineCurve, \
                        GeomAbs_OtherCurve
from OCC.GeomAbs import GeomAbs_Plane, GeomAbs_Cylinder, \
                        GeomAbs_SurfaceOfExtrusion, GeomAbs_Cone, \
                        GeomAbs_BSplineSurface, GeomAbs_SurfaceOfRevolution, \
                        GeomAbs_Torus
# from OCC.TopoDS import topods_Face, topods_Edge, TopoDS_Iterator
from OCC.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface
from core_topology_traverse import Topo
# from enum import Enum
import logging
import ipdb

from dataIO import read_step_file, Display
import os.path


class RecognizeTopo():
    def __init__(self, shp, Debug=False):
        """[summary]

        Arguments:
            shp {TopoDS_Shape} -- [description]
        """
        self.__isDebug = Debug
        # Todo: need to change this into TopAbs_XXXX enumeration
        self.__shapeTypeList = ['vertex', 'edge', 'wire', 'face', 'shell',
                                'solid', 'comp_solid', 'compound']
        self.shape = shp
        self.topo = Topo(shp)

        self.faces, self.edges = {}, {}

        self.shape_type = self.__get_shape_type()

        self.edge_types = ['line', 'circle', 'ellipse', 'hyperbola',
                           'parabola', 'bezierCurve', 'bspineCurve', 'else']
        self.face_types = ['plane', 'cylinder', 'cone', 'BslineSurface',
                           'torus', 'surfaceOfExtrusion',
                           'surfaceOfRevolution', 'else']

        shape_idx = self.__shapeTypeList.index(self.shape_type)
        if shape_idx >= self.__shapeTypeList.index('solid'):
            self.solids = list(self.topo.solids())
        if shape_idx >= self.__shapeTypeList.index('face'):
            self.faces = self.__extract_faceType_all()
        if shape_idx >= self.__shapeTypeList.index('edge'):
            self.edges = self.__extract_edgeType_all()
        '''
        else:
            print('other topods type not yet implemented')
        '''

    def planes(self):
        return self.faces['plane']

    def cylinders(self):
        return self.faces['cylinder']

    def surfOfExt(self):
        return self.faces['surfaceOfExtrusion']

    def surfOfRev(self):
        return self.faces['surfaceOfExtrusion']

    def lines(self):
        return self.edges['line']

    def circles(self):
        return self.edges['circle']

    def ellipses(self):
        return self.edges['ellipse']

    def __get_shape_type(self):
        element_list = [self.topo.vertices(),
                        self.topo.edges(),
                        self.topo.wires(),
                        self.topo.faces(),
                        self.topo.shells(),
                        self.topo.solids(),
                        self.topo.comp_solids(),
                        self.topo.compounds()]
        level = 0
        listLen = [len(list(element_list[0]))]
        for i in range(1, len(element_list)):
            numberOfShapes = len(list(element_list[i]))
            if self.__isDebug:
                listLen.append(numberOfShapes)
            if numberOfShapes >= 1:
                level = level + 1
        print('level = ', level)

        if self.__isDebug:
            for k in range(0, len(element_list)-1):
                if listLen[k] == 0 and listLen[k+1] != 0:
                    logging.warning('[Error] Wrong hierachy about the shapes, \
                Ex. compounds in not alway on higher level than comp_solids?')

        for i in range(0, len(self.__shapeTypeList)):
            if level == i:
                shape_type = self.__shapeTypeList[i]
        if level >= len(self.__shapeTypeList):
            logging.warning('Error, some logic of this method is wrong')
            shape_type = ''
        return shape_type

    def __extract_faceType(self, a_face):
        surf = BRepAdaptor_Surface(a_face, True)
        surf_type = surf.GetType()
        if surf_type == GeomAbs_Plane:
            return {'plane': a_face}
        elif surf_type == GeomAbs_Cylinder:
            return {'cylinder': a_face}
        elif surf_type == GeomAbs_Cone:
            return {'cone': a_face}
        elif surf_type == GeomAbs_Torus:
            return {'torus': a_face}
        elif surf_type == GeomAbs_SurfaceOfExtrusion:
            return {'surfaceOfExtrusion': a_face}
        elif surf_type == GeomAbs_SurfaceOfRevolution:
            return {'surfaceOfRevolution': a_face}
        elif surf_type == GeomAbs_BSplineSurface:
            return {'BSplineSurface': a_face}
        else:
            print('face type not yet implemented !!!!!!')
            return {'else': a_face}

    def __extract_faceType_all(self):
        faces = {'plane': [], 'cylinder': [], 'else': []}

        for face in self.topo.faces():
            a_face = self.__extract_faceType(face)
            key = list(a_face.keys())[0]
            if key in self.face_types:
                if key in faces:
                    faces[key].append(a_face[key])
                else:
                    faces[key] = [a_face[key]]
            else:
                print('keys from this value is not defined in this class!!')
        return faces

    def __extract_edgeType(self, an_edge):
        edge = BRepAdaptor_Curve(an_edge)
        edge_type = edge.GetType()
        if edge_type == GeomAbs_Line:
            return {'line': an_edge}
        elif edge_type == GeomAbs_Circle:
            return {'circle': an_edge}
        elif edge_type == GeomAbs_Ellipse:
            return {'ellipse': an_edge}
        elif edge_type == GeomAbs_Parabola:
            return {'parabola': an_edge}
        elif edge_type == GeomAbs_Hyperbola:
            return {'hyperbola': an_edge}
        elif edge_type == GeomAbs_BezierCurve:
            return {'bezierCurve': an_edge}
        elif edge_type == GeomAbs_BSplineCurve:
            return {'bsplineCurve': an_edge}
        elif edge_type == GeomAbs_OtherCurve:
            return {'otherCurve': an_edge}
        else:
            print('edge type is not able to be recognized !!!!!!')
            return {'else': an_edge}

    def __extract_edgeType_all(self):
        edges = {'line': [], 'circle': [], 'ellipse': [], 'hyperbola': [],
                 'parabola': [], 'bezierCurve': [], 'bspineCurve': [],
                 'else': []}
        # loop over edges only
        for edg in self.topo.edges():
            # call the recognition function
            an_edge = self.__extract_edgeType(edg)
            for key in self.edge_types:
                if key in an_edge:
                    edges[key].append(an_edge[key])
        return edges


if __name__ == '__main__':
    logging.basicConfig(filename="logging.txt", filemode='w', level=logging.warning)

    fileList = ['lf064-01.stp',
                'cylinder_group_test.stp',
                'cylinder_cut.stp',
                'cylinder_cut2.stp',
                'face_recognition_sample_part.stp',
                'cylinder_with_side_hole.stp',
                'cylinder_with_side_slot.stp',
                'cylinder_with_slot.stp',
                'cylinders.stp',
                'compound_solid_face-no-contact.stp']
    shapeFromModel = read_step_file(os.path.join('..', 'models', fileList[9]))

    test = RecognizeTopo(shapeFromModel)
    # gui = Display(shapeFromModel, run_display=True)    

