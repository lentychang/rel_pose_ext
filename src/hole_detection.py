

from OCC.STEPControl import STEPControl_Reader
from OCC.IFSelect import IFSelect_RetDone, IFSelect_ItemsByEntity
from OCC.GeomAbs import GeomAbs_Line, GeomAbs_Circle, GeomAbs_Ellipse, GeomAbs_Hyperbola, GeomAbs_Parabola, GeomAbs_BezierCurve, GeomAbs_BSplineCurve, GeomAbs_OtherCurve
from OCC.GeomAbs import GeomAbs_Plane, GeomAbs_Cylinder, GeomAbs_SurfaceOfExtrusion, GeomAbs_Cone, GeomAbs_BSplineSurface, GeomAbs_SurfaceOfRevolution, GeomAbs_Torus
from OCC.TopoDS import topods_Face, topods_Edge, TopoDS_Iterator
from OCC.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface
from OCC.Display.SimpleGui import init_display

import os.path
import sys

from math import pi, radians, degrees
import ipdb
# from core_topology_traverse import Topo
from core_topology_traverse import *
import logging


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
        print("Error: can't read file.")
        sys.exit(0)
    return a_shape


def extract_faceType(a_face):
    surf = BRepAdaptor_Surface(a_face, True)
    surf_type = surf.GetType()
    if surf_type == GeomAbs_Plane:
        gp_pln = surf.Plane()
        return {'plane': [a_face, gp_pln]}
    elif surf_type == GeomAbs_Cylinder:
        gp_cyl = surf.Cylinder()
        return {'cylinder': [a_face, gp_cyl]}
    else:
        print('face type not yet implemented !!!!!!')
        return {'else': [a_face, surf]}


def extract_faceType_all(shape):
    shp = Topo(shape)    
    face_types = ['plane', 'cylinder', 'else']
    faces = {'plane': [], 'cylinder': [], 'else':[]}

    for face in shp.faces():
        a_face = extract_faceType(face)
        for key in face_types:
            if key in a_face:
                faces[key].append(a_face[key])
    return faces


def get_cylinders(shape):
    faces = extract_faceType_all(shape)
    cylinder_pair_list = faces['cylinder']
    return cylinder_pair_list


def extract_edgeType(an_edge):
    edge = BRepAdaptor_Curve(an_edge)
    edge_type = edge.GetType()
    if edge_type == GeomAbs_Line:
        gp_lin = edge.Line()
        return {'line': [an_edge, gp_lin]}
    elif edge_type == GeomAbs_Circle:
        gp_circ = edge.Circle()
        return {'circle': [an_edge, gp_circ]}
    elif edge_type == GeomAbs_Ellipse:
        gp_elips = edge.Ellipse()
        return {'ellipse': [an_edge, gp_elips]}
    elif edge_type == GeomAbs_Parabola:
        gp_parab = edge.Parabola()
        return {'parabola': [an_edge, gp_parab]}
    elif edge_type == GeomAbs_Hyperbola:
        gp_hypr = edge.Hyperbola()
        return {'hyperbola': [an_edge, gp_hypr]}    
    elif edge_type == GeomAbs_BezierCurve:
        # gp_parab = edge.Parabola()
        return {'bezierCurve': [an_edge, edge]}
    elif edge_type == GeomAbs_BSplineCurve:
        # gp_parab = edge.Parabola()
        return {'bsplineCurve': [an_edge, edge]}  
    elif edge_type == GeomAbs_OtherCurve:
        # gp_parab = edge.Parabola()
        return {'otherCurve': [an_edge, edge]}              
    else:
        print('edge type is not able to be recognized !!!!!!')
        return {'else': [an_edge, edge]}  


def extract_edgeType_all(shape):
    shp = Topo(shape)
    edge_types = ['line', 'circle', 'ellipse', 'hyperbola', 'parabola', 'bezierCurve', 'bspineCurve', 'else']
    edges = {'line': [], 'circle': [], 'ellipse': [], 'hyperbola': [], 'parabola': [], 'bezierCurve': [], 'bspineCurve': [], 'else': []}

    # loop over edges only
    #ipdb.set_trace()
    for edg in shp.edges():
        # call the recognition function
        an_edge = extract_edgeType(edg)
        for key in edge_types:
            if key in an_edge:
                edges[key].append(an_edge[key])
    return edges


def get_lines(shape):
    edges = extract_edgeType_all(shape)
    line_pair_list = edges['line']
    return line_pair_list


def get_circles(shape):
    edges = extract_edgeType_all(shape)
    circle_pair_list = edges['circle']
    return circle_pair_list


def cylinder_group_coaxial(cylinders, tol_ang = 0.5, tol_lin = 0.1):
    """According to cylinders' axis, categorize a list of cylinders into a dictionary by using its axis as a key.

    Arguments:
        cylinders {[[DopoDS_Face, gp_cyl],...]} -- each element [DopoDS_Face, gp_cyl] in the list contains both topology and geometry shapeType.
        tol_ang {float} -- [Unit - degree] the angle between these two axis below this value will be recognize as two parallel axis
        tol_lin

    Returns:
        {'string': [DopoDS_Face, gp_cyl]} -- [description]
    """
    logging.debug('Entering group_coaxial')   
    tol_rad = radians(tol_ang)
    skipList = []
    cyl_ax_grp = {}
    for i in range(0,len(cylinders)):
        if i in skipList:
            continue

        axis1 = cylinders[i][1].Axis()
        location1 = cylinders[i][1].Location()
        key = 'Axis=%.3f,%.3f,%.3f,Loc=%.3f,%.3f,%.3f' %(axis1.Direction().X(), 
                                                axis1.Direction().Y(),
                                                axis1.Direction().Z(),
                                                location1.X(), 
                                                location1.Y(), 
                                                location1.Z())
        
        if key not in cyl_ax_grp:
            cyl_ax_grp[key] = [cylinders[i]]
        else:
            logging.warning('Error !!! Please check the logic again !')
        
        for j in range(i+1,len(cylinders)):
            logging.debug('i = %d, j = %d' %(i,j))

            if j in skipList:
                logging.debug('skip !!')
                continue

            axis2 = cylinders[j][1].Axis()
            if axis1.IsCoaxial(axis2, tol_rad, tol_lin):
                logging.debug('Coaxial !!')
                cyl_ax_grp[key].append(cylinders[j])
                skipList.append(j)
    return cyl_ax_grp


def cylinder_group_radius(cyl_ax_grp, tol = 0.1):
    logging.debug('Entering group_radius')

    keyList = list(cyl_ax_grp.keys())    
    cyl_rad_grp = {}
    for k in keyList:
        cylinders = cyl_ax_grp[k]
        skipList = []
        for i in range(0,len(cylinders)):
            if i in skipList:
                continue
            radius1 = cylinders[i][1].Radius()
            key = '%s,r=%.3f' %(k,radius1)
            
            if key not in cyl_rad_grp:
                cyl_rad_grp[key] = [cylinders[i]]
            else:
                logging.warning('Error !!! Please check the logic again !')
            
            for j in range(i+1,len(cylinders)):
                logging.debug('i = %d, j = %d' %(i,j))
                if j in skipList:
                    continue

                radius2 = cylinders[j][1].Radius()
                if abs(radius1-radius2) <= tol:
                    logging.debug('Same radius found')
                    cyl_rad_grp[key].append(cylinders[j])
                    skipList.append(j)
    return cyl_rad_grp


def select_edge():
    global display
    global shapeFromModel
    display.unregister_callback(click_face)
    display.SetSelectionModeEdge()
    display.DisplayShape(shapeFromModel, update=True)
    display.register_select_callback(click_edge)


def click_edge(edg_local, *kwargs):
    """ This is the function called every time
    an edge is clicked in the 3d view
    """
    # shp = A list of TopoDS_Shape; type=Face, if click a place without model, it is null
    # kwargs xy coordinate in 2D where mouse is clicked
    global shapeFromModel
    print("\n\n")
    print("Clicked !!")
    
    for edge in edg_local:  # this should be a TopoDS_Face TODO check it is
        print("Edge selected: ", edge)  # TopoDS_Shape
        edge_topo = Topo(shapeFromModel)
        #wires = list(edge_topo.wires_from_edge(edge))
        faces = list(edge_topo.faces_from_edge(edge))
        print('faces:',faces)
        # print(len(wires))                
        #print('first Wire',list(edge_topo.edges_from_wire(wires[0])))
        #print('Second Wire',list(edge_topo.edges_from_wire(wires[1])))

        #print("\nAfter conversion:\n", topods_Edge(edge))  # TopoDS_Edge
        #recognize_edge(topods_Edge(edge))


def select_face():
    global display
    global shapeFromModel
    display.unregister_callback(click_edge)
    display.SetSelectionModeFace()
    display.DisplayShape(shapeFromModel, update=True)
    display.register_select_callback(click_face)


def click_face(shp_local, *kwargs):
    """ This is the function called every time
    a face is clicked in the 3d view
    """
    # shp = A list of TopoDS_Shape; type=Face, if click a place without model, it is null
    # kwargs xy coordinate in 2D where mouse is clicked
    print(shp_local)
    global shapeFromModel
    print("\n\n")
    print("Click Face !!")

    for shape in shp_local:  # this should be a TopoDS_Face TODO check it is
        print("Face selected: %s,\nHashCode:%d" %(shape, shape.HashCode(100000000)))  # TopoDS_Shape
        shp_topo = Topo(shapeFromModel)
        print('number of wires from cylinder face:', len(list(shp_topo.wires_from_face(shape))))
        cylinder_wire = list(shp_topo.wires_from_face(shape))[0]
        cylinder_edges = list(shp_topo.edges_from_wire(cylinder_wire))
        print('cylinder_edges:',cylinder_edges)
        edges_list_from_neighbeor_wire = []
        # take out the edge which is at open end of the cylinder
        for edge in cylinder_edges:
            wires = list(shp_topo.wires_from_edge(edge))
            wires.remove(cylinder_wire)

            n = 0
            for wire in wires:
                edges_list_from_neighbeor_wire.append(list(shp_topo.edges_from_wire(wire)))
                print('check, n should be always zero. n:',n)
                n = n+1

        count_list = []
        for i in range(0,len(edges_list_from_neighbeor_wire)):
            count = 0
            for j in range(0,len(edges_list_from_neighbeor_wire[i])):
                for k in range(0,len(edges_list_from_neighbeor_wire)):
                    #print(edges_list_from_neighbeor_wire[i][j])
                    #print(edges_list_from_neighbeor_wire[k])
                    for l in range(len(edges_list_from_neighbeor_wire[k])):
                        if edges_list_from_neighbeor_wire[i][j].HashCode(10000000) == edges_list_from_neighbeor_wire[k][l].HashCode(10000000): 
                            count = count + 1
                        print('here!! i = %d, j = %d, k = %d, l = %d,count = %d' %(i,j,k,l, count))                
            count = count - len(edges_list_from_neighbeor_wire[i])
            count_list.append(count)

        print(count_list)
        print(edges_list_from_neighbeor_wire[0])
        t = edges_list_from_neighbeor_wire
        ipdb.set_trace()
        print(edges_list_from_neighbeor_wire[1])
        print(edges_list_from_neighbeor_wire[2])
        print(edges_list_from_neighbeor_wire[3])
        #vertices = list(shp_topo.wires_from_face(cylinder_wire))

        #faces = shp_topo.faces_from_wire(wires)
        #print(faces)
        #print("\nAfter conversion:\n", topods_Face(shape))  # TopoDS_Face
        #recognize_face(topods_Face(shape))


if __name__ == '__main__':
    logging.basicConfig(filename="logging.txt", filemode='w', level=logging.warning)
    run_display = True
    if(run_display):
        display, start_display, add_menu, add_function_to_menu = init_display()
        display.SetSelectionModeEdge()
        display.register_select_callback(click_edge)
        # first loads the STEP file and display
    fileList = ['lf064-01.stp', 
                'cylinder_group_test.stp', 
                'cylinder_cut.stp', 
                'cylinder_cut2.stp',
                'face_recognition_sample_part.stp',
                'cylinder_with_side_hole.stp',
                'cylinder_with_side_slot.stp',
                'cylinder_with_slot.stp',
                'cylinders.stp']
    shapeFromModel = read_step_file(os.path.join('.', 'models', fileList[6]))

    if(run_display):
        display.DisplayShape(shapeFromModel, update=True)
        add_menu('recognition')
        # add_function_to_menu('recognition', recognize_batch)
        add_menu('Selection Mode')
        add_function_to_menu('Selection Mode', select_edge)
        add_function_to_menu('Selection Mode', select_face)

        start_display()

    cylinder_pairs = get_cylinders(shapeFromModel)
    cylinder_axis_group = cylinder_group_coaxial(cylinder_pairs)
    
    cylinder_axis_r_group = cylinder_group_radius(cylinder_axis_group)

    t = cylinder_axis_r_group
    key_list = list(t.keys())
    '''
    print('\n\n\n')
    topo_shape = Topo(shapeFromModel)
    for i in key_list:
        grp_list = t[i]
        if len(grp_list)>=2:
            for j in range(0,len(grp_list)):
                cylinder1 = grp_list[j][0]
                cirle_list = extract_edgeType_all(list(topo_shape.edges_from_face(cylinder1)))['circle']
                if len(cirle_list) > 0 :
                    
                else:
                    logging.warning('Theres no circle curve in the cylinder !!')

                for k in range(j+1,len(grp_list)):
                    cylinder2 = grp_list[k][0]

        else:
    '''


    tmp = Topo(shapeFromModel)
    a = list(tmp.edges_from_face(t[k[0]][0][0]))
    b = list(tmp.edges_from_face(t[k[0]][1][0]))
    print(a)
    print(b)

    c0 = list(tmp.wires_from_edge(b[0]))
    c1 = list(tmp.edges_from_wire(c0[0]))


    ipdb.set_trace(context=7)
    #print(cylinder_group_radius(cylinder_axis_group)['ax=0.000,1.000,0.000,-8.000,0.000,0.000,r=2.500'])


    # comparing cylinder by direction

    '''            
    axis1 = cylinders[0][1].Axis().Direction()
    axis2 = cylinders[1][1].Axis().Direction()
    print(axis1.X(), axis1.Y(), axis1.Z())
    print(axis2.X(), axis2.Y(), axis2.Z())
    print( axis1.IsEqual(axis2, pi/180.0))exit

    '''