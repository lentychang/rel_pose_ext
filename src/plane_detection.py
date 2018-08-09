

from step_utils import read_step_file
from topo2 import RecognizeTopo
from OCC.Display.SimpleGui import init_display
import os.path
import logging
from OCC.BRepAdaptor import BRepAdaptor_Surface, BRepAdaptor_Curve
import ipdb

if __name__ == '__main__':

    ##############################
    logging.basicConfig(filename="logging.txt", filemode='w',
                        level=logging.warning)
    run_display = True
    if(run_display):
        display, start_display, add_menu, add_function_to_menu = init_display()
        display.SetSelectionModeEdge()
        # display.register_select_callback(click_edge)
        # first loads the STEP file and display
    fileList = ['lf064-01.stp', 'cylinder_group_test.stp', 'cylinder_cut.stp',
                'cylinder_cut2.stp', 'face_recognition_sample_part.stp',
                'cylinder_with_side_hole.stp', 'cylinder_with_side_slot.stp',
                'cylinder_with_slot.stp', 'cylinders.stp']
    shapeFromModel = read_step_file(os.path.join('..', 'models', fileList[6]))

    # group planes with normal vector
    planeList = RecognizeTopo(shapeFromModel).planes()
    pln_dict = {}
    for pln in planeList:
        gp_pln = BRepAdaptor_Surface(pln).Plane()
        normal = gp_pln.Axis().Direction()
        key = '%.3f,%.3f,%.3f' % (normal.X(), normal.Y(), normal.Z())
        if key not in pln_dict.keys():
            pln_dict[key] = [pln]
        else:
            pln_dict[key].append(pln)
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
    
    ipdb.set_trace()

    if(run_display):
        display.DisplayShape(shapeFromModel, update=True)
        add_menu('recognition')
        # add_function_to_menu('recognition', recognize_batch)
        add_menu('Selection Mode')
        # add_function_to_menu('Selection Mode', select_edge)
        # add_function_to_menu('Selection Mode', select_face)

        start_display()
    ##############################
