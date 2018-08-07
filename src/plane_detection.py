

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
