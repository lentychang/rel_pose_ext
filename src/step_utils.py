

from OCC.STEPControl import STEPControl_Reader
from OCC.IFSelect import IFSelect_RetDone, IFSelect_ItemsByEntity
import os
import os.path
import sys
import logging

from OCC.Display.SimpleGui import init_display


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


if __name__ == '__main__':
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

    if(run_display):
        display.DisplayShape(shapeFromModel, update=True)
        add_menu('recognition')
        # add_function_to_menu('recognition', recognize_batch)
        add_menu('Selection Mode')
        # add_function_to_menu('Selection Mode', select_edge)
        # add_function_to_menu('Selection Mode', select_face)
        start_display()
