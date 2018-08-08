

from OCC.STEPControl import STEPControl_Reader
from OCC.IFSelect import IFSelect_RetDone, IFSelect_ItemsByEntity
import os
import os.path
import sys
import logging
import ipdb


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


class Display():
    def __init__(self, shp, run_display=False, *args, **kwargs):
        self.shape = shp
        if(run_display):
            self.display, self.start_display, self.add_menu, self.add_function_to_menu = init_display()
            self.display.register_select_callback(self.click_edge)
            self.display.SetSelectionModeEdge()

            self.display.DisplayShape(self.shape, update=True)
            self.add_menu('Selection Mode')
            self.add_function_to_menu('Selection Mode', self.edge_select_mode)
            self.add_function_to_menu('Selection Mode', self.face_select_mode)
            self.start_display()

    def update_shape(self, shp):
        self.shape = shp
        self.display.DisplayShape(self.shape, update=True)

    def edge_select_mode(self):
        self.display.unregister_callback(self.click_face)
        self.display.SetSelectionModeEdge()
        self.display.DisplayShape(self.shape, update=True)
        self.display.register_select_callback(self.click_edge)

    def click_edge(self, edg_local, *kwargs):
        """ This is the function called every time
        an edge is clicked in the 3d view
        """
        # shp = A list of TopoDS_Shape; type=Face, if click a place without model, it is null
        # kwargs xy coordinate in 2D where mouse is clicked
        print("\n\n")
        print("Clicked !!")
        for edge in edg_local:  # this should be a TopoDS_Face TODO check it is
            print("Edge selected: ", edge)  # TopoDS_Shape

    def face_select_mode(self):
        self.display.unregister_callback(self.click_edge)
        self.display.SetSelectionModeFace()
        self.display.DisplayShape(self.shape, update=True)
        self.display.register_select_callback(self.click_face)

    def click_face(self, shp_local, *kwargs):
        """ This is the function called every time
        a face is clicked in the 3d view
        """
        # shp = A list of TopoDS_Shape; type=Face, if click a place without model, it is null
        # kwargs xy coordinate in 2D where mouse is clicked

        print("Click Face !!")

        for shape in shp_local:  # this should be a TopoDS_Face TODO check it is
            print("Face selected: ", shape)  # TopoDS_Shape


if __name__ == '__main__':
    logging.basicConfig(filename="logging.txt", filemode='w',
                        level=logging.warning)
    fileList = ['lf064-01.stp', 'cylinder_group_test.stp', 'cylinder_cut.stp',
                'cylinder_cut2.stp', 'face_recognition_sample_part.stp',
                'cylinder_with_side_hole.stp', 'cylinder_with_side_slot.stp',
                'cylinder_with_slot.stp', 'cylinders.stp']
    shapeFromModel = read_step_file(os.path.join('..', 'models', fileList[6]))
    showShapes = shapeFromModel

    Display(showShapes, run_display=True)

    
