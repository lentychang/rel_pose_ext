#!/usr/bin/python3
import ipdb
from OCC.Bnd import Bnd_B3f, Bnd_Box
from OCC.BRepBndLib import brepbndlib
from OCC.gp import gp_Pnt, gp_XYZ
from OCC.IntTools import IntTools_FaceFace
from OCC.BRepAlgo import BRepAlgo_Common


from .core_topology_traverse import Topo
from .dataIO import Display, read_step_file, read_stp_solid_withTf


def test_bndBox():
    # simple interference check through bounding box

    shp = read_step_file("/home/lenty/exchange/tempData/models/lf064-01.stp")
    brepbnd = brepbndlib()
    bndBox = Bnd_Box()
    # display = Display(shp, run_display=True)

    brepbnd.Add(shp, bndBox)
    p1 = gp_Pnt(55, 30, 7)
    print(bndBox.IsOut(p1))

    b3f = Bnd_B3f()
    p2 = gp_Pnt(0, 0, 0)
    b3f.Add(p2)
    print(b3f.IsOut(gp_XYZ(0, 0, 1)))
    ipdb.set_trace()
    print(b3f.IsOut(gp_Pnt(0, 0, 1)))


def bndBox_is_interfered(shp1, shp2):
    brepbnd = brepbndlib()
    bndBox1 = Bnd_Box()
    bndBox2 = Bnd_Box()
    brepbnd.Add(shp1, bndBox1)
    brepbnd.Add(shp2, bndBox2)
    return not (bndBox1.IsOut(bndBox2))


def infinite_faces_are_interferenced(solid1, solid2):

    # tolerance is take from faces themselves
    faces1 = list(Topo(solid1).faces())
    faces2 = list(Topo(solid2).faces())
    interf = IntTools_FaceFace()

    interf_count = 0
    for face1 in faces1:
        for face2 in faces2:

            interf = IntTools_FaceFace()
            interf.Perform(face1, face2)
            if interf.Points().Length() != 0:
                interf_count += 1
            if interf.Lines().Length() != 0:
                interf_count += 1
                display2 = Display(face1, run_display=False)
                display2.add_shape(face2)
                # display2.open()

    if interf_count > 0:
        return True
    else:
        return False


def solid_interfered_exact(solid1, solid2):
    # ps. could be implemented also with BRepAlgoAPI?
    # brep_api = BRepAlgoAPI_Common()
    # [BOPAlgo_COMMON, BOPAlgo_FUSE, BOPAlgo_CUT, BOPAlgo_CUT21, BOPAlgo_SECTION, BOPAlgo_UNKNOWN]

    builder = BRepAlgo_Common(solid1, solid2)
    # [TopAbs_IN, TopAbs_OUT, TopAbs_ON, TopAbs_UNKNOWN]
    builder.Perform(0, 0)
    intersectionShp = builder.Shape()

    shp_topo = Topo(intersectionShp)
    # display = Display(intersectionShp, run_display=False)
    interf_count = shp_topo.number_of_comp_solids() + shp_topo.number_of_solids() + shp_topo.number_of_shells() + shp_topo.number_of_edges() + shp_topo.number_of_vertices() + shp_topo.number_of_wires()

    if interf_count > 0:
        return True
    else:
        return False


def test():
    modelDir = "/home/lenty/exchange/tempData/models"
    file1 = "lf064-01"
    file2 = "lf064-02"
    topods_solid1 = read_stp_solid_withTf(modelDir, file1, [0, 0, 0], [0, 0, 0, 1], unitIsMM=True)
    topods_solid2 = read_stp_solid_withTf(modelDir, file2, [-40, 10, 2], [0, 0, 0, 1], unitIsMM=True)
    display = Display(topods_solid1, run_display=False)
    # display.add_shape(topods_solid2)
    assert bndBox_is_interfered(shp1=topods_solid1, shp2=topods_solid2) is True, "Function bndBox_is_interfered failed test, check algorithm"
    assert infinite_faces_are_interferenced(solid1=topods_solid1, solid2=topods_solid2) is True, "Function bndBox_is_interfered failed test, check algorithm"
    assert solid_interfered_exact(solid1=topods_solid1, solid2=topods_solid2) is True, "Function solid_interfered_exact failed test, check algorithm"

    topods_solid2 = read_stp_solid_withTf(modelDir, file2, [-40, 10, 16], [0, 0, 0, 1], unitIsMM=True)
    assert bndBox_is_interfered(shp1=topods_solid1, shp2=topods_solid2) is False, "Function bndBox_is_interfered failed test, check algorithm"
    assert infinite_faces_are_interferenced(solid1=topods_solid1, solid2=topods_solid2) is False, "Function bndBox_is_interfered failed test, check algorithm"
    assert solid_interfered_exact(solid1=topods_solid1, solid2=topods_solid2) is False, "Function solid_interfered_exact failed test, check algorithm"

    topods_solid2 = read_stp_solid_withTf(modelDir, file2, [-55, 30, 10], [0, 0, 0, 1], unitIsMM=True)
    display.add_shape(topods_solid2)
    # display.open()
    assert bndBox_is_interfered(shp1=topods_solid1, shp2=topods_solid2) is True, "Function bndBox_is_interfered failed test, check algorithm"
    assert infinite_faces_are_interferenced(solid1=topods_solid1, solid2=topods_solid2) is True, "Function bndBox_is_interfered failed test, check algorithm"
    assert solid_interfered_exact(solid1=topods_solid1, solid2=topods_solid2) is False, "Function solid_interfered_exact failed test, check algorithm"
    print("Test passes!")


if __name__ == "__main__":
    test()
