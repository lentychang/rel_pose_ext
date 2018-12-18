#!/usr/bin/python3
import ipdb
from OCC.Bnd import Bnd_B3f, Bnd_Box
from OCC.BRepBndLib import brepbndlib
from OCC.gp import gp_Pnt, gp_XYZ
from OCC.IntTools import IntTools_FaceFace

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


def solidsAreInterfered(topods_solid1, topods_solid2):
    # check solids interences by looping into all faces and check wheather they interfere with each other

    faces1 = list(Topo(topods_solid1).faces())
    faces2 = list(Topo(topods_solid2).faces())
    interf = IntTools_FaceFace()

    interf_count = 0
    for face1 in faces1:
        for face2 in faces2:
            interf.Perform(face1, face2)
            if interf.Points().Length() != 0 or interf.Lines().Length() != 0:
                interf_count += 1

    if interf_count > 0:
        print("[WARN] Two given solids interfere with each other!!")
        return True
    else:
        return False


def test_solidsAreInterfered():
    modelDir = "/home/lenty/exchange/tempData/models"
    file1 = "lf064-01"
    file2 = "lf064-02"
    topods_solid1 = read_stp_solid_withTf(modelDir, file1, [0, 0, 0], [0, 0, 0, 1], unitIsMM=True)
    topods_solid2 = read_stp_solid_withTf(modelDir, file2, [-40, 10, 5.01], [0, 0, 0, 1], unitIsMM=True)
    display = Display(topods_solid1, run_display=False)
    display.add_shape(topods_solid2)
    display.open()
    print("interference:{0}".format(solidsAreInterfered(topods_solid1, topods_solid2)))


if __name__ == "__main__":
    # test()
    test_solidsAreInterfered()
