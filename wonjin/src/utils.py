import os
import threading
import datetime
from enum import IntEnum

from vtkmodules.vtkCommonColor import vtkNamedColors
from vtkmodules.vtkFiltersSources import (
    vtkSphereSource,
    vtkCylinderSource,
    vtkCubeSource,
)
from vtkmodules.vtkRenderingCore import (
    vtkActor,
    vtkPolyDataMapper,

)


def make_cube_source(x, y, z):
    cube_source = vtkCubeSource()
    cube_source.SetXLength(x)
    cube_source.SetYLength(y)
    cube_source.SetZLength(z)
    cube_source.SetCenter(0, 0, -2)
    # cube_source.GetCenter()

    return cube_source


def make_sphere_source(radius):
    sphere_source = vtkSphereSource()
    sphere_source.SetRadius(radius)
    sphere_source.SetPhiResolution(100)
    sphere_source.SetThetaResolution(100)
    sphere_source.GetCenter()

    return sphere_source


def make_cylinder_source(radius, height):
    cylinder_source = vtkCylinderSource()
    cylinder_source.SetRadius(radius)
    cylinder_source.SetHeight(height)
    cylinder_source.SetResolution(100)

    return cylinder_source


def make_mapper_actor(source, transform=None, colors=vtkNamedColors()):
    source_mapper = vtkPolyDataMapper()
    source_mapper.SetInputConnection(source.GetOutputPort())

    actor = vtkActor()
    actor.SetMapper(source_mapper)
    actor.GetProperty().SetColor(colors.GetColor3d("Cornsilk"))

    if transform:
        actor.SetUserTransform(transform)
    # actor.SetPosition(position[0], position[1], position[2])

    return actor


# ─────────── 쿼터니언→회전 행렬 변환 ───────────
def quat_to_matrix(w, x, y, z):
    r00 = 1 - 2 * (y * y + z * z)
    r01 = 2 * (x * y - z * w)
    r02 = 2 * (x * z + y * w)
    r10 = 2 * (x * y + z * w)
    r11 = 1 - 2 * (x * x + z * z)
    r12 = 2 * (y * z - x * w)
    r20 = 2 * (x * z - y * w)
    r21 = 2 * (y * z + x * w)
    r22 = 1 - 2 * (x * x + y * y)
    return (r00, r01, r02, 0, r10, r11, r12, 0, r20, r21, r22, 0, 0, 0, 0, 1)


def quat_inverse(w, x, y, z):
    return (w, -x, -y, -z)


def quat_multiply(w1, x1, y1, z1, w2, x2, y2, z2):
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return (w, x, y, z)


def get_timestamped_filename():
    now = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    os.makedirs("./Recorded_Data", exist_ok=True)
    return f"./Recorded_Data/sensor_data_{now}.csv"


class SensorIndex(IntEnum):
    PELVIS = 0
    LEFT_LEG = 1
    LEFT_CALF = 2
    LEFT_FOOT = 3
    RIGHT_LEG = 4
    RIGHT_CALF = 5
    RIGHT_FOOT = 6


# ─────────── 전역 변수 ───────────
sensor_transforms = {
    SensorIndex.PELVIS: [
        0, 1, 0, 0,
        0, 0, 1, 0,
        1, 0, 0, 0,
        0, 0, 0, 1
    ],
    SensorIndex.LEFT_LEG: [
        0, -1, 0, 0,
        0, 0, 1, 0,
        -1, 0, 0, 0,
        0, 0, 0, 1
    ],
    SensorIndex.LEFT_CALF: [
        0, 1, 0, 0,
        0, 0, 1, 0,
        1, 0, 0, 0,
        0, 0, 0, 1
    ],
    SensorIndex.LEFT_FOOT: [
        0, 1, 0, 0,
        0, 0, 1, 0,
        -1, 0, 0, 0,
        0, 0, 0, 1
    ],
    SensorIndex.RIGHT_LEG: [
        0, 1, 0, 0,
        0, 0, 1, 0,
        -1, 0, 0, 0,
        0, 0, 0, 1
    ],
    SensorIndex.RIGHT_CALF: [
        0, 1, 0, 0,
        0, 0, 1, 0,
        1, 0, 0, 0,
        0, 0, 0, 1
    ],
    SensorIndex.RIGHT_FOOT: [
        0, 1, 0, 0,
        0, 0, 1, 0,
        -1, 0, 0, 0,
        0, 0, 0, 1
    ],
}

latest_quat = [(1.0, 0.0, 0.0, 0.0)] * 7   # [(w, x, y, z)], 7개 센서
lock = threading.Lock()

saving = False
sensor_data = []  # 센서별 쿼터니언 기록

calibration_offsets = [(1.0, 0.0, 0.0, 0.0)] * 7