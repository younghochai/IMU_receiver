import time

from vtk import vtkMatrix4x4

from src.utils import (
    SensorIndex,
    lock,
    sensor_data,
    latest_quat,
    calibration_offsets,
    saving,
    sensor_transforms,
    quat_inverse,
    quat_multiply,
    quat_to_matrix,
)


class TimerCallback:
    def __init__(self, actor, transform, position_x, position_y, renWin, sensor_idx):
        self.actor = actor
        self.renWin = renWin
        self.transform = transform
        self.pos_x = position_x
        self.pos_y = position_y
        self.sensor_idx = sensor_idx
        self.actor.SetUserTransform(self.transform)

        self.B = vtkMatrix4x4()

        for sensor_idx in SensorIndex:
            self.B.DeepCopy(sensor_transforms[SensorIndex(sensor_idx)])

        self.B_inv = vtkMatrix4x4()
        vtkMatrix4x4.Invert(self.B, self.B_inv)

    def __call__(self, obj, event):
        global saving, sensor_data

        # 센서 데이터 읽기 (락 걸기)
        with lock:
            w1, x1, y1, z1 = latest_quat[0]
            w2, x2, y2, z2 = latest_quat[1]
            w3, x3, y3, z3 = latest_quat[2]
            w4, x4, y4, z4 = latest_quat[3]
            w5, x5, y5, z5 = latest_quat[4]
            w6, x6, y6, z6 = latest_quat[5]
            w7, x7, y7, z7 = latest_quat[6]

        # 저장 중일 때, sensor_idx == 0 (한 타이머에서만 저장 처리)
        if self.sensor_idx == 0 and saving:
            timestamp = time.time()
            sensor_data.append(
                [
                    timestamp,
                    w1,
                    x1,
                    y1,
                    z1,
                    w2,
                    x2,
                    y2,
                    z2,
                    w3,
                    x3,
                    y3,
                    z3,
                    w4,
                    x4,
                    y4,
                    z4,
                    w5,
                    x5,
                    y5,
                    z5,
                    w6,
                    x6,
                    y6,
                    z6,
                    w7,
                    x7,
                    y7,
                    z7,
                ]
            )

        raw_w, raw_x, raw_y, raw_z = latest_quat[self.sensor_idx]
        calib_w, calib_x, calib_y, calib_z = calibration_offsets[self.sensor_idx]

        # 보정 쿼터니언 구하기
        inv_calib = quat_inverse(calib_w, calib_x, calib_y, calib_z)
        w, x, y, z = quat_multiply(
            raw_w,
            raw_x,
            raw_y,
            raw_z,
            inv_calib[0],
            inv_calib[1],
            inv_calib[2],
            inv_calib[3],
        )
        Mq = vtkMatrix4x4()
        Mq.DeepCopy(quat_to_matrix(w, x, -y, -z))

        tmp = vtkMatrix4x4()
        vtkMatrix4x4.Multiply4x4(self.B, Mq, tmp)

        Mprime = vtkMatrix4x4()
        vtkMatrix4x4.Multiply4x4(tmp, self.B_inv, Mprime)

        self.transform.SetMatrix(Mprime)
        self.transform.Translate(self.pos_x, self.pos_y, 0)

        self.renWin.Render()
