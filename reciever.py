import socket
import threading
import logging

import vtk

# ─────────── 로깅 세팅 ───────────
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(levelname)s: %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)

# ─────────── 전역 변수 ───────────
latest_quat = [(1.0, 0.0, 0.0, 0.0)]  # [(w, x, y, z)]
lock = threading.Lock()

# ─────────── 네트워크 스레드 ───────────
def network_thread(host='127.0.0.1', port=65431):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen(1)
    logger.info(f"Waiting for connection on {host}:{port}")
    conn, addr = server.accept()
    logger.info(f"Connected by {addr}")

    while True:
        try:
            data = conn.recv(1024)       # 블로킹 호출
            if not data:
                continue
            text = data.decode('utf-8').rstrip('\n')
            parts = text.split(';')
            for p in parts:
                if not p:
                    continue
                vals = p.split(',')
                if len(vals) == 4:
                    w, x, y, z = map(float, vals)
                    logger.info(f"Received quaternion: w={w:.3f}, x={x:.3f}, y={y:.3f}, z={z:.3f}")
                    with lock:
                        latest_quat[0] = (w, x, y, z)
                    break
        except Exception as e:
            logger.error(f"Network error: {e}")
            continue

# ─────────── 쿼터니언→회전 행렬 변환 ───────────
def quat_to_matrix(w, x, y, z):
    r00 = 1 - 2*(y*y + z*z)
    r01 =     2*(x*y - z*w)
    r02 =     2*(x*z + y*w)
    r10 =     2*(x*y + z*w)
    r11 = 1 - 2*(x*x + z*z)
    r12 =     2*(y*z - x*w)
    r20 =     2*(x*z - y*w)
    r21 =     2*(y*z + x*w)
    r22 = 1 - 2*(x*x + y*y)
    return (
        r00, r01, r02, 0,
        r10, r11, r12, 0,
        r20, r21, r22, 0,
          0,   0,   0, 1
    )

# ─────────── VTK 키 이벤트 콜백 ───────────
def keypress_callback(obj, event):
    if obj.GetKeySym().lower() == 'k':
        logger.info("Key 'k' pressed: exiting.")
        obj.TerminateApp()

# ─────────── VTK 타이머 콜백 ───────────
class TimerCallback:
    def __init__(self, actor, renWin):
        self.actor  = actor
        self.renWin = renWin
        self.transform = vtk.vtkTransform()
        self.actor.SetUserTransform(self.transform)

        # 좌표계 X축 반전을 위한 Basis 행렬 B (LH→RH 변환)
        self.B = vtk.vtkMatrix4x4()
        self.B.DeepCopy((
            -1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1
        ))
        # B의 역행렬 (reflection은 자기 자신의 역)
        self.B_inv = vtk.vtkMatrix4x4()
        vtk.vtkMatrix4x4.Invert(self.B, self.B_inv)

    def __call__(self, obj, event):
        with lock:
            w, x, y, z = latest_quat[0]
        # 1) 쿼터니언 회전 행렬 생성
        Mq = vtk.vtkMatrix4x4()
        Mq.DeepCopy(quat_to_matrix(w, x, y, z))
        # 2) 축 변환
        tmp = vtk.vtkMatrix4x4()
        vtk.vtkMatrix4x4.Multiply4x4(self.B, Mq, tmp)
        Mprime = vtk.vtkMatrix4x4()
        vtk.vtkMatrix4x4.Multiply4x4(tmp, self.B_inv, Mprime)
        # 3) 최종 행렬 적용 및 렌더
        self.transform.SetMatrix(Mprime)
        self.renWin.Render()

# ─────────── 메인 ───────────
if __name__ == '__main__':
    # 1) 네트워크 쓰레드 시작 (데이터 처리만 담당)
    threading.Thread(target=network_thread, daemon=True).start()

    # 2) VTK 파이프라인 생성
    cube = vtk.vtkCubeSource()
    cube.Update()

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(cube.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    renderer = vtk.vtkRenderer()
    renderer.AddActor(actor)
    renderer.SetBackground(0.1, 0.1, 0.1)

    # 카메라(Back view) 설정
    cam = renderer.GetActiveCamera()
    cam.SetPosition(0, 0, -5)
    cam.SetFocalPoint(0, 0, 0)
    cam.SetViewUp(0, 1, 0)
    renderer.ResetCameraClippingRange()

    renWin = vtk.vtkRenderWindow()
    renWin.AddRenderer(renderer)
    renWin.SetSize(800, 600)

    iren = vtk.vtkRenderWindowInteractor()
    iren.SetRenderWindow(renWin)
    iren.AddObserver('KeyPressEvent', keypress_callback)

    # 3) 타이머 콜백 등록 (10ms 주기)
    cb = TimerCallback(actor, renWin)
    iren.AddObserver('TimerEvent', cb)
    iren.Initialize()
    iren.CreateRepeatingTimer(10)

    # 4) 렌더링 시작
    renWin.Render()
    iren.Start()

    logger.info("Application exited.")
