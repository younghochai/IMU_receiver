import socket
import threading
import csv
import logging

from src.utils import (
    get_timestamped_filename,
    latest_quat, calibration_offsets, lock, sensor_data
)

# logger = logging.getLogger(__name__)

# ─────────── 로깅 세팅 ───────────
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s: %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger(__name__)


def network_thread(host="127.0.0.1", port=65431):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen(1)
    logger.info(f"Waiting for connection on {host}:{port}")
    conn, addr = server.accept()
    logger.info(f"Connected by {addr}")

    while True:
        try:
            data = conn.recv(1024)
            if not data:
                continue
            text = data.decode("utf-8").rstrip("\n")
            parts = text.split(";")

            for idx, p in enumerate(parts):
                if not p:
                    continue
                vals = p.split(",")
                if len(vals) == 4:
                    w, x, y, z = map(float, vals)
                    # logger.info(f"Received sensor{idx+1}: w={w:.3f}, x={x:.3f}, y={y:.3f}, z={z:.3f}")
                    with lock:
                        if idx < len(latest_quat):
                            latest_quat[idx] = (w, x, y, z)

        except Exception as e:
            logger.error(f"Network error: {e}")
            continue


def start_data_thread():
    threading.Thread(target=network_thread, daemon=True).start()


def keypress_callback(obj, event):
    global saving, sensor_data

    key = obj.GetKeySym().lower()

    if key == "r":
        logger.info("저장 시작")
        saving = True
        sensor_data.clear()

    elif key == "s":
        logger.info("저장 종료 및 CSV 저장")
        saving = False
        filename = get_timestamped_filename()
        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "timestamp",
                "sensor1_w", "sensor1_x", "sensor1_y", "sensor1_z",
                "sensor2_w", "sensor2_x", "sensor2_y", "sensor2_z",
                "sensor3_w", "sensor3_x", "sensor3_y", "sensor3_z",
                "sensor4_w", "sensor4_x", "sensor4_y", "sensor4_z",
                "sensor5_w", "sensor5_x", "sensor5_y", "sensor5_z",
                "sensor6_w", "sensor6_x", "sensor6_y", "sensor6_z",
                "sensor7_w", "sensor7_x", "sensor7_y", "sensor7_z",
            ])
            writer.writerows(sensor_data)
        logger.info(f"전체 센서 데이터 CSV 저장됨: {filename}")

    elif key == 'c':
        with lock:
            for i in range(7):
                calibration_offsets[i] = latest_quat[i]
        logger.info("캘리브레이션 완료")
