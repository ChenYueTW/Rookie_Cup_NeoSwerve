from cscore import CameraServer
from networktables import NetworkTablesInstance
from networktables import NetworkTables
import time
import pupil_apriltags as apriltag  # For Windows
import cv2
import numpy as np

# 設置 RoboRIO 的 IP
roborio_ip = '10.87.25.2'

# 相機參數
camera_width, camera_height = 1280, 800  # 相機解析度
hfov = 70         # 水平視野
camera_xCenter, camera_yCenter = camera_width / 2, camera_height / 2   # 目標像素座標（畫面中心）
print("START")

# NetworkTablesIstance 設定
ntinst = NetworkTablesInstance.getDefault()
ntinst.startClient(roborio_ip)
ntinst.startDSClient()            # 啟動與 Driver Station 的連接

# NetworkTable 設定
suffleboard_tab = NetworkTables.getTable("raspberrypi")
cameraserver_tab = NetworkTables.getTable("CameraServer")
camera_entry = suffleboard_tab.getEntry("CameraLink")
camera_url = "http://10.87.25.106:1181/stream.mjpg"
camera_entry.setString(camera_url)

time.sleep(1)

# CameraServer 設定
CameraServer.enableLogging()
camera = CameraServer.startAutomaticCapture("RaspPI", 0)
# camera.setResolution(camera_width, camera_height)
# camera.setFPS(30)

# OpenCV 設定
sink = CameraServer.getVideo(camera)
print("Set up sinks array")
frame_buffer = np.zeros((camera_width, camera_height, 3), dtype=np.uint8)
print("Set up frame buffers")
mjpeg_server = CameraServer.putVideo("ProcessedVideo", camera_width, camera_height)
print("Set up outputs array")

at_detector = apriltag.Detector(families='tag36h11')  # For Windows

def pixel_to_angle(x, y):
    # 計算垂直視野 VFOV
    vfov = hfov * (camera_height / camera_width)

    # 計算角度
    tx = (x / camera_width - 0.5) * hfov
    ty = (0.5 - y / camera_height) * vfov

    return tx, ty

def put_value(tx, ty, tid):
    suffleboard_tab.putNumber("tx", tx)
    suffleboard_tab.putNumber("ty", ty)
    suffleboard_tab.putNumber("tid", tid)

put_value(0.0, 0.0, 0.0)

try:
    while True:
        start_time = time.time()
        # 捕獲影像幀
        ret, frame = sink.grabFrame(frame_buffer)
        
        # 檢測 AprilTags
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = at_detector.detect(gray)

        # 偵測有沒有掃到東西
        if tags == []:
            put_value(0.0, 0.0, 0.0)
            continue 
        
        for t in tags:
            # 獲取 AprilTag 的中心坐標
            (cX, cY) = (int(t.center[0])), (int(t.center[1])) # X 和 Y 坐標（整數）

            # 中心座標轉tx, ty
            tx, ty = pixel_to_angle(cX, cY)
            
            # 顯示發送的訊息
            print(f"tx: {tx:.1f} | ty: {ty:.1f} | Tag ID: {t.tag_id}")
            
            # 繪製 AprilTag 的邊界框和中心點
            b = (tuple(t.corners[0].astype(int))[0], tuple(t.corners[0].astype(int))[1])
            c = (tuple(t.corners[1].astype(int))[0], tuple(t.corners[1].astype(int))[1])
            d = (tuple(t.corners[2].astype(int))[0], tuple(t.corners[2].astype(int))[1])
            a = (tuple(t.corners[3].astype(int))[0], tuple(t.corners[3].astype(int))[1])
            
            # 畫出 AprilTag 的邊界框
            cv2.line(frame, a, b, (255, 0, 255), 2, lineType=cv2.LINE_AA)
            cv2.line(frame, b, c, (255, 0, 255), 2, lineType=cv2.LINE_AA)
            cv2.line(frame, c, d, (255, 0, 255), 2, lineType=cv2.LINE_AA)
            cv2.line(frame, d, a, (255, 0, 255), 2, lineType=cv2.LINE_AA)
            
            # 畫出 AprilTag 的中心點
            cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
            cv2.putText(frame, f"id: {t.tag_id}", (a[0], a[1] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, f"X: {cX} Y: {cY}", (a[0], a[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, f"tx: {tx:.1f} ty: {ty:.1f}", (a[0], a[1] - 0), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Put to NetworkTable
            put_value(tx, ty, t.tag_id)
            # 控制傳送頻率
            # time.sleep(0.01)  # 每秒發送一次訊息 (可根據需要調整)

        end_time = time.time()
        fps = 1.0 / (end_time - start_time)
        cv2.putText(
            frame,
            f"{fps:.1f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2,
        )
        mjpeg_server.putFrame(frame)

except KeyboardInterrupt:
    # 捕捉 Ctrl+C 終止程式的動作
    print("Program terminated by user.")
