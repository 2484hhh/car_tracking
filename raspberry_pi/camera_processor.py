import cv2
import threading
import numpy as np
from ultralytics import YOLO
from car_controller import CarController

class CameraProcessor:
    def __init__(self):
        # 初始化摄像头
        self.usb_cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
        self.csi_cap = cv2.VideoCapture("/dev/video1")
        self.car_controller = CarController()
        # 加载YOLO模型（使用已训练的YOLOv8）
        self.model = YOLO('yolov8n.pt')

    def line_following_processing(self, frame):
        """循线处理"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
        height, width = binary.shape
        roi = binary[int(height * 0.6):height, :]
        contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                center_offset = cx - width // 2
                if center_offset > 20:
                    self.car_controller.send_command('LINE', {'dir': 'R'})
                elif center_offset < -20:
                    self.car_controller.send_command('LINE', {'dir': 'L'})
                else:
                    self.car_controller.send_command('LINE', {'dir': 'F'})
        else:
            self.car_controller.send_command('LINE', {'dir': 'S'})
        return binary

    def object_detection_processing(self, frame):
        """YOLO目标检测处理"""
        results = self.model.predict(source=frame, imgsz=640, conf=0.5, verbose=False)
        result_frame = results[0].plot()
        detected = set()
        for box in results[0].boxes:
            cls = int(box.cls[0])
            label = self.model.names[cls]
            detected.add(label)
            print(f"检测到目标: {label}")
        # 结构化指令发送
        if "stop sign" in detected:
            self.car_controller.send_command('DETECT', {'action': 'STOP'})
        elif "person" in detected:
            self.car_controller.send_command('DETECT', {'action': 'STOP'})
        elif "car" in detected:
            self.car_controller.send_command('DETECT', {'action': 'HORN'})
        # 可扩展更多目标与动作
        return result_frame

    def process_csi_camera(self):
        """CSI摄像头处理"""
        import time
        while True:
            ret, frame = self.csi_cap.read()
            if not ret:
                print("无法从CSI摄像头读取图像！")
                break
            # YOLO目标检测
            result_frame = self.object_detection_processing(frame)
            cv2.imshow("CSI Camera - YOLO", result_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            time.sleep(3)

    def process_usb_camera(self):
        """USB摄像头处理"""
        import time
        while True:
            ret, frame = self.usb_cap.read()
            if not ret:
                print("无法从USB摄像头读取图像！")
                break
            # 循线处理
            line_frame = self.line_following_processing(frame)
            cv2.imshow("USB Camera - Line Following", line_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            time.sleep(0.15)

    def start_processing(self):
        """启动多线程"""
        csi_thread = threading.Thread(target=self.process_csi_camera)
        csi_thread.daemon = True
        csi_thread.start()
        usb_thread = threading.Thread(target=self.process_usb_camera)
        usb_thread.daemon = True
        usb_thread.start()
        try:
            while True:
                # 可在此处读取反馈并处理
                feedback = self.car_controller.read_feedback()
                if feedback:
                    print(f"主控收到Arduino反馈: {feedback}")
                if cv2.waitKey(10) & 0xFF == ord('q'):
                    break
        finally:
            self.car_controller.close()
            cv2.destroyAllWindows()
