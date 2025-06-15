import sys
import threading
import numpy as np
import cv2
from PyQt6.QtCore import QObject, QThread, pyqtSignal, Qt, QSize
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
from ctypes import *
from ultralytics import YOLO
import time
import os 
sys.path.append("/opt/MVS/Samples/aarch64/Python/MvImport")
from MvCameraControl_class import *

class HikrobotCamera(QObject):
    frame_ready = pyqtSignal(QImage)
    
    def __init__(self, width=1280, height=720):
        super().__init__()
        self.img_w = width
        self.img_h = height
        self.cam = None
        self.data_buf = None
        self.nPayloadSize = 0
        self.running = False
        
    def init_camera(self):
        # Enum devices
        deviceList = MV_CC_DEVICE_INFO_LIST()
        tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE
        ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
        if ret != 0 or deviceList.nDeviceNum == 0:
            raise RuntimeError("No Hikrobot camera found")
        
        # Create and configure camera
        self.cam = MvCamera()
        stDeviceList = cast(deviceList.pDeviceInfo[0], POINTER(MV_CC_DEVICE_INFO)).contents
        if self.cam.MV_CC_CreateHandle(stDeviceList) != 0:
            raise RuntimeError("Failed to create camera handle")
        
        # Open device
        if self.cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0) != 0:
            raise RuntimeError("Failed to open camera")
        
        # Configure camera (your existing settings)
        self.cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
        self.cam.MV_CC_SetFloatValue("AcquisitionFrameRate", 20.0)
        self.cam.MV_CC_SetIntValue("Width", 372)
        self.cam.MV_CC_SetIntValue("Height", 2048)
        self.cam.MV_CC_SetFloatValue("ExposureTime", 45000.0)
        self.cam.MV_CC_SetFloatValue("Gain", 22.9042)
        self.cam.MV_CC_SetEnumValue("AcquisitionMode", 2)
        
        # Get payload size
        stParam = MVCC_INTVALUE()
        memset(byref(stParam), 0, sizeof(MVCC_INTVALUE))
        if self.cam.MV_CC_GetIntValue("PayloadSize", stParam) != 0:
            raise RuntimeError("Failed to get payload size")
        self.nPayloadSize = stParam.nCurValue
        self.data_buf = (c_ubyte * self.nPayloadSize)()
        
    def start_capture(self):
        if self.cam.MV_CC_StartGrabbing() != 0:
            raise RuntimeError("Failed to start grabbing")
        self.running = True
        self.thread = threading.Thread(target=self._capture_thread)
        self.thread.start()
        
    def stop_capture(self):
        self.running = False
        if hasattr(self, 'thread') and self.thread.is_alive():
            self.thread.join()
        if self.cam:
            self.cam.MV_CC_StopGrabbing()
            self.cam.MV_CC_CloseDevice()
            self.cam.MV_CC_DestroyHandle()
    
    def _capture_thread(self):
        stFrameInfo = MV_FRAME_OUT_INFO_EX()
        memset(byref(stFrameInfo), 0, sizeof(stFrameInfo))
        cam = Detection("../../model/face/face.pt")
        while self.running:
            ret = self.cam.MV_CC_GetOneFrameTimeout(self.data_buf, self.nPayloadSize, stFrameInfo, 1000)
            if ret == 0:
                # Process frame
                img = np.asarray(self.data_buf)
                img = img.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth, -1)
                img = cv2.resize(img, (self.img_w, self.img_h))
                # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                
                img,det = cam.process_frame(img)
                # Convert to QImage and emit
                h, w, ch = img.shape
                bytes_per_line = ch * w
                qt_img = QImage(img.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
                self.frame_ready.emit(qt_img)

class CameraViewer(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Hikrobot Camera Viewer")
        self.setGeometry(100, 100, 1280, 720)
        
        # Setup UI
        self.video_label = QLabel()
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout = QVBoxLayout()
        layout.addWidget(self.video_label)
        self.setLayout(layout)
        
        # Setup camera
        self.camera = HikrobotCamera()
        try:
            self.camera.init_camera()
            self.camera.frame_ready.connect(self.update_frame)
            self.camera.start_capture()
        except Exception as e:
            print(f"Camera initialization failed: {e}")
            self.close()
    
    def update_frame(self, image):
        # Scale if needed
        scaled_img = image.scaled(
            self.video_label.size(), 
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation
        )
        self.video_label.setPixmap(QPixmap.fromImage(scaled_img))
    
    def closeEvent(self, event):
        self.camera.stop_capture()
        event.accept()


class Detection:
    def __init__(self,model_path):
        self.model_path = model_path
        self.model = YOLO(model_path)

    def process_frame(self, img, start_time=None):
        """
        Processes a frame to detect objects, draw bounding boxes, and optionally calculate FPS.

        Args:
            img: The input frame as a NumPy array (OpenCV format).
            start_time: (optional) The time when the frame was received, used for FPS calculation.

        Returns:
            The processed frame with bounding boxes drawn (NumPy array).  Returns original if no detections.
            Also returns the list of detections.
        """
        results = self.model.predict(img, verbose=False, stream = True)
        detections = []
        for result in results:
            for *xyxy, conf, cls in result.boxes.data.tolist():
                x1, y1, x2, y2 = map(int, xyxy)
                center_u = (x1 + x2) // 2
                center_v = (y1 + y2) // 2
                bbox_h = abs(y2 - y1)
                class_id = int(cls)

                if bbox_h == 0:
                    continue  # Skip zero-height boxes

                # Draw the bounding box on the frame.  Use a color tuple (B, G, R).
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) # Add this line before cv2.rectangle
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Green rectangle, 2 pixels thick
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) # Convert back to RGB

                # You could add text here, too, using cv2.putText, to show class name or confidence.

                # detections.append({
                #     'box': (x1, y1, x2, y2),
                #     'center': (center_u, center_v),
                #     'height': bbox_h,
                #     'class_id': class_id,
                #     'confidence': conf
                # })
        return img, detections


if __name__ == "__main__":
    app = QApplication(sys.argv)
    viewer = CameraViewer()
    viewer.show()
    sys.exit(app.exec())