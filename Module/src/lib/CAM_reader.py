import cv2
import sys
import threading
import numpy as np
from PyQt6.QtCore import QObject, QThread, pyqtSignal, Qt, QSize
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
from ctypes import *
import os
import time

from ultralytics import YOLO
import subprocess

sys.path.append("/home/ubuntu/Documents/programs/CV/Module/src/lib") # Default path for aarch64 /home/ubuntu/Documents/programs/CV/Module/src/lib
sys.path.append("/home/ubuntu/Documents/programs/CV/Module/src/")

from detection import Detector
import can
import multiprocessing
import csv
from datetime import datetime

# Add the path to MvImport.  This path may need to be adjusted
# to match your installation.

sys.path.append("/opt/MVS/Samples/aarch64/Python/MvImport") # Default path for aarch64

model_path = "../../model/patria/best.pt"
from MvCameraControl_class import *

class CameraFeed(QThread):
    frame_ready = pyqtSignal(QImage)
    def __init__(self, width=320, height=240, record_queue = multiprocessing.Queue()):
        super().__init__()
        self.img_w = width
        self.img_h = height
        self.cam = None
        self.data_buf = None
        self.nPayloadSize = 0
        self.running = False
        self.model = Detector(model_path)
        self.radar_queue = []
        self.record_queue = record_queue
        self.csv_path = "../radar_matching.csv"
        
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
        self.cam.MV_CC_SetFloatValue("ExposureTime", 15000.0)
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
        self.thread = threading.Thread(target=self.run)
        self.thread.start()
        
    def stop(self):
        self.running = False
        if hasattr(self, 'thread') and self.thread.is_alive():
            self.thread.join()
        if self.cam:
            self.cam.MV_CC_StopGrabbing()
            self.cam.MV_CC_CloseDevice()
            self.cam.MV_CC_DestroyHandle()

    def parse_target_info(self, data):
        """
        Parse the 8-byte CAN frame into structured object data based on MR76 logic.
        """
        data_stream = [0] * 8
        data_stream[0] = data[0]  # Object ID
        dist_long_raw = ((data[1] << 5) | (data[2] >> 3)) & 0x1FFF # Distance Longitudinal (13 bits)
        dist_lat_raw = ((data[2] & 0x07) << 8) | data[3] # Distance Lateral (11 bits)
        vel_long_raw = ((data[4] << 6) | (data[5] >> 2)) & 0x3FFF # Velocity Longitudinal (14 bits)
        vel_lat_raw = ((data[5] & 0x03) << 8) | ((data[6] & 0xE0) >> 5) # Velocity Lateral (11 bits)    
        obj_class = (data[6] >> 3) & 0x03 # Object Class (2 bits)
        dyn_prop = data[6] & 0x07 # Dynamic Property (3 bits)
        rcs_raw = data[7] # Radar Cross Section (8 bits)

        parsed = {
            "id": data_stream[0],
            "dist_long": dist_long_raw * 0.2 - 500,
            "dist_lat": dist_lat_raw * 0.2 - 204.6,
            "vel_rel_long": vel_long_raw * 0.25 - 128.0,
            "vel_rel_lat": vel_lat_raw * 0.25 - 64.0,
            "object_class": obj_class,
            "dynamic_prop": dyn_prop,
            "rcs": rcs_raw * 0.5 - 64.0
        }

        return parsed

    def can_listener(self):
        bus = can.interface.Bus(channel='vcan0', interface='socketcan')
        while True:
            msg = bus.recv()
            if msg is None or len(msg.data) < 8:
                continue
            try:
                parsed = self.parse_target_info(list(msg.data))
                """
                print((
                    parsed["id"],
                    parsed["dist_long"],
                    parsed["dist_lat"],
                    parsed["vel_rel_long"],
                    parsed["vel_rel_lat"]
                ))
                """
                self.radar_queue.append(parsed)
            except Exception as e:
                print(f"[ERROR] Failed to parse frame: {e}")

    def run(self):
        stFrameInfo = MV_FRAME_OUT_INFO_EX()
        memset(byref(stFrameInfo), 0, sizeof(stFrameInfo))
        fieldnames = ['timestamp', 'id', 'dist_lat', 'dist_long', 'vel_rel_long', 'vel_rel_lat', 'object_class', 'dynamic_prop', 'rcs', 'class', 'x_vision']
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
        can_thread = threading.Thread(target=self.can_listener)
        can_thread.start()

        while self.running:
            ret = self.cam.MV_CC_GetOneFrameTimeout(self.data_buf, self.nPayloadSize, stFrameInfo, 1000)
            if ret == 0:
                img = np.asarray(self.data_buf)
                img = img.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth, -1)
                img = cv2.resize(img, (self.img_w, self.img_h))
                # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                
                try:
                    detected_objects = self.model.detect_n_seg(img)
                    detected_objects.sort(key=lambda obj: obj.box[0] if obj.box else float('inf'))
                    sorted_labels = []
                    for each_object in detected_objects:
                        cv2.rectangle(img, (each_object.box[0], each_object.box[1]), (each_object.box[2], each_object.box[3]), (0,255,0), 2)
                        cv2.putText(img, each_object.object_class, (each_object.box[2], each_object.box[3]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
                        sorted_labels.append(each_object.object_class)
                    # print(sorted_labels)
                except:
                    pass
                # Convert to QImage and emit
                self.record_queue.put(img)
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                h, w, ch = img.shape
                bytes_per_line = ch * w
                qt_img = QImage(img.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
                self.frame_ready.emit(qt_img)
                
            sorted_radar = sorted(self.radar_queue, key=lambda x: x['dist_lat'])
            if len(sorted_radar) > 0 and len(sorted_radar) == len(detected_objects):
                for idx, item in enumerate(sorted_radar):
                    sorted_radar[idx]["class"] = detected_objects[idx].object_class
                    sorted_radar[idx]["x_vision"] = detected_objects[idx].box[0]
                    sorted_radar[idx]["timestamp"] = datetime.now()
                    print(f"{idx}. ID: {item['id']}, dist_lat: {item['dist_lat']:.2f}, dist_long: {item['dist_long']:.2f}, "
                        f"vel_rel_long: {item['vel_rel_long']:.2f}, vel_rel_lat: {item['vel_rel_lat']:.2f}, "
                        f"object_class: {item['object_class']}, dynamic_prop: {item['dynamic_prop']}, rcs: {item['rcs']:.1f}, "
                        f"class: {item['class']}, x_vision: {item['x_vision']} ")

                    
                    with open(self.csv_path, 'a', newline='') as f:
                        writer = csv.DictWriter(f, fieldnames=fieldnames)
                        writer.writerow(sorted_radar[idx])
                print("/n")
            self.radar_queue=[] 
