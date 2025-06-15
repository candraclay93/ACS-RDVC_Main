# camera_module.py

import numpy as np
from ctypes import *
import sys
import os
import time
import cv2
sys.path.append("/opt/MVS/Samples/aarch64/Python/MvImport")

try:
    from MvCameraControl_class import *
except ImportError:
    print("[ERROR] Failed to import MVS SDK Python wrapper.")
    exit(1)


class HikvisionCamera:
    def __init__(self):
        self.cam = MvCamera()
        self.payload_size = 0
        self.is_open = False

    def open(self):
        device_list = MV_CC_DEVICE_INFO_LIST()
        ret = MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, device_list)
        if ret != 0 or device_list.nDeviceNum == 0:
            raise Exception("[ERROR] No Hikvision camera found!")

        device_info = cast(device_list.pDeviceInfo[0], POINTER(MV_CC_DEVICE_INFO)).contents

        ret = self.cam.MV_CC_CreateHandle(device_info)
        if ret != 0:
            raise Exception("[ERROR] Create handle failed")

        ret = self.cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        if ret != 0:
            raise Exception("[ERROR] Open device failed")

        self.cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)

        ret = self.cam.MV_CC_StartGrabbing()
        if ret != 0:
            raise Exception("[ERROR] Start grabbing failed")

        # Get payload size
        stParam = MVCC_INTVALUE()
        memset(byref(stParam), 0, sizeof(MVCC_INTVALUE))
        ret = self.cam.MV_CC_GetIntValue("PayloadSize", stParam)
        if ret != 0:
            raise Exception("[ERROR] Failed to get payload size")
        self.payload_size = stParam.nCurValue

        self.is_open = True
        print("[SUCCESS] Camera opened and grabbing started.")

    def grab_frame(self, timeout=1000):
        """Grab one frame and return as a NumPy array."""
        if not self.is_open:
            raise Exception("Camera is not open. Call open() first.")

        data_buf = (c_ubyte * self.payload_size)()
        stFrameInfo = MV_FRAME_OUT_INFO_EX()
        memset(byref(stFrameInfo), 0, sizeof(stFrameInfo))

        ret = self.cam.MV_CC_GetOneFrameTimeout(data_buf, self.payload_size, stFrameInfo, timeout)
        if ret != 0:
            return None

        img = np.frombuffer(data_buf, dtype=np.uint8)

        if stFrameInfo.enPixelType == PixelType_Gvsp_Mono8:
            img = img.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth))
        elif stFrameInfo.enPixelType == PixelType_Gvsp_RGB8_Packed:
            img = img.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth, 3))
            img = img[..., ::-1]  # Convert RGB to BGR
        else:
            print(f"[WARN] Unsupported pixel format: {stFrameInfo.enPixelType}")
            return None

        return img
    
    def capture_checkerboards(self, save_dir="../../data/images/checkerboard_images", max_images=20, interval_sec=2):
        """Capture checkerboard images with countdown between captures."""
        if not self.is_open:
            raise Exception("Camera must be opened first.")

        os.makedirs(save_dir, exist_ok=True)
        print(f"[REC] Capturing {max_images} images every {interval_sec} seconds into '{save_dir}'")
        
        img_count = 0

        try:
            while img_count < max_images:
                if img_count % 5:
                    print("==================")
                # Countdown before capture
                for remaining in range(interval_sec, 0, -1):
                    print(f"⏳ Capture in {remaining} second(s)...", end='\r')
                    # print("==================")
                    time.sleep(1)

                frame = self.grab_frame()
                if frame is None:
                    print("[WARN] Failed to grab frame, retrying...")
                    continue

                filename = os.path.join(save_dir, f"checkerboard_{img_count:02d}.jpg")
                cv2.imwrite(filename, frame)
                print(f"[SUCCESS] Saved {filename}")
                img_count += 1

        except KeyboardInterrupt:
            print("\nCapture interrupted by user.")

        print(f"\nCapture completed. {img_count} images saved to '{save_dir}'.")

    def record_video(self, filename="output.avi"):
        """Continuously record video until Ctrl+C is pressed (no FPS constraint)."""
        if not self.is_open:
            raise Exception("Camera must be opened first.")

        # Get frame to determine resolution
        frame = self.grab_frame()
        if frame is None:
            raise Exception("[ERROR] Unable to grab frame to determine resolution.")

        height, width = frame.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        video_writer = cv2.VideoWriter(filename, fourcc,10, (width, height))  # 0 FPS = no constraint

        if not video_writer.isOpened():
            raise Exception("[ERROR] Failed to open video writer.")

        print(f"[REC] Recording video to '{filename}' until Ctrl+C is pressed...")

        frames_written = 0

        try:
            while True:
                frame = self.grab_frame()
                if frame is None:
                    print("⚠️ Frame grab failed, skipping.")
                    continue

                if len(frame.shape) == 2:
                    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

                video_writer.write(frame)
                frames_written += 1

        except KeyboardInterrupt:
            print("\n[REC] Recording stopped by user.")

        video_writer.release()
        print(f"[SUCCESS] Video saved: {filename} ({frames_written} frames).")



    def close(self):
        if self.is_open:
            self.cam.MV_CC_StopGrabbing()
            self.cam.MV_CC_CloseDevice()
            self.cam.MV_CC_DestroyHandle()
            self.is_open = False
            print("[SUCCESS] Camera closed.")


