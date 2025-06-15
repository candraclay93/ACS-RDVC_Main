import cv2
import numpy as np
import time
from ctypes import *
import sys
from ultralytics import YOLO
import can
import math
sys.path.append("/opt/MVS/Samples/aarch64/Python/MvImport")

# Load the MVS SDK shared library
try:
    from MvCameraControl_class import *
except ImportError:
    print("[ERROR] Failed to import MVS SDK Python wrapper.")
    exit(1)

# Load the YOLOv8 face detection model
model = YOLO("/home/ubuntu/Documents/programs/CV/Module/model/face/face.pt")

class CanReaderThread():

    def __init__(self, interface="can0"):
        super().__init__()
        self.interface = interface
        self.running = True
        self.object_counter = 0
        self.total_objects = 0
        self.cycles = 0
        self.objects = {}

    def run(self):
        try:
            bus = can.interface.Bus(channel=self.interface, bustype="socketcan")
            print(f"[CanReaderThread] Listening on {self.interface}")

            while self.running:
                msg = bus.recv(timeout=0.1)
                if msg:
                    if msg.arbitration_id == 0x62B and len(msg.data) == 8:
                        self.parse_object_data(msg.data)
                    elif msg.arbitration_id == 0x62A and len(msg.data) == 8:
                        self.update_objects(msg.data)

        except Exception as e:
            print("[CanReaderThread] CAN Error:", e)

    def parse_object_data(self, data):
        object_id = data[0] + 1
        distance_long = ((data[1] << 8) | (data[2] & 0xF8)) >> 3
        distance_long = distance_long * 0.2 - 500
        
        distance_lat = ((data[2] & 0x07) << 8) | data[3]
        distance_lat = distance_lat * 0.2 - 204.2
        
        velocity_long = (data[4] << 8) | (data[5] & 0xC0)
        velocity_long = (velocity_long >> 6) * 0.25 - 128.0
        
        velocity_lat = ((data[5] & 0x3F) << 8) | (data[6] & 0xE0)
        velocity_lat = (velocity_lat >> 5) * 0.25 - 64.0
        
        obj_class = (data[6] & 0x18) >> 3
        obj_state = data[6] & 0x07
        rcs = data[7] * 0.5 - 64.0

        class_label = CLASS_MAP.get(obj_class, f"Unknown({obj_class})")
        state_label = STATE_MAP.get(obj_state, f"Unknown({obj_state})")
        if rcs < 10:
            parsed = {
                "Time": datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3],
                "Object ID": object_id,
                "Dist Long (m)": round(distance_long, 1),
                "Dist Lat (m)": round(distance_lat, 1),
                "Vel Long (m/s)": round(velocity_long, 2),
                "Vel Lat (m/s)": round(velocity_lat, 2),
                "Class": class_label,
                "State": state_label,
                "RCS (dBsm)": round(rcs, 1),
                "Distance": round(math.sqrt(distance_lat**2 + distance_long**2), 1)
            }

            self.objects[object_id] = parsed
            self.object_counter += 1

            raw_hex = ' '.join(f'{b:02X}' for b in data)
            print("\n=== NEW OBJECT DATA ===")
            print(f"RAW: ID=0x62B DATA=[{raw_hex}]")
            print("PARSED:")
            for k, v in parsed.items():
                print(f"  {k}: {v}")

    def update_objects(self, data):
        self.total_objects = data[0]
        self.cycles = (data[1] << 8) | data[2]
        
        print(f"\n=== UPDATE CYCLE {self.cycles} ===")
        print(f"Total objects: {self.total_objects}")
        
        if self.object_counter == self.total_objects:
            for obj_id, obj_data in self.objects.items():
                self.new_data.emit(obj_data)
        
        self.object_counter = 0
        self.objects.clear()

    def stop(self):
        self.running = False
        self.quit()
        self.wait()

class Face3DCoordinateCalculator:
    def __init__(self, calib_path='calibration_img'):
        from src.lib.calibration_module import CameraCalibration
        self.calibrator = CameraCalibration(calib_path)
        self.calibrator.checkerboard(7,7,0.04)  # Initialize calibration
        self.avg_face_width = 0.15  # Average face width in meters (15cm)
        
    def calculate_3d_coordinates(self, bbox, img_width, img_height):
        """Calculate 3D coordinates from face bounding box"""
        # Get face center in image coordinates
        x_center = (bbox[0] + bbox[2]) / 2
        y_center = (bbox[1] + bbox[3]) / 2
        
        # Estimate depth based on face width
        face_width_px = bbox[2] - bbox[0]
        focal_length = self.calibrator.mtx[0,0]  # Get focal length from calibration
        depth = (focal_length * self.avg_face_width) / face_width_px
        
        # Convert to 3D camera coordinates
        image_point = np.array([[x_center, y_center]], dtype=np.float32)
        undistorted_point = cv2.undistortPoints(image_point, self.calibrator.mtx, self.calibrator.dist)
        
        x_n = undistorted_point[0][0][0]
        y_n = undistorted_point[0][0][1]
        
        # Calculate 3D coordinates
        X = x_n * depth
        Y = y_n * depth
        Z = depth
        
        return X, Y, Z

def main():
    # Initialize 3D coordinate calculator
    coord_calculator = Face3DCoordinateCalculator(calib_path="data/images/checkerboard_images/lt1_pantry")
    
    # Create camera object
    cam = MvCamera()

    # Prepare device list
    deviceList = MV_CC_DEVICE_INFO_LIST()
    ret = MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, deviceList)
    if ret != 0 or deviceList.nDeviceNum == 0:
        print("❌ No Hikvision camera found!")
        return

    # Get first device
    stDeviceInfo = cast(deviceList.pDeviceInfo[0], POINTER(MV_CC_DEVICE_INFO)).contents

    # Create handle and open device
    ret = cam.MV_CC_CreateHandle(stDeviceInfo)
    if ret != 0:
        print(f"❌ Create handle failed: {ret}")
        return

    ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
    if ret != 0:
        print(f"❌ Open device failed: {ret}")
        return

    # Turn off trigger mode (free run)
    cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)

    # Start grabbing
    ret = cam.MV_CC_StartGrabbing()
    if ret != 0:
        print(f"❌ Start grabbing failed: {ret}")
        return

    # Get payload size
    stParam = MVCC_INTVALUE()
    memset(byref(stParam), 0, sizeof(MVCC_INTVALUE))
    ret = cam.MV_CC_GetIntValue("PayloadSize", stParam)
    if ret != 0:
        print("❌ Failed to get payload size")
        return

    payload_size = stParam.nCurValue
    data_buf = (c_ubyte * payload_size)()

    print("✅ Grabbing frames. Press ESC to quit.")
    while True:
        # Frame info
        stFrameInfo = MV_FRAME_OUT_INFO_EX()
        memset(byref(stFrameInfo), 0, sizeof(stFrameInfo))

        # Get one frame
        ret = cam.MV_CC_GetOneFrameTimeout(data_buf, payload_size, stFrameInfo, 1000)
        if ret == 0:
            # Convert buffer to image
            img = np.frombuffer(data_buf, dtype=np.uint8)
            
            if stFrameInfo.enPixelType == PixelType_Gvsp_Mono8:  # Grayscale
                img = img.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth))
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            elif stFrameInfo.enPixelType == PixelType_Gvsp_RGB8_Packed:  # RGB
                img = img.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth, 3))
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            else:
                print("❌ Unsupported pixel format:", stFrameInfo.enPixelType)
                continue

            # Run face detection
            results = model.predict(img, verbose=False)
            
            # Process each detection
            for result in results:
                for *xyxy, conf, cls in result.boxes.data.tolist():
                    if conf > 0.5:  # Confidence threshold
                        x1, y1, x2, y2 = map(int, xyxy)
                        
                        # Calculate 3D coordinates
                        X, Y, Z = coord_calculator.calculate_3d_coordinates(
                            [x1, y1, x2, y2],
                            stFrameInfo.nWidth,
                            stFrameInfo.nHeight
                        )
                        
                        # Print to terminal
                        print(f"X (right/left): {X:.2f} meters")
                        print(f"Y (up/down): {Y:.2f} meters")
                        print(f"Z (distance): {Z:.2f} meters")
                        print(f"Confidence: {conf:.2f}")
                        
                

    # Cleanup
    cam.MV_CC_StopGrabbing()
    cam.MV_CC_CloseDevice()
    cam.MV_CC_DestroyHandle()
    cv2.destroyAllWindows()
    print("✅ Done.")

if __name__ == "__main__":
    main()