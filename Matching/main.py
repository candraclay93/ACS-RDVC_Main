import cv2
import numpy as np
import time
from ctypes import *
import sys
from ultralytics import YOLO
from Canreader import CanReaderThread  # Your radar reader
from Sensorfusion import SensorFusion
import math
import os
sys.path.append("/opt/MVS/Samples/aarch64/Python/MvImport")

# Load the MVS SDK shared library
try:
    from MvCameraControl_class import *
except ImportError:
    print("❌ Failed to import MVS SDK Python wrapper.")
    exit(1)

class Face3DCoordinateCalculator:
    def __init__(self, calib_path='calibration_img'):
        sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
        from calibration_module import CameraCalibration
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
    
class VisionProcessor:
    def __init__(self, calib_path):
        self.model = YOLO("/home/ubuntu/Documents/programs/CV/face.pt")
        self.coord_calculator = Face3DCoordinateCalculator(calib_path)
        self.track_id_counter = 0
        self.track_history = {}

    def process_frame(self, img, img_width, img_height):
        vision_objects = []
        results = self.model.predict(img, verbose=False)
        
        for result in results:
            for *xyxy, conf, cls in result.boxes.data.tolist():
                if conf > 0.5:  # Confidence threshold
                    x1, y1, x2, y2 = map(int, xyxy)
                    
                    # Calculate 3D coordinates
                    X, Y, Z = self.coord_calculator.calculate_3d_coordinates(
                        [x1, y1, x2, y2],
                        img_width,
                        img_height
                    )
                    
                    # Simple tracking by position (replace with proper tracker if needed)
                    track_id = self._assign_track_id(X, Y, Z)
                    
                    vision_objects.append({
                        'track_id': track_id,
                        'x': X,
                        'y': Y,
                        'z': Z,
                        'confidence': float(conf),
                        'bbox': [x1, y1, x2, y2],
                        'class': 'face'
                    })
        
        return vision_objects

    def _assign_track_id(self, x, y, z, threshold=0.5):
        """Simple tracking by position (replace with proper tracker)"""
        current_time = time.time()
        for tid, history in list(self.track_history.items()):
            last_pos = history[-1]
            distance = math.sqrt(
                (x - last_pos['x'])**2 +
                (y - last_pos['y'])**2 +
                (z - last_pos['z'])**2
            )
            if distance < threshold and (current_time - last_pos['time']) < 1.0:
                self.track_history[tid].append({'x': x, 'y': y, 'z': z, 'time': current_time})
                return tid
        
        # If no match, create new track
        new_id = self.track_id_counter
        self.track_id_counter += 1
        self.track_history[new_id] = [{'x': x, 'y': y, 'z': z, 'time': current_time}]
        return new_id

def main():
    # Initialize systems
    fusion = SensorFusion()
    radar_reader = CanReaderThread()
    vision_processor = VisionProcessor(calib_path="../checkerboard_images/lt1_pantry")
    
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

    print("✅ Starting sensor fusion. Press Ctrl+C to stop.")
    
    try:
        while True:
            # Process radar data (non-blocking)
            radar_objects = radar_reader.get_latest()  # Implement this method to get latest radar data
            if radar_objects:
                fusion.update_radar(radar_objects)
            
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

                # Process the frame with vision processor
                vision_objects = vision_processor.process_frame(
                    img, 
                    stFrameInfo.nWidth,  # Width of the frame
                    stFrameInfo.nHeight  # Height of the frame
                )
                fusion.update_vision(vision_objects)
                
                # Perform fusion
                fused_objects = fusion.fuse_data()
                
                # Optional: Process fused objects further
                for obj in fused_objects:
                    # Your custom logic here (for example, display or store fused objects)
                    pass
            
            time.sleep(0.01)  # Prevent CPU overload

    except KeyboardInterrupt:
        print("\n🛑 Stopping sensor fusion...")
    
    # Cleanup
    cam.MV_CC_StopGrabbing()
    cam.MV_CC_CloseDevice()
    cam.MV_CC_DestroyHandle()
    radar_reader.stop()
    print("✅ System stopped.")

if __name__ == "__main__":
    main()


# import time
# from Canreader import CanReaderThread
# can_reader = CanReaderThread()
# can_reader.start()

# # ... somewhere in your main loop
# time.sleep(3)
# latest_objects = can_reader.get_latest_objects()
# for obj in latest_objects:
#     print("👉 Tracked Object:", obj)

# # On shutdown
# can_reader.stop()
# can_reader.join()
