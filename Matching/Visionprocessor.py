import cv2
import numpy as np
import time
from ultralytics import YOLO
import math 
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from calibration_module import CameraCalibration

class VisionProcessor:
    def __init__(self, calib_path):
        self.model = YOLO("/home/ubuntu/Documents/programs/CV/face.pt")  # Face detection model
        self.calibrator = CameraCalibration(calib_path)
        self.calibrator.checkerboard(7, 7, 0.04)
        self.track_id_counter = 0
        self.track_history = {}
        self.avg_face_width = 0.135  # meters

    def process_frame(self, img):
        """Process image frame and return 3D object detections"""
        vision_objects = []
        results = self.model.predict(img, verbose=False)
        
        for result in results:
            for *xyxy, conf, cls in result.boxes.data.tolist():
                if conf > 0.5:  # Confidence threshold
                    x1, y1, x2, y2 = map(int, xyxy)
                    X, Y, Z = self._calculate_3d_coordinates(
                        [x1, y1, x2, y2], 
                        img.shape[1], 
                        img.shape[0]
                    )
                    
                    track_id = self._assign_track_id(X, Y, Z)
                    vision_objects.append({
                        'track_id': track_id,
                        'x': X,
                        'y': Y,
                        'z': Z,
                        'confidence': float(conf),
                        'bbox': [x1, y1, x2, y2],
                        'class': 'face',
                        'timestamp': time.time()
                    })
        
        return vision_objects

    def _calculate_3d_coordinates(self, bbox, img_width, img_height):
        """Convert 2D bbox to 3D coordinates"""
        x_center = (bbox[0] + bbox[2]) / 2
        y_center = (bbox[1] + bbox[3]) / 2
        
        # Estimate depth
        face_width_px = bbox[2] - bbox[0]
        focal_length = self.calibrator.mtx[0,0]
        depth = (focal_length * self.avg_face_width) / face_width_px
        
        # Convert to 3D
        img_point = np.array([[x_center, y_center]], dtype=np.float32)
        undistorted = cv2.undistortPoints(img_point, self.calibrator.mtx, self.calibrator.dist)
        
        x_n = undistorted[0][0][0]
        y_n = undistorted[0][0][1]
        
        return x_n * depth, y_n * depth, depth

    def _assign_track_id(self, x, y, z, threshold=0.3):
        """Simple object tracking"""
        current_time = time.time()
        for tid, history in list(self.track_history.items()):
            if len(history) == 0:
                continue
                
            last_pos = history[-1]
            distance = math.sqrt(
                (x - last_pos['x'])**2 +
                (y - last_pos['y'])**2 +
                (z - last_pos['z'])**2
            )
            
            if distance < threshold and (current_time - last_pos['time']) < 1.0:
                self.track_history[tid].append({
                    'x': x, 'y': y, 'z': z, 
                    'time': current_time
                })
                return tid
        
        # New object
        new_id = self.track_id_counter
        self.track_id_counter += 1
        self.track_history[new_id] = [{
            'x': x, 'y': y, 'z': z,
            'time': current_time
        }]
        return new_id