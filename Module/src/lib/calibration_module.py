# calibration_module.py
import cv2
import numpy as np
import os
import glob
from ultralytics import YOLO
from lib.camera_module import HikvisionCamera


class CameraCalibration:
    def __init__(self, calib_path: str = 'calibration_img'):
        self.calib_path = calib_path
        self.mtx = None
        self.dist = None
        self.tvecs = None
        self.rvecs = None
        self.h = 6
        self.w = 9

    def checkerboard(self, h: int = 6, w: int = 8, square_size: float = 0.03,scale_percent:int=50):
        self.h = h
        self.w = w
        BOARD = (h, w)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        objpoints = []
        imgpoints = []
        objp = np.zeros((1, BOARD[0] * BOARD[1], 3), np.float32)
        objp[0, :, :2] = np.mgrid[0:BOARD[0], 0:BOARD[1]].T.reshape(-1, 2)
        objp *= square_size

        images = glob.glob(f'{self.calib_path}/*.jpg')
        for fname in images:
            img = cv2.imread(fname)
            width = int(img.shape[1] * scale_percent / 100)
            height = int(img.shape[0] * scale_percent / 100)
            img = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            ret, corners = cv2.findChessboardCorners(
                gray, BOARD,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
            )
            if ret:
                objpoints.append(objp)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners2)

        ret, self.mtx, self.dist, self.rvecs, self.tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, gray.shape[::-1], None, None
        )

        return self.mtx, self.dist, self.rvecs, self.tvecs

    def pixel_to_camera_coords(self, u: int, v: int, depth: float):
        image_point = np.array([[u, v]], dtype=np.float32)
        undistorted = cv2.undistortPoints(image_point, self.mtx, self.dist)
        x_n, y_n = undistorted[0][0]
        X_c = x_n * depth
        Y_c = y_n * depth
        Z_c = depth
        return np.array([X_c, Y_c, Z_c])

    def detect_and_project(self, model_path="yolov8s.pt", avg_person_height=1.7):
        cam = HikvisionCamera()
        model = YOLO(model_path)

        cam.open()
        img = cam.grab_frame()
        # cam.close()

        results = model.predict(img, verbose=False)
        person_coords = []

        for result in results:
            for *xyxy, conf, cls in result.boxes.data.tolist():
                class_id = int(cls)
                label = model.names[class_id]
                if label == "person":
                    x1, y1, x2, y2 = map(int, xyxy)
                    bbox_height = abs(y2 - y1)

                    # Estimate depth using real height / pixel height
                    estimated_depth = (self.mtx[1, 1] * avg_person_height) / bbox_height

                    center_u = (x1 + x2) // 2
                    center_v = (y1 + y2) // 2

                    world_coord = self.pixel_to_camera_coords(center_u, center_v, estimated_depth)

                    person_coords.append({
                        'pixel': (center_u, center_v),
                        'bbox_height_px': bbox_height,
                        'depth_m': estimated_depth,
                        'world': world_coord
                    })

                    print(f"🧍 Person @ pixel ({center_u},{center_v}) | depth ~ {estimated_depth:.2f} m | world: {world_coord}")

        return person_coords
    
    def show_checkerboard_corners(self, scale_percent=100):
        """
        Detect and visualize checkerboard corners on calibration images with optional resizing.
        Args:
            scale_percent: Resize scale percentage (e.g., 50 for 50%)
        """
        detect_path = os.path.join(self.calib_path, "detect_corner")
        os.makedirs(detect_path, exist_ok=True)

        images = glob.glob(f'{self.calib_path}/*.jpg')
        if not images:
            print(f"No images found in {self.calib_path}")
            return

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        board_size = (self.h, self.w)

        for fname in images:
            img = cv2.imread(fname)

            if scale_percent != 100:
                width = int(img.shape[1] * scale_percent / 100)
                height = int(img.shape[0] * scale_percent / 100)
                img = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            ret, corners = cv2.findChessboardCorners(
                gray, board_size,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
            if ret:
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                img_marked = cv2.drawChessboardCorners(img.copy(), board_size, corners2, ret)
                save_name = os.path.join(detect_path, os.path.basename(fname))
                cv2.imwrite(save_name, img_marked)
                print(f"[SUCCESS] Saved with corners: {save_name}")
            else:
                print(f"[ERROR] No corners detected in {os.path.basename(fname)}")   


           

    def get_calibration_img(self,max_img:int=15,capture_delay:int=2):
        os.makedirs(self.calib_path, exist_ok=True)
        img_counter = 0
        cam = HikvisionCamera()
        # try:
        cam.open()
        try:
            img = cam.grab_frame()
        except Exception as e:
            print(f"[ERROR] Camera init failed: {e}")
            
        print(f"[CAP] Capturing {max_img} checkerboard images every {capture_delay} sec.")
   