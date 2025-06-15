from src.lib.camera_module import HikvisionCamera
import cv2
from ultralytics import YOLO
from src.lib.calibration_module import CameraCalibration

# camera = HikvisionCamera()
# calib = CameraCalibration()
# ret = camera.open()
# model = YOLO("yolov8n")

# cam = HikvisionCamera()
# try:
#     cam.open()
#     cam.capture_checkerboards(save_dir="./checkerboard_images/lt1_pantry7", max_images=20, interval_sec=5)
# finally:
#     cam.close()

calib = CameraCalibration('data/images/checkerboard_images/lt1_pantry6')
h,w = 6,8
square_size = 0.03
# calib.get_calibration_img()
mtx, dist, rvecs,tvecs = calib.checkerboard(h,w,square_size)
calib.show_checkerboard_corners(scale_percent=50)

print("\nCamera Matrix (intrinsic parameters):")
print(mtx)

print("\nDistortion Coefficients:")
print(dist)

print("\nRotation Vectors (in meters):")
for i, rvec in enumerate(rvecs):
    print(f"Image {i+1}: rvec = {rvec.ravel()}")

# Show translation vectors (tvecs)
print("\nTranslation Vectors (in meters):")
for i, tvec in enumerate(tvecs):
    print(f"Image {i+1}: tvec = {tvec.ravel()}")