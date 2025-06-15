import cv2
import numpy as np
import time
from collections import OrderedDict
from ctypes import *
import sys
from ultralytics import YOLO
from calibration_module import CameraCalibration

sys.path.append("/opt/MVS/Samples/aarch64/Python/MvImport")

try:
    from MvCameraControl_class import *
except ImportError:
    print("Failed to import MVS SDK Python wrapper.")
    exit(1)

AVG_PERSON_HEIGHT_M = 1.7
AVG_FACE_HEIGHT_M = 0.24

model = YOLO("face.pt")
model.to("cuda")
calib = CameraCalibration('data/images/checkerboard_images/lt1_pantry6')

h, w = 7, 7
square_size = 0.04
scale = 75
print("📷 Starting camera calibration...")
mtx, dist, rvecs, tvecs = calib.checkerboard(h, w, square_size, scale_percent=scale)

fy = mtx[1, 1]

def estimate_depth(bbox_height_px, fy, mode="person"):
    if bbox_height_px == 0:
        return 0
    if mode == "person":
        return (fy * AVG_PERSON_HEIGHT_M) / bbox_height_px
    elif mode == "wajah":
        return (fy * AVG_FACE_HEIGHT_M) / bbox_height_px

def pixel_to_camera_coords(u, v, depth, camera_matrix, dist_coeffs):
    image_point = np.array([[[u, v]]], dtype=np.float32)
    normalized = cv2.undistortPoints(image_point, camera_matrix, dist_coeffs)
    x_n, y_n = normalized[0][0]
    X_c = x_n * depth
    Y_c = y_n * depth
    return np.array([X_c, Y_c, depth])

def process_frame(img, start_time, log_file=None): # Removed 'self' parameter
    """
    Processes a frame to detect objects, draw bounding boxes, and log information.

    Args:
        img: The input frame as a NumPy array (BGR format for OpenCV).
        start_time: The time when the video capture started, used for timestamping.
        log_file: A file object to write the detection logs to. If None, prints to console.

    Returns:
        The processed frame with bounding boxes drawn (BGR format).
        The list of detections.
    """
    # Access global 'model' directly
    results = model.predict(img, verbose=False) 
    detections = []
    
    # Ensure img is writable if it's not already
    if not img.flags['WRITEABLE']:
        img = img.copy()

    for result in results:
        for *xyxy, conf, cls in result.boxes.data.tolist():
            x1, y1, x2, y2 = map(int, xyxy)
            center_u = (x1 + x2) // 2
            center_v = (y1 + y2) // 2
            bbox_h = abs(y2 - y1)
            class_id = int(cls)

            if bbox_h == 0:
                continue  # Skip zero-height boxes

            # Access global 'model.names' directly
            label = model.names[class_id].lower() 
            mode = "wajah" if "wajah" in label else "person"
            
            # Access global 'fy', 'mtx', 'dist' directly
            depth = estimate_depth(bbox_h, fy, mode) 
            world_coord = pixel_to_camera_coords(center_u, center_v, depth, mtx, dist)

            detections.append({
                'center_x': center_u,
                'cx_intrinsic' : mtx[0, 2],
                'cy_intrinsic' : mtx[1, 2],
                'bbox': (x1, y1, x2 - x1, bbox_h),
                'depth_m': round(depth, 2),
                'class': label,
                'camera_coords': world_coord,
                'timestamp_sec': round(time.time() - start_time, 3)
            })

            # Draw the bounding box on the frame (assuming img is BGR)
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Green rectangle, 2 pixels thick
            cv2.putText(img, f"{label} {round(conf, 2)}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


    if detections:
        detections.sort(key=lambda d: d['center_x'])
        sorted_faces = OrderedDict((idx + 1, det) for idx, det in enumerate(detections))

        log_output = ""
        for idx, det in sorted_faces.items():
            log_output += f"ID {idx}: {det['class'].capitalize()}\n"
            log_output += f"  - BBox: {det['bbox']} (center X: {det['center_x']})\n"
            log_output += f"Intrinsic Principal Point (cx, cy): {det['cx_intrinsic']:.2f}, {det['cy_intrinsic']:.2f}\n"
            log_output += f"  - Depth: {det['depth_m']} m\n"
            log_output += f"  - Camera Coords: X={det['camera_coords'][0]:.2f}, Y={det['camera_coords'][1]:.2f}, Z={det['camera_coords'][2]:.2f}\n"
            log_output += f"  - Timestamp: {det['timestamp_sec']} sec\n\n"
        log_output += "================================================\n"

        if log_file:
            log_file.write(log_output)
        else:
            print(log_output)

    return img, detections # Returns BGR image and detections

def infer_from_video(video_path):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print("Failed to open video file.")
        return

    start_time = time.time()
    print("Video inference started...\n")

    with open("data/detections_log.txt", "w") as log_file:  # overwrite existing file
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            process_frame(frame, start_time, log_file)

    cap.release()
    print("✅ Video inference finished")

def infer_from_camera():
    cam = MvCamera()
    deviceList = MV_CC_DEVICE_INFO_LIST()
    ret = MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, deviceList)
    if ret != 0 or deviceList.nDeviceNum == 0:
        print("No Hikvision camera found!")
        return

    stDeviceInfo = cast(deviceList.pDeviceInfo[0], POINTER(MV_CC_DEVICE_INFO)).contents
    ret = cam.MV_CC_CreateHandle(stDeviceInfo)
    if ret != 0:
        print(f"Create handle failed: {ret}")
        return

    ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
    if ret != 0:
        print(f"Open device failed: {ret}")
        return

    cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
    cam.MV_CC_StartGrabbing()

    stParam = MVCC_INTVALUE()
    memset(byref(stParam), 0, sizeof(MVCC_INTVALUE))
    ret = cam.MV_CC_GetIntValue("PayloadSize", stParam)
    if ret != 0:
        print("Failed to get payload size")
        return

    payload_size = stParam.nCurValue
    data_buf = (c_ubyte * payload_size)()
    print("📡 Live detection started... Press Ctrl+C to stop.\n")
    start_time = time.time()

    try:
        while True:
            stFrameInfo = MV_FRAME_OUT_INFO_EX()
            memset(byref(stFrameInfo), 0, sizeof(stFrameInfo))
            ret = cam.MV_CC_GetOneFrameTimeout(data_buf, payload_size, stFrameInfo, 1000)
            if ret != 0:
                print("Frame grab failed")
                continue

            img = np.frombuffer(data_buf, dtype=np.uint8)
            if stFrameInfo.enPixelType == PixelType_Gvsp_RGB8_Packed:
                img = img.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth, 3))
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif stFrameInfo.enPixelType == PixelType_Gvsp_Mono8:
                img = img.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth))
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            else:
                print("Unsupported pixel format:", stFrameInfo.enPixelType)
                continue

            process_frame(img, start_time)

    except KeyboardInterrupt:
        print("Interrupted by user")

    cam.MV_CC_StopGrabbing()
    cam.MV_CC_CloseDevice()
    cam.MV_CC_DestroyHandle()
    print("Camera released")

if __name__ == "__main__":
    USE_VIDEO_FILE = True
    VIDEO_PATH = "data/vid/garage_lt1_test.mp4"

    if USE_VIDEO_FILE:
        infer_from_video(VIDEO_PATH)
    else:
        infer_from_camera()
