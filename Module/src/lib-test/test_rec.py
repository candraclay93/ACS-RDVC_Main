
from src.lib.camera_module import HikvisionCamera
if __name__ == "__main__":
    cam = HikvisionCamera()
    try:
        cam.open()
        cam.record_video(filename="data/vid/pantry_lt1.mp4")
    finally:
        cam.close()
