import cv2 
from lib.CAM_reader import HikvisionCamera, Detection
import time

face_model = "../../model/face/face.pt"
def main():
    """
    Main function to capture and display video from a Hikvision camera using cv2.imshow.
    """
    try:
        hik_cam = HikvisionCamera()
        hik_cam.open()
        cam = Detection(face_model)
    except Exception as e:
        print(f"Error: {e}")
        return  # Exit if camera initialization fails

    try:
        while True:
            frame = hik_cam.grab_frame()
            # frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
            frame,det = cam.process_frame(frame)
            if frame is not None:
                frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
                cv2.imshow("Hikvision Camera Feed", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
                    break
            else:
                print("Error grabbing frame")
                time.sleep(0.1) # prevent a tight loop
    except Exception as e:
        print(f"Error during video capture or display: {e}")
    finally:
        hik_cam.close()
        cv2.destroyAllWindows()  # Ensure window is closed

if __name__ == "__main__":
    main()