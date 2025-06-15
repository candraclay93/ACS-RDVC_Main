import cv2
from datetime import datetime, timedelta
import numpy as np
from multiprocessing import Queue
import os
import time

class VideoRecorder:
    def __init__(self, input_queue: Queue):
        self.input_queue = input_queue
        self.fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # MP4 codec
        self.fps = 15.0  # Frames per second
        self.frame_size = (640, 480)  # Default frame size

    def record(self):
        while True:
            start_time = datetime.now()
            end_time = start_time + timedelta(minutes=5)
            log_path = f"../../log"
            os.makedirs(log_path, exist_ok=True)
            filename = f"{log_path}/{start_time.strftime('%Y%m%d-%H%M%S')}_{end_time.strftime('%Y%m%d-%H%M%S')}.mp4"
            
            out = None  # Initialize writer
            
            while datetime.now() < end_time:
                if not self.input_queue.empty():
                    frame = self.input_queue.get()
                    if frame is not None:
                        if out is None:  # Initialize writer only once per file
                            self.frame_size = (frame.shape[1], frame.shape[0])
                            out = cv2.VideoWriter(filename, self.fourcc, self.fps, self.frame_size)
                        out.write(frame)
                time.sleep(0.05)  # Prevent CPU overload
            
            if out is not None:
                out.release()  # Release writer after recording
                print(f"Saved: {filename}")
