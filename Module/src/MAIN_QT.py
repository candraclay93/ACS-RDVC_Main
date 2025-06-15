import multiprocessing as mp
import signal
import sys
from lib.CANBUS_reader import can_listener

processes = [] # Keep track of processes

def terminate_processes(signal, frame):
    """
    Signal handler to terminate child processes.
    """
    print("Terminating processes...")
    for p in processes:
        if p.is_alive():
            p.terminate()
            p.join()
    sys.exit(0)

if __name__ == "__main__":
    mp.set_start_method('spawn')  # safer for GUI multiprocessing on Linux
    
    from lib.CANBUS_viewer import run_gui
    from lib.CAM_reader import CameraFeed
    from lib.video_manager import VideoRecorder
    
    # Set up the signal handler for Ctrl+C and other termination signals
    signal.signal(signal.SIGINT, terminate_processes)
    signal.signal(signal.SIGTERM, terminate_processes)
    record_queue = mp.Queue(maxsize=2)
    recorder = VideoRecorder(record_queue)
    queue = mp.Queue()
    can_process = mp.Process(target=can_listener, args=(queue,))
    record_process = mp.Process(target=recorder.record, args=())
    processes.append(can_process) #add to process list
    processes.append(record_process)
    can_process.start()
    record_process.start()

    # start the camera feed process
    #camera_feed_process = mp.Process(target=CameraFeed)
    #processes.append(camera_feed_process) # add to process list
    #camera_feed_process.start()

    try:
        run_gui(queue, record_queue)
    except Exception as e:
        print(f"Exception in run_gui: {e}") #catch exceptions
        terminate_processes(None, None) # Terminate on error
    finally: #make sure processes are cleaned
        can_process.join()
        record_process.join()
        #camera_feed_process.join()

    print("Main process exiting.")
