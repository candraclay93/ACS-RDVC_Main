import can
import datetime
import math
import time
import threading
from collections import deque

CLASS_MAP = {0: "Point", 1: "Vehicle", 2: "Pedestrian", 3: "Bicycle"}
STATE_MAP = {0: "Initial", 1: "New", 2: "Tracked", 3: "Lost", 4: "Merged"}

class CanReaderThread(threading.Thread):
    def __init__(self, interface="can0"):
        super().__init__()
        self.interface = interface
        self.running = True
        self.objects = {}
        self.track_history = {}
        self.max_history = 5

    def connect(self):
        self.bus = can.interface.Bus(channel=self.interface, bustype="socketcan")
        print(f"[Radar] Connected to {self.interface}")

    def get_latest_objects(self):
        """Returns filtered and temporally smoothed radar objects"""
        current_objects = []
        current_time = time.time()
        
        # Apply temporal filtering
        for obj_id, history in list(self.track_history.items()):
            if len(history) == 0:
                continue
            
            # Simple moving average
            avg_x = sum(h['x'] for h in history) / len(history)
            avg_z = sum(h['z'] for h in history) / len(history)
            
            current_objects.append({
                "Object ID": obj_id,
                "Dist Lat (m)": avg_x,
                "Dist Long (m)": avg_z,
                "RCS": history[-1]["RCS"],
                "timestamp": current_time,
                "raw_data": history[-1]['raw_data']  # Keep most recent full data
            })
            
        return current_objects

    def update(self):
        """Reads and processes CAN messages"""
        msg = self.bus.recv(timeout=0.1)
        if not msg:
            return

        if msg.arbitration_id == 0x62B and len(msg.data) == 8:
            self._parse_object_data(msg.data)
        elif msg.arbitration_id == 0x62A and len(msg.data) == 8:
            self._update_objects(msg.data)

    def _parse_object_data(self, data):
        """Parse individual object CAN message"""
        object_id = data[0] + 1
        distance_long = ((data[1] << 8) | (data[2] & 0xF8)) >> 3
        distance_long = distance_long * 0.2 - 500
        
        distance_lat = ((data[2] & 0x07) << 8) | data[3]
        distance_lat = distance_lat * 0.2 - 204.2
        
        rcs = data[7] * 0.5 - 64.0

        # if rcs < 10:  # Filter low RCS objects
        obj_data = {
            "Object ID": object_id,
            "Dist Lat (m)": distance_lat,
            "Dist Long (m)": distance_long,
            "RCS": rcs,
            "timestamp": time.time()
        }

        # Update tracking history
        if object_id not in self.track_history:
            self.track_history[object_id] = deque(maxlen=self.max_history)
        
        self.track_history[object_id].append({
            'x': distance_lat,
            'z': distance_long,
            'RCS':rcs,
            'timestamp': time.time(),
            'raw_data': obj_data
        })

    def _update_objects(self, data):
        """Handle object list update message"""
        self.total_objects = data[0]
        self.cycles = (data[1] << 8) | data[2]
        # print(f"\n[Radar] Cycle {self.cycles} - {self.total_objects} objects")

    def stop(self):
        """Stop the thread and the connection"""
        self.running = False
        self.bus.shutdown()

    def run(self):
        """Override the run method to process CAN messages in a separate thread"""
        self.connect()  # Establish connection on thread start
        while self.running:
            self.update()
            time.sleep(0.05)  # Prevent CPU overload
        print("[Radar] Thread stopped.")

# Example of how to use the CanReader in your main script:

if __name__ == "__main__":
    # Create CanReader instance and start the thread
    radar_reader = CanReaderThread(interface="can0")
    radar_reader.start()

    try:
        # Example usage: get latest radar objects in a loop
        while True:
            radar_objs = radar_reader.get_latest_objects()
            
            # 👇 Print each radar object ID
            if radar_objs:
                print("\n📡 Radar Object IDs:")
                for robj in radar_objs:
                    print(f" - ID: {robj['Object ID']}, X: {robj['Dist Lat (m)']:.2f}, Z: {robj['Dist Long (m)']:.2f}, RCS: {robj['RCS']}")
            # print(radar_objects)  # Process radar objects as needed
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n🛑 Stopping radar reader thread...")
    
    # Stop the radar reader thread safely
    radar_reader.stop()
    radar_reader.join()  # Wait for the thread to finish before exiting
    print("✅ System stopped.")
