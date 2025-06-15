import math
import time
from collections import deque

class SensorFusion:
    def __init__(self):
        self.radar_objects = {}
        self.vision_objects = {}
        self.fused_objects = []
        self.match_threshold = 0.4  # meters
        self.track_history = {}  # For temporal filtering
        self.max_history = 5     # Number of frames to keep in history

    def update_radar(self, radar_data):
        """Update radar objects with temporal filtering"""
        current_time = time.time()
        for obj in radar_data:
            obj_id = obj["Object ID"]
            
            # Initialize tracking history if new object
            if obj_id not in self.track_history:
                self.track_history[obj_id] = deque(maxlen=self.max_history)
            
            # Store current measurement
            self.track_history[obj_id].append({
                'x': obj["Dist Lat (m)"],
                'z': obj["Dist Long (m)"],
                'timestamp': current_time,
                'data': obj
            })
            
            # Apply simple moving average filter
            x_avg = sum(t['x'] for t in self.track_history[obj_id]) / len(self.track_history[obj_id])
            z_avg = sum(t['z'] for t in self.track_history[obj_id]) / len(self.track_history[obj_id])
            
            self.radar_objects[obj_id] = {
                'x': x_avg,
                'y': 0,  # Radar doesn't provide height
                'z': z_avg,
                'timestamp': current_time,
                'raw_data': obj
            }

    def update_vision(self, vision_data):
        """Update vision objects with temporal filtering"""
        current_time = time.time()
        for obj in vision_data:
            obj_id = obj.get("track_id", hash(tuple(obj.values())))  # Fallback ID if no tracking
            
            if obj_id not in self.track_history:
                self.track_history[obj_id] = deque(maxlen=self.max_history)
            
            self.track_history[obj_id].append({
                'x': obj["x"],
                'z': obj["z"],
                'timestamp': current_time,
                'data': obj
            })
            
            # Apply simple moving average filter
            x_avg = sum(t['x'] for t in self.track_history[obj_id]) / len(self.track_history[obj_id])
            z_avg = sum(t['z'] for t in self.track_history[obj_id]) / len(self.track_history[obj_id])
            
            self.vision_objects[obj_id] = {
                'x': x_avg,
                'y': obj["y"],
                'z': z_avg,
                'timestamp': current_time,
                'raw_data': obj
            }

    def fuse_data(self):
        """Match radar and vision objects with temporal consistency"""
        self.fused_objects = []
        matched_vision_ids = set()
        
        for radar_id, radar_obj in self.radar_objects.items():
            best_match = None
            min_distance = float('inf')
            
            for vision_id, vision_obj in self.vision_objects.items():
                if vision_id in matched_vision_ids:
                    continue
                
                # Calculate distance in x-z plane
                distance = math.sqrt(
                    (radar_obj['x'] - vision_obj['x'])**2 +
                    (radar_obj['z'] - vision_obj['z'])**2
                )
                
                if distance < self.match_threshold and distance < min_distance:
                    min_distance = distance
                    best_match = vision_id
            
            if best_match:
                vision_obj = self.vision_objects[best_match]
                
                # Weighted average based on sensor confidence
                radar_conf = 0.7  # Radar typically more accurate for distance
                vision_conf = 0.3
                
                fused_obj = {
                    'fusion_id': f"{radar_id}-{best_match}",
                    'x': (radar_obj['x'] * radar_conf + vision_obj['x'] * vision_conf),
                    'y': vision_obj['y'],  # Use vision's y-coordinate
                    'z': (radar_obj['z'] * radar_conf + vision_obj['z'] * vision_conf),
                    'distance': min_distance,
                    'radar_data': radar_obj['raw_data'],
                    'vision_data': vision_obj['raw_data'],
                    'timestamp': time.time()
                }
                
                self.fused_objects.append(fused_obj)
                matched_vision_ids.add(best_match)
                
                # Print matching info
                print(f"\n🔗 MATCHED OBJECT (Distance: {min_distance:.2f}m)")
                print(f"Radar ID: {radar_id} | Vision ID: {best_match}")
                print(f"Fused Position: X={fused_obj['x']:.2f}m, Y={fused_obj['y']:.2f}m, Z={fused_obj['z']:.2f}m")
        
        # Cleanup old objects
        self._cleanup_old_objects()
        return self.fused_objects

    def _cleanup_old_objects(self, max_age=1.0):
        """Remove objects older than max_age seconds"""
        current_time = time.time()
        self.radar_objects = {k: v for k, v in self.radar_objects.items() 
                             if current_time - v['timestamp'] < max_age}
        self.vision_objects = {k: v for k, v in self.vision_objects.items() 
                             if current_time - v['timestamp'] < max_age}