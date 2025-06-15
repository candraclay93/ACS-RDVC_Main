import time
import math
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from Canreader import CanReaderThread
from Visionprocessor import VisionProcessor
from camera_module import HikvisionCamera  # Your camera interface

class SensorFusionSystem:
    def __init__(self, calib_path):
        self.radar = CanReaderThread()
        self.camera = HikvisionCamera()
        self.vision = VisionProcessor(calib_path)
        self.fusion_results = []
        self.match_threshold = 0.4  # meters

    def run(self):
        try:
            self.radar.connect()
            self.camera.open()
            
            print("✅ Sensor fusion started. Press Ctrl+C to stop")
            while True:
                # Process radar
                self.radar.update()
                radar_objs = self.radar.get_latest_objects()
                
                # Process camera
                img = self.camera.grab_frame()
                # if ret:
                vision_objs = self.vision.process_frame(img)
                # print(f"RADAR OBJS : {radar_objs}")
                print(f"VISION OBJS :{vision_objs}")

                # if len(vision_objs)>0:
                #     for robj in radar_objs:
                #         rz = robj["Dist Long (m)"]  # Z-axis from radar
                #         print("================================================================")
                #         for vobj in vision_objs:
                #             vz = vobj["z"]  # Z-axis from vision
                #             z_diff = abs(rz - vz)
                #             if z_diff < 5:

                #                 print(f"Radar ID{robj['Object ID']} (RCS: {robj['RCS']})  vs Vision ID {vobj['track_id']} ")
                #                 print(f"Radar ID depth: {rz} ; Vision ID depth: {vz}")
                #                 print(f"Z Difference = {z_diff:.2f} m")


                self._fuse_data(radar_objs, vision_objs)
                
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\n🛑 Stopping system...")
        finally:
            self.radar.stop()
            self.camera.close()
            print("✅ System stopped")

    def _fuse_data(self, radar_objects, vision_objects):
        """Match radar and vision detections"""
        matched_pairs = []
        matched_vision_ids = set()
        
        for robj in radar_objects:
            best_match = None
            min_distance = float('inf')
            
            for vobj in vision_objects:
                if vobj['track_id'] in matched_vision_ids:
                    continue
                
                # Calculate x-z plane distance (ignore height y)
                distance = math.sqrt(
                    (abs(robj["Dist Long (m)"]**2 - vobj['z']**2))
                )
                
                if distance < self.match_threshold and distance < min_distance:
                    min_distance = distance
                    best_match = vobj
            
            if best_match:
                # Weighted average (70% radar, 30% vision)
                fused_x = robj["Dist Lat (m)"] * 0.7 + best_match['x'] * 0.3
                fused_z = robj["Dist Long (m)"] * 0.7 + best_match['z'] * 0.3
                
                fused_obj = {
                    'fusion_id': f"R{robj['Object ID']}-V{best_match['track_id']}",
                    'x': fused_x,
                    'y': best_match['y'],  # Use vision's height
                    'z': fused_z,
                    'distance': min_distance,
                    'radar_data': robj,
                    'vision_data': best_match,
                    'timestamp': time.time()
                }
                
                self.fusion_results.append(fused_obj)
                matched_vision_ids.add(best_match['track_id'])
                
                # Print results
                print(f"\n🔗 Fused Object {fused_obj['fusion_id']}")
                print(f"Position: X={fused_x:.2f}m, Y={best_match['y']:.2f}m, Z={fused_z:.2f}m")
                print(f"Match Distance: {min_distance:.2f}m")
                print(f"Radar RCS: {robj.get('RCS (dBsm)', 'N/A')} dBsm")
                print(f"Vision Confidence: {best_match['confidence']:.2f}")

    def match_vision_to_radar(vision_objs, radar_objs, z_threshold=0.8, time_threshold=1.0):
        matches = []

        for vobj in vision_objs:
            vz = vobj['z']
            vtime = vobj['timestamp']
            best_match = None
            min_z_diff = float('inf')

            for robj in radar_objs:
                rz = robj['Dist Long (m)']
                rtime = robj['timestamp']
                z_diff = abs(vz - rz)
                time_diff = abs(vtime - rtime)

                if z_diff <= z_threshold and time_diff <= time_threshold:
                    if z_diff < min_z_diff:
                        min_z_diff = z_diff
                        best_match = robj

            if best_match:
                matches.append({
                    "track_id": vobj['track_id'],
                    "vision_z": vz,
                    "radar_id": best_match['Object ID'],
                    "radar_z": best_match['Dist Long (m)'],
                    "z_diff": min_z_diff,
                    "vision_time": vobj['timestamp'],
                    "radar_time": best_match['timestamp'],
                    "time_diff": abs(vobj['timestamp'] - best_match['timestamp']),
                    "radar_rcs": best_match['RCS']
                })

        return matches
if __name__ == "__main__":
    fusion_system = SensorFusionSystem(calib_path="../checkerboard_images/lt1_pantry")
    fusion_system.run()