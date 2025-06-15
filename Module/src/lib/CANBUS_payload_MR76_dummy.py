import can
import time

# Initialize CAN bus (virtual for testing)
bus = can.Bus(channel='vcan0', interface='socketcan')

def encode_frame(obj_id, dist_long, dist_lat, vel_long, vel_lat, obj_class, dyn_prop, rcs):
    frame = [0] * 8

    # Encode object ID
    frame[0] = obj_id & 0xFF

    # Encode distance longitudinal (depth)
    dist_long_raw = int((dist_long + 500) / 0.2)
    dist_long_raw = max(0, min(dist_long_raw, 0x1FFF))  # 13 bits
    frame[1] = (dist_long_raw >> 5) & 0xFF
    frame[2] = ((dist_long_raw & 0x1F) << 3)

    # Encode distance lateral (camera X coordinate)
    dist_lat_raw = int((dist_lat + 204.6) / 0.2)
    dist_lat_raw = max(0, min(dist_lat_raw, 0x7FF))  # 11 bits
    frame[2] |= (dist_lat_raw >> 8) & 0x07
    frame[3] = dist_lat_raw & 0xFF

    # Encode velocity longitudinal (depth direction)
    vel_long_raw = int((vel_long + 128.0) / 0.25)
    vel_long_raw = max(0, min(vel_long_raw, 0x3FFF))  # 14 bits
    frame[4] = (vel_long_raw >> 6) & 0xFF
    frame[5] = ((vel_long_raw & 0x3F) << 2) & 0xFC

    # Encode velocity lateral (camera X direction)
    vel_lat_raw = int((vel_lat + 64.0) / 0.25)
    vel_lat_raw = max(0, min(vel_lat_raw, 0x7FF))  # 11 bits
    frame[5] |= (vel_lat_raw >> 8) & 0x03
    frame[6] = ((vel_lat_raw & 0xFF) << 5) & 0xE0

    # Object class and dynamic property
    frame[6] |= ((obj_class & 0x03) << 3)
    frame[6] |= (dyn_prop & 0x07)

    # Encode RCS
    rcs_raw = int((rcs + 64.0) / 0.5)
    rcs_raw = max(0, min(rcs_raw, 0xFF))
    frame[7] = rcs_raw

    return [b & 0xFF for b in frame]

# Provided face detection data
face_data = [{'cam_coord': 0.06, 'depth': 1.31, 'ID': 1, 'timestamp': 5.714},
 {'cam_coord': 1.14, 'depth': 2.15, 'ID': 2, 'timestamp': 5.714},
 {'cam_coord': 1.19, 'depth': 1.21, 'ID': 1, 'timestamp': 5.83},
 {'cam_coord': 1.18, 'depth': 1.16, 'ID': 1, 'timestamp': 5.947},
 {'cam_coord': 1.12, 'depth': 1.21, 'ID': 1, 'timestamp': 6.062},
 {'cam_coord': 1.13, 'depth': 2.05, 'ID': 1, 'timestamp': 6.179},
 {'cam_coord': 1.11, 'depth': 2.01, 'ID': 1, 'timestamp': 6.295},
 {'cam_coord': 1.1, 'depth': 1.99, 'ID': 1, 'timestamp': 6.411},
 {'cam_coord': 1.08, 'depth': 1.95, 'ID': 1, 'timestamp': 6.527},
 {'cam_coord': 1.1, 'depth': 1.99, 'ID': 1, 'timestamp': 6.643},
 {'cam_coord': 1.12, 'depth': 2.04, 'ID': 1, 'timestamp': 6.76},
 {'cam_coord': 1.19, 'depth': 2.19, 'ID': 1, 'timestamp': 6.876},
 {'cam_coord': 1.19, 'depth': 2.15, 'ID': 1, 'timestamp': 6.992},
 {'cam_coord': 1.18, 'depth': 2.18, 'ID': 1, 'timestamp': 7.108},
 {'cam_coord': 1.11, 'depth': 2.19, 'ID': 1, 'timestamp': 7.224},
 {'cam_coord': 1.09, 'depth': 2.18, 'ID': 1, 'timestamp': 7.341},
 {'cam_coord': 1.05, 'depth': 2.18, 'ID': 1, 'timestamp': 7.457},
 {'cam_coord': 0.85, 'depth': 3.24, 'ID': 1, 'timestamp': 7.573},
 {'cam_coord': 0.87, 'depth': 2.0, 'ID': 2, 'timestamp': 7.573},
 {'cam_coord': 0.78, 'depth': 3.22, 'ID': 1, 'timestamp': 7.689},
 {'cam_coord': 0.87, 'depth': 2.16, 'ID': 2, 'timestamp': 7.689},
 {'cam_coord': 0.72, 'depth': 3.37, 'ID': 1, 'timestamp': 7.805},
 {'cam_coord': 0.82, 'depth': 2.07, 'ID': 2, 'timestamp': 7.806},
 {'cam_coord': 0.65, 'depth': 3.37, 'ID': 1, 'timestamp': 7.922},
 {'cam_coord': 0.65, 'depth': 3.35, 'ID': 1, 'timestamp': 8.038},
 {'cam_coord': 0.67, 'depth': 3.39, 'ID': 1, 'timestamp': 8.154},
 {'cam_coord': 0.66, 'depth': 3.37, 'ID': 1, 'timestamp': 8.27},
 {'cam_coord': 0.79, 'depth': 2.21, 'ID': 2, 'timestamp': 8.271},
 {'cam_coord': 0.66, 'depth': 3.37, 'ID': 1, 'timestamp': 8.387},
 {'cam_coord': 0.78, 'depth': 2.14, 'ID': 2, 'timestamp': 8.387},
 {'cam_coord': 0.66, 'depth': 3.43, 'ID': 1, 'timestamp': 8.503},
 {'cam_coord': 0.87, 'depth': 2.21, 'ID': 2, 'timestamp': 8.503},
 {'cam_coord': 0.76, 'depth': 3.45, 'ID': 1, 'timestamp': 8.62},
 {'cam_coord': 0.97, 'depth': 2.11, 'ID': 2, 'timestamp': 8.619},
 {'cam_coord': 1.02, 'depth': 3.59, 'ID': 1, 'timestamp': 8.736},
 {'cam_coord': 1.04, 'depth': 2.16, 'ID': 2, 'timestamp': 8.736},
 {'cam_coord': 1.0, 'depth': 3.12, 'ID': 1, 'timestamp': 8.852},
 {'cam_coord': 1.07, 'depth': 2.21, 'ID': 2, 'timestamp': 8.852},
 {'cam_coord': 1.01, 'depth': 3.17, 'ID': 1, 'timestamp': 8.968},
 {'cam_coord': 1.08, 'depth': 2.11, 'ID': 2, 'timestamp': 8.968},
 {'cam_coord': 1.05, 'depth': 3.41, 'ID': 1, 'timestamp': 9.084},
 {'cam_coord': 1.08, 'depth': 1.96, 'ID': 2, 'timestamp': 9.084},
 {'cam_coord': 1.03, 'depth': 3.49, 'ID': 1, 'timestamp': 9.201},
 {'cam_coord': 1.08, 'depth': 1.96, 'ID': 2, 'timestamp': 9.201},
 {'cam_coord': 0.96, 'depth': 3.47, 'ID': 1, 'timestamp': 9.317},
 {'cam_coord': 1.08, 'depth': 1.97, 'ID': 2, 'timestamp': 9.317},
 {'cam_coord': 1.0, 'depth': 3.53, 'ID': 1, 'timestamp': 9.433},
 {'cam_coord': 1.1, 'depth': 1.98, 'ID': 2, 'timestamp': 9.433},
 {'cam_coord': 1.06, 'depth': 1.91, 'ID': 1, 'timestamp': 9.549},
 {'cam_coord': 1.46, 'depth': 3.53, 'ID': 1, 'timestamp': 9.665},
 {'cam_coord': 1.0, 'depth': 1.82, 'ID': 2, 'timestamp': 9.665},
 {'cam_coord': 1.59, 'depth': 3.62, 'ID': 1, 'timestamp': 9.783},
 {'cam_coord': 0.99, 'depth': 1.8, 'ID': 2, 'timestamp': 9.783},
 {'cam_coord': 1.46, 'depth': 3.35, 'ID': 1, 'timestamp': 9.899},
 {'cam_coord': 0.97, 'depth': 1.76, 'ID': 2, 'timestamp': 9.899},
 {'cam_coord': 1.46, 'depth': 3.35, 'ID': 1, 'timestamp': 10.015},
 {'cam_coord': 1.01, 'depth': 1.89, 'ID': 2, 'timestamp': 10.015},
 {'cam_coord': 1.01, 'depth': 1.82, 'ID': 1, 'timestamp': 10.245},
 {'cam_coord': 1.01, 'depth': 1.83, 'ID': 1, 'timestamp': 10.362},
 {'cam_coord': 1.06, 'depth': 1.96, 'ID': 1, 'timestamp': 10.478},
 {'cam_coord': 1.15, 'depth': 2.15, 'ID': 1, 'timestamp': 10.594},
 {'cam_coord': 1.19, 'depth': 2.23, 'ID': 1, 'timestamp': 10.71},
 {'cam_coord': 1.25, 'depth': 2.38, 'ID': 1, 'timestamp': 10.827},
 {'cam_coord': 1.23, 'depth': 2.45, 'ID': 1, 'timestamp': 10.943},
 {'cam_coord': 1.14, 'depth': 2.39, 'ID': 1, 'timestamp': 11.059},
 {'cam_coord': 1.71, 'depth': 3.09, 'ID': 2, 'timestamp': 11.059},
 {'cam_coord': 1.17, 'depth': 2.45, 'ID': 1, 'timestamp': 11.175},
 {'cam_coord': 1.75, 'depth': 3.15, 'ID': 2, 'timestamp': 11.175},
 {'cam_coord': 1.22, 'depth': 2.54, 'ID': 1, 'timestamp': 11.291},
 {'cam_coord': 1.78, 'depth': 3.2, 'ID': 2, 'timestamp': 11.291},
 {'cam_coord': 1.18, 'depth': 2.56, 'ID': 1, 'timestamp': 11.522},
 {'cam_coord': 1.72, 'depth': 3.1, 'ID': 2, 'timestamp': 11.522},
 {'cam_coord': 1.01, 'depth': 2.62, 'ID': 1, 'timestamp': 11.638},
 {'cam_coord': 1.71, 'depth': 3.09, 'ID': 2, 'timestamp': 11.638},
 {'cam_coord': 0.88, 'depth': 2.75, 'ID': 1, 'timestamp': 11.754},
 {'cam_coord': 1.73, 'depth': 3.14, 'ID': 2, 'timestamp': 11.754},
 {'cam_coord': 0.84, 'depth': 2.88, 'ID': 1, 'timestamp': 11.87},
 {'cam_coord': 1.72, 'depth': 3.15, 'ID': 2, 'timestamp': 11.871},
 {'cam_coord': 0.76, 'depth': 2.9, 'ID': 1, 'timestamp': 11.987},
 {'cam_coord': 1.69, 'depth': 3.19, 'ID': 2, 'timestamp': 11.987},
 {'cam_coord': 0.61, 'depth': 2.79, 'ID': 1, 'timestamp': 12.103},
 {'cam_coord': 1.69, 'depth': 3.24, 'ID': 2, 'timestamp': 12.103},
 {'cam_coord': 0.57, 'depth': 2.78, 'ID': 1, 'timestamp': 12.22},
 {'cam_coord': 1.67, 'depth': 3.19, 'ID': 2, 'timestamp': 12.22},
 {'cam_coord': 0.63, 'depth': 2.93, 'ID': 1, 'timestamp': 12.336},
 {'cam_coord': 1.7, 'depth': 3.24, 'ID': 2, 'timestamp': 12.336},
 {'cam_coord': 0.65, 'depth': 3.09, 'ID': 1, 'timestamp': 12.452},
 {'cam_coord': 1.7, 'depth': 3.24, 'ID': 2, 'timestamp': 12.452},
 {'cam_coord': 0.63, 'depth': 3.12, 'ID': 1, 'timestamp': 12.568},
 {'cam_coord': 1.7, 'depth': 3.26, 'ID': 2, 'timestamp': 12.568},
 {'cam_coord': 0.55, 'depth': 2.85, 'ID': 1, 'timestamp': 12.685},
 {'cam_coord': 1.71, 'depth': 3.27, 'ID': 2, 'timestamp': 12.685},
 {'cam_coord': 0.61, 'depth': 3.04, 'ID': 1, 'timestamp': 12.801},
 {'cam_coord': 1.7, 'depth': 3.26, 'ID': 2, 'timestamp': 12.801},
 {'cam_coord': 0.68, 'depth': 2.96, 'ID': 1, 'timestamp': 12.918},
 {'cam_coord': 1.69, 'depth': 3.24, 'ID': 2, 'timestamp': 12.918},
 {'cam_coord': 0.76, 'depth': 3.04, 'ID': 1, 'timestamp': 13.034},
 {'cam_coord': 1.7, 'depth': 3.26, 'ID': 2, 'timestamp': 13.034},
 {'cam_coord': 0.77, 'depth': 3.04, 'ID': 1, 'timestamp': 13.151},
 {'cam_coord': 1.7, 'depth': 3.26, 'ID': 2, 'timestamp': 13.15},
 {'cam_coord': 0.73, 'depth': 3.19, 'ID': 1, 'timestamp': 13.267},
 {'cam_coord': 1.72, 'depth': 3.33, 'ID': 2, 'timestamp': 13.267},
 {'cam_coord': 0.68, 'depth': 3.33, 'ID': 1, 'timestamp': 13.383},
 {'cam_coord': 1.71, 'depth': 3.47, 'ID': 2, 'timestamp': 13.383},
 {'cam_coord': 0.63, 'depth': 3.2, 'ID': 1, 'timestamp': 13.5},
 {'cam_coord': 1.6, 'depth': 3.29, 'ID': 2, 'timestamp': 13.5},
 {'cam_coord': 0.63, 'depth': 3.17, 'ID': 1, 'timestamp': 13.616},
 {'cam_coord': 1.54, 'depth': 3.15, 'ID': 2, 'timestamp': 13.616},
 {'cam_coord': 0.66, 'depth': 3.24, 'ID': 1, 'timestamp': 13.732},
 {'cam_coord': 1.61, 'depth': 3.24, 'ID': 2, 'timestamp': 13.732},
 {'cam_coord': 0.65, 'depth': 3.12, 'ID': 1, 'timestamp': 13.849},
 {'cam_coord': 1.49, 'depth': 2.93, 'ID': 2, 'timestamp': 13.849},
 {'cam_coord': 0.7, 'depth': 3.24, 'ID': 1, 'timestamp': 13.965},
 {'cam_coord': 1.48, 'depth': 2.89, 'ID': 2, 'timestamp': 13.965},
 {'cam_coord': 0.68, 'depth': 3.15, 'ID': 1, 'timestamp': 14.081},
 {'cam_coord': 1.28, 'depth': 2.56, 'ID': 2, 'timestamp': 14.082},
 {'cam_coord': 0.63, 'depth': 3.14, 'ID': 1, 'timestamp': 14.199},
 {'cam_coord': 1.06, 'depth': 2.27, 'ID': 2, 'timestamp': 14.198},
 {'cam_coord': 0.59, 'depth': 2.99, 'ID': 1, 'timestamp': 14.315},
 {'cam_coord': 1.3, 'depth': 2.9, 'ID': 2, 'timestamp': 14.315},
 {'cam_coord': 0.59, 'depth': 2.95, 'ID': 1, 'timestamp': 14.431},
 {'cam_coord': 1.13, 'depth': 2.62, 'ID': 2, 'timestamp': 14.431}]
# Sort by timestamp
face_data.sort(key=lambda x: x['timestamp'])

# Calculate velocities
face_data_with_velocity = []
for i, curr in enumerate(face_data):
    if i == 0:
        vx = vz = 0.0
    else:
        prev = face_data[i - 1]
        dt = curr['timestamp'] - prev['timestamp']
        if dt == 0:
            vx = vz = 0.0
        else:
            vx = (curr['cam_coord'] - prev['cam_coord']) / dt
            vz = (curr['depth'] - prev['depth']) / dt

    face_data_with_velocity.append({
        'id': curr['ID'],
        'depth': curr['depth'],
        'cam_coord': curr['cam_coord'],
        'timestamp': curr['timestamp'],
        'vx': vx,
        'vz': vz
    })

# Send frames with velocity
while True:
    face_data_with_velocity.sort(key=lambda x: x['timestamp'])

    # Get the reference start time (wall clock)
    start_time = time.time()
    base_timestamp = face_data_with_velocity[0]['timestamp']

    for idx, face in enumerate(face_data_with_velocity, start=1):
        target_time = start_time + (face['timestamp'] - base_timestamp)
        now = time.time()
        wait_time = target_time - now
        if wait_time > 0:
            time.sleep(wait_time)

        frame = encode_frame(
            obj_id=face['id'],
            dist_long=face['depth'],
            dist_lat=face['cam_coord'],
            vel_long=face['vz'],
            vel_lat=face['vx'],
            obj_class=1,
            dyn_prop=0,
            rcs=-15
        )

        msg = can.Message(arbitration_id=0x70, data=frame, is_extended_id=False)
        try:
            bus.send(msg)
            print(f"[{face['timestamp']:.3f}s] Sent frame: ID={face['id']}, dist_long={face['depth']:.2f}m, "
                f"dist_lat={face['cam_coord']:.2f}m, vel_long={face['vz']:.2f}m/s, "
                f"vel_lat={face['vx']:.2f}m/s, frame={frame}")
        except can.CanError as e:
            print(f"CAN send failed: {e}")
