# can_reader.py
import multiprocessing as mp
import can

def parse_target_info(data):
    """
    Parse the 8-byte CAN frame into structured object data based on MR76 logic.
    """
    data_stream = [0] * 8
    data_stream[0] = data[0]  # Object ID
    dist_long_raw = ((data[1] << 5) | (data[2] >> 3)) & 0x1FFF # Distance Longitudinal (13 bits)
    dist_lat_raw = ((data[2] & 0x07) << 8) | data[3] # Distance Lateral (11 bits)
    vel_long_raw = ((data[4] << 6) | (data[5] >> 2)) & 0x3FFF # Velocity Longitudinal (14 bits)
    vel_lat_raw = ((data[5] & 0x03) << 8) | ((data[6] & 0xE0) >> 5) # Velocity Lateral (11 bits)    
    obj_class = (data[6] >> 3) & 0x03 # Object Class (2 bits)
    dyn_prop = data[6] & 0x07 # Dynamic Property (3 bits)
    rcs_raw = data[7] # Radar Cross Section (8 bits)

    parsed = {
        "id": data_stream[0],
        "dist_long": dist_long_raw * 0.2 - 500,
        "dist_lat": dist_lat_raw * 0.2 - 204.6,
        "vel_rel_long": vel_long_raw * 0.25 - 128.0,
        "vel_rel_lat": vel_lat_raw * 0.25 - 64.0,
        "object_class": obj_class,
        "dynamic_prop": dyn_prop,
        "rcs": rcs_raw * 0.5 - 64.0
    }

    return parsed

def can_listener(queue: mp.Queue):
    bus = can.interface.Bus(channel='vcan0', interface='socketcan')
    while True:
        msg = bus.recv()
        if msg is None or len(msg.data) < 8:
            continue
        try:
            parsed = parse_target_info(list(msg.data))
            # print((
            #     parsed["id"],
            #     parsed["dist_long"],
            #     parsed["dist_lat"],
            #     parsed["vel_rel_long"],
            #     parsed["vel_rel_lat"]
            # ))
            queue.put(parsed)
        except Exception as e:
            print(f"[ERROR] Failed to parse frame: {e}")
