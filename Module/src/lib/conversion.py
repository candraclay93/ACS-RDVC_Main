import re
import pandas as pd

log = """
ID: 1 | Depth: 1.31 | Camera Coords: 0.06 | Timestamp: 5.714 sec
ID: 2 | Depth: 2.15 | Camera Coords: 1.14 | Timestamp: 5.714 sec
ID: 1 | Depth: 1.21 | Camera Coords: 1.19 | Timestamp: 5.830 sec
ID: 1 | Depth: 1.16 | Camera Coords: 1.18 | Timestamp: 5.947 sec
ID: 1 | Depth: 1.21 | Camera Coords: 1.12 | Timestamp: 6.062 sec
ID: 1 | Depth: 2.05 | Camera Coords: 1.13 | Timestamp: 6.179 sec
ID: 1 | Depth: 2.01 | Camera Coords: 1.11 | Timestamp: 6.295 sec
ID: 1 | Depth: 1.99 | Camera Coords: 1.10 | Timestamp: 6.411 sec
ID: 1 | Depth: 1.95 | Camera Coords: 1.08 | Timestamp: 6.527 sec
ID: 1 | Depth: 1.99 | Camera Coords: 1.10 | Timestamp: 6.643 sec
ID: 1 | Depth: 2.04 | Camera Coords: 1.12 | Timestamp: 6.760 sec
ID: 1 | Depth: 2.19 | Camera Coords: 1.19 | Timestamp: 6.876 sec
ID: 1 | Depth: 2.15 | Camera Coords: 1.19 | Timestamp: 6.992 sec
ID: 1 | Depth: 2.18 | Camera Coords: 1.18 | Timestamp: 7.108 sec
ID: 1 | Depth: 2.19 | Camera Coords: 1.11 | Timestamp: 7.224 sec
ID: 1 | Depth: 2.18 | Camera Coords: 1.09 | Timestamp: 7.341 sec
ID: 1 | Depth: 2.18 | Camera Coords: 1.05 | Timestamp: 7.457 sec
ID: 1 | Depth: 3.24 | Camera Coords: 0.85 | Timestamp: 7.573 sec
ID: 2 | Depth: 2.00 | Camera Coords: 0.87 | Timestamp: 7.573 sec
ID: 1 | Depth: 3.22 | Camera Coords: 0.78 | Timestamp: 7.689 sec
ID: 2 | Depth: 2.16 | Camera Coords: 0.87 | Timestamp: 7.689 sec
ID: 1 | Depth: 3.37 | Camera Coords: 0.72 | Timestamp: 7.805 sec
ID: 2 | Depth: 2.07 | Camera Coords: 0.82 | Timestamp: 7.806 sec
ID: 1 | Depth: 3.37 | Camera Coords: 0.65 | Timestamp: 7.922 sec
ID: 1 | Depth: 3.35 | Camera Coords: 0.65 | Timestamp: 8.038 sec
ID: 1 | Depth: 3.39 | Camera Coords: 0.67 | Timestamp: 8.154 sec
ID: 1 | Depth: 3.37 | Camera Coords: 0.66 | Timestamp: 8.270 sec
ID: 2 | Depth: 2.21 | Camera Coords: 0.79 | Timestamp: 8.271 sec
ID: 1 | Depth: 3.37 | Camera Coords: 0.66 | Timestamp: 8.387 sec
ID: 2 | Depth: 2.14 | Camera Coords: 0.78 | Timestamp: 8.387 sec
ID: 1 | Depth: 3.43 | Camera Coords: 0.66 | Timestamp: 8.503 sec
ID: 2 | Depth: 2.21 | Camera Coords: 0.87 | Timestamp: 8.503 sec
ID: 1 | Depth: 3.45 | Camera Coords: 0.76 | Timestamp: 8.620 sec
ID: 2 | Depth: 2.11 | Camera Coords: 0.97 | Timestamp: 8.619 sec
ID: 1 | Depth: 3.59 | Camera Coords: 1.02 | Timestamp: 8.736 sec
ID: 2 | Depth: 2.16 | Camera Coords: 1.04 | Timestamp: 8.736 sec
ID: 1 | Depth: 3.12 | Camera Coords: 1.00 | Timestamp: 8.852 sec
ID: 2 | Depth: 2.21 | Camera Coords: 1.07 | Timestamp: 8.852 sec
ID: 1 | Depth: 3.17 | Camera Coords: 1.01 | Timestamp: 8.968 sec
ID: 2 | Depth: 2.11 | Camera Coords: 1.08 | Timestamp: 8.968 sec
ID: 1 | Depth: 3.41 | Camera Coords: 1.05 | Timestamp: 9.084 sec
ID: 2 | Depth: 1.96 | Camera Coords: 1.08 | Timestamp: 9.084 sec
ID: 1 | Depth: 3.49 | Camera Coords: 1.03 | Timestamp: 9.201 sec
ID: 2 | Depth: 1.96 | Camera Coords: 1.08 | Timestamp: 9.201 sec
ID: 1 | Depth: 3.47 | Camera Coords: 0.96 | Timestamp: 9.317 sec
ID: 2 | Depth: 1.97 | Camera Coords: 1.08 | Timestamp: 9.317 sec
ID: 1 | Depth: 3.53 | Camera Coords: 1.00 | Timestamp: 9.433 sec
ID: 2 | Depth: 1.98 | Camera Coords: 1.10 | Timestamp: 9.433 sec
ID: 1 | Depth: 1.91 | Camera Coords: 1.06 | Timestamp: 9.549 sec
ID: 1 | Depth: 3.53 | Camera Coords: 1.46 | Timestamp: 9.665 sec
ID: 2 | Depth: 1.82 | Camera Coords: 1.00 | Timestamp: 9.665 sec
ID: 1 | Depth: 3.62 | Camera Coords: 1.59 | Timestamp: 9.783 sec
ID: 2 | Depth: 1.80 | Camera Coords: 0.99 | Timestamp: 9.783 sec
ID: 1 | Depth: 3.35 | Camera Coords: 1.46 | Timestamp: 9.899 sec
ID: 2 | Depth: 1.76 | Camera Coords: 0.97 | Timestamp: 9.899 sec
ID: 1 | Depth: 3.35 | Camera Coords: 1.46 | Timestamp: 10.015 sec
ID: 2 | Depth: 1.89 | Camera Coords: 1.01 | Timestamp: 10.015 sec
ID: 1 | Depth: 1.82 | Camera Coords: 1.01 | Timestamp: 10.245 sec
ID: 1 | Depth: 1.83 | Camera Coords: 1.01 | Timestamp: 10.362 sec
ID: 1 | Depth: 1.96 | Camera Coords: 1.06 | Timestamp: 10.478 sec
ID: 1 | Depth: 2.15 | Camera Coords: 1.15 | Timestamp: 10.594 sec
ID: 1 | Depth: 2.23 | Camera Coords: 1.19 | Timestamp: 10.710 sec
ID: 1 | Depth: 2.38 | Camera Coords: 1.25 | Timestamp: 10.827 sec
ID: 1 | Depth: 2.45 | Camera Coords: 1.23 | Timestamp: 10.943 sec
ID: 1 | Depth: 2.39 | Camera Coords: 1.14 | Timestamp: 11.059 sec
ID: 2 | Depth: 3.09 | Camera Coords: 1.71 | Timestamp: 11.059 sec
ID: 1 | Depth: 2.45 | Camera Coords: 1.17 | Timestamp: 11.175 sec
ID: 2 | Depth: 3.15 | Camera Coords: 1.75 | Timestamp: 11.175 sec
ID: 1 | Depth: 2.54 | Camera Coords: 1.22 | Timestamp: 11.291 sec
ID: 2 | Depth: 3.20 | Camera Coords: 1.78 | Timestamp: 11.291 sec
ID: 1 | Depth: 2.56 | Camera Coords: 1.18 | Timestamp: 11.522 sec
ID: 2 | Depth: 3.10 | Camera Coords: 1.72 | Timestamp: 11.522 sec
ID: 1 | Depth: 2.62 | Camera Coords: 1.01 | Timestamp: 11.638 sec
ID: 2 | Depth: 3.09 | Camera Coords: 1.71 | Timestamp: 11.638 sec
ID: 1 | Depth: 2.75 | Camera Coords: 0.88 | Timestamp: 11.754 sec
ID: 2 | Depth: 3.14 | Camera Coords: 1.73 | Timestamp: 11.754 sec
ID: 1 | Depth: 2.88 | Camera Coords: 0.84 | Timestamp: 11.870 sec
ID: 2 | Depth: 3.15 | Camera Coords: 1.72 | Timestamp: 11.871 sec
ID: 1 | Depth: 2.90 | Camera Coords: 0.76 | Timestamp: 11.987 sec
ID: 2 | Depth: 3.19 | Camera Coords: 1.69 | Timestamp: 11.987 sec
ID: 1 | Depth: 2.79 | Camera Coords: 0.61 | Timestamp: 12.103 sec
ID: 2 | Depth: 3.24 | Camera Coords: 1.69 | Timestamp: 12.103 sec
ID: 1 | Depth: 2.78 | Camera Coords: 0.57 | Timestamp: 12.220 sec
ID: 2 | Depth: 3.19 | Camera Coords: 1.67 | Timestamp: 12.220 sec
ID: 1 | Depth: 2.93 | Camera Coords: 0.63 | Timestamp: 12.336 sec
ID: 2 | Depth: 3.24 | Camera Coords: 1.70 | Timestamp: 12.336 sec
ID: 1 | Depth: 3.09 | Camera Coords: 0.65 | Timestamp: 12.452 sec
ID: 2 | Depth: 3.24 | Camera Coords: 1.70 | Timestamp: 12.452 sec
ID: 1 | Depth: 3.12 | Camera Coords: 0.63 | Timestamp: 12.568 sec
ID: 2 | Depth: 3.26 | Camera Coords: 1.70 | Timestamp: 12.568 sec
ID: 1 | Depth: 2.85 | Camera Coords: 0.55 | Timestamp: 12.685 sec
ID: 2 | Depth: 3.27 | Camera Coords: 1.71 | Timestamp: 12.685 sec
ID: 1 | Depth: 3.04 | Camera Coords: 0.61 | Timestamp: 12.801 sec
ID: 2 | Depth: 3.26 | Camera Coords: 1.70 | Timestamp: 12.801 sec
ID: 1 | Depth: 2.96 | Camera Coords: 0.68 | Timestamp: 12.918 sec
ID: 2 | Depth: 3.24 | Camera Coords: 1.69 | Timestamp: 12.918 sec
ID: 1 | Depth: 3.04 | Camera Coords: 0.76 | Timestamp: 13.034 sec
ID: 2 | Depth: 3.26 | Camera Coords: 1.70 | Timestamp: 13.034 sec
ID: 1 | Depth: 3.04 | Camera Coords: 0.77 | Timestamp: 13.151 sec
ID: 2 | Depth: 3.26 | Camera Coords: 1.70 | Timestamp: 13.150 sec
ID: 1 | Depth: 3.19 | Camera Coords: 0.73 | Timestamp: 13.267 sec
ID: 2 | Depth: 3.33 | Camera Coords: 1.72 | Timestamp: 13.267 sec
ID: 1 | Depth: 3.33 | Camera Coords: 0.68 | Timestamp: 13.383 sec
ID: 2 | Depth: 3.47 | Camera Coords: 1.71 | Timestamp: 13.383 sec
ID: 1 | Depth: 3.20 | Camera Coords: 0.63 | Timestamp: 13.500 sec
ID: 2 | Depth: 3.29 | Camera Coords: 1.60 | Timestamp: 13.500 sec
ID: 1 | Depth: 3.17 | Camera Coords: 0.63 | Timestamp: 13.616 sec
ID: 2 | Depth: 3.15 | Camera Coords: 1.54 | Timestamp: 13.616 sec
ID: 1 | Depth: 3.24 | Camera Coords: 0.66 | Timestamp: 13.732 sec
ID: 2 | Depth: 3.24 | Camera Coords: 1.61 | Timestamp: 13.732 sec
ID: 1 | Depth: 3.12 | Camera Coords: 0.65 | Timestamp: 13.849 sec
ID: 2 | Depth: 2.93 | Camera Coords: 1.49 | Timestamp: 13.849 sec
ID: 1 | Depth: 3.24 | Camera Coords: 0.70 | Timestamp: 13.965 sec
ID: 2 | Depth: 2.89 | Camera Coords: 1.48 | Timestamp: 13.965 sec
ID: 1 | Depth: 3.15 | Camera Coords: 0.68 | Timestamp: 14.081 sec
ID: 2 | Depth: 2.56 | Camera Coords: 1.28 | Timestamp: 14.082 sec
ID: 1 | Depth: 3.14 | Camera Coords: 0.63 | Timestamp: 14.199 sec
ID: 2 | Depth: 2.27 | Camera Coords: 1.06 | Timestamp: 14.198 sec
ID: 1 | Depth: 2.99 | Camera Coords: 0.59 | Timestamp: 14.315 sec
ID: 2 | Depth: 2.90 | Camera Coords: 1.30 | Timestamp: 14.315 sec
ID: 1 | Depth: 2.95 | Camera Coords: 0.59 | Timestamp: 14.431 sec
ID: 2 | Depth: 2.62 | Camera Coords: 1.13 | Timestamp: 14.431 sec
"""

# Regular expression to match the pattern
pattern = re.compile(
    r"ID:\s*(\d+)\s*\|\s*Depth:\s*([\d.]+)\s*\|\s*Camera\s+Coords:\s*([\d.]+)\s*\|\s*Timestamp:\s*([\d.]+)"
)

# Parse the log into structured data
data = []
for match in pattern.finditer(log):
    id_, depth, cam_coords, timestamp = match.groups()
    data.append({
        "ID": int(id_),
        "Depth": float(depth),
        "Camera_Coords": float(cam_coords),
        "Timestamp": float(timestamp)
    })
# Display the result
from pprint import pprint
# print(repr(log))
print(match.groups())
pprint(data)