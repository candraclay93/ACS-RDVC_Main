import re

# Load the input text from a file (or paste it directly here as a string)
with open("../detections_log.txt", "r") as f:
    text = f.read()

# Regex to match blocks with ID, Depth, Camera Coords X, and Timestamp
pattern = re.compile(
    r"ID\s+(\d+):.*?"                          # ID
    r"- Depth:\s*([\d.]+)\s*m.*?"              # Depth
    r"Camera Coords:\s*X=([-\d.]+),.*?"        # Camera X
    r"Timestamp:\s*([\d.]+)\s*sec",            # Timestamp
    re.DOTALL
)

# Write the extracted and formatted output
with open("output_coords.txt", "w") as out:
    for match in pattern.finditer(text):
        obj_id, depth, cam_x, timestamp = match.groups()
        out.write(f"ID: {obj_id} | Depth: {depth} | Camera Coords: {cam_x} | Timestamp: {timestamp} sec\n")

print("✅ Done. Output saved to 'output_coords.txt'")
