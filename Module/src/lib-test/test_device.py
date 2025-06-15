from ultralytics import YOLO
import torch

model = YOLO("model/face/face.pt")

# Force to GPU
if torch.cuda.is_available():
    model.to("cuda")
else:
    print("[WARN] CUDA not available, using CPU")

print("Model device:", model.device)

