from ultralytics import YOLO

# --- Configuration ---
# The App will update this path based on what you type in the box
MODEL_PATH = "runs/detect/train5/weights/best.pt"
# ---------------------

# Load your custom-trained PyTorch model
print(f"Loading model from: {MODEL_PATH}")
model = YOLO(MODEL_PATH)

# Export the model to TensorRT format
print("Starting TensorRT export... This may take a few minutes.")
model.export(format='engine') 
print(f"✅ TensorRT export complete! .engine file created in the same folder.")
