from ultralytics import YOLO

# --- Configuration ---
# Path to your dataset's YAML file
DATASET_YAML_PATH = "Yolo_inferencing-10/data.yaml"
EPOCHS = 25 
IMAGE_SIZE = 640
CUSTOM_NAME = "NormModel" # <--- The App will change this line
# ---------------------

# Load a pretrained model
model = YOLO('yolov8n.pt')

# Train the model
if __name__ == '__main__':
    results = model.train(
        data=DATASET_YAML_PATH,
        epochs=EPOCHS,
        imgsz=IMAGE_SIZE,
        device=0,
        name=CUSTOM_NAME,   # Uses the variable above
    )

print(f"✅ Training complete! Model saved in 'runs/detect/{CUSTOM_NAME}'")
