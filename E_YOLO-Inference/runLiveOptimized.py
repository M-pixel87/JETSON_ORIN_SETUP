import cv2
from ultralytics import YOLO

# --- Configuration ---
# Path to your optimized TensorRT model
MODEL_PATH = "runs/detect/train5/weights/best.engine"
CAMERA_INDEX = 0
# ---------------------

# Load the optimized TensorRT model
try:
    print(f"Attempting to load: {MODEL_PATH}")
    model = YOLO(MODEL_PATH)
except Exception as e:
    print(f"Error loading model: {e}")
    print("Check if the file name in the 'Optimize' tab matches your training name.")
    exit()

# Initialize the camera
cap = cv2.VideoCapture(CAMERA_INDEX)
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

print("Running live inference with optimized model...")
print("Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret: break
    
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    
    # Run inference
    results = model(frame, verbose=False) 

    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            confidence = box.conf[0]
            box_width = x2 - x1
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            
            position = ""
            if center_x < frame_width / 3: position = "Left"
            elif center_x < frame_width * (2/3): position = "Middle"
            else: position = "Right"

            print(f"Pos: {position} | Acc: {int(confidence*100)}% | Width: {int(box_width)}")

    annotated_frame = results[0].plot()
    cv2.imshow("Optimized Model - Live", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
