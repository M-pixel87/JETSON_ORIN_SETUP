from ultralytics import YOLO
import cv2

# --- Configuration ---
# Path to your custom-trained model
MODEL_PATH = "runs/detect/train5/weights/best.pt"
CAMERA_INDEX = 0
# ---------------------

# Load your custom model
model = YOLO(MODEL_PATH)

# Initialize the camera
cap = cv2.VideoCapture(CAMERA_INDEX)
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

print("Running inference with custom model...")
print("Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Run inference on the frame
    results = model(frame)

    # Get the annotated frame with bounding boxes
    annotated_frame = results[0].plot()

    # Display the frame
    cv2.imshow("Custom Object Detection", annotated_frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
