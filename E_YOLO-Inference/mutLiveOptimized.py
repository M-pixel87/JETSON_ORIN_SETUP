import cv2
from ultralytics import YOLO
import time
import threading
from queue import Queue

# --- Configuration ---
MODEL_PATH = 'runs/detect/train/weights/best.engine' 
CAMERA_INDEX = 0
# ---------------------

# A thread-safe class to handle camera reading
class CameraStream:
    def __init__(self, src=0):
        self.stream = cv2.VideoCapture(src)
        if not self.stream.isOpened():
            print(f"Error: Could not open camera {src}.")
            raise IOError
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False
        # Use a queue to store frames, with a max size of 1 to always get the latest frame
        self.q = Queue(maxsize=1) 

    def start(self):
        # Start the thread to read frames from the video stream
        threading.Thread(target=self.update, args=(), daemon=True).start()
        return self

    def update(self):
        # Keep looping infinitely until the thread is stopped
        while not self.stopped:
            if not self.grabbed:
                self.stop()
                return
            (self.grabbed, frame) = self.stream.read()
            # If the queue is full, the oldest frame is dropped, and the new one is added.
            if not self.q.full():
                self.q.put(frame)

    def read(self):
        # Return the latest frame from the queue
        return self.q.get()

    def stop(self):
        # Indicate that the thread should be stopped
        self.stopped = True
        self.stream.release()

# --- Main Program ---

# Load the model
model = YOLO(MODEL_PATH)

# Initialize and start the camera stream thread
print("Starting camera stream...")
cap = CameraStream(src=CAMERA_INDEX).start()
time.sleep(1.0) # Give the camera stream some time to start

print("Running live inference...")
print("Press 'q' to quit.")

# Get frame width once
frame_width = int(cap.stream.get(cv2.CAP_PROP_FRAME_WIDTH))

while True:


    # ... inside your while True loop ...
    
    # Grab the latest frame from the threaded stream
    frame = cap.read()
    if frame is None:
        break
    
    # Run inference on the current frame
    results = model(frame, verbose=False) 
    
    # --- FAST PART: Just Process the Data ---
    for result in results:
        for box in result.boxes:
            # Get the values you need
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            confidence = box.conf[0]
            class_id = int(box.cls[0])
            class_name = model.names[class_id]
            
            # Now just use the data! For example, print it:
            print(f"Detected: {class_name} with {confidence:.2f} confidence at [{x1}, {y1}, {x2}, {y2}]")


# Clean up
print("Stopping...")
cap.stop()
cv2.destroyAllWindows()
