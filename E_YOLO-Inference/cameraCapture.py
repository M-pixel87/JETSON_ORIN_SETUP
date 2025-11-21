import cv2
import os
import time

# --- Configuration ---
SAVE_DIR = "images"  # Folder to save the images
CAMERA_INDEX = 0      # 0 for default camera, change if you have multiple cameras
# ---------------------

# Create the save directory if it doesn't exist
if not os.path.exists(SAVE_DIR):
    os.makedirs(SAVE_DIR)

# Initialize the camera
cap = cv2.VideoCapture(CAMERA_INDEX)
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

print("Camera feed is active.")
print("Press 's' to save a frame. Press 'q' to quit.")

img_counter = 0

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        print("Error: Can't receive frame. Exiting ...")
        break

    # Display the resulting frame
    cv2.imshow('Camera Feed - Press "s" to save, "q" to quit', frame)

    # Wait for keypress
    key = cv2.waitKey(1) & 0xFF

    if key == ord('s'):
        # Save the frame
        img_name = os.path.join(SAVE_DIR, f"image_{int(time.time())}_{img_counter}.jpg")
        cv2.imwrite(img_name, frame)
        print(f"✅ Saved {img_name}")
        img_counter += 1
    elif key == ord('q'):
        # Quit the loop
        print("Quitting...")
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
