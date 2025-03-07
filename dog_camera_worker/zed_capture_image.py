import cv2
import time
import os

# Create a directory to save images if it doesn't exist
save_path = "captured_images"
os.makedirs(save_path, exist_ok=True)

# Initialize the webcam
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

print("Press 'q' to exit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame.")
        break

    height, width, _ = frame.shape  # Get original dimensions

    # Crop the first half of the image
    # Split it in the middle
    cropped_frame = frame[:, :width // 2]

    # Resize to 320x200
    resized_frame = cv2.resize(cropped_frame, (320, 200))

    # Generate filename
    filename = os.path.join(save_path, "latest.jpg")

    # Remove the previous frame
    if os.path.exists(filename):
        os.remove(filename)

    # Save the new image
    cv2.imwrite(filename, resized_frame)
    print(f"Saved: {filename}")

    # Wait for 1 second before capturing the next image
    time.sleep(1)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
