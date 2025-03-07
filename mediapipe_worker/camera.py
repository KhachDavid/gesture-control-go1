#!/usr/bin/env python3
import asyncio
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from aiohttp import web

# ---------------------------------------------------------------------
# Initialize MediaPipe gesture recognition
base_options = python.BaseOptions(model_asset_path='gesture_recognizer.task')
options = vision.GestureRecognizerOptions(base_options=base_options)
recognizer = vision.GestureRecognizer.create_from_options(options)

# For drawing (if needed)
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

# Global variable to store the current gesture result
current_gesture = "none"

# Gesture command mapping
GESTURE_COMMANDS = {
    "thumb_up": "up",
    "thumb_down": "down",
    "pointing_up": "forward",
    "victory": "back",
    "iloveyou": "right",
    "open_palm": "left",
    "closed_fist": "hand"
}

# ---------------------------------------------------------------------
# Async task that processes frames in real-time
async def gesture_loop():
    global current_gesture
    cap = cv2.VideoCapture(0)  # Open webcam

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            await asyncio.sleep(0.1)
            continue

        # Convert OpenCV BGR frame to RGB (as required by MediaPipe)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Convert to a MediaPipe Image
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb)

        # Run gesture recognition
        recognition_result = recognizer.recognize(mp_image)

        if recognition_result.gestures and len(recognition_result.gestures[0]) > 0:
            top_gesture = recognition_result.gestures[0][0]
            gesture_category = top_gesture.category_name.lower()
            current_gesture = GESTURE_COMMANDS.get(gesture_category, gesture_category)
            print(f"Detected gesture: {gesture_category} (mapped as {current_gesture})")
        else:
            current_gesture = "none"
            print("No gesture detected.")

        await asyncio.sleep(0.1)  # Small delay to control loop speed

    cap.release()
    cv2.destroyAllWindows()

# ---------------------------------------------------------------------
# HTTP handler returning the most recent gesture as JSON.
async def handle_get_gesture(request):
    return web.json_response({"gesture": current_gesture})

# ---------------------------------------------------------------------
# Create the aiohttp web app and add a startup routine to launch the gesture task.
async def create_app():
    app = web.Application()
    app.router.add_get('/gesture', handle_get_gesture)

    # On startup, begin the background gesture loop.
    async def on_startup(app):
        app['gesture_task'] = asyncio.create_task(gesture_loop())

    # On shutdown, stop the background task.
    async def on_cleanup(app):
        task = app.get('gesture_task')
        if task:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass

    app.on_startup.append(on_startup)
    app.on_cleanup.append(on_cleanup)
    return app

# ---------------------------------------------------------------------
# Main entry point: start the web server.
if __name__ == '__main__':
    app = asyncio.run(create_app())
    web.run_app(app, host='0.0.0.0', port=8000)

