import asyncio
import cv2
import glob
import sys
import math
import os
import subprocess
from importlib.resources import files

from matplotlib import pyplot as plt

from PIL import Image

import mediapipe as mp
from mediapipe.framework.formats import landmark_pb2
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

from frame_msg import FrameMsg, TxSprite, TxImageSpriteBlock

# Initialize mediapipe gesture recognition
base_options = python.BaseOptions(model_asset_path='gesture_recognizer.task')
options = vision.GestureRecognizerOptions(base_options=base_options)
recognizer = vision.GestureRecognizer.create_from_options(options)

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

DOG_FRAMES_DIR = "dog_frames"
# if in test mode, use the test directory
args = sys.argv 
if len(args) > 1 and args[1] == "test":
    REMOTE_HOST = "0.0.0.0"  # IP of computer running the camera
else:
    REMOTE_HOST = "192.168.12.33"  # IP of computer running the camera
HTTP_PORT = "9000"


async def main():
    await check_camera_feed()

async def check_camera_feed():
    frame = FrameMsg()
    
    try:
        await frame.connect()
        await frame.send_break_signal()

        # Let the user know we're starting
        await frame.send_lua("frame.display.text('Loading...',1,1);frame.display.show();print(1)", await_print=True)

        # debug only: check our current battery level
        batt_mem = await frame.send_lua('print(frame.battery_level() .. " / " .. collectgarbage("count"))', await_print=True)
        print(f"Battery Level/Memory used: {batt_mem}")

        # send the std lua files to Frame that handle data accumulation and camera
        for stdlua in ['data', 'camera', 'image_sprite_block']:
            await frame.upload_file_from_string(files("frame_msg").joinpath(f"lua/{stdlua}.min.lua").read_text(), f"{stdlua}.min.lua")

    except Exception as e:
        print(f"Error: {e}")
        
    # Send the main lua application from this project to Frame that will run the app
    # to display the text when the messages arrive
    # We rename the file slightly when we copy it, although it isn't necessary
    #await frame.upload_file("lua/camera_sprite_frame_app.lua", "frame_app.lua")
    await frame.upload_file("lua/compressed_prog_sprite_frame_app.lua", "frame_app.lua")
    # attach the print response handler so we can see stdout from Frame Lua print() statements
    # If we assigned this handler before the frameside app was running,
    # any await_print=True commands will echo the acknowledgement byte (e.g. "1"), but if we assign
    # the handler now we'll see any lua exceptions (or stdout print statements)
    frame.attach_print_response_handler()

    # "require" the main lua file to run it
    # Note: we can't await_print here because the require() doesn't return - it has a main loop
    await frame.send_lua("require('frame_app')", await_print=False)

    # give Frame a moment to start the frameside app,
    # based on how much work the app does before it's ready to process incoming data
    await asyncio.sleep(0.5)

    # Now that the Frameside app has started there is no need to send snippets of Lua
    # code directly (in fact, we would need to send a break_signal if we wanted to because
    # the main app loop on Frame is running).
    # From this point we do message-passing with first-class types and send_message() (or send_data())
    # give the frame some time for the autoexposure loop to run (50 times; every 0.1s)
    await asyncio.sleep(3.0)

    # start the photo capture loop and the ros2 controlling interface
    running = True  # Variable to control photo capture loop
    images = []
    results = []

    i = 0  # Counter for photos
    while running:  # Keep capturing photos
        
         # Create async tasks for parallel execution
        await fetch_and_display_dog_frame(frame)

        i += 1
        await asyncio.sleep(0.01)  # Small delay

    # stop the photo handler and clean up resources

    display_batch_of_images_with_gestures_and_hand_landmarks(images, results)
    frame.detach_print_response_handler()
    await frame.stop_frame_app()


async def fetch_and_display_dog_frame(frame):
    """Fetches the latest image from the Unitree robot and displays it on Frame."""
    transfer_latest_image_from_robot()  # Fetch the image

    # Display it on Frame
    await display_latest_dog_frame(frame)

# Fetch last captured image from dog_frames
async def display_latest_dog_frame(f):
    """Displays the last saved image from the local dog_frames directory."""
    try:
        # Get a list of all .jpg files in the directory
        img = Image.open("/home/david/Frames/gesture-ar-robot-control/dog_frames/latest.jpg")
        # Get the first half of the image's width not the height because it is a stereo image

        # Pack the image into a TxSprite object
        sprite = TxSprite.from_image_bytes(img, max_pixels=64000, compress=True)
        isb = TxImageSpriteBlock(image=sprite)
        
        await f.send_message(0x20, isb.pack())
        # then send all the slices
        for spr in isb.sprite_lines:
            await f.send_message(0x20, spr.pack())

        print(f"Displaying latest image")
    
    except Exception as e:
        print(f"Error while displaying the image: {e}")


def transfer_latest_image_from_robot():
    """Transfers the latest image from the robot to `dog_frames/` using http."""
    # Use http to copy the latest file
    try:
        # use wget to download the latest image from the robot
        # wget http://192.168.12.33:9000/captured_images/latest.jpg
        subprocess.run(["wget", "-q", "-O", f"{DOG_FRAMES_DIR}/latest.jpg", f"http://{REMOTE_HOST}:{HTTP_PORT}/captured_images/latest.jpg"],
                       check=True)

        # Make sure the file is valid and fully downloaded before proceeding
        if os.path.getsize(f"{DOG_FRAMES_DIR}/latest.jpg") < 1000:  # File too small = likely incomplete
            raise ValueError(f"Downloaded image is too small! ({os.path.getsize(f'{DOG_FRAMES_DIR}/latest.jpg')} bytes)")

    except subprocess.CalledProcessError as e:
        print(f"HTTP transfer failed: {e}")


def display_batch_of_images_with_gestures_and_hand_landmarks(images, results):
    """Displays a batch of images with the gesture category and its score along with the hand landmarks."""
    # Images and labels.
    images = [image.numpy_view() for image in images]
    # rotate images back to original orientation
    images = [cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE) for image in images]
    
    gestures = [top_gesture for (top_gesture, _) in results]
    multi_hand_landmarks_list = [multi_hand_landmarks for (_, multi_hand_landmarks) in results]

    # Auto-squaring: this will drop data that does not fit into square or square-ish rectangle.
    rows = int(math.sqrt(len(images)))
    cols = len(images) // rows

    # Size and spacing.
    FIGSIZE = 13.0
    SPACING = 0.1
    subplot=(rows,cols, 1)
    if rows < cols:
        plt.figure(figsize=(FIGSIZE,FIGSIZE/cols*rows))
    else:
        plt.figure(figsize=(FIGSIZE/rows*cols,FIGSIZE))

    # Display gestures and hand landmarks.
    for i, (image, gestures) in enumerate(zip(images[:rows*cols], gestures[:rows*cols])):
        title = f"{gestures.category_name} ({gestures.score:.2f})"
        dynamic_titlesize = FIGSIZE*SPACING/max(rows,cols) * 40 + 3
        annotated_image = image.copy()

        for hand_landmarks in multi_hand_landmarks_list[i]:
          hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
          hand_landmarks_proto.landmark.extend([
            landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in hand_landmarks
          ])

          mp_drawing.draw_landmarks(
            annotated_image,
            hand_landmarks_proto,
            mp_hands.HAND_CONNECTIONS,
            mp_drawing_styles.get_default_hand_landmarks_style(),
            mp_drawing_styles.get_default_hand_connections_style())

        subplot = display_one_image(annotated_image, title, subplot, titlesize=dynamic_titlesize)

    # Layout.
    plt.tight_layout()
    plt.subplots_adjust(wspace=SPACING, hspace=SPACING)
    plt.show()


def display_one_image(image, title, subplot, titlesize=16):
    """Displays one image along with the predicted category name and score."""
    plt.subplot(*subplot)
    plt.imshow(image)
    if len(title) > 0:
        plt.title(title, fontsize=int(titlesize), color='black', fontdict={'verticalalignment':'center'}, pad=int(titlesize/1.5))
    return (subplot[0], subplot[1], subplot[2]+1)


if __name__ == "__main__":
    asyncio.run(main())
