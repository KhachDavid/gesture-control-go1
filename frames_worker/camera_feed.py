import asyncio
import sys
import os
import subprocess
from importlib.resources import files
from PIL import Image

from frame_msg import FrameMsg, TxSprite, TxImageSpriteBlock

# Get the absolute path of the directory where the script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

DOG_FRAMES_DIR = os.path.join(script_dir, "dog_frames")  # Adjust if needed
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

    # Now that the Frameside app has started there is no need to send snippets of Lua
    # code directly (in fact, we would need to send a break_signal if we wanted to because
    # the main app loop on Frame is running).
    # From this point we do message-passing with first-class types and send_message() (or send_data())
    # give the frame some time for the autoexposure loop to run (50 times; every 0.1s)
    await asyncio.sleep(3.0)

    # start the photo capture loop and the ros2 controlling interface
    running = True  # Variable to control photo capture loop
    while running:  # Keep capturing photos
        
         # Create async tasks for parallel execution
        await fetch_and_display_dog_frame(frame)
        await asyncio.sleep(0.01)  # Small delay

    # stop the photo handler and clean up resources
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
        #img = Image.open(f"{DOG_FRAMES_DIR}/latest.jpg")
        # Can you change image open to be absolute path?
        # Like the absolute path from /home...
        image_path = os.path.join(DOG_FRAMES_DIR, "latest.jpg")
        img = Image.open(image_path)
        print(f"Image path: {image_path}")

        if img is None:
            print("No image found.")
            return

        sprite = TxSprite.from_image_bytes(img, max_pixels=40000)
        isb = TxImageSpriteBlock(image=sprite, sprite_line_height=16, progressive_render=True)

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
        print("Transferred the file")

    except subprocess.CalledProcessError as e:
        print(f"HTTP transfer failed: {e}")

if __name__ == "__main__":
    asyncio.run(main())
