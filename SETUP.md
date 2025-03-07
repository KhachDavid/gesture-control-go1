# How to run

Before starting the process, make sure you can run unitree_ws. The instructions are included in unitree_ws/README.md.

1. Connect to Unitree's WiFi network. Upon turning it on, you should see a network named `UNITREE-XXXXXX`. Connect to it using the image below as a reference.

![Unitree WiFi](
    readme_assets/unitree_wifi.png
)

2. Run the following command to start the ROS2 controller for the Unitree GO1 robot:

```bash
colcon build # in your unitree workspace
source install/setup.bash
ros2 run unitree_legged_real ros2_udp highlevel
ros2 run unitree_legged_real frames_open_palm # creates the node that maps gestures to robot commands
```

3. Run the dog camera worker:

This is a simple script that reads the camera feed and saves the last frame to a file for the AR glasses to display. The Jetson Nano is connected to the ZED camera, and the script is run on the Jetson Nano. The scripts are setup as startup scripts, so they should run automatically when the Jetson Nano is powered on

Ensure that these are run inside the dog_camera_worker directory, so http://localhost:9000/captured_images/latest.jpg is accessible.

```bash
cd dog_camera_worker
python3 zed_capture_image.py &
python3 -m http.server 9000
```

4. Now that the robot is running, the camera feed is being captured, we can run the AR Glasses to see the dog's POV:

```bash
cd frames_worker
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
python3 main.py
```

If your Brilliant Frames are on, this script should pair with your glasses and start displaying the camera feed from the dog.

5. Finally, we can run the hand gesture recognition. This will start a web application with a single endpoint that returns the detected gesture. The endpoint is accessed by a recevier script that then sends the gesture to the robot controller.

```bash
cd mediapipe_worker
# The venv inside frames_worker should satisfy the package requirements
source ../frames_worker/venv/bin/activate
python3 camera.py
```

6. Now that we have a web server with the most recent gesture, we can run the receiver script that sends the gesture to the robot controller:

```bash
cd dog_controller
source ../frames_worker/venv/bin/activate
python3 main.py
```

7. Now you can control the robot with your hand gestures! The robot should move according to the gestures you make. You should see the robot's POV on the display of the AR glasses.