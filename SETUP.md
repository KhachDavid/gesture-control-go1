# How to run

Before starting the process, make sure you can run unitree_ws. The instructions are included in unitree_ws/README.md.

1. Connect to Unitree's WiFi network. Upon turning it on, you should see a network named `UNITREE-XXXXXX`. The password is `00000000`.

2. Run the following command to start the ROS2 controller for the Unitree GO1 robot:

```bash
./scripts
```

3. Now that the robot is running, the camera feed is being captured, we can run the AR Glasses to see the dog's POV:

```bash
cd frames_worker
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
python3 camera_feed.py
```

If your Brilliant Frames are on, this script should pair with your glasses and start displaying the camera feed from the dog.

4. Now you can control the robot with your hand gestures! The robot should move according to the gestures you make. You should see the robot's POV on the display of the AR glasses.
