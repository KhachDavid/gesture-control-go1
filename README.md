# Teleoperating Quadrupeds with AR Glasses

<iframe style="width: 100%; max-width: 600px; height: auto; min-height: 400px; 
            @media (min-width: 768px) { width: 80%; max-width: 800px; } 
            @media (min-width: 1200px) { width: 60%; max-width: 1000px; }" src="https://www.youtube.com/embed/POHBXnE-2EU?si=vtejNMxPS6ItHHtg&amp;start=4" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
<p style="
    font-size: 0.8em;
    font-style: italic;
    color: #777;
">
Knocking Pins using hand gestures and feedback from the AR Glasses
</p>
</br>

<iframe style="width: 100%; max-width: 600px; height: auto; min-height: 400px; 
            @media (min-width: 768px) { width: 80%; max-width: 800px; } 
            @media (min-width: 1200px) { width: 60%; max-width: 1000px; }" src="https://www.youtube.com/embed/fxmVGViPP74?si=WH9VBvqyr7Oq93H6" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
            <p style="
    font-size: 0.8em;
    font-style: italic;
    color: #777;
">
Closeup of what the AR Glasses display
</p>

Github Repository is [here](https://github.com/KhachDavid/gesture-control-go1)
</br></br>
## Project Overview
AR technology has spent the last decade on standby to be the next big breakthrough. This started in 2013 when Google released its consumer AR glasses, the "Google Glass", which ended up in their graveyard in 2023. This is a drop in the bucket of failed AR technologies, and it seems like AR will never receive mainstream adoption. Recently, Meta came out with the new consumer Ray Ban AR Glasses, claiming to have revolutionized the industry. However, they have a big problem: lack of developer support. While the Ray Ban Meta glasses can benefit people with low-vision, we as developers are not given an opportunity to develop applications for this technology.

In the sea of closed-source AR devices, Brilliant Labs is a company that attempts to restore faith in AR technology. Their frames are fully open-source and support Python, Flutter, and JavaScript SDKs, opening up many opportunities for applying AR technology in a meaningful way. The Brilliant Frames support a fully programmable 640x400 color OLED display and a fully capable 720p low power color camera. I found this to be adequate for applying mediapipe by using the glasses' camera (shown on the disassembled image later in this article). To maximize the frames per second (FPS) of the quadruped's point of view, and to minimize latency, I used central computer camera instead of the one on the glasses. It is possible to take and receive photos on the glasses simultaneously, but it limits the FPS and slows down the whole operation. 

<img src="https://raw.githubusercontent.com/KhachDavid/static/refs/heads/main/block_diagram.png" style="width: 100%; max-width: 800px; height: auto; 
            @media (min-width: 768px) { width: 100%; max-width: 800px; } 
            @media (min-width: 1200px) { width: 100%; max-width: 1000px; }" />
<p style="
    font-size: 0.8em;
    font-style: italic;
    color: #777;
">
Block Diagram: Input from the camera and output on the display
</p>
</br>

## Unitree GO1 Quadruped Robot

The GO1 robot is a quadruped robot developed by Unitree Robotics. It is controlled using a ROS2 C++ API that communicates with the robot through a UDP connection. The API provides functions to control the robot's joints, read sensor data, and get camera feed. However, I am using a camera feed. Instead a ZED camera is mounted on top of the robot and connected to a Jetson Nano, which runs the [dog_camera_worker](https://github.com/KhachDavid/gesture-control-go1/tree/main/dog_camera_worker) upon startup. The robot control pipeline is adapted from the repository [unitree_legged_sdk](https://github.com/katie-hughes/unitree_ros2). A new ROS2 node, `frames_open_palm`, was created to control the robot. The [node](https://github.com/KhachDavid/gesture-control-go1/blob/main/ros_workspace/src/unitree_legged_real/src/frames_open_palm.cpp) has a variable that stores the current gesture. 

```cpp
std::string current_gesture_ = "";
```

This is updated via a subscription to the `hgr_topic` topic. The subscriber callback receives strings representing gestures and sets the current gesture accordingly. 

```cpp
sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "hgr_topic", 10,
            [this](std_msgs::msg::Int32::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Received gesture code: '%d'", msg->data);
                this->current_gesture_ = mapGestureCodeToString(msg->data);
            });
```


In the high-frequency control loop, the node evaluates the current gesture and formulates a corresponding command message. Depending on the gesture (such as "up," "down," "left," "right," "forward," "back," or "hand"), the node configures parameters like velocity, gait type, and motion mode. Then, it publishes a message to the "high_cmd" topic.

## AR Glasses

<img src="https://raw.githubusercontent.com/KhachDavid/gesture-control-go1/refs/heads/main/readme_assets/glasses_disassembled.jpeg" style="width: 100%; max-width: 600px; height: auto; 
            @media (min-width: 768px) { width: 80%; max-width: 800px; } 
            @media (min-width: 1200px) { width: 60%; max-width: 1000px; }" />
<p style="
    font-size: 0.8em;
    font-style: italic;
    color: #777;
">
AR Glasses Disassembled
</p>
</br>

The AR glasses are a central component for providing the operator with real-time visual feedback directly from the robot's onboard ZED camera. The **Brilliant Frames** are entirely open-source. For this project, I developed the graphics for it, which is documented [here](https://docs.brilliant.xyz/frame/hardware/#display). Below are the key developments:

Displaying images on the glasses:

- I wrote a custom Python TxSprite class was written to convert images into a sprite format compatible with the glasses. Due to the availability of 4 bits per pixel, the images were quantized to 16 colors (huge shoutout to github user [@CitizenOneX](https://github.com/citizenonex) for his help in this process). The "Brilliant Frames" company promoted my code into a pip library for future developers to use and can be found [here](https://pypi.org/project/frame-msg/). My full contributed code can be seen [here](https://github.com/CitizenOneX/frame_msg_python/blob/main/src/frame_msg/tx_sprite.py). 

- I updated and flashed Lua scripts to ensure that bitmapped images (with necessary color quantization for a monochrome display) could be rendered on the glasses. For the purpose of this project, the following file is flashed in `camera_feed.py` in the frames_worker:

```python
await frame.upload_file("lua/compressed_prog_sprite_frame_app.lua", "frame_app.lua")
```

Image Feedback from the Robot:

- Early experiments streamed images using SCP for convenience. After rapid prototyping, the pipeline was changed to use HTTP transfer (yielding approximately 0.3-second latency) for improved reliability.

- Improvements included parallelization of the Mediapipe pipeline and the robot’s live stream using asyncio, leading to a more robust closed-loop control demonstration.

- Experimental modifications with LZ4 compression, enabled by firmware release v25.031.0924 (which added an API for decompression), further accelerated the display. This firmware was published by Brilliant Frames on GitHub and can be found [here](https://github.com/brilliantlabsAR/frame-codebase/releases/tag/v25.031.0924  
).

## Gesture Recognition

I made gesture recognition using the MediaPipe library. MediaPipe is a cross-platform framework for building multimodal applied machine learning pipelines. It was developed by Google and is used in a variety of applications, including gesture recognition. MediaPipe has a pre-trained model for hand gesture recognition that can recognize 8 different gestures. The model is based on a convolutional neural network (CNN) and is trained on a large dataset of hand gestures. In the original implementation, mediapipe was a standalone script. This created a massive control latency. Inspired from Ava Zahedi's mediapipe integration into ROS2, I managed to bring the latency to near zero, where it feels very smooth, living up the teleoperation name. Below is an image of the available gestures:

<img src="https://raw.githubusercontent.com/KhachDavid/static/refs/heads/main/final_gesture_mapping.png" style="width: 100%; max-width: 600px; height: auto; 
            @media (min-width: 768px) { width: 80%; max-width: 800px; } 
            @media (min-width: 1200px) { width: 60%; max-width: 1000px; }" />
<p style="
    font-size: 0.8em;
    font-style: italic;
    color: #777;
">
Initial gesture_recognizer.task model gestures
</p>
</br>


## Integration, Advanced Gestures, and Future Modes

After the successful integration of gesture-based controls with visual feedback, my focus turned to advanced control modes. The Unitree GO1 successfully performed all required movements (left/right/up/down) and even executed a dance routine in response to an “I love you” gesture in ASL. A desired feature I have yet to implement is an IMU-based control mode. When teleoperating the robot, it is useful to turn the robot's head independently, without moving the body. Another enhancement would be developing a side mount and porting the entire program so it runs as a startup script on a raspberry pi. This will create a standalone hardware that is ready to go at the push of a button. 

<img src="https://raw.githubusercontent.com/KhachDavid/static/refs/heads/main/side_mount.png" style="width: 100%; max-width: 600px; height: auto; 
            @media (min-width: 768px) { width: 80%; max-width: 800px; } 
            @media (min-width: 1200px) { width: 60%; max-width: 1000px; }" />
<p style="
    font-size: 0.8em;
    font-style: italic;
    color: #777;
">
Side Mount of the AR Glasses. Concept Design Found on Brilliant Frames Discord
</p>
</br>

## Conclusion

This project demonstrates the feasibility of closed-loop control of a quadruped robot, using high-level hand gestures and AR-enabled visual feedback. The integration of ROS2-based control, Mediapipe hand tracking, and AR glasses for real-time imaging forms a robust proof-of-concept for teleoperation in a resource constrained environment. This not only achieves closed-loop control but also opens the door to future enhancements such as IMU-based head movement control.

## Sources and References

- [Brilliant Frames](https://brilliant.xyz/pages/developers)
- [MediaPipe](https://google.github.io/mediapipe/)
- [Unitree SDK](https://github.com/katie-hughes/unitree_ros2)
- [Unitree Documentation](https://unitree-docs.readthedocs.io/en/latest/get_started/Go1_Edu.html)
- [Mediapipe Integration with ROS2](https://github.com/avazahedi/go1-gesture-command)
