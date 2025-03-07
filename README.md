# Quadruped Robotics with Hand Gesture Recognition and AR Glasses

<video src="https://github.com/user-attachments/assets/f5e1d19e-e097-401d-8b28-f44feecbcfc0"></video>

## Introduction

This project combines quadruped robotics, computer vision for hand gesture recognition, and AR-based feedback for immersive control. The aim is to control the Unitree GO1 robot with high-level hand gestures (detected using Mediapipe) while receiving live image feedback on AR glasses. The system is composed of four main sections:

• Unitree GO1 Quadruped Robot

• Mediapipe Hand Gesture Recognition

• AR Glasses as a visual interface

• Integration & Feedback Pipeline over ROS2 and Bluetooth

![Project Diagram](
    readme_assets/animated_block_diagram.gif
)

## Unitree GO1 Quadruped Robot

The GO1 robot is a quadruped robot developed by Unitree Robotics. It is controlled using a C++ API that communicates with the robot through a UDP connection. The API provides functions to control the robot's joints, read sensor data, and get the robot's camera feed. It is adapted from the repository [unitree_legged_sdk](https://github.com/katie-hughes/unitree_ros2). A new ROS2 node was created to control the robot using the API. The node has a variable that stores the current gesture, which is updated via a subscription to the "active_gesture" topic. The subscriber callback receives strings representing gestures and sets the current gesture accordingly. In the high-frequency control loop, the node evaluates the current gesture and formulates a corresponding command message. Depending on the gesture (such as "up," "down," "left," "right," "forward," "back," or "hand"), the node configures parameters like velocity, gait type, and motion mode, then publishes a message to the "high_cmd" topic. This mechanism enables the robot to perform various maneuvers by directly converting high-level gesture inputs into precise low-level motion commands.

## Mediapipe Hand Gesture Recognition

Hand gesture recognition is implemented using Google’s Mediapipe. A pre-trained model detects 21 key points on the hand, and the angles between those key points are used to classify gestures. The detected gesture is published on the “active_gesture” topic, which then controls the robot’s movement. This simple but effective approach was enhanced through numerous experiments, several weeks of tinkering with the model’s output plotting and threshold calibration. Early tests showed reliable gesture detection, and later refinements included improvements to the real-time performance and robustness to different hand positions.

## AR Glasses and Live Camera Feed Integration

The AR glasses are a central component for providing the operator with real-time visual feedback directly from the robot’s onboard camera. Key developments and milestones include:

• Displaying Images on the Glass

– A custom Python TxSprite class was written to convert images into a sprite format compatible with the glasses.

– Lua scripts were updated and flashed to ensure that bitmapped images (with necessary color quantization for a monochrome display) could be reliably rendered.

• Image Feedback from the Robot

– Early experiments streamed images using SCP for convenience. After rapid prototyping, the pipeline was changed to use HTTP transfer (yielding approximately 0.3-second latency) for improved reliability.

– Improvements included parallelization of the Mediapipe pipeline and the robot’s live stream using asyncio, leading to a more robust closed-loop control demonstration.

– Experimental modifications with LZ4 compression, enabled by firmware release v25.031.0924 (which added an API for decompression), further accelerated the display, particularly in high-speed scenarios.

The development logs detail tests ranging from early image grabbing to the full integration where the robot’s camera feed is seamlessly pushed via Bluetooth from the main computer (which orchestrates the entire pipeline) to the AR glasses.

## Integration, Advanced Gestures, and Future Modes

After the initial integration of gesture-based controls with visual feedback, the focus turned to advanced control modes and enhanced robotics behavior. Recent journal entries detail the following progress: • Advanced Gestures and Dog Movements

– The Unitree GO1 successfully performed all required movements (left/right/up/down) and even executed a handshake routine in response to a “fist” gesture.

– A custom demonstration allowed the dog to perform a dance routine as a playful response when certain gestures were recognized.

• IMU and Optical Flow for Alternative Control Modes

– A proposed stretch goal is to implement two new control modes using the AR glasses:

1. “Pointing” Mode: leveraging optical flow to move the dog towards a location indicated by the user’s gesture. This mode uses odometry to counteract challenges such as occlusion.

2. “IMU” Mode: based on head movements detected by the glasses’ IMU. Despite challenges in filtering out sensor noise (including the need for debounce routines and potentially Kalman filters), early experiments with roll and pitch have already yielded promising results.

– Discussions with other developers (e.g., CitizenOne on GitHub) have informed attempts to integrate more robust filtering methods for accurate heading/yaw from the magnetometer, though calibration remains nontrivial.

• Hardware and Firmware Enhancements

– A new Raspberry Pi Zero-based camera setup was tested and reconfigured to reduce latency between the dog’s camera capture and the AR glasses display. Plans for a Pi Zero 2 (supporting Mediapipe natively) were set once the hardware becomes available.

– Firmware updates on the glasses and adjustments in the communication protocols (moving between HTTP and Bluetooth) have progressively optimized the closed-loop control interface.

## Project Timeline and Tasks Overview

A detailed journal of tasks and milestones is maintained, with key completions and upcoming challenges, such as:

• Setting Up the Glasses

– Preliminary tests and fully functional demos for image feedback and gesture recognition were completed by mid-January.

– A custom Bluetooth package was created and integrated successfully.

• Moving Unitree GO1

– The integration of the ROS2 C++ SDK into the development environment was successfully handled, with the robot responding correctly to commands from the gesture recognition pipeline by late January.

• Advanced Gestures and Image Feedback via AR Glasses

– Progressive updates (from February through early March) included the integration of LZ4 image compression, improved closed-loop control demonstrations, and experimental modifications with the Raspberry Pi Zero trials, each documented by week.

• Future Tasks and Improvements

– Ongoing work involves finalizing the IMU control mode, particularly calibrating the magnetometer and filtering IMU noise.

– Investigations into simpler yet effective Optical Flow approaches will be pursued to further eliminate latency in the “pointing” control mode.

– There is an intention to extend the demonstration to generalize control modules for other robotics platforms (such as drones and robot arms) in the future.

The project’s repository, related forks (including a modified Frames Python SDK), and further details can be found via the links provided throughout the documentation.

## Conclusion

This project demonstrates the feasibility of controlling a sophisticated quadruped robot using high-level hand gestures and AR-enabled visual feedback. Starting with basic movement commands and evolving into advanced control modes, the iterative development process has been guided by a detailed journal documenting week-by-week progress. The integration of ROS2-based control, Mediapipe hand tracking, and AR glasses for real-time imaging forms a robust proof-of-concept that not only achieves closed-loop control but also opens the door to future enhancements such as IMU-based head movement control and optical flow for spatial navigation.

By sharing these insights and the challenges encountered, from meeting hardware limitations to overcoming communication latencies, the project contributes openly to the robotics and AR communities and lays the foundation for more immersive, intuitive human–robot interfaces.

## References 

1. https://github.com/google-ai-edge/mediapipe-samples/blob/7268248f1401d6f9a72a48361004c185922eeb3b/examples/gesture_recognizer/python/gesture_recognizer.ipynb

2. https://ai.google.dev/edge/mediapipe/solutions/vision/gesture_recognizer/index#models

3. https://medium.com/@fixitblog/solved-unitree-go1-unitree-legged-sdk-39-make-39-error-8a1dcd0af26c

4. https://github.com/katie-hughes/unitree_ros2

5. Additional logs and code repositories at https://github.com/KhachDavid/frames-sdk-image-module and related forks.


Last Updated: March 7, 2025

AR Glasses and Robots,  In Pursuit of More Immersive Controls

David K.

Portfolio post: https://davidk.tech/#/project/controlling_unitree_with_ar_glasses
