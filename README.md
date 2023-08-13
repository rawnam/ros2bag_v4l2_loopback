# ros2bag_v4l2_loopback

**ros2bag_v4l2_loopback** is a utility designed to read image topics from a ROS2 bag and publish them over a v4l2 loopback device. This emulates a camera feed, making it possible to process pre-recorded ROS2 data as if it were coming from a live camera.

## Prerequisites

- OpenCV
- v4l2loopback kernel module
- ROS2
- rosbag2_compression

## Setup

Before using this utility, ensure that the v4l2loopback module is loaded with the required parameters:

```bash
sudo modprobe v4l2loopback \
        devices=2 exclusive_caps=1,1 video_nr=5,6 \
        card_label="Gst VideoTest","OpenCV Camera"
```
## Usage

To run the utility, execute:

```bash
ros2 run ros2bag_v4l2_loopback ros2bag_v4l2_loopback_node [path_to_compressed_bag] [topic_name] [video_out_number]
```

## Description

The program begins by initializing the ROS2 bag reader and the v4l2 output device. It reads the image messages from the specified ROS2 bag topic, processes them using OpenCV (converting them to RGB format), and then writes them to the v4l2 loopback device, emulating a live camera feed. The images can also be visualized in a GUI window.

While processing, the utility ensures that the time intervals between frames match the time intervals recorded in the ROS2 bag. This is to maintain the authenticity of the original data's timing.

Additionally, the program calculates and prints the average frames per second (FPS) to the console.

## Ending
The utility runs until all frames in the ROS2 bag are processed, or until the user exits by pressing the ESC key. On completion, it prints a farewell message and exits.

## Contribution
Feel free to contribute to this utility by submitting pull requests or opening issues for any bugs or feature requests.

This README provides an overview of the utility, setup instructions, usage details, and a brief explanation of the program's functionality. Adjustments might be necessary based on any additional context or specific details related to the utility.