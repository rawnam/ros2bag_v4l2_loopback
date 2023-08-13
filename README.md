# ros2bag_v4l2_loopback
A simple application to read image topics from a ros2bag and publish over a v4l2 loopback device to emulate a camera


sudo modprobe v4l2loopback \
        devices=2 exclusive_caps=1,1 video_nr=5,6 \
        card_label="Gst VideoTest","OpenCV Camera"

ros2 run ros2bag_v4l2_loopback ros2bag_v4l2_loopback_node /home/arukan/data/aras_data_12_07_2023/camera_front_ros2_bag_2023_07_12_15_50_14 /camera_front/image_raw 6
