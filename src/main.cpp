#include <opencv2/opencv.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include "rosbag2_compression/sequential_compression_reader.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"  
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"

#ifndef VID_WIDTH
#define VID_WIDTH 1920
#endif

#ifndef VID_HEIGHT
#define VID_HEIGHT 1020
#endif

#define VIDEO_OUT "/dev/video6"

int main(int argc, char* argv[]) {

    if (argc < 3) {
        std::cerr << "Usage: <program_name> <path_to_compressed_bag> <topic_name>" << std::endl;
        return 1;
    }

    std::string bag_path = argv[1];
    std::string topic_name = argv[2];

    rosbag2_compression::SequentialCompressionReader reader;  
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path;
    rosbag2_cpp::ConverterOptions converter_options;  

    reader.open(storage_options, converter_options);

    // open output device
    int output = open(VIDEO_OUT, O_RDWR);
    if(output < 0) {
        std::cerr << "ERROR: could not open output device!\n" << strerror(errno);
        return -2;
    }

    // configure params for output device
    struct v4l2_format vid_format;
    memset(&vid_format, 0, sizeof(vid_format));
    vid_format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    vid_format.fmt.pix.width = VID_WIDTH;
    vid_format.fmt.pix.height = VID_HEIGHT;
    vid_format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
    vid_format.fmt.pix.sizeimage = VID_WIDTH * VID_HEIGHT * 3;
    vid_format.fmt.pix.field = V4L2_FIELD_NONE;

    if (ioctl(output, VIDIOC_S_FMT, &vid_format) < 0) {
        std::cerr << "ERROR: unable to set video format!\n" << strerror(errno);
        return -1;
    }

    // create GUI window
    const char* gui = "gui";
    cv::namedWindow(gui);
    cv::setWindowTitle(gui, "ROS2 Bag to V4L2");

    while (reader.has_next()) {
        auto serialized_msg = reader.read_next();

        if (serialized_msg->topic_name != topic_name) {
            continue;
        }

        rclcpp::SerializedMessage rclcpp_serialized_msg(*(serialized_msg->serialized_data));
        sensor_msgs::msg::Image::SharedPtr image_msg = std::make_shared<sensor_msgs::msg::Image>();
        rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
        serializer.deserialize_message(&rclcpp_serialized_msg, image_msg.get());

        if (image_msg) {
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

                // Convert the BGR image to RGB since we're writing in RGB24 format
                cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2RGB);

                // show frame
                cv::imshow(gui, cv_ptr->image);

                // write frame to output device
                size_t written = write(output, cv_ptr->image.data, cv_ptr->image.total() * cv_ptr->image.elemSize());
                if (written < 0) {
                    std::cerr << "ERROR: Could not write to the v4l2 device!\n" << strerror(errno);
                    close(output);
                    break;
                }

                // wait for user to finish program pressing ESC
                if (cv::waitKey(10) == 27)
                    break;

            } catch (cv_bridge::Exception& e) {
                std::cerr << "cv_bridge exception: " << e.what() << std::endl;
                return 1;
            }
        }
    }

    std::cout << "\n\nFinish, bye!\n";
    return 0;
}
