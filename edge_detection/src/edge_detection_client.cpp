#include "ros/ros.h"
#include <ros/package.h>
#include <edge_detection/EdgeDetector.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "edge_detection/edge_detect_srv.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "edge_detection_client");
    if (argc != 2) {
        ROS_INFO("Usage: edge_detection_client <image_file_name>");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<edge_detection::edge_detect_srv>("edge_detection/edge_detector_service");
    edge_detection::edge_detect_srv srv;

    // Construct the full path to the image file
    std::string data_directory = ros::package::getPath("edge_detection");
    std::string image_file_path = data_directory + "/data/" + argv[1];

    // Load the input image from file into a cv::Mat
    cv::Mat input_image = cv::imread(image_file_path);

    if (input_image.empty()) {
        ROS_ERROR("Failed to load the input image: %s", image_file_path.c_str());
        return 1;
    }

    // Display the loaded image for debugging purposes
    cv::imshow("Input Image", input_image);
    cv::waitKey(0);

    // Automatically determine expected dimensions
    int expected_width = input_image.cols;
    int expected_height = input_image.rows;

    // Check image encoding
    if (input_image.channels() != 3 || input_image.type() != CV_8UC3) {
        ROS_ERROR("Image encoding is not in 'bgr8' format (3 channels of 8-bit unsigned integers per channel).");
        return 1;
    }

    // Convert the cv::Mat image to a sensor_msgs::Image message
    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", input_image).toImageMsg();

    // Print out image properties for debugging
    ROS_INFO("Image dimensions: width=%d, height=%d", image_msg->width, image_msg->height);
    ROS_INFO("Image encoding: %s", image_msg->encoding.c_str());
    ROS_INFO("Image step: %d", image_msg->step);

    // Set the image dimensions and encoding explicitly
    image_msg->width = expected_width;
    image_msg->height = expected_height;
    image_msg->encoding = "bgr8"; // Ensure encoding is set to "bgr8"
    image_msg->step = input_image.step;

    // Print updated image properties
    ROS_INFO("Updated image dimensions: width=%d, height=%d", image_msg->width, image_msg->height);
    ROS_INFO("Updated image encoding: %s", image_msg->encoding.c_str());

    // Set the image message as part of the service request
    srv.request.input_image_msg = *image_msg;

    if (client.call(srv)) {
        if (!srv.response.output_image_msg.data.empty()) {
            // Convert the sensor_msgs/Image to a cv::Mat
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(srv.response.output_image_msg, sensor_msgs::image_encodings::BGR8);
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return 1;
            }

            // Display the detected edges image
            cv::imshow("Edge Detection Result", cv_ptr->image);
            cv::waitKey(0);

        } else {
            ROS_WARN("Received an empty image from the service.");
        }
    } else {
        ROS_ERROR("Failed to call service EdgeDetector");
        return 1;
    }

    return 0;
}
