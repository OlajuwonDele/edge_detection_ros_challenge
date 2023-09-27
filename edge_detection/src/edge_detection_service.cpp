#include "ros/ros.h"
#include <edge_detection/EdgeDetector.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "edge_detection/edge_detect_srv.h"

using namespace edge_detection;

bool edge_detect(edge_detection::edge_detect_srv::Request &req,
                 edge_detection::edge_detect_srv::Response &res) {
    // Create an instance of the EdgeDetector class
    edge_detection::EdgeDetector detector;

    // Convert the ROS sensor_msgs::Image to a cv::Mat using cv_bridge
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(req.input_image_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false; // Return false to indicate an error
    }

    cv::Mat inputImage = cv_ptr->image;

    // Create an output image with the same dimensions as the input image
    cv::Mat outputImage(inputImage.rows, inputImage.cols, inputImage.type());

    // Call the detectEdges function to process the input image and store the result in the outputImage
    // This ensures that the dimensions of outputImage are preserved.
    detector.detectEdges(inputImage, outputImage);

    // Verify the dimensions and step of the output image
    ROS_INFO("Output image dimensions: width=%d, height=%d", outputImage.cols, outputImage.rows);
    ROS_INFO("Output image step: %d", outputImage.step);
    ROS_INFO("Output image encoding: %s", outputImage.type() == CV_8UC3 ? "bgr8" : "unknown");


    // Convert the cv::Mat outputImage to a ROS sensor_msgs::Image message
    sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(req.input_image_msg.header, req.input_image_msg.encoding, outputImage).toImageMsg();
    // out_msg->step = outputImage.step;
    // Verify the dimensions and step of the output message
    ROS_INFO("Output message dimensions: width=%u, height=%u, encoding=%s", out_msg->width, out_msg->height, out_msg->encoding.c_str());
    ROS_INFO("Output message step: %d", out_msg->step);

    // Set the response image
    res.output_image_msg = *out_msg;

    // Log the request and response data for debugging
    ROS_INFO("Received an image for edge detection.");
    ROS_INFO("Sending back response: Detected Edges, width=%u, height=%u, encoding=%s", res.output_image_msg.width, res.output_image_msg.height, res.output_image_msg.encoding.c_str());

    return true; // Return true to indicate a successful service call
}

int main(int argc, char **argv) {
    // Initializing the node
    ros::init(argc, argv, "edge_detection_service");
    ros::NodeHandle n;

    // Create a service server to handle edge detection requests.
    ros::ServiceServer service = n.advertiseService("edge_detection/edge_detector_service", edge_detect);

    ROS_INFO("Ready to perform edge detection.");
    ros::spin();

    return 0;
}
