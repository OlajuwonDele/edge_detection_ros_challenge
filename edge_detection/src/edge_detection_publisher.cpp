#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "edge_detection/EdgeDetector.hpp"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <mutex> 
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/buffer.h> 
#include <tf2_ros/transform_listener.h>




class EdgeDetectionNode {
public:
    EdgeDetectionNode(ros::NodeHandle& nh)
    : nh_(nh), it_(nh) {
    // tf_buffer_ = new tf2_ros::Buffer();
        
    //     // Create a TF2 listener and populate the buffer
    // tf_listener_ = new tf2_ros::TransformListener(*tf_buffer_);
    image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &EdgeDetectionNode::imageCallback, this);
    depth_points_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &EdgeDetectionNode::depthPointsCallback, this);
    camera_info_sub_ = nh_.subscribe("/camera/depth/camera_info", 1, &EdgeDetectionNode::cameraInfoCallback, this);
    
    edge_pub_ = it_.advertise("/edge_detection/edge_image", 1);
    input_msg_pub_ = it_.advertise("/edge_detection/input_image", 1);


    point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("/edge_detection/edge_points", 1);
    edge_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/edge_detection/edge_markers", 1);

    robot_pose_sub_ = nh_.subscribe("/tf", 1, &EdgeDetectionNode::robotPoseCallback, this);

    

    // cv::namedWindow("Input Image");
    // cv::namedWindow("Edge Detection");

    // cv::startWindowThread();
}



    ~EdgeDetectionNode() {
        // cv::destroyWindow("Input Image");
        // cv::destroyWindow("Edge Detection");
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        // Lock the image_mutex_ to ensure exclusive access to image_msg_
        std::lock_guard<std::mutex> lock(image_mutex_);
        // Store the received image message
        input_image_msg_ = msg;

        try {
            // Convert the ROS sensor_msgs::Image to a cv::Mat using cv_bridge
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            cv::Mat inputImage = cv_ptr->image;

            // Create an instance of the EdgeDetector class
            edge_detection::EdgeDetector detector;
            cv::Mat edges(inputImage.rows, inputImage.cols, inputImage.type());
            // Call the detectEdges function to process the input image
            detector.detectEdges(inputImage, edges);

            // Convert the cv::Mat edges to a ROS sensor_msgs::Image message
            edge_msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", edges).toImageMsg();
            edge_pub_.publish(edge_msg_);  
            input_msg_pub_.publish(*msg);

            // Display the input image and edge-detected image
            // cv::imshow("Input Image", inputImage);
            // cv::imshow("Edge Detection", edges);
            // cv::waitKey(1);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void depthPointsCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        // Lock the point cloud info mutex
        std::lock_guard<std::mutex> lock(point_cloud_mutex_);
        depth_points_msg_ = msg;

        // Check if the depth point cloud data is empty or invalid
    if (!depth_points_msg_->data.empty() && depth_points_msg_->width > 0 && depth_points_msg_->height > 0) {
        // Data is valid
    } else {
        ROS_WARN("Received empty or invalid depth point cloud data.");
        depth_points_msg_.reset(); // Clear the invalid data
    }
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
        // Lock the camera info mutex
        std::lock_guard<std::mutex> lock(camera_info_mutex_);
        camera_info_msg_ = msg;

         // Check if the camera info data is valid
    if (camera_info_msg_->K[0] > 0.0 && camera_info_msg_->K[4] > 0.0) {
        // Focal lengths are positive (valid)
    } else {
        ROS_WARN("Received invalid camera info data.");
        camera_info_msg_.reset(); // Clear the invalid data
    }
    }

    void robotPoseCallback(const tf2_msgs::TFMessage::ConstPtr& msg) {
    for (const geometry_msgs::TransformStamped& transform : msg->transforms) {
        if (transform.header.frame_id == "link6" && transform.child_frame_id == "link7") {
            std::lock_guard<std::mutex> lock(robot_pose_mutex_);
            // Check if the timestamp is different before updating robot_pose_
            if (transform.header.stamp != robot_pose_.header.stamp) {
                robot_pose_.header = transform.header;
                robot_pose_.pose.position.x = transform.transform.translation.x;
                robot_pose_.pose.position.y = transform.transform.translation.y;
                robot_pose_.pose.position.z = transform.transform.translation.z;
                robot_pose_.pose.orientation = transform.transform.rotation;
            }
        }
    }
}




    // Implement the pixel-to-3D conversion and point cloud publishing logic here
    void publishPointCloud() {
    // Lock the relevant mutexes as needed
    std::lock_guard<std::mutex> lock_image(image_mutex_);
    std::lock_guard<std::mutex> lock_point_cloud(point_cloud_mutex_);
    std::lock_guard<std::mutex> lock_camera_info(camera_info_mutex_);

    if (!edge_msg_ || !depth_points_msg_ || !camera_info_msg_) {
        ROS_WARN("Waiting for input data...");
        return;
    }

    // Convert depth points message to a point cloud
    sensor_msgs::PointCloud2::ConstPtr depth_points_msg = depth_points_msg_;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*depth_points_msg, pcl_pc2);

    // Convert the point cloud to a pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
    pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);

    // Create a sensor_msgs::PointCloud message
    sensor_msgs::PointCloud point_cloud_msg;
    point_cloud_msg.header = edge_msg_->header;

    // Get camera parameters
    const sensor_msgs::CameraInfo& cam_info = *camera_info_msg_;
    double fx = cam_info.K[0];  // Focal length in x-direction (fx)
    double fy = cam_info.K[4];  // Focal length in y-direction (fy)
    double cx = cam_info.K[2];  // Principal point in x-direction (cx)
    double cy = cam_info.K[5];  // Principal point in y-direction (cy)

    for (int v = 0; v < input_image_msg_->height; ++v) {
        for (int u = 0; u < input_image_msg_->width; ++u) {
            // Calculate the index for accessing depth data in the point cloud
            pcl::PointXYZRGB& point = pcl_cloud(u, v);

            // Calculate the 3D coordinates (x, y, z) using camera parameters
            double depth = point.z;
            double x = (u - cx) * depth / fx;
            double y = (v - cy) * depth / fy;

            // Add the 3D point to the sensor_msgs::PointCloud message
            geometry_msgs::Point32 p;
            p.x = x;
            p.y = y;
            p.z = depth;
            point_cloud_msg.points.push_back(p);

            // Add the point to the edge_points vector
            edge_points.push_back(p); // Add the 3D point to edge_points
        }
    }

    // Publish the point cloud
    point_cloud_pub_.publish(point_cloud_msg);
}


    void publishEdgeMarkers() {
    if (edge_points.empty()) {
        ROS_WARN("No edge points available...");
        return;
    }

    //ROS_INFO("Publishing edge markers. Number of edge points: %zu", edge_points.size());

    // Create and publish a MarkerArray message
    visualization_msgs::MarkerArray marker_array;
    for (int i = 0; i < edge_points.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header = edge_msg_->header;
        marker.header.frame_id = "frame0";
        marker.header.stamp = ros::Time();
        marker.ns = "edge_points";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        // Convert geometry_msgs::Point32 to geometry_msgs::Point
        geometry_msgs::Point point;
        point.x = edge_points[i].x;
        point.y = edge_points[i].y;
        point.z = edge_points[i].z;
        marker.pose.position = point;

        marker.scale.x = 1.0; 
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0; // Fully opaque
        marker_array.markers.push_back(marker);
    }
    //ROS_INFO("Publishing edge markers. Number of markers: %zu", marker_array.markers.size());
    edge_marker_pub_.publish(marker_array);
}


    void publishTF() {
        std::lock_guard<std::mutex> lock_robot_pose(robot_pose_mutex_);
        
        // Create a TF transform message
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "link6";
        transformStamped.child_frame_id = "link7";
        
        // Set the translation and rotation using robot_pose_
        transformStamped.transform.translation.x = robot_pose_.pose.position.x;
        transformStamped.transform.translation.y = robot_pose_.pose.position.y;
        transformStamped.transform.translation.z = robot_pose_.pose.position.z;
        transformStamped.transform.rotation = robot_pose_.pose.orientation;
        // ROS_INFO("Quaternion: x=%f, y=%f, z=%f, w=%f", 
        //         robot_pose_.pose.orientation.x, 
        //         robot_pose_.pose.orientation.y, 
        //         robot_pose_.pose.orientation.z, 
        //         robot_pose_.pose.orientation.w);


        // Publish the TF transform
        tf_broadcaster_.sendTransform(transformStamped);
    }

  




private:
    ros::NodeHandle nh_;

    ros::Subscriber depth_points_sub_;
    ros::Subscriber camera_info_sub_;
    ros::Subscriber robot_pose_sub_;

    ros::Publisher point_cloud_pub_;
    ros::Publisher edge_marker_pub_;

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    image_transport::Publisher edge_pub_;
    image_transport::Publisher input_msg_pub_;

    sensor_msgs::ImageConstPtr input_image_msg_;
    sensor_msgs::ImageConstPtr edge_msg_;
    sensor_msgs::PointCloud2ConstPtr depth_points_msg_;
    sensor_msgs::CameraInfoConstPtr camera_info_msg_;

    std::vector<geometry_msgs::Point32> edge_points;

    geometry_msgs::PoseStamped robot_pose_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2_ros::Buffer tf_buffer_;
    // Mutexes for protecting shared data
    std::mutex image_mutex_;
    std::mutex point_cloud_mutex_;
    std::mutex camera_info_mutex_;
    std::mutex robot_pose_mutex_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "edge_detection_node");
    ros::NodeHandle nh;

    EdgeDetectionNode node(nh);

    ros::Rate loop_rate(30);  // Set to same as camera in RViz

    while (ros::ok()) {
        node.publishPointCloud();
        node.publishEdgeMarkers();
        node.publishTF();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
