#include "rclcpp/rclcpp.hpp"                        // Include the main ROS 2 header
#include "sensor_msgs/msg/image.hpp"                // Include the Image message
#include <image_transport/image_transport.hpp>      // Include the image transport library
#include <cv_bridge/cv_bridge.h>                    // Include the OpenCV bridge for ROS 2
#include <opencv2/opencv.hpp>                       // Include the main OpenCV header

const int FRAMES_PER_SECOND = 60;                   // Define the frames per second

/**
 * Class representing a Camera Publisher node
 * 
 * This class captures images from the camera and publishes them as ROS 2 Image messages.
 */
class CameraPublisher : public rclcpp::Node
{
public:
    /**
     * Construct a new Camera Publisher object
     * 
     * The constructor initializes the camera and sets up the publisher and timer.
     */
    CameraPublisher() : Node("camera_publisher"), cap(0) // Initialize the camera here
    {
        // Check if the camera is opened successfully
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the camera");
            rclcpp::shutdown(); // If the camera cannot be opened, shutdown the node
        }

        // Define the QoS profile for the image publisher
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        qos.keep_last(1);

        // Create an image publisher with the specified QoS profile
        publisher_ = image_transport::create_publisher(this, "camera_image", qos.get_rmw_qos_profile());
        
        // Create a timer to capture and publish images at a fixed rate
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / FRAMES_PER_SECOND),
            std::bind(&CameraPublisher::timerCallback, this));
    }

private:
    /**
     * Timer callback function
     * 
     * This function captures a new frame from the camera and publishes it as an Image message.
     */
    void timerCallback()
    {
        cv::Mat frame;
        cap >> frame; // Capture a new frame
        if (!frame.empty()) {
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            publisher_.publish(*msg); // Publish the image message
        }
    }

    cv::VideoCapture cap;                       // OpenCV video capture object
    image_transport::Publisher publisher_;      // Image transport publisher
    rclcpp::TimerBase::SharedPtr timer_;        // ROS 2 timer
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                             // Initialize the ROS 2 system
    auto node = std::make_shared<CameraPublisher>();      // Create a CameraPublisher node
    if (rclcpp::ok()) {
        rclcpp::spin(node);                               // Spin the node to process callbacks
    }
    rclcpp::shutdown();                                   // Shutdown the ROS 2 system
    return 0;
}