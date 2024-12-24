#include <memory>                                // Include the memory header for smart pointers
#include <string>                                // Include the string header
#include <numeric>                               // Include the numeric header for std::accumulate
#include "rclcpp/rclcpp.hpp"                     // Include the main ROS 2 header
#include "rclcpp/generic_subscription.hpp"       // Include the generic subscription header

using std::placeholders::_1;                     // Use the placeholder for bind

std::string topic_to_subb = "camera_image/basic"; // Define the topic to subscribe to

/**
 * Class representing a Generic Subscriber node
 * 
 * This class subscribes to a specified topic and processes messages in a generic way.
 */
class GenericSubscriber : public rclcpp::Node
{
public:
    /**
     * Construct a new Generic Subscriber object
     * 
     * The constructor initializes the node, retrieves topic types, and sets up subscriptions.
     */
    GenericSubscriber()
    : Node("generic_subscriber")
    {
        RCLCPP_INFO(this->get_logger(), "Started, listening on: %s", topic_to_subb.c_str());

        // Retrieve the list of topic names and types
        auto data = rclcpp::Node::get_topic_names_and_types();

        // Print each topic name and its types
        for (const auto& topic : data) {
            std::cout << "Topic: " << topic.first << " | Types: ";
            for (const auto& type : topic.second) {
                std::cout << type << " ";
            }
            std::cout << std::endl;
        }

        // Retrieve the message types for the specified topic
        auto msg_type = data["/" + topic_to_subb];

        // Concatenate the message types into a single string
        std::string concatenated_msg_type = std::accumulate(msg_type.begin(), msg_type.end(), std::string(),
            [](const std::string &a, const std::string &b) -> std::string {
                return a + (a.empty() ? "" : " ") + b;
            });

        RCLCPP_INFO(this->get_logger(), concatenated_msg_type.c_str());

        // Create a subscription for each message type
        for (const auto& type : msg_type) {
            std::cout << type << std::endl;
            rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();
            std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> call = 
                std::bind(&GenericSubscriber::topic_callback, this, std::placeholders::_1);
            subscription_ = this->create_generic_subscription(topic_to_subb, type, qos, call);
        }
    }

private:
    /**
     * Callback function for topic subscription
     * 
     * This function is called whenever a message is received on the subscribed topic.
     * 
     * - msg The received serialized message
     */
    void topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg)
    {
        size_t msg_size = msg->size();
        RCLCPP_INFO(this->get_logger(), "Data received, message size: %zu bytes", msg_size);
    }

    std::shared_ptr<rclcpp::GenericSubscription> subscription_; // Generic subscription object
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);                                 // Initialize the ROS 2 system
    rclcpp::spin(std::make_shared<GenericSubscriber>());      // Create and spin the GenericSubscriber node
    rclcpp::shutdown();                                       // Shutdown the ROS 2 system
    return 0;
}
