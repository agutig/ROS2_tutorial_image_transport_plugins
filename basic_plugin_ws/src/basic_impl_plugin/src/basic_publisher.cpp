/*********************************************************************
* This code is a variation by agutig (Álvaro Gutiérrez) of the examples 
* provided by Willow Garage at:
* https://github.com/ros-perception/image_transport_plugins/tree/humble. 
* As a result, this software inherits the original license provided below.
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/


#include "basic_impl_plugin/basic_publisher.h"
#include <pluginlib/class_list_macros.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

#include "coded_interfaces/msg/basic.hpp" //Import the personalized interface.

namespace basic_impl_plugin { //IMPORTANT THE NAMESPACE.
  
  /**
  In this script, the defined functions in basic_publisher.h are developed.
  */

  void BasicPublisher::advertiseImpl(
      rclcpp::Node* node,
      const std::string &base_topic,
      rmw_qos_profile_t custom_qos,
      rclcpp::PublisherOptions options) {
        
  /**
    * advertiseImpl implementation. Configures the QoS that will be used by the publisher plugin and must be
    * perfectly paired with the subscribeImpl implementation of the subscriber plugin. 
    */

      std::string transport_topic = base_topic + "/" + getTransportName(); // Get full transport name

      //Configure QoS, this code tries to find and apply the QoS configuration used by the
      //Raw video code.
      rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(custom_qos));
      qos_profile.reliability(custom_qos.reliability);
      qos_profile.durability(custom_qos.durability);
      qos_profile.liveliness(custom_qos.liveliness);

      publisher_ = node->create_publisher<coded_interfaces::msg::Basic>(
          transport_topic,
          qos_profile,
          options);
  }

  void BasicPublisher::publish(const sensor_msgs::msg::Image& message, const PublishFn& publish_fn) const {

  /**
   * Recives a sensor_msgs::msg::Image message through the message vairable. You can operate the message as desired
   * but at the end, it must be converted to coded_interfaces::msg::Basic. For demonstration, this function flips
   * upside down the image to ensure that the both plugins operate properly.
   * 
   * - message The image message to publish.
   * - publish_fn The function to call to publish the message. Just call the function.
   */


    //This lines convert the message sensor_msgs::msg::Image into an OPenCV image (cv::Mat) to apply
    // Cool maths to it
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "cv_bridge exception: %s", e.what());
        return;
    }

    // Invert the image (flip vertically)
    cv::Mat inverted_image;
    cv::flip(cv_ptr->image, inverted_image, 0);


    //Create the new basic msg and fill it with known information.
    /* For a reference, this is how Basic msg structure looks like
      # Basic.msg

      uint16 original_width
      uint16 original_height

      uint8[] vector_r #R matrix flattened into a vector 
      uint8[] vector_g #G matrix flattened into a vector  
      uint8[] vector_b #B matrix flattened into a vector  
    */

    coded_interfaces::msg::Basic new_message; //Create empty msg
    new_message.original_width = inverted_image.cols; // Add original img width
    new_message.original_height = inverted_image.rows; // Add original img height

    // Flatten the image into a single vector and assign it to the message
    cv::Mat channels[3];
    // Separate the image into R, G, B channels
    cv::split(inverted_image, channels);

    // Fill the msg with the flattened R-G-B channels. 
    new_message.vector_r.assign(channels[2].data, channels[2].data + channels[2].total());
    new_message.vector_g.assign(channels[1].data, channels[1].data + channels[1].total());
    new_message.vector_b.assign(channels[0].data, channels[0].data + channels[0].total());

    // Publish the Basic message
    publish_fn(new_message);

    //This is a log to check that everything is going well but dont forget to delete it before using
    //your final implementation
    RCLCPP_INFO(rclcpp::get_logger("basic_publisher"), "Published message on topic: %s with transport: %s", this->getTopic().c_str(), getTransportName().c_str());
  }


} // namespace basic_publisher


