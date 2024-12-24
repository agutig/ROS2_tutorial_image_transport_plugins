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

#include "basic_impl_plugin/basic_subscriber.h"
#include <pluginlib/class_list_macros.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "coded_interfaces/msg/basic.hpp" //Basic interface!

namespace basic_impl_plugin {

  /**
  In this script, the defined functions in basic_subscriber.h are developed.
  */

  void BasicSubscriber::subscribeImpl(
  rclcpp::Node* node,
  const std::string &base_topic,
  const Callback &callback,
  rmw_qos_profile_t custom_qos,
  rclcpp::SubscriptionOptions options) {

  /**
    * subscribeImpl implementation. Configures the QoS that will be used by the subscriber plugin and must be
    * perfectly paired with the advertiseImpl implementation of the publisher plugin. 
    */

    std::string transport_topic = base_topic + "/" + getTransportName(); //Get transport full name

    //Configure Qos, this configuration analises and replicated the publisher one.
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    qos_profile.reliability(custom_qos.reliability);
    qos_profile.durability(custom_qos.durability);
    qos_profile.liveliness(custom_qos.liveliness);

    // Applies configuration
    subscription_ = node->create_subscription<coded_interfaces::msg::Basic>(
      transport_topic,
      qos_profile,
      [this, callback](const coded_interfaces::msg::Basic::SharedPtr msg) {
        this->internalCallback(msg, callback);
      },
      options);
  }

  void BasicSubscriber::internalCallback(
      const coded_interfaces::msg::Basic::ConstSharedPtr& message,
      const Callback& user_cb) {

    /** internalCallback
     *
     * Callback for handling coded_interfaces::msg::Basic messages.
     * Converts the coded_interfaces::msg::Basic message to sensor_msgs::msg::Image and calls the user's callback.
     *
     * - message The received coded_interfaces::msg::Basic message.
     * - user_cb The callback function to call with the converted sensor_msgs::msg::Image message.
     */



    // Just a log to check everything is fine, remeber to delete it for the final implementation
    RCLCPP_INFO(rclcpp::get_logger("basic_subscriber"), "internalCallback activated");

    auto image_msg = std::make_shared<sensor_msgs::msg::Image>(); //Creates image tipe to publish


    //Fill the image with default options
    image_msg->header.stamp = rclcpp::Clock().now();
    image_msg->header.frame_id = "basic_frame"; 
    image_msg->encoding = "rgb8"; 
    image_msg->is_bigendian = false; 

    //Fill the message with information extrated from our basic type.
    image_msg->height = message->original_height;
    image_msg->width = message->original_width;
    image_msg->step = image_msg->width * 3; 

    //Checks if the vector R,G and B from Basic image are fine.
    size_t expected_size = image_msg->height * image_msg->width;
    if (message->vector_r.size() != expected_size || 
        message->vector_g.size() != expected_size || 
        message->vector_b.size() != expected_size) {
        // Just a log to check everything is fine, remeber to delete it for the final implementation
        RCLCPP_ERROR(rclcpp::get_logger("basic_subscriber"), "Vector sizes do not match expected image size (%zu)", expected_size);
        return;
    }

    // Reconstrucs the image from the RGB vector from basic msg to a final image
    image_msg->data.resize(expected_size * 3);
    for (size_t i = 0; i < expected_size; ++i) {
        image_msg->data[3 * i] = message->vector_r[i];
        image_msg->data[3 * i + 1] = message->vector_g[i];
        image_msg->data[3 * i + 2] = message->vector_b[i];
    }
    user_cb(image_msg);
  }


} // namespace basic_subscriber
