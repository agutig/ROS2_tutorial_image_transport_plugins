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


#ifndef BASIC_PLUGIN_BASIC_SUBSCRIBER_H
#define BASIC_PLUGIN_BASIC_SUBSCRIBER_H

#include <rclcpp/rclcpp.hpp>
#include <image_transport/simple_subscriber_plugin.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "coded_interfaces/msg/basic.hpp" // Import the personalized interface.

namespace basic_impl_plugin { //IMPORTANT THE NAMESPACE, to keep it clean i made the same namespace for both pub and sub plugins

/**
 * A SimpleSubscriberPlugin example for understanding image_transport_plugins by
 * subscribing to coded_interfaces::msg::Basic and converting it to sensor_msgs::msg::Image.
 */

class BasicSubscriber final : public image_transport::SimpleSubscriberPlugin<coded_interfaces::msg::Basic>
/**
  * Base class for the "basic" subscriber plugin. It converts:
  * video "coded" (coded_interfaces::msg::Basic) --> video raw (sensor_msgs::msg::Image)
*/
{
public:
  /** ~BasicSubscriber
   * 
   * Destructor for BasicSubscriber.
  */
  virtual ~BasicSubscriber() = default;

  /** getTransportName
   * 
  * Gets the transport name for this plugin. This is the identifier of the full transport (both plugins)
  * (subscriber and publisher) and has to be the same in both header files.
  * 
  */
  virtual std::string getTransportName() const
  {
    return "basic_impl";
  }

protected:

  /** internalCallback
   * 
   * Callback method for handling coded_interfaces::msg::Basic messages.
   * Converts the coded_interfaces::msg::Basic message to sensor_msgs::msg::Image.
   * 
   * - message The received coded_interfaces::msg::Basic message.
   * - user_cb The callback function to call with the converted sensor_msgs::msg::Image message.
   */

  void internalCallback(const coded_interfaces::msg::Basic::ConstSharedPtr& message, const Callback& user_cb) override;


  /** subscribeImpl
   * 
   * Configures the QoS settings.
   * 
   * - node The ROS 2 node.
   * - base_topic The base topic name.
   * - callback The callback function.
   * - custom_qos The QoS profile.
   * - options Subscription options.
   */

  void subscribeImpl(
  rclcpp::Node* node,
  const std::string &base_topic,
  const Callback &callback,
  rmw_qos_profile_t custom_qos,
  rclcpp::SubscriptionOptions options);

  rclcpp::Subscription<coded_interfaces::msg::Basic>::SharedPtr subscription_; // Necesary if using subscribeImpl

}; // namespace basic_subscriber
}

#endif // BASIC_PLUGIN_BASIC_SUBSCRIBER_H