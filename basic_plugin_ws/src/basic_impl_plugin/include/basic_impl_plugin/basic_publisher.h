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

#ifndef BASIC_IMAGE_TRANSPORT_BASIC_PUBLISHER_H
#define BASIC_IMAGE_TRANSPORT_BASIC_PUBLISHER_H

#include <sensor_msgs/msg/image.hpp>
#include <image_transport/simple_publisher_plugin.hpp>
#include <rclcpp/node.hpp>

#include "coded_interfaces/msg/basic.hpp" //Import the personalized interface.

namespace basic_impl_plugin { //IMPORTANT THE NAMESPACE, to keep it clean i made the same namespace for both pub and sub plugins

/**
 * A SimplePublisherPlugin example for understanding image_transport_plugins by
 * publishing sensor_msgs::msg::Image as coded_interfaces::msg::Basic.
 */

class BasicPublisher : public image_transport::SimplePublisherPlugin<coded_interfaces::msg::Basic>
/**
  * Base class for the "basic" publisher plugin. It converts:
  * video raw (sensor_msgs::msg::Image) --> video "coded" (coded_interfaces::msg::Basic) 
*/

{
public:

    /** ~BASICPublisher
     * 
     * Destructor for BASICPublisher.
     */
    virtual ~BasicPublisher() = default;

    /** getTransportName()
     * 
     * Gets the transport name for this plugin. This is the identifier of the full transport (both plugins)
     * (subscriber and publsiher) and has to be the same in both header files
     */
    virtual std::string getTransportName() const override
    {
        return "basic_impl";
    }


protected:

    /** publish
     *  
     * Method that manages the modifications to sensor_msgs::msg::Image message to make it a
     * coded_interfaces::msg::Basic message.
     * 
     * - message The image message to publish. Dont modify this parameter here.
     * - publish_fn The function to call to publish the message. Dont modify this parameter here.
     */
    virtual void publish(const sensor_msgs::msg::Image& message, const PublishFn& publish_fn) const override;


    /** advertiseImpl
     *  
     * Advertises a topic with the given QoS settings.
     * 
     * - node The ROS 2 node.
     * - base_topic The base topic name.
     * - custom_qos The QoS profile.
     * - options Publisher options.
     */
    void advertiseImpl(
    rclcpp::Node* node,
    const std::string &base_topic,
    rmw_qos_profile_t custom_qos,
    rclcpp::PublisherOptions options);

    rclcpp::Publisher<coded_interfaces::msg::Basic>::SharedPtr publisher_; //Necesary if using advertiseImpl

    
};

}; // namespace basic_plugin

#endif // BASIC_IMAGE_TRANSPORT_BASIC_PUBLISHER_H