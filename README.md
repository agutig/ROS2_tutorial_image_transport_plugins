# ROS2 Tutorial: Image Transport Plugins (C++) [ENGLISH/INGLES]
* Author: Agutig (Álvaro "Guti" Gutiérrez García)
* Last Review: December 2024
* ROS 2 Version: Humble.
* Spanish version of the [Tutorial](https://github.com/agutig/ROS2_tutorial_image_transport_plugins/blob/main/README_ESP.md)

## Summary

This tutorial provides a detailed explanation of how to develop a video plugin for the [Image_transport](http://wiki.ros.org/image_transport) package. A basic and detailed example, named 'basic_plugin,' is presented, covering the minimum requirements, including interfaces and files. From this example, the tutorial will progress to more complex concepts and functions. All the code used in the tutorial can be found and utilized at this [link](https://github.com/agutig/ROS2_tutorial_image_transport_plugins/tree/main/basic_plugin_ws/src"¡).


## Introduction

Image_transport is a key ROS 2 package designed for capturing and transmitting images and videos across the node network. This package supports the use of "plugins" that allow raw images to be processed before publication. These plugins are modular and can be swapped in real time during execution without needing to be launched as a conventional node.

This feature is particularly valuable for the development of image and video encoders, which optimize the volume of transmitted data — a critical consideration given the potentially large size of videos — thereby helping to maintain the efficiency of the robot's network. Existing plugins, mainly used for encoding, are available on [GitHub.](https://github.com/ros-perception/image_transport_plugins.) However, there are many possibilities yet to be explored, so the first step in developing a new plugin would be defining its desired functionalities.

Image_transport plugins can be designed for two main schemes:

1. **Direct Processing**: This consists of receiving a video signal in raw format (`msg:image`), modifying it as needed, and transmitting the resulting image in the same format (`msg:image`). A practical example of this approach is using a plugin that automatically adjusts the video resolution to a 16:9 aspect ratio, thus ensuring consistent image dimensions regardless of the camera used. Other applications may include image rotation, color transformations, or similar adjustments. This type of processing is performed by a single plugin (publisher plugin).

2. **Dual Processing (Codecs)**: This scheme is used by video codecs and involves using two coordinated plugins (publisher-subscriber) that communicate using an interface other than (`msg:image`). To simplify its explanation, its functionality will be described as follows:

   - **Publisher Plugin (Encoder)**: Receives the video signal, encodes it to reduce its size, converts the content into a different interface that allows greater compression (`msg:codec_interface`), and transmits it.

   - **Subscriber Plugin (Decoder)**: Another plugin receives the message (`msg:codec_interface`) and decodes it back into (`msg:image`) so it can be displayed.

Since the dual plugin processing scheme is more complex and also facilitates explaining the direct processing scheme, a detailed explanation of this will be provided, accompanied by a code example available in the GitHub repository at the following [link.](https://github.com/ros-perception/image_transport_plugins.) This code will not only be used to explain the tutorial but also serve as a reference for similar projects. Additionally, an example of a custom interface is included to simplify the process.

The tutorial will begin with the simplest possible example and will gradually expand with additional features to explain more advanced aspects of image_transport plugins.

## Additional Notes

It is also important to note that there are two types of class implementations for image transport plugins:

- `image_transport::PublisherPlugin<>` | `image_transport::SubscriberPlugin<>`
- `image_transport::SimplePublisherPlugin<>` | `image_transport::SimpleSubscriberPlugin<>`

The original version (`PublisherPlugin` and `SubscriberPlugin`) is more comprehensive but less commonly used in current implementations. The simplified version (`SimplePublisherPlugin` and `SimpleSubscriberPlugin`) is more widespread and allows for easier implementation with more default parameters.


## Basic Tutorial

### Step 1: Create the Package
Create the package as you would normally. In this package, we will store the code for our plugins, and it is advisable not to mix this code with anything unrelated to these plugins to keep everything clean and organized.


```Terminal
ros2 pkg create --build-type ament_cmake <package_name>
```

### Step 2: Create a Custom Interface
If you are following a single subscriber plugin scheme, you can skip this step since you will only handle `sensor_msgs::msg::Image` interfaces for both input and output. However, for dual-plugin schemes, such as a video codec, it will be necessary to design a custom interface for communication between the publisher plugin and the subscriber plugin.

For this tutorial, a specific interface was developed that allows breaking down a raw image into its three main channels: Red (R), Green (G), and Blue (B). Each channel will be transformed from a matrix of specific dimensions (e.g., 1280x720) into a flat vector with a length equivalent to the number of pixels in the image (in this case, 921,600). Additionally, two integer variables, `original_width` and `original_height`, are included to reorganize the vectors and reconstruct the image to its original dimensions in the proposed interface.


```cpp
# BASIC.msg

uint16 original_width
uint16 original_height

uint8[] vector_r #R matrix flattened into a vector 
uint8[] vector_g #G matrix flattened into a vector 
uint8[] vector_b #B matrix flattened into a vector 
```
To create a custom interface, I recommend following this other tutorial I have uploaded at this [link](https://github.com/agutig/ROS2_tutorial_custom_interfaces).

### Step 3: Publisher Plugin Header Definition (.h)

Create the publisher definition in the `include` folder within the `publisher.h` file. This file will require:

- **Namespace**: Use a namespace to identify the plugin classes. Whether you are using an individual plugin or combining a publisher plugin with a subscriber, it is advisable to use the same namespace throughout the project for simplicity. Typically, the package name is used.

- **Method `getTransportName()`**: This method returns the identifier for the video stream, which is crucial for correct communication between plugins in dual-plugin structures. When using an image transport plugin, a suffix is usually added to the topic `/camera_image`, such as `/camera_image/basic`.

- **Publish Function**: Define the function that will modify the message and retransmit it. The implementation of this function will be specified later and is not included directly in the `.h` file.

  ```cpp
  virtual void publish(const sensor_msgs::msg::Image& message, const PublishFn& publish_fn) const override;
  ```

Finally, the code of this file would look like:

```cpp
  #ifndef BASIC_PLUGIN_BASIC_SUBSCRIBER_H
  #define BASIC_PLUGIN_BASIC_SUBSCRIBER_H

  #include <rclcpp/rclcpp.hpp>
  #include <image_transport/simple_subscriber_plugin.hpp>
  #include <sensor_msgs/msg/image.hpp>

  #include "coded_interfaces/msg/basic.hpp" // Import the personalized interface.

  namespace basic_plugin { //IMPORTANT THE NAMESPACE, to keep it clean i made the same namespace for both pub and sub plugins

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
      return "basic";
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

  }; // namespace basic_subscriber
  }

  #endif // BASIC_PLUGIN_BASIC_SUBSCRIBER_H    
```


### Step 4: Publisher Description (.cpp)
Next, proceed to create the class implementation by creating a file with the same name but with a `.cpp` extension in the `src` folder. In this file:

* Use the previously defined namespace.
* Create the function that will be called each time a video message is received (`publish`).

```cpp
void BASICPublisher::publish(const sensor_msgs::msg::Image& message, const PublishFn& publish_fn) const
```
Where:
* Within this function, using the input parameter, you can call the `PublishFn` function (passed as an input parameter) to publish the new message.
* The received message is stored in the variable `message`.

```cpp
#include "basic_plugin/basic_publisher.h"
#include <pluginlib/class_list_macros.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

#include "coded_interfaces/msg/basic.hpp" //Import the personalized interface.

namespace basic_plugin { //IMPORTANT THE NAMESPACE.
  
  /**
  In this script, the defined functions in basic_publisher.h are developed.
  */

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
```

### Step 5: Subscriber Plugin Header Definition (.h)

Similar to step 2, create a file named `x_subscriber.h` in `include/package_name`, containing a subscriber class that inherits from `image_transport::SimpleSubscriberPlugin`. In this file, you should consider the following key points:

* `getTransportName()` must return the same value as the publisher to ensure proper coordination between both plugins.
* `internalCallback` will be the function triggered when a message is received. Its first parameter, `message`, is a pointer referencing the received message, so it must match the type designed for your interface (`coded_interfaces::msg::Basic::ConstSharedPtr&`).
* For clarity, it is recommended to use the same namespace as before.

The code looks like this:

```cpp
#ifndef BASIC_PLUGIN_BASIC_SUBSCRIBER_H
#define BASIC_PLUGIN_BASIC_SUBSCRIBER_H

#include <rclcpp/rclcpp.hpp>
#include <image_transport/simple_subscriber_plugin.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "coded_interfaces/msg/basic.hpp" // Import the personalized interface.

namespace basic_plugin { //IMPORTANT THE NAMESPACE, to keep it clean i made the same namespace for both pub and sub plugins

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
    return "basic";
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

}; // namespace basic_subscriber
}

#endif // BASIC_PLUGIN_BASIC_SUBSCRIBER_H

```

### Step 6: Subscriber Description (.cpp)

In this file, we will focus on designing the code for the `internalCallback` method, which will be triggered when a message of type `coded_interfaces::msg::Basic` arrives from the publisher plugin. To do this, we will operate on the content of `message`, which can be accessed using the syntax `message->data_name`. We will process the information contained in this message to reconstruct an image that can be interpreted as video and publish it, i.e., convert the data to the `sensor_msgs::msg::Image` interface.

In this process:

1. Create an empty message of type `sensor_msgs::msg::Image` and populate it with information that is known. The original interface can be found at this [link](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html).

  ```msg
      # This message contains an uncompressed image
      # (0, 0) is at top-left corner of image
      #

      Header header        # Header timestamp should be acquisition time of image
                          # Header frame_id should be optical frame of camera
                          # origin of frame should be optical center of camera
                          # +x should point to the right in the image
                          # +y should point down in the image
                          # +z should point into to plane of the image
                          # If the frame_id here and the frame_id of the CameraInfo
                          # message associated with the image conflict
                          # the behavior is undefined

      uint32 height         # image height, that is, number of rows
      uint32 width          # image width, that is, number of columns

      string encoding       # Encoding of pixels -- channel meaning, ordering, size
                            # taken from the list of strings in include/sensor_msgs/image_encodings.h

      uint8 is_bigendian    # is this data bigendian?
      uint32 step           # Full row length in bytes
      uint8[] data          # actual matrix data, size is (step * rows)
  ```


2. Extract the R, G, and B vectors from the interface and resize them to match the image's width and height (`original_height` and `original_width`).
3. Rearrange the matrices to add them to the `uint8[] data` vector.

Note that it was decided NOT to invert the image to ensure that both plugins work correctly by simply observing the resulting video.

An example would be: 

```cpp

#include "basic_plugin/basic_subscriber.h"
#include <pluginlib/class_list_macros.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "coded_interfaces/msg/basic.hpp" //Basic interface!

namespace basic_plugin {

  /**
  In this script, the defined functions in basic_subscriber.h are developed.
  */
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

```

### Step 7: Create a `manifest.cpp` File

The purpose of this file is simply to pair which of your defined classes can replace the classes in image transport through `<pluginlib/class_list_macros.hpp>`. This is achieved by adding the following lines of code:


```cpp
#include <pluginlib/class_list_macros.hpp>
#include "basic_plugin/basic_publisher.h"
#include "basic_plugin/basic_subscriber.h"

// Just to export the plugin, copy this structure.
PLUGINLIB_EXPORT_CLASS(yourPluginPackage::PluginPublisher, image_transport::PublisherPlugin)
PLUGINLIB_EXPORT_CLASS(yourPluginPackage::PluginSubscriber, image_transport::SubscriberPlugin)
```

These lines can technically be added directly to both `publisher.cpp` and `subscriber.cpp`. However, following the structure commonly used for these plugins, it is clearer to keep them in a separate `.cpp` file located in the `/src` folder, as this is where the plugins will be exported.



 ```cpp
  #include <pluginlib/class_list_macros.hpp>
  #include "basic_plugin/basic_publisher.h"
  #include "basic_plugin/basic_subscriber.h"

  //Just to export the plugin, copy this structure.
  PLUGINLIB_EXPORT_CLASS(basic_plugin::BasicPublisher, image_transport::PublisherPlugin)
  PLUGINLIB_EXPORT_CLASS(basic_plugin::BasicSubscriber, image_transport::SubscriberPlugin)
 ```

### Step 8: `plugins.xml`

Finally, create an `.xml` file to define how the plugins will be configured. Ideally, it should be named something like `plugins.xml` or `transport_image_plugins.xml`. 

This file is crucial as it identifies the created plugins and their associated classes. The file should follow this structure, where each `<class>` represents a plugin:

```xml
<library path="packet_name">

  <class>Plugin1</class>
  <class>Plugin1</class>
  .
  .
  .
</library>
```

and each class tag must have the following parameters:

```xml
  <class name="nombre" type="name_space::RolClass" base_class_type="image_transport::PublisherPlugin" >
```

  * **name**: An identifier for the plugin. It MUST follow the structure `original_package_name` + "/" + the transport name defined in the headers of the publisher or subscriber plugin (IMPORTANT) + "_pub" or "_sub" accordingly. Otherwise, it will cause an error (and it won’t notify you!). In our case, it would be `image_transport/basic_pub` or `image_transport/basic_sub`.

  **Note**: This error is very common. Special emphasis is placed on ensuring this is configured correctly.

  <img src="_other_files/important.png" width="900" height="200">

  To ensure functionality, plugins operate as interchangeable classes, which requires replacing a predefined original class with another that can function similarly (your own plugin/class).

  * **type**: `"namespace::RoleClass"`. I usually use a common namespace for all my plugins in a package and name the namespace the same as my package for simplicity. This is the class you create to replace the original.

  * **base_class_type**: `"original_node::PluginClassToReplace"`. This is the original class you are replacing.


Here is an example of code from this file: 

```xml
<!-- This is file is key, it needs to be properly configure to make your plugin work-->
<library path="basic_plugin"> <!-- I usually use the package name here-->

  <!-- class (one per plugin)

    * name= an identifier for the plugin, i use original_node/transportName_rol ||Note i recommend use as rol "pub" or "sub",since otherwise
    can produce you issues.

    To ensure functionality, plugins operate as interchangeable classes, necessitating the substitution of an original class predefined
    with another that can work similar (Your own plugin/class)

    * type="name_space::RolClass . I usually use a common name_space for all my plugins in a package and i name the name_space the same
    as my package namae for simplicity. This is the class the you create to replace the original one.

    * base_class_type= "original_node::PlugingClassToReplace" . This is the original class you are replacing it.

   -->
  <class name="image_transport/basic_pub" type="basic_plugin::BasicPublisher" base_class_type="image_transport::PublisherPlugin">
    <description>
    This is a custom video publisher plugin for image transport that transforms and msg:image into a
    personalized interface (Basic). Also flips the image for system comprobation.
    </description>
  </class>


  <class name="image_transport/basic_sub" type="basic_plugin::BasicSubscriber" base_class_type="image_transport::SubscriberPlugin">
    <description>
    This is a custom video publisher plugin for image transport that transforms and msg:Basic back into a msg:image.Once installed
    this image can be seen with rqt for any image topic.
    </description>
  </class>
</library>
```

### Step 9: Modify the `CMakeLists.txt` File

The `CMakeLists.txt` file will primarily function as usual, adding the necessary dependencies for your code and modifying other compilation sections. However, you will also need to add these lines to ensure the plugins can be exported.


```txt
# Dependencies, the same like always
.
.
.
find_package(coded_interfaces REQUIRED)       #Your personalized interface
find_package(pluginlib REQUIRED)              #Required for plugins


# Add the created code (Remember manifest.cpp!)
add_library(${PROJECT_NAME} SHARED
  src/basic_publisher.cpp
  src/basic_subscriber.cpp
  src/manifest.cpp
)


ament_target_dependencies(${PROJECT_NAME}
  .
  .
  .
  pluginlib
  coded_interfaces #La interfaz personalizada!
)

# Add this lines too, they are key for plugins
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES
  plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

include_directories(
  include
)

#Export the plugin --> pluginlib_export_plugin_description_file(original_node plugins.xml)
pluginlib_export_plugin_description_file(image_transport plugins.xml)
```

Siguiendo con el ejemplo propuesto, el CMakeLists.txt completo se veria así:

```Txt
cmake_minimum_required(VERSION 3.8)
project(basic_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies, the same like always
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)              #Required for plugins
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(coded_interfaces REQUIRED)       #Your personalized interface


# Add the created code (Remember manifest.cpp!)
add_library(${PROJECT_NAME} SHARED
  src/basic_publisher.cpp
  src/basic_subscriber.cpp
  src/manifest.cpp
)

# Add the dependencies (yes,again)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  pluginlib
  image_transport
  sensor_msgs
  cv_bridge
  OpenCV
  coded_interfaces
)

# Add this lines too, they are key for plugins
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES
  plugins.xml
  DESTINATION share/${PROJECT_NAME}
)


include_directories(
  include
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
################################################

#Export the plugin --> pluginlib_export_plugin_description_file(original_node plugins.xml)
pluginlib_export_plugin_description_file(image_transport plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

```

### Step 10: Modify the package.xml 

Finally, we only have to modify the package.xml . This step is very simple and consists of adding the following lines:

```xml
<depend>pluginlib</depend> <!-- Add this -->
<depend>cv_bridge</depend>
<depend>custom_interfaces</depend> <!-- Add your personalized interfaces -->

<image_transport plugin="${prefix}/plugins.xml" /> <!-- Add this -->
```

The final result of this file could look like: 
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>basic_plugin</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="agutig @github">agutig</maintainer>
   <license>BSD</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>image_transport</depend>
  <depend>pluginlib</depend> <!-- Add this -->
  <depend>cv_bridge</depend>
  <depend>coded_interfaces</depend> <!-- Add your personalized interfaces -->

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
    <image_transport plugin="${prefix}/plugins.xml" /> <!-- Add this -->
  </export>
</package>
```

If you have followed these steps, we will have completed the basic tutorial and it will only be necessary to compile and check that everything works correctly.
Your folder structure should look something similar to:

```Ascii
basic_plugin_ws
│
└───src
    │   manifest.cpp
    │   CMakeLists.txt
    │   package.xml
    │   plugins.xml
    │
    ├───basic_plugin
    │   │
    │   ├───include
    │   │   └───basic_plugin
    │   │           basic_publisher.h
    │   │           basic_subscriber.h
    │   │
    │   └───src
    │           basic_publisher.cpp
    │           basic_subscriber.cpp
```

### Step 11: Compile

Compile the workspace you are working in to ensure no typographical errors have been made. If everything is correct, the plugins should be detected when `image_transport` is launched.

```terminal
colcon build
source install/setup.bash
```

Since plugins cannot be launched like a regular node, to verify if everything is working correctly, I have created a set of tools available at [cam_video_example](https://github.com/agutig/ROS2_tutorial_image_transport_plugins/tree/main/other_tools_ws/src/cam_video_example). Specifically, we will focus on the `camera_publisher` node (`camera_publisher.cpp`), which will start a video stream.

1. Copy and paste the contents of the `cam_video_example` package into your workspace and test it.
2. Compile and source it (Step 11).
3. Launch the `camera_publisher` node.


```terminal
ros2 run cam_video_example camera_publisher
```

4. Open a new terminal in the workspace, source the code, and launch `rqt`.


```terminal
source install/setup.bash
rqt
```

`rqt` is a ROS tool that allows us to easily visualize topics, and with it, we can see the opened video stream. To do this, activate an image viewer such as:

<img src="_other_files/other.png" width="500" height="300">

Once the viewer is open, you can select the topic we opened (`/camera_image`), where the raw image will be displayed. By clicking on that button, you should be able to find versions of this same topic with `/` + the image transports of your plugins.

<img src="_other_files/other2.png" width="200">

<img src="_other_files/other3.png" width="200">

If these three conditions are met, everything is working correctly:

  * The `rqt` viewer displays the video image inverted.
  * The terminal where you launched `rqt` shows the log `"internalCallback activated"`.
  * The terminal where you launched `cam_video_example` shows the log `"Published message on topic: /camera_image/your_transport with transport: your_transport"`.

**Note**: The logs are marked with comments in the `.cpp` files so that they can be removed when no longer needed.

Finally, we have covered the simplest possible implementation of an `image_transport` plugin. However, there are still functions within the `SimpleImagePublisher` and `SimpleImageSubscriber` classes to explore, which we can use to model communication to our preferences. Specifically, we will address the following situation.

If you launched the video with `cam_video_example`, you might have encountered one of the following errors/warnings in the `rqt` terminal:

* `[WARN] [1722273109.670950949] [rqt_gui_cpp_node_40296]: New publisher discovered on topic '/camera_image/basic', offering incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY`

* `[ERROR] [1722273112.346690801] [rqt_gui_cpp_node_40296]: SubscriberPlugin::subscribeImpl with five arguments has not been overridden`

This happens because `SimplePublisher` and `SimpleSubscriber` internally configure the `advertiseImpl` and `subscribeImpl` methods with default settings to simplify the implementation. These methods are responsible for announcing and configuring the QoS policies of the communication between plugins. While their default configuration is robust, they may cause minor issues when implementing plugins, making it advisable to override them to ensure proper functionality.


## IMPL Methods

As previously explained, the IMPL methods (`advertiseImpl`, `subscribeImpl`) are designed to configure the QoS settings for communication between plugins. These methods are NOT required unless errors are occurring, or there is an explicit need to change the QoS between the publisher and subscriber plugins. By default, these methods replicate the QoS of the original video stream without adding extra complexity, as demonstrated in the example code. Since these methods require coordination, it is recommended to implement both if one is modified.

From a technical perspective, it is recommended to configure real-time video transmission using a "non-reliable" scheme (Best-effort), allowing the middleware to use the UDP protocol. Unlike TCP, UDP does not implement packet retransmission mechanisms in case of loss, which is advantageous for applications requiring low latency, such as real-time video.

When a packet is initially lost, it results in a degradation of video quality. Attempting to recover the lost packet via retransmission would be unnecessary, as the corresponding content will have been superseded by newer packets, causing the lost packet to be discarded. Additionally, retransmissions consume bandwidth that could otherwise be allocated to current packets, increasing network congestion. This congestion can create a domino effect that affects video smoothness and may even block the transmission by saturating network resources.

However, forcing a QoS configuration different from that of the raw transport can cause incompatibilities. Therefore, for this tutorial, we will simply configure the QoS to replicate the original stream's configuration.

All the code for this second part of the tutorial can be found at: `basic_plugin_ws/src/basic_plugin_impl`.


 * ### advertiseImpl (Located in the SimplePublisher)

  1. We define the method in the basic_publisher.h adding the following lines.
  ```cpp
    void advertiseImpl(
      rclcpp::Node* node,
      const std::string &base_topic,
      rmw_qos_profile_t custom_qos,
      rclcpp::PublisherOptions options);
  ```

  2. We also define a publisher in basic_publisher.h
  ```cpp
  rclcpp::Publisher<coded_interfaces::msg::Basic>::SharedPtr publisher_;
  ```


  3. Finally we write the implementation of the method in the basic_publisher.cpp file.

  The implementation of the method is a bit open depending on the QoS criteria you are looking for but here I leave you an implementation that replicates the QoS of the data it receives. Remember that it must be the same configuration in the SubscribeImpl method that we are going to do.

  ```
  void BasicPublisher::advertiseImpl(
        rclcpp::Node* node,
        const std::string &base_topic,
        rmw_qos_profile_t custom_qos,
        rclcpp::PublisherOptions options) {

        std::string transport_topic = base_topic + "/" + getTransportName();

        rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(custom_qos));
        qos_profile.reliability(custom_qos.reliability);
        qos_profile.durability(custom_qos.durability);
        qos_profile.liveliness(custom_qos.liveliness);

        publisher_ = node->create_publisher<coded_interfaces::msg::Basic>(
            transport_topic,
            qos_profile,
            options);
        }
  ```


 * ### subscribeImpl (Located in the SimpleSubscriber )

  1. We define the method in the basic_subscriber.h by adding the following lines.
  ```cpp
    void subscribeImpl(
    rclcpp::Node* node,
    const std::string &base_topic,
    const Callback &callback,
    rmw_qos_profile_t custom_qos,
    rclcpp::SubscriptionOptions options);
  ```

  2. We also define a publisher in basic_subscriber.h

  ```cpp
  rclcpp::Subscription<coded_interfaces::msg::Basic>::SharedPtr subscription_;
  ```

  3. Finally we write the implementation of the method in the file basic_subscriber.cpp

  The implementation of the method must represent the same configuration previously established in the advertiseImpl (or at least, compatible)

  ```cpp
  void BasicSubscriber::subscribeImpl(
  rclcpp::Node* node,
  const std::string &base_topic,
  const Callback &callback,
  rmw_qos_profile_t custom_qos,
  rclcpp::SubscriptionOptions options) {

  std::string transport_topic = base_topic + "/" + getTransportName();

    qos_profile.reliability(custom_qos.reliability);
    qos_profile.durability(custom_qos.durability);
    qos_profile.liveliness(custom_qos.liveliness);

    subscription_ = node->create_subscription<coded_interfaces::msg::Basic>(
      transport_topic,
      qos_profile,
      [this, callback](const coded_interfaces::msg::Basic::SharedPtr msg) {
        this->internalCallback(msg, callback);
      },
      options);
  }
  ```


## Referencias

* [http://wiki.ros.org/pluginlib](http://wiki.ros.org/pluginlib) 
* [https://github.com/ros-perception/image_transport_plugins](https://github.com/ros-perception/image_transport_plugins)
* [https://docs.ros.org/en/api/image_transport/html/classimage__transport_1_1PublisherPlugin.html](https://docs.ros.org/en/api/image_transport/html/classimage__transport_1_1PublisherPlugin.html)
* [https://docs.ros.org/en/api/image_transport/html/classimage__transport_1_1SubscriberPlugin.html](https://docs.ros.org/en/api/image_transport/html/classimage__transport_1_1SubscriberPlugin.html)