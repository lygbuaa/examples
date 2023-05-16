// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.hpp"
#include "logging_utils.h"

using std::placeholders::_1;
constexpr static char _SUB_IMAGE_TOPIC[] = "/carla/ego_vehicle/camera_svc_front/image";
constexpr static char _PUB_IMAGE_TOPIC[] = "/image_pub";
constexpr static char _OUTPUT_PATH[] = "/home/hugoliu/github/colcon_ws/examples/output/";

class CvBridgeDemo : public rclcpp::Node
{
public:
  CvBridgeDemo()  : Node("CvBridgeDemo")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(_PUB_IMAGE_TOPIC, 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(_SUB_IMAGE_TOPIC, 10, std::bind(&CvBridgeDemo::topic_callback, this, _1));
    // RCLCPP_INFO(this->get_logger(), "listening on %s", _SUB_IMAGE_TOPIC);
    RLOGI("listening on %s", _SUB_IMAGE_TOPIC);
  }

private:
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
    static int id = 0;
    // RCLCPP_INFO(this->get_logger(), "recv image [%d], w: %d, h: %d", id, msg->width, msg->height);
    double t_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    RLOGI("recv image [%d], time: %.6f, w: %d, h: %d", id, t_sec, msg->width, msg->height);

    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        // RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        RLOGE("cv_bridge exception: %s", e.what());
        return;
    }

    //save image
    if(id % 100 == 0){
      const cv::Mat& img = cv_ptr->image;
      const std::string filepath = _OUTPUT_PATH + std::to_string(id) + ".png";
      try
      {
        cv::imwrite(filepath.c_str(), img);
        sensor_msgs::msg::Image::SharedPtr ptr = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
        publisher_->publish(*ptr.get());
      }
      catch (cv::Exception& e)
      {
          // RCLCPP_ERROR(this->get_logger(), "cv::imwrite exception: %s", e.what());
          RLOGE("cv::imwrite exception: %s", e.what());
          return;
      }
      // RCLCPP_INFO(this->get_logger(), "save image %s", filepath.c_str());
      RLOGI("save image %s", filepath.c_str());
    }
    id += 1;
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  for(int i = 0; i < argc; i++){
      fprintf(stderr, "argv[%d] = %s\n", i, argv[i]);
  }

  _print_ros_env_();
  rclcpp::init(argc, argv);
  _run_logger_test_();
  
  rclcpp::executors::MultiThreadedExecutor executor;

  rclcpp::Node::SharedPtr g_node = std::make_shared<CvBridgeDemo>();
  RCLCPP_INFO(g_node->get_logger(), "you can also do logging with g_node->get_logger()");

  /* add all nodes into executor */
  executor.add_node(g_node);
  /* only spin one node */
  // rclcpp::spin(g_node);
  /* spin multi node */
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
