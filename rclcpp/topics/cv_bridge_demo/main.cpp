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
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.hpp"

using std::placeholders::_1;
constexpr static char _IMAGE_TOPIC[] = "/carla/ego_vehicle/camera_svc_front/image";
constexpr static char _OUTPUT_PATH[] = "/home/hugoliu/github/colcon_ws/examples/output/";

class CvBridgeDemo : public rclcpp::Node
{
public:
  CvBridgeDemo()  : Node("CvBridgeDemo")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(_IMAGE_TOPIC, 10, std::bind(&CvBridgeDemo::topic_callback, this, _1));
    RCLCPP_INFO(this->get_logger(), "listening on %s", _IMAGE_TOPIC);
  }

private:
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
    static int id = 0;
    RCLCPP_INFO(this->get_logger(), "recv image [%d], w: %d, h: %d", id, msg->width, msg->height);
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    //save image
    if(id % 100 == 0){
      const cv::Mat& img = cv_ptr->image;
      const std::string filepath = _OUTPUT_PATH + std::to_string(id) + ".png";
      try
      {
        cv::imwrite(filepath.c_str(), img);
      }
      catch (cv::Exception& e)
      {
          RCLCPP_ERROR(this->get_logger(), "cv::imwrite exception: %s", e.what());
          return;
      }
      RCLCPP_INFO(this->get_logger(), "save image %s", filepath.c_str());
    }
    id += 1;
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CvBridgeDemo>());
  rclcpp::shutdown();
  return 0;
}
