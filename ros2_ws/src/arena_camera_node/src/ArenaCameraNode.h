#pragma once

// TODO
// - remove m_ before private members
// - add const to member functions
// fix includes in all files
// - should we rclcpp::shutdown in construction instead
//

#include <functional>  // std::bind , std::placeholders

// ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>           // WallTimer
#include <sensor_msgs/msg/image.hpp>  //image msg published
#include <std_srvs/srv/trigger.hpp>   // Trigger

// arena sdk
#include "Arena/ArenaApi.h"

class ArenaCameraNode : public rclcpp::Node
{
 public:
  ArenaCameraNode() : Node("arena_camera_node")
  {
    // set stdout buffer size for ROS defined size BUFSIZE
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    RCLCPP_INFO(this->get_logger(), "Initialising %s node", this->get_name());
    parse_parameters_();
    initialize_();
    RCLCPP_INFO(this->get_logger(), "Created %s node", this->get_name());
  }

 private:
  std::shared_ptr<Arena::ISystem> m_pSystem;
  std::shared_ptr<Arena::IDevice> m_pDevice;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pub_;
  rclcpp::TimerBase::SharedPtr m_wait_for_device_timer_callback_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_trigger_an_image_srv_;

  std::string serial_;
  bool is_passed_serial_;

  size_t width_;
  bool is_passed_width;

  size_t height_;
  bool is_passed_height;

  double gain_;
  bool is_passed_gain_;

  double exposure_time_;
  bool is_passed_exposure_time_;

  std::string pixelformat_pfnc_;
  std::string pixelformat_ros_;
  bool is_passed_pixelformat_ros_;
  std::string test_pattern;
  double frame_rate;
  bool trigger_mode_activated_;
  bool use_ptp;

  std::string frame_id;

  std::string pub_qos_history_;
  bool is_passed_pub_qos_history_;

  size_t pub_qos_history_depth_;
  bool is_passed_pub_qos_history_depth_;

  std::string pub_qos_reliability_;
  bool is_passed_pub_qos_reliability_;

  void parse_parameters_();
  void initialize_();

  void wait_for_device_timer_callback_();

  void run_();
  // TODO :
  // - handle misconfigured device
  Arena::IDevice* create_device_ros_();
  void configure_camera();
  void set_nodes_load_default_profile_();
  void set_resolution();
  void set_nodes_gain_();
  void set_nodes_pixelformat_();
  void set_nodes_exposure_();
  void set_nodes_trigger_mode_();
  void set_nodes_test_pattern_image_();
  void wait_for_ptp_sync();
  void publish_image();

  void publish_an_image_on_trigger_(
      std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void msg_form_image_(Arena::IImage* pImage,
                       sensor_msgs::msg::Image& image_msg);
};
