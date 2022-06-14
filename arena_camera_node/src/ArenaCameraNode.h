#pragma once

// TODO
// - remove m_ before private members
// - add const to member functions
// fix includes in all files
// - should we rclcpp::shutdown in construction instead
//

#include <functional>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "Arena/ArenaApi.h"

class ArenaCameraNode : public rclcpp::Node
{
 public:
    explicit ArenaCameraNode(); //(const rclcpp::NodeOptions &options);
    ~ArenaCameraNode();

 private:
  Arena::ISystem* m_pSystem;
  Arena::IDevice* m_pDevice;
  bool connected;
  bool shutdown = false;
  std::thread camera_thread;

  sensor_msgs::msg::CameraInfo camera_info_msg;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub;
  rclcpp::TimerBase::SharedPtr m_wait_for_device_timer_callback_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_trigger_an_image_srv_;

  std::string serial_;

  int width_;
  bool is_passed_width;
  int height_;
  bool is_passed_height;

  double gain_;
  bool is_passed_gain_;
  double exposure_time_;
  bool is_passed_exposure_time_;
  std::string pixelformat_pfnc_;
  std::string pixelformat_ros_;
  std::string test_pattern;
  double frame_rate;
  bool trigger_mode_activated_;
  bool use_ptp;

  std::string camera_info_path;
  std::string camera_frame_id;

  std::string pub_qos_history_;

  int pub_qos_history_depth_;
  bool is_passed_pub_qos_history_depth_;

  std::string pub_qos_reliability_;

  void initialize_();

  void discover_camera();

  void loop();
  // TODO :
  // - handle misconfigured device
  Arena::IDevice* create_device_ros_(std::vector<Arena::DeviceInfo>& device_infos);
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
