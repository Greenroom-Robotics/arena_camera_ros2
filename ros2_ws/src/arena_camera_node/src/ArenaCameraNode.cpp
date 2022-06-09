#include <stdexcept>
#include <string>
#include <chrono>
#include <thread>

#include "rmw/types.h"
#include "camera_calibration_parsers/parse.hpp"

#include "ArenaCameraNode.h"
#include "light_arena/deviceinfo_helper.h"
#include "rclcpp_adapter/pixelformat_translation.h"
#include "rclcpp_adapter/quilty_of_service_translation.cpp"

constexpr int GETIMAGE_TIMEOUT = 1000;


ArenaCameraNode::ArenaCameraNode() : Node("arena_camera_node")
{
    serial_ = this->declare_parameter<std::string>("serial", "");

    pixelformat_ros_ = this->declare_parameter("pixelformat", "");

    camera_frame_id = this->declare_parameter("camera_frame_id", "lucid_camera");
    camera_info_path = this->declare_parameter("camera_info_path", "");

    width_ = this->declare_parameter("width", 0);
    is_passed_width = width_ > 0;

    height_ = this->declare_parameter("height", 0);
    is_passed_height = height_ > 0;

    gain_ = this->declare_parameter("gain", -1.0);
    is_passed_gain_ = gain_ >= 0;

    exposure_time_ = this->declare_parameter("exposure_time", -1.0);
    is_passed_exposure_time_ = exposure_time_ >= 0;

    trigger_mode_activated_ = this->declare_parameter("trigger_mode", false);
    frame_rate = this->declare_parameter("frame_rate", 0.0);

    test_pattern = this->declare_parameter("test_pattern", "");

    use_ptp = this->declare_parameter("ptp", false);

    pub_qos_history_ = this->declare_parameter("qos_history", "");

    pub_qos_history_depth_ = this->declare_parameter("qos_history_depth", 0);
    is_passed_pub_qos_history_depth_ = pub_qos_history_depth_ > 0;

    pub_qos_reliability_ = this->declare_parameter("qos_reliability", "");

    if (!camera_info_path.empty()) {
        std::string camera_name;
        if (camera_calibration_parsers::readCalibration(camera_info_path, camera_name, camera_info_msg)) {
            RCLCPP_INFO(get_logger(), "Parsed camera info for '%s'", camera_name.c_str());
            camera_info_msg.header.frame_id = camera_frame_id;
            camera_info_pub = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
        } else {
            RCLCPP_ERROR(get_logger(), "Cannot read camera info from path '%s'", camera_info_path.c_str());
            camera_info_pub = nullptr;
        }
    } else {
        RCLCPP_WARN(get_logger(), "No camera info path provided. camera_info will not be published.");
        camera_info_pub = nullptr;
    }

    initialize_();
    RCLCPP_INFO(this->get_logger(), "Created %s node", this->get_name());
}

ArenaCameraNode::~ArenaCameraNode() {
    if (m_pSystem) {
        if (m_pDevice) {
            m_pSystem->DestroyDevice(m_pDevice);
            RCLCPP_INFO(this->get_logger(), "Device has been destroyed");
        }

        Arena::CloseSystem(m_pSystem);
        RCLCPP_INFO(this->get_logger(), "System is destroyed");
    }
}

void ArenaCameraNode::initialize_()
{
  using namespace std::chrono_literals;
  m_pSystem = Arena::OpenSystem();

  //
  // CHECK DEVICE CONNECTION ( timer ) --------------------------------------
  //
  // TODO
  // - Think of design that allow the node to start stream as soon as
  // it is initialized without waiting for spin to be called
  // - maybe change 1s to a smaller value
  m_wait_for_device_timer_callback_ = this->create_wall_timer(
      1s, std::bind(&ArenaCameraNode::wait_for_device_timer_callback_, this));

  using namespace std::placeholders;
  m_trigger_an_image_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "trigger_capture",
      std::bind(&ArenaCameraNode::publish_an_image_on_trigger_, this, _1, _2));

  rclcpp::SensorDataQoS pub_qos_;
  // QoS history
  if (!pub_qos_history_.empty()) {
    if (is_supported_qos_histroy_policy(pub_qos_history_)) {
      pub_qos_.history(
          K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY[pub_qos_history_]);
    } else {
      RCLCPP_ERROR(this->get_logger(), "%s is not supported for this node", pub_qos_history_.c_str());
      // TODO
      // should throw instead??
      // should this keeps shutting down if for some reasons this node is kept
      // alive
      throw;
    }
  }
  // QoS depth
  if (is_passed_pub_qos_history_depth_ &&
      K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY[pub_qos_history_] ==
          RMW_QOS_POLICY_HISTORY_KEEP_LAST) {
    // TODO
    // test err msg withwhen -1
    pub_qos_.keep_last(pub_qos_history_depth_);
  }

  // Qos reliability
  if (!pub_qos_reliability_.empty()) {
    if (is_supported_qos_reliability_policy(pub_qos_reliability_)) {
      pub_qos_.reliability(
          K_CMDLN_PARAMETER_TO_QOS_RELIABILITY_POLICY[pub_qos_reliability_]);
    } else {
      RCLCPP_ERROR(this->get_logger(), "%s is not supported for this node", pub_qos_reliability_.c_str());
      throw;
    }
  }

  m_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", pub_qos_);

  std::stringstream pub_qos_info;
  auto pub_qos_profile = pub_qos_.get_rmw_qos_profile();
  pub_qos_info
      << "[QoS] History: "
      << K_QOS_HISTORY_POLICY_TO_CMDLN_PARAMETER[pub_qos_profile.history]
      << " | Depth: " << pub_qos_profile.depth << " | Reliability: "
      << K_QOS_RELIABILITY_POLICY_TO_CMDLN_PARAMETER[pub_qos_profile.reliability];

  RCLCPP_INFO(this->get_logger(), pub_qos_info.str().c_str());
}

void ArenaCameraNode::wait_for_device_timer_callback_()
{
  // something happened while checking for cameras
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for arena camera. Exiting.");
    rclcpp::shutdown();
  }

  // camera discovery
  m_pSystem->UpdateDevices(100);  // in millisec
  auto device_infos = m_pSystem->GetDevices();

  // no camera is connected
  if (!device_infos.size()) {
    RCLCPP_INFO(this->get_logger(), "No arena camera is connected. Waiting for device(s)...");
  }
  // at least one is found
  else {
    m_wait_for_device_timer_callback_->cancel();
    RCLCPP_INFO(this->get_logger(), "%ld arena device(s) has been discovered.", device_infos.size());
    run_();
  }
}

void ArenaCameraNode::run_()
{
    m_pDevice = create_device_ros_();
    configure_camera();

  if (use_ptp) {
      wait_for_ptp_sync();
  }

  m_pDevice->StartStream();

  if (!trigger_mode_activated_) {
    while (rclcpp::ok()) {
      publish_image();
    }
  } else {
    // else ros::spin will
  }
}

void ArenaCameraNode::publish_image()
{
  Arena::IImage* pImage = nullptr;
  try {
    auto image_msg = std::make_unique<sensor_msgs::msg::Image>();
    pImage = m_pDevice->GetImage(GETIMAGE_TIMEOUT);

    if (pImage != nullptr) {
      if (!pImage->IsIncomplete()) {
        msg_form_image_(pImage, *image_msg);
        m_pub_->publish(std::move(image_msg));

        if (camera_info_pub != nullptr) {
            camera_info_msg.header.stamp = image_msg->header.stamp;
            camera_info_pub->publish(camera_info_msg);
        }
        RCLCPP_DEBUG(this->get_logger(), "image %ld published", pImage->GetFrameId());
      } else {
        RCLCPP_WARN(this->get_logger(), "Incomplete frame was returned by GetImage - discarding frame");
      }
      // the buffer has been copied to the msg so this is free to be re-queued
      this->m_pDevice->RequeueBuffer(pImage);
    } else {
      RCLCPP_ERROR(this->get_logger(), "nullptr was returned by GetImage");
    }

  } catch (std::exception& e) {
    if (pImage) {
        this->m_pDevice->RequeueBuffer(pImage);
        pImage = nullptr;
    }
    RCLCPP_ERROR(this->get_logger(), "Exception occurred while publishing an image: %s", e.what());
  }
}

void ArenaCameraNode::msg_form_image_(Arena::IImage* pImage,
                                      sensor_msgs::msg::Image& image_msg)
{
    if (use_ptp) {
        image_msg.header.stamp.sec =
                static_cast<uint32_t>(pImage->GetTimestampNs() / 1000000000);
        image_msg.header.stamp.nanosec =
                static_cast<uint32_t>(pImage->GetTimestampNs() % 1000000000);
    } else {
        image_msg.header.stamp = get_clock()->now(); // this is rudimentary
    }
    image_msg.header.frame_id = camera_frame_id;
    image_msg.height = pImage->GetHeight();
    image_msg.width = pImage->GetWidth();
    image_msg.encoding = pixelformat_ros_; // TODO - better way to map this than playing with strings
    image_msg.is_bigendian = pImage->GetPixelEndianness() ==
                             Arena::EPixelEndianness::PixelEndiannessBig;
    auto pixel_length_in_bytes = pImage->GetBitsPerPixel() / 8;
    auto width_length_in_bytes = image_msg.width * pixel_length_in_bytes;
    auto image_data_length_in_bytes = width_length_in_bytes * image_msg.height;
    image_msg.step =
        static_cast<sensor_msgs::msg::Image::_step_type>(width_length_in_bytes);

    image_msg.data.resize(image_data_length_in_bytes);
    std::memcpy(&image_msg.data[0], pImage->GetData(),
                image_data_length_in_bytes);
}

void ArenaCameraNode::publish_an_image_on_trigger_(
    std::shared_ptr<std_srvs::srv::Trigger::Request> request /*unused*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!trigger_mode_activated_) {
    std::string msg =
        "Failed to trigger image because the device is not in trigger mode."
        "run `ros2 run arena_camera_node run --ros-args -p trigger_mode:=true`";
    RCLCPP_WARN(this->get_logger(), msg.c_str());
    response->message = msg;
    response->success = false;
  }

  RCLCPP_INFO(this->get_logger(), "A client triggered an image request");

  try {
    // trigger
    bool triggerArmed = false;
    auto waitForTriggerCount = 10;
    do {
      // infinite loop when I step in (sometimes)
      triggerArmed =
          Arena::GetNodeValue<bool>(m_pDevice->GetNodeMap(), "TriggerArmed");

      if (triggerArmed == false && (waitForTriggerCount % 10) == 0) {
        RCLCPP_INFO(this->get_logger(), "waiting for trigger to be armed");
      }

    } while (triggerArmed == false);

    RCLCPP_DEBUG(this->get_logger(), "trigger is armed; triggering an image");
    Arena::ExecuteNode(m_pDevice->GetNodeMap(), "TriggerSoftware");

    publish_image();
    response->message = "Ok";
    response->success = true;

    RCLCPP_INFO(this->get_logger(), "Image published");
  }

  catch (std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception occurred while triggering an image: %s", e.what());
    response->message = "Error";
    response->success = false;
  }

  catch (GenICam::GenericException& e) {
    RCLCPP_ERROR(this->get_logger(), "GenICam Exception occurred while triggering an image %s", e.what());
    response->message = "Error";
    response->success = false;
  }
}

Arena::IDevice* ArenaCameraNode::create_device_ros_()
{
  m_pSystem->UpdateDevices(100);  // in millisec
  auto device_infos = m_pSystem->GetDevices();
  if (!device_infos.size()) {
    // TODO: handle disconnection
    throw std::runtime_error(
        "camera(s) were disconnected after they were discovered");
  }

  auto index = 0;
  if (!serial_.empty()) {
    index = DeviceInfoHelper::get_index_of_serial(device_infos, serial_);
  }

  auto pDevice = m_pSystem->CreateDevice(device_infos.at(index));
  RCLCPP_INFO(this->get_logger(), "Device created %s", DeviceInfoHelper::info(device_infos.at(index)).c_str());
  return pDevice;
}

void ArenaCameraNode::wait_for_ptp_sync()
{
    using namespace std::chrono_literals;
    auto nodemap = m_pDevice->GetNodeMap();

    RCLCPP_INFO(this->get_logger(), "Checking for PTP sync...");

    while (true) {
        int64_t skew = Arena::GetNodeValue<int64_t>(nodemap, "PtpOffsetFromMaster");

        if (skew == -1) {
            std::this_thread::sleep_for(500ms);
        } else {
            RCLCPP_INFO(this->get_logger(), "PTP synced with %ld ns skew.", skew);
            break;
        }
    }
}

void ArenaCameraNode::configure_camera()
{
  set_nodes_load_default_profile_();

  if (is_passed_width || is_passed_height)
    set_resolution();

  set_nodes_gain_();
  set_nodes_pixelformat_();
  set_nodes_exposure_();
  set_nodes_trigger_mode_();

  auto nodemap = m_pDevice->GetNodeMap();

  if (frame_rate > 0.0) {
      // TODO https://support.thinklucid.com/knowledgebase/how-to-increase-maximum-allowable-exposure-time/
      Arena::SetNodeValue<bool>(nodemap, "AcquisitionFrameRateEnable", true);
      Arena::SetNodeValue<double>(nodemap, "AcquisitionFrameRate", frame_rate);
      RCLCPP_INFO(this->get_logger(), "Acquisition frame rate is set to %f.", frame_rate);
  }

  if (use_ptp) {
      Arena::SetNodeValue<bool>(nodemap, "PtpEnable", true);
      Arena::SetNodeValue<bool>(nodemap, "PtpSlaveOnly", true);
      RCLCPP_INFO(this->get_logger(), "PTP is enabled in slave configuration.");
  }

  if (!test_pattern.empty()) {
    set_nodes_test_pattern_image_();
  }
}

void ArenaCameraNode::set_nodes_load_default_profile_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  // device run on default profile all the time if no args are passed
  // otherwise, overwise only these params
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "UserSetSelector", "Default");
  // execute the profile
  Arena::ExecuteNode(nodemap, "UserSetLoad");
  RCLCPP_INFO(this->get_logger(), "Default profile is loaded");
}

void ArenaCameraNode::set_resolution()
{
  auto nodemap = m_pDevice->GetNodeMap();

  if (is_passed_width) {
    Arena::SetNodeValue<int64_t>(nodemap, "Width", width_);
  } else {
    width_ = Arena::GetNodeValue<int64_t>(nodemap, "Width");
  }

  if (is_passed_height) {
    Arena::SetNodeValue<int64_t>(nodemap, "Height", height_);
  } else {
    height_ = Arena::GetNodeValue<int64_t>(nodemap, "Height");
  }

  RCLCPP_INFO(this->get_logger(),"Image resolution set to %d x %d", width_, height_);
}

void ArenaCameraNode::set_nodes_gain_()
{
  if (is_passed_gain_) {  // not default
    auto nodemap = m_pDevice->GetNodeMap();
    Arena::SetNodeValue<double>(nodemap, "Gain", gain_);
    RCLCPP_INFO(this->get_logger(), "Gain set to %f", gain_);
  }
}

void ArenaCameraNode::set_nodes_pixelformat_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  // TODO ---------------------------------------------------------------------
  // PIXEL FORMAT HANDLING

  if (!pixelformat_ros_.empty()) {
    pixelformat_pfnc_ = K_ROS2_PIXELFORMAT_TO_PFNC[pixelformat_ros_];
    if (pixelformat_pfnc_.empty()) {
      throw std::invalid_argument("pixelformat is not supported!");
    }

    try {
      Arena::SetNodeValue<GenICam::gcstring>(nodemap, "PixelFormat",
                                             pixelformat_pfnc_.c_str());
      RCLCPP_INFO(this->get_logger(), "PixelFormat set to %s", pixelformat_pfnc_.c_str());

    } catch (GenICam::GenericException& e) {
      // TODO
      // an rcl expectation might be expected
      auto x = std::string("pixelformat is not supported by this camera");
      x.append(e.what());
      throw std::invalid_argument(x);
    }
  } else {
    pixelformat_pfnc_ =
        Arena::GetNodeValue<GenICam::gcstring>(nodemap, "PixelFormat");
    pixelformat_ros_ = K_PFNC_TO_ROS2_PIXELFORMAT[pixelformat_pfnc_];

    if (pixelformat_ros_.empty()) {
      RCLCPP_WARN(this->get_logger(),
          "the device current pixelformat value is not supported by ROS2. "
          "please use --ros-args -p pixelformat:=\"<supported pixelformat>\".");
      // TODO
      // print list of supported pixelformats
    }
  }
}

void ArenaCameraNode::set_nodes_exposure_()
{
  if (is_passed_exposure_time_) {
    auto nodemap = m_pDevice->GetNodeMap();
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ExposureAuto", "Off");
    Arena::SetNodeValue<double>(nodemap, "ExposureTime", exposure_time_);
  }
}

void ArenaCameraNode::set_nodes_trigger_mode_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  if (trigger_mode_activated_) {
    if (exposure_time_ < 0) {
      RCLCPP_WARN(this->get_logger(),
          "Avoid long waits wating for triggered images by providing proper "
          "exposure_time.");
    }
    // Enable trigger mode before setting the source and selector
    // and before starting the stream. Trigger mode cannot be turned
    // on and off while the device is streaming.

    // Make sure Trigger Mode set to 'Off' after finishing this example
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode", "On");

    // Set the trigger source to software in order to trigger buffers
    // without the use of any additional hardware.
    // Lines of the GPIO can also be used to trigger.
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerSource",
                                           "Software");
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerSelector",
                                           "FrameStart");
    RCLCPP_WARN(this->get_logger(), "Trigger mode is activated. To trigger an image run `ros2 run arena_camera_node trigger_image`");
  }
  // unset device from being in trigger mode if user did not pass trigger
  // mode parameter because the trigger nodes are not rest when loading
  // the user default profile
  else {
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode", "Off");
  }
}

// just for debugging
void ArenaCameraNode::set_nodes_test_pattern_image_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TestPattern", test_pattern.c_str());
  RCLCPP_INFO(this->get_logger(), "Setting test pattern to %s", test_pattern.c_str());
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArenaCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
