#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include "rclcpp/client.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class TriggerArenaImageClientNode : public rclcpp::Node
{
 public:
  explicit TriggerArenaImageClientNode(
      const std::string& node_name = "trigger_arena_image_client")
      : Node(node_name)
  {
    std::string srvs_name = "/arena_camera_node/trigger_image";

    m_cli_ = this->create_client<std_srvs::srv::Trigger>(srvs_name);

    // wait if the service is not available
    while (!m_cli_->wait_for_service(1s) && rclcpp::ok()) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
      } else {
        RCLCPP_INFO(this->get_logger(), "%s service not available, waiting again... ->", srvs_name.c_str());
      }
    };
    // log_debug_('service is available now')
    // log_debug('trigger request object has been created')
  }

  void send_request()
  {
    auto req_ = std::make_shared<std_srvs::srv::Trigger::Request>();
    m_result = m_cli_->async_send_request(req_); // TODO when moved away from galactic add .future.share()
  }

  std::shared_future<std::shared_ptr<std_srvs::srv::Trigger_Response>> m_result;

 private:
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m_cli_;
};

int main(int argc, char** argv)
{
  // TODO
  // - should take number of images to trigger
  // - add debug logs

  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto client_node = std::make_shared<TriggerArenaImageClientNode>(
      "trigger_arena_image_client");
  client_node->send_request();
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(client_node, client_node->m_result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto response = client_node->m_result.get();
    if (response->success) {
      RCLCPP_INFO(client_node->get_logger(), "SUCCESS : %s", response->message.c_str());
    } else {
      RCLCPP_ERROR(client_node->get_logger(), "FAILURE : %s", response->message.c_str());
    }
  } else {
    RCLCPP_ERROR(client_node->get_logger(),
                 "Failed to trigger image; issue with spin");
  }

  rclcpp::shutdown();

  return 0;
}