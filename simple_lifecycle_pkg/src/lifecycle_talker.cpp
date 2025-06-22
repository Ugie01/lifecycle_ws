#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode
{
public:
    LifecycleTalker() : rclcpp_lifecycle::LifecycleNode("lifecycle_talker")
    {
        RCLCPP_INFO(get_logger(), "LifecycleTalker Node Start");
    }

protected:
    // 상태 전환 콜백 함수들
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(get_logger(), "Configuring from state: %s", previous_state.label().c_str());

        // Publisher와 Timer 생성
        publisher_ = create_publisher<std_msgs::msg::String>("messages", 10);
        timer_ = create_wall_timer(1s, std::bind(&LifecycleTalker::publish_message, this));

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(get_logger(), "Activating from state: %s", previous_state.label().c_str());

        // Publisher 활성화
        publisher_->on_activate();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(get_logger(), "Deactivating from state: %s", previous_state.label().c_str());

        // Publisher 비활성화
        publisher_->on_deactivate();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(get_logger(), "Cleaning up from state: %s", previous_state.label().c_str());

        // 리소스 정리
        publisher_.reset();
        timer_.reset();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(get_logger(), "Shutting down from state: %s", previous_state.label().c_str());

        // 모든 리소스 정리
        publisher_.reset();
        timer_.reset();
        return CallbackReturn::SUCCESS;
    }

private:
    void publish_message()
    {
        if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        {
            auto message = std_msgs::msg::String();
            message.data = "Lifecycle Hello World #" + std::to_string(count_++);
            publisher_->publish(message);
            RCLCPP_INFO(get_logger(), "Publishing: '%s'", message.data.c_str());
        }
    }

    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_ = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LifecycleTalker>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
