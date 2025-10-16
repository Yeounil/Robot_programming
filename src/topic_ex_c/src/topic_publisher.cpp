#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;

class MinPub : public rclcpp::Node
{
	public:
		MinPub()
		: Node("topic_pub"), count_(0)
		{
			publisher_ = this->create_publisher<std_msgs::msg::Int32>("count", 10);
			timer_ = this->create_wall_timer(
				500ms, std::bind(&MinPub::timer_callback, this));
		}
		
	private:
		void timer_callback()
		{
			auto message = std_msgs::msg::Int32();
			message.data = count_++;
			//RCLCPP_INFO(this->get_logger(), "Publishing: %d", message.data);
			publisher_->publish(message);
		}
		
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
		size_t count_;
		
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MinPub>());
	rclcpp::shutdown();
	return 0;
}
		
		
		
