#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"

using std::placeholders::_1;

class MinSub : public rclcpp::Node
{
	public:
		MinSub()
		: Node("topic_sub")
		{
			subscription_ = this->create_subscription<std_msgs::msg::Int32>(
			"count", 10, std::bind(&MinSub::topic_callback, this, _1));
		}
		
	private:
		void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) const
		{
			//RCLCPP_INFO(this->get_logger(), "I heard: %d", msg->data);
			printf("%d\n", msg->data);
		}
		rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MinSub>());
	rclcpp::shutdown();
	return 0;
}
