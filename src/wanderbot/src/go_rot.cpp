#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class GoRot : public rclcpp::Node {
	public:
		GoRot() : Node("go_rot") {
			pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
			timer_ = this->create_wall_timer(100ms, std::bind(&GoRot::timer_cb, this));
		}
	
	private:
		void timer_cb() {
			auto cmd = geometry_msgs::msg::Twist();
			cmd.linear.x = 0.1;
			pub_->publish(cmd);
		}
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<GoRot>());
	rclcpp::shutdown();
	return 0;
}
