#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class avoid : public rclcpp::Node {
	public:
		avoid() : Node("avoid") {
			pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
			timer_ = this->create_wall_timer(100ms, std::bind(&avoid::timer_cb, this));
			// sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 1, std::bind(&GoScan::scan_cb, this, _1))
			sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(1).best_effort(), std::bind(&avoid::scan_cb, this, _1));
		}
	
	private:
		void timer_cb() {
			auto cmd = geometry_msgs::msg::Twist();
			
			if (turn == 0) { // go and check straight
			cmd.linear.x = 0.15;
			cmd.angular.z = 0;
			
			} else if (turn == 1) { // turn right
				if (rotate_cnt < ROTATE_DURATION) {
					cmd.linear.x = 0.0;
					cmd.angular.z = -1.0;
					rotate_cnt++;
				} else {
					rotate_cnt = 0;
					turn = 2;
		}
				
			} else if (turn == 2) { // go and check left
				cmd.linear.x = 0.15;
				cmd.angular.z = 0;
				
			} else if (turn == 3) { // turn left
				if (rotate_cnt < ROTATE_DURATION) {
					cmd.linear.x = 0.0;
					cmd.angular.z = 1.0;
					rotate_cnt++;
				} else {
					rotate_cnt = 0;
					turn = 0;
		}
			}
				
			pub_->publish(cmd);
		}
		
		void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
			if(msg->ranges[0] < range_max && msg->ranges[0] > range_min) range_ahead_ = msg->ranges[0];
			else if(msg->ranges[0] > range_max) range_ahead_ = range_max;
			
			if(msg->ranges[90] < range_max && msg->ranges[90] > range_min) range_left_ = msg->ranges[90];
			else if(msg->ranges[90] > range_max) range_left_ = range_max;
			
			printf("range[0]: %f, range[90]:%f\t", range_ahead_, range_left_);
			
			if (turn == 0 && range_ahead_ < 0.6) 
			{
				turn = 1; // turn right
			}
			
			if (turn == 2 && range_left_ > 0.6) {
				turn = 3; // turn left
			} 
			
			
			if (turn == 0) printf("turn = %d, Go Straight\n", turn);
			else if (turn == 1) printf("turn = %d, Turn Right\n", turn);
			else if (turn == 2) printf("turn = %d, Going\n", turn);
			else if (turn == 3) printf("turn = %d, Turn Left\n", turn);
			else printf("No Info\n");
			
		}
		

		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
		float range_ahead_;
		float range_left_;
		
		float range_min = 0.2; //msg->range_min;
		float range_max = 30; //msg->range_max;
			
		int rotate_cnt = 0;
		const int ROTATE_DURATION = 15.7; // 100ms x 15 = 1.5s

		int turn;	
		
};

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<avoid>());
	rclcpp::shutdown();
	return 0;
}
