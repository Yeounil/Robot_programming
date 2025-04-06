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

class GoScan : public rclcpp::Node {
	public:
		GoScan() : Node("go_scan") {
			pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
			timer_ = this->create_wall_timer(100ms, std::bind(&GoScan::timer_cb, this));
			// sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 1, std::bind(&GoScan::scan_cb, this, _1))
			sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(1).best_effort(), std::bind(&GoScan::scan_cb, this, _1));
		}
	
	private:
		void timer_cb() {
			auto cmd = geometry_msgs::msg::Twist();
			
			if(min_dist > range_min && min_dist < range_max) {
				if (turn == 0) { // go
				cmd.linear.x = 0.2;
				cmd.angular.z = 0;
				} else if (turn == 1) { // turn left
					cmd.linear.x = 0.05;
					cmd.angular.z = 0.4;
				} else if (turn == 2) { //turn right
					cmd.linear.x = 0.05;
					cmd.angular.z = -0.4;
				}
			}
				
			pub_->publish(cmd);
		}
		
		void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
			range_ahead_ = msg->ranges[0];
			if (range_ahead_ < range_min || range_ahead_ > range_max) range_ahead_ = range_max;
			
			// 0~359 left
			for(int i = 345; i < 360; i++) { // right
				if (rrange > msg->ranges[i] && msg->ranges[i] > range_min && msg->ranges[i] < range_max) { 
					rrange = msg->ranges[i];
					//min_right = i;
				}
			}
			
			for (int i=0;i < 15; i++) { // left
				if (lrange > msg->ranges[i] && msg->ranges[i] > range_min && msg->ranges[i] < range_max) {
					lrange = msg->ranges[i];
					//min_left = i;
				}
			}
			
			
			if(lrange > rrange) {
				min_dist = rrange;
				turn = 1;  // turn left
			} else {
				min_dist = lrange;
				turn = 2; // turn right
			}
			
			if (min_dist >= range_ahead_) {
				min_dist = range_ahead_;
				turn = 1; // turn left
			}
			
			if (min_dist > 0.7) turn = 0; // no turn, go straight
			
			
			printf("range ahead: %f / lrange: %f / rrange: %f, min : %f\n", range_ahead_, lrange, rrange, min_dist);
			
			if (turn == 0) printf("Go Straght\n");
			else if (turn == 1) printf("Turn Left\n");
			else if (turn == 2) printf("Turn Right\n");
			else printf("No Info\n");
			
		}
		

		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
		float range_ahead_;
		
		float range_min = 0.2; //msg->range_min;
		float range_max = 30; //msg->range_max;
			
		float rrange = range_max; // right min distance
		float lrange = range_max; // left min distance
		float min_dist;
		int turn;	
		
};

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<GoScan>());
	rclcpp::shutdown();
	return 0;
}
