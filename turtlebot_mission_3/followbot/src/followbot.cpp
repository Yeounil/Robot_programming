#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class FollowBot : public rclcpp::Node {
	public:
		FollowBot() : Node("followbot") {
			pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
			timer_ = this->create_wall_timer(100ms, std::bind(&FollowBot::timer_cb, this));
			sub_ = this->create_subscription<sensor_msgs::msg::Image>("image_raw", 1, std::bind(&FollowBot::img_cb, this, _1));
		}
	
	private:
		void timer_cb() {
			auto cmd = geometry_msgs::msg::Twist();
			float Kp_, Kd_;
			float abs_err = std::abs(err_);
			//cmd.linear.x = 0.4;
			float derivative = err_ - prev_err_;
			if(abs_err < 50){
				Kp_ = 0.003;
				Kd_ = 0.0005;
			} else if(abs_err < 150){
				Kp_ = 0.005;
				Kd_ = 0.001;
			} else{
				Kp_ = 0.007;
				Kd_ = 0.0015;
			}
			//cmd.angular.z = -(Kp_ * err_ + Kd_ *derivative);
			//if(std::abs(cmd.angular.z) > 1) cmd.linear.x = 0.15;
			cmd.linear.x = 0.22;
			cmd.angular.z = -(0.008 * err_ + 0.001 * derivative);
			printf("cmd.linear.x : %.2f, err : %.2f, angular.z : %f \n", cmd.linear.x, err_, cmd.angular.z);
			prev_err_ = err_;
			pub_ -> publish(cmd);
		}
		
		void img_cb(const sensor_msgs::msg::Image::SharedPtr msg) {
			cv_bridge::CvImagePtr cv_ptr;
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			int h = cv_ptr->image.rows;
			int w = cv_ptr->image.cols;
			cv::Mat hsv_img;
			cv::GaussianBlur(cv_ptr->image, cv_ptr->image, cv::Size(5, 5), 0);
			cvtColor(cv_ptr->image, hsv_img, CV_BGR2HSV);
			
			cv::Mat mask;
			cv::Scalar lower_yellow = cv::Scalar(20, 100, 100);
			cv::Scalar upper_yellow = cv::Scalar(40, 255, 255);
			inRange(hsv_img, lower_yellow, upper_yellow, mask);
			
			int search_top = 3 * h / 4;
			int search_bot = 3 * h / 4  + 45;

			
			for(int y=0; y < h; y++){
				for(int x=0; x < w; x++){
					if(y<search_top) mask.at<uchar>(y,x) = 0;
					else if(y >= search_bot) mask.at<uchar>(y,x) = 0; 
				}
			}
			cv::Moments m = moments(mask);
			if(m.m00 > 0){
				int cx = m.m10 / m.m00;
				int cy = m.m01 / m.m00;
				cv::circle(cv_ptr->image, cv::Point(cx, cy), 20, CV_RGB(255,0,0), -1);
				err_ = cx - w/2;
			}
			//imshow("road window", cv_ptr->image);
			cv::waitKey(3);
		}
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
		float err_;
		float prev_err_ = 0.0;
};

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<FollowBot>());
	rclcpp::shutdown();
	return 0;
}
			
		
		
