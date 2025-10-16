#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler

def main():
	rclpy.init()
	navigator = BasicNavigator()
	
	# set initial pose
	initial_pose = PoseStamped()
	initial_pose.header.frame_id = 'map'
	initial_pose.header.stamp = navigator.get_clock().now().to_msg()
	initial_pose.pose.position.x = 0.0
	initial_pose.pose.position.y = 0.0
	
	quat = quaternion_from_euler(0,0,0)
	initial_pose.pose.orientation.x = quat[0]
	initial_pose.pose.orientation.y = quat[1]
	initial_pose.pose.orientation.z = quat[2]
	initial_pose.pose.orientation.w = quat[3]
	navigator.setInitialPose(initial_pose)
	
	
	#navigator.waitUntilNav2Active() # autostart
	navigator.lifecycleStartup() # no autostart
	
	# change map
	#navigator.changeMap('/home/kohjj27/map.yaml')
	
	# clear
	# navigator.clearAllcostmaps()
	
	#goto our demos first goal pose
	#goal1
	goal_pose1 = PoseStamped()
	goal_pose1.header.frame_id = 'map'
	goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
	
	goal_pose1.pose.position.x = 1.9
	goal_pose1.pose.position.y = 0.0
	quat = quaternion_from_euler(0,0,-3.1416/2)
	
	goal_pose1.pose.orientation.x = quat[0]
	goal_pose1.pose.orientation.y = quat[1]
	goal_pose1.pose.orientation.z = quat[2]
	goal_pose1.pose.orientation.w = quat[3]
	
	
	#goal2
	goal_pose2 = PoseStamped()
	goal_pose2.header.frame_id = 'map'
	goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
	
	goal_pose2.pose.position.x = 1.9
	goal_pose2.pose.position.y = -1.0
	quat = quaternion_from_euler(0,0,-3.1416/2)
	
	goal_pose2.pose.orientation.x = quat[0]
	goal_pose2.pose.orientation.y = quat[1]
	goal_pose2.pose.orientation.z = quat[2]
	goal_pose2.pose.orientation.w = quat[3]
	
	'''
	#goal3
	goal_pose3 = PoseStamped()
	goal_pose3.header.frame_id = 'map'
	goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
	
	goal_pose3.pose.position.x = -0.2
	goal_pose3.pose.position.y = -1.0
	quat = quaternion_from_euler(0,0,3.1416/2)
	
	goal_pose3.pose.orientation.x = quat[0]
	goal_pose3.pose.orientation.y = quat[1]
	goal_pose3.pose.orientation.z = quat[2]
	goal_pose3.pose.orientation.w = quat[3]
	'''
	
	#goal4	
	goal_pose4 = PoseStamped()
	goal_pose4.header.frame_id = 'map'
	goal_pose4.header.stamp = navigator.get_clock().now().to_msg()
	
	goal_pose4.pose.position.x = -0.0
	goal_pose4.pose.position.y = -2.0
	quat = quaternion_from_euler(0,0,3.1416/2)
	
	goal_pose4.pose.orientation.x = quat[0]
	goal_pose4.pose.orientation.y = quat[1]
	goal_pose4.pose.orientation.z = quat[2]
	goal_pose4.pose.orientation.w = quat[3]
	
	# goal5
	goal_pose5 = PoseStamped()
	goal_pose5.header.frame_id = 'map'
	goal_pose5.header.stamp = navigator.get_clock().now().to_msg()
	
	goal_pose5.pose.position.x = 1.9
	goal_pose5.pose.position.y = -2.0
	quat = quaternion_from_euler(0,0,0)
	
	goal_pose5.pose.orientation.x = quat[0]
	goal_pose5.pose.orientation.y = quat[1]
	goal_pose5.pose.orientation.z = quat[2]
	goal_pose5.pose.orientation.w = quat[3]
	
	
	#sanity check a valid path exists
	# path = navigator.getPath(initial_pose, goal_pose)
	
	navigator.goToPose(goal_pose1)
	
	i = 0
	while not navigator.isTaskComplete():
	
		###
		# implement some code for your application
		#####################
		
		#do something with feedback
		i = i+1
		feedback = navigator.getFeedback()
		if feedback and i % 5 == 0:
			print(
				'Estimated time of arrival:  ' + '{0:.0f}'.format(
				Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) + ' seconds.'
			)
			
			# timeout to demo cancellation
			if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
				navigator.cancelTask()
	
	result = navigator.getResult()
	if result == TaskResult.SUCCEEDED: 
		print('Goal succeeded!')
	elif result == TaskResult.CANCELED:
		print('Goal was canceled!')
	elif result == TaskResult.FAILED:
		print('Goal failed!')
	else:
		print('Goal1 has an invalid return status!')
	
			
	navigator.goToPose(goal_pose2)
	
	i = 0
	while not navigator.isTaskComplete():
	
		###
		# implement some code for your application
		#####################
		
		#do something with feedback
		i = i+1
		feedback = navigator.getFeedback()
		if feedback and i % 5 == 0:
			print(
				'Estimated time of arrival:  ' + '{0:.0f}'.format(
				Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) + ' seconds.'
			)
			
			# timeout to demo cancellation
			if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
				navigator.cancelTask()
	
	result = navigator.getResult()
	if result == TaskResult.SUCCEEDED: 
		print('Goal succeeded!')
	elif result == TaskResult.CANCELED:
		print('Goal was canceled!')
	elif result == TaskResult.FAILED:
		print('Goal failed!')
	else:
		print('Goal2 has an invalid return status!')
	
	
	
	'''		
	navigator.goToPose(goal_pose3)
	i = 0
	while not navigator.isTaskComplete():
	
		###
		# implement some code for your application
		#####################
		
		#do something with feedback
		i = i+1
		feedback = navigator.getFeedback()
		if feedback and i % 5 == 0:
			print(
				'Estimated time of arrival:  ' + '{0:.0f}'.format(
				Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) + ' seconds.'
			)
			
			# timeout to demo cancellation
			if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
				navigator.cancelTask()
	'''
	
	
	
	result = navigator.getResult()
	if result == TaskResult.SUCCEEDED: 
		print('Goal succeeded!')
	elif result == TaskResult.CANCELED:
		print('Goal was canceled!')
	elif result == TaskResult.FAILED:
		print('Goal failed!')
	else:
		print('Goal3 has an invalid return status!')
			
	navigator.goToPose(goal_pose4)
	i = 0
	while not navigator.isTaskComplete():

		i = i+1
		feedback = navigator.getFeedback()
		if feedback and i % 5 == 0:
			print(
				'Estimated time of arrival:  ' + '{0:.0f}'.format(
				Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) + ' seconds.'
			)
			
			# timeout to demo cancellation
			if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
				navigator.cancelTask()
	
	result = navigator.getResult()
	if result == TaskResult.SUCCEEDED: 
		print('Goal succeeded!')
	elif result == TaskResult.CANCELED:
		print('Goal was canceled!')
	elif result == TaskResult.FAILED:
		print('Goal failed!')
	else:
		print('Goal has an invalid return status!')
	
	
			
	navigator.goToPose(goal_pose5)
	i = 0
	while not navigator.isTaskComplete():
	
		###
		# implement some code for your application
		#####################
		
		#do something with feedback
		i = i+1
		feedback = navigator.getFeedback()
		if feedback and i % 5 == 0:
			print(
				'Estimated time of arrival:  ' + '{0:.0f}'.format(
				Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) + ' seconds.'
			)
			
			# timeout to demo cancellation
			if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
				navigator.cancelTask()
			
			
			
	# depending on the return code
	result = navigator.getResult()
	if result == TaskResult.SUCCEEDED: 
		print('Goal succeeded!')
	elif result == TaskResult.CANCELED:
		print('Goal was canceled!')
	elif result == TaskResult.FAILED:
		print('Goal failed!')
	else:
		print('Goal has an invalid return status!')
		
	navigator.lifecycleShutdown()
	
	exit(0)
	

if __name__ == '__main__':
	main()
	
				
		
		
	
	
	
	
