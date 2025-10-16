#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler
import math

def make_pose(x, y, yaw_rad):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    quat = quaternion_from_euler(0, 0, yaw_rad)
    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]
    return pose

def go_and_retry(pose, max_retry=3):
    for attempt in range(1, max_retry + 1):
        navigator.goToPose(pose)
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                print(f"[INFO] ETA: {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9:.1f}s")

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"[OK] Goal reached on attempt {attempt}")
            return True
        else:
            print(f"[WARN] Goal failed (attempt {attempt})")

    print("[FAIL] All attempts failed.")
    return False


def main():
    global navigator
    rclpy.init()
    navigator = BasicNavigator()

    # 초기 위치
    initial_pose = make_pose(0.0, 0.0, 0.0)
    navigator.setInitialPose(initial_pose)
    navigator.lifecycleStartup()

    # 목표 지점 설정: ㄹ자 경로
    goals = [
        make_pose(2.0, 0.0, -math.pi/2),         # G1: 직선 후 우회전
        make_pose(2.0, -1.0, -math.pi/2),        # G2: 직진 후 우회전
        make_pose(0.0, -2.0, math.pi),        # G3: 직진 끝
        make_pose(2.0, -2.0, math.pi),    # G4: 좌회전해서 복귀
    ]

    for i, goal in enumerate(goals):
        print(f"[MOVE] Heading to Goal {i+1}")
        success = go_and_retry(goal)
        if not success:
            break

    navigator.lifecycleShutdown()
    print("[DONE] All goals processed.")
    exit(0)

if __name__ == '__main__':
    main()
