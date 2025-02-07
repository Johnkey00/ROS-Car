#!/usr/bin/env python3

import rospy
import math
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf

def get_robot_pose():
    msg = rospy.wait_for_message('/odom', Odometry)
    position = msg.pose.pose.position
    orientation_q = msg.pose.pose.orientation
    _, _, yaw = tf.transformations.euler_from_quaternion(
        [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    )
    return position.x, position.y, yaw

def move_to_target(pub, target_x, target_y, linear_speed=0.5, angular_speed=0.5):
    rate = rospy.Rate(10)  # 10 Hz
    cmd = Twist()

    current_x, current_y, current_yaw = get_robot_pose()

    target_angle = math.atan2(target_y - current_y, target_x - current_x)
    print(f"Target Angle: {math.degrees(target_angle)} degrees")

    while not rospy.is_shutdown():
        _, _, current_yaw = get_robot_pose()
        angle_diff = target_angle - current_yaw

        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        if abs(angle_diff) < 0.05:
            break

        cmd.angular.z = angular_speed if angle_diff > 0 else -angular_speed
        cmd.linear.x = 0
        pub.publish(cmd)
        rate.sleep()

    cmd.angular.z = 0
    pub.publish(cmd)

    rospy.sleep(1)

    distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
    move_time = distance / linear_speed

    cmd.linear.x = linear_speed
    start_time = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start_time < move_time and not rospy.is_shutdown():
        pub.publish(cmd)
        rate.sleep()

    cmd.linear.x = 0
    pub.publish(cmd)

    print(f"Arrived at ({target_x}, {target_y})")

def rotate_to_angle(pub, target_yaw, angular_speed=0.5):
    rate = rospy.Rate(10)
    cmd = Twist()

    while not rospy.is_shutdown():
        _, _, current_yaw = get_robot_pose()
        angle_diff = target_yaw - current_yaw

        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        if abs(angle_diff) < 0.05:
            break

        cmd.angular.z = angular_speed if angle_diff > 0 else -angular_speed
        pub.publish(cmd)
        rate.sleep()

    cmd.angular.z = 0
    pub.publish(cmd)
    print(f"Reached final orientation: {math.degrees(target_yaw)} degrees")

def move_through_points(points):
    rospy.init_node('move_robot', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    for i, (x, y, wait_time) in enumerate(points):
        print(f"Going to {i+1}th point ({x}, {y})")
        move_to_target(pub, x, y)
        print(f"Arrived at({x}, {y}), Wait for {wait_time} seconds...")
        time.sleep(wait_time)
        
    rotate_to_angle(pub, 0.0)    
    print("Target Done")

if __name__ == "__main__":
    try:
        waypoints = [
            (1, 1, 2),
            (2, -1, 2),
            (3, 2, 5),
            (0, 0, 2)
        ]
        move_through_points(waypoints)
    except rospy.ROSInterruptException:
        pass


