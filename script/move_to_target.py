#!/usr/bin/env python3
import rospy
import math
import time
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
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
    rate = rospy.Rate(10)
    cmd = Twist()

    current_x, current_y, current_yaw = get_robot_pose()
    target_angle = round(math.atan2(target_y - current_y, target_x - current_x), 2)
    print(f"Target Angle: {round(math.degrees(target_angle), 2)} degrees")

    while not rospy.is_shutdown():
        _, _, current_yaw = get_robot_pose()
        angle_diff = target_angle - current_yaw
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        if abs(angle_diff) < 0.01:
            break

        cmd.angular.z = angular_speed if angle_diff > 0 else -angular_speed
        cmd.linear.x = 0
        pub.publish(cmd)
        rate.sleep()

    cmd.angular.z = 0
    pub.publish(cmd)
    print("rotation published")
    rospy.sleep(1)

    distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
    cmd.linear.x = linear_speed
    while not rospy.is_shutdown():
        current_x, current_y, _ = get_robot_pose()
        distance_error = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        if distance_error < 0.05:
            break
        cmd.linear.x = linear_speed
        cmd.angular.z = 0
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
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.sleep(1)

    for i, (x, y, wait_time) in enumerate(points):
        print(f"Going to {i+1}th point ({x}, {y})")
        current_pose = get_robot_pose()
        current_xy = (current_pose[0], current_pose[1])
        target_xy = (x, y)
        rospy.sleep(1)
        move_to_target(cmd_pub, x, y)
        print(f"Arrived at ({x}, {y}), waiting for {wait_time} seconds...")
        time.sleep(wait_time)
        
    rotate_to_angle(cmd_pub, 0.0)    
    print("Target Done")

if __name__ == "__main__":
    try:
        waypoints = [
            (1, 1, 2),
            (2, -1, 2),
            (3, 2, 5),
            (0, 0, 0)
        ]
        move_through_points(waypoints)
    except rospy.ROSInterruptException:
        pass
