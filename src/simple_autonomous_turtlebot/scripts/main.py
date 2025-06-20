#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib
import numpy as np
import sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Define all zones
all_zones = {
    "kitchen": {
        "top_left": (8.9, 2.7),
        "bottom_right": (5.4, -4.3),
        "step": 0.4
    },
    "living": {
        "top_left": (4.4, 5.1),
        "bottom_right": (-2.3, -3.8),
        "step": 0.4
    },
    "bedroom": {
        "top_left": (-2.6, 1.7),
        "bottom_right": (-8.9, -4.0),
        "step": 0.4
    }
}

close_obstacle = False

def scan_callback(msg):
    global close_obstacle
    ranges = [r for r in msg.ranges if r > 0.0]
    min_dist = min(ranges) if ranges else 99.0
    close_obstacle = min_dist < 0.4
    if close_obstacle:
        rospy.logwarn("⚠️ Obstacle too close: %.2f m", min_dist)

def generate_zone_grid(top_left, bottom_right, step):
    x1, y1 = top_left
    x2, y2 = bottom_right
    x_min, x_max = sorted([x1, x2])
    y_min, y_max = sorted([y1, y2])
    x_vals = np.arange(x_min, x_max, step)
    y_vals = np.arange(y_min, y_max, step)

    grid = []
    for i, x in enumerate(x_vals):
        row = [(x, y) for y in (y_vals if i % 2 == 0 else reversed(y_vals))]
        grid.extend(row)
    return grid

def send_goal(x, y, timeout=20):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("⏳ Waiting for move_base...")
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0

    rospy.loginfo("🚶 Moving to (%.2f, %.2f)", x, y)
    client.send_goal(goal)
    success = client.wait_for_result(rospy.Duration(timeout))
    if success and client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("✅ Arrived at (%.2f, %.2f)", x, y)
        return True
    client.cancel_goal()
    return False

def perform_recovery():
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    twist = Twist()

    rospy.logwarn("🔄 Performing recovery...")

    twist.linear.x = -0.2
    pub.publish(twist)
    rospy.sleep(2.0)

    twist.linear.x = 0
    twist.angular.z = 0.6
    pub.publish(twist)
    rospy.sleep(2.5)

    twist.angular.z = 0
    pub.publish(twist)
    rospy.loginfo("↩️ Recovery complete.")

def try_shifted_goals(x, y, timeout=15):
    offsets = [
        (0.0, 0.0),
        (0.2, 0.0), (-0.2, 0.0),
        (0.0, 0.2), (0.0, -0.2),
        (0.2, 0.2), (-0.2, -0.2)
    ]
    for dx, dy in offsets:
        if send_goal(x + dx, y + dy, timeout):
            return True
    perform_recovery()
    return False

if __name__ == '__main__':
    rospy.init_node("selectable_zone_cleaner")
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.loginfo("🤖 Zone-cleaning robot started.")

    # Zone selection
    if len(sys.argv) > 1:
        selected_zone = sys.argv[1]
        if selected_zone in all_zones:
            zones = [(selected_zone, all_zones[selected_zone])]
            rospy.loginfo("📍 Selected zone: %s", selected_zone)
        else:
            rospy.logerr("❌ Zone '%s' not defined. Available: %s", selected_zone, ", ".join(all_zones.keys()))
            sys.exit(1)
    else:
        zones = list(all_zones.items())
        rospy.loginfo("🧭 No zone specified. Cleaning ALL zones.")

    # Generate waypoints
    all_points = []
    for name, config in zones:
        grid = generate_zone_grid(config["top_left"], config["bottom_right"], config["step"])
        rospy.loginfo("📦 Zone '%s': %d points", name, len(grid))
        all_points.extend(grid)

    rate = rospy.Rate(0.01)

    while not rospy.is_shutdown():
        skipped = []
        for (x, y) in all_points:
            if rospy.is_shutdown():
                break
            if close_obstacle:
                rospy.logwarn("⚠️ Obstacle near (%.2f, %.2f). Skipping...", x, y)
                perform_recovery()
                skipped.append((x, y))
                continue
            if not try_shifted_goals(x, y):
                rospy.logwarn("🚫 Unreachable: (%.2f, %.2f)", x, y)
                skipped.append((x, y))
        rospy.loginfo("✅ Pass complete. Skipped %d points. Restarting...", len(skipped))
        rate.sleep()

