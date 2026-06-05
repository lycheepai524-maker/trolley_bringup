#!/usr/bin/env python3
"""
Navigate to a named location.

Uses the move_base_simple/goal topic interface (one-shot goal publish).

Usage: rosrun trolley_bringup goto_location.py <name>
"""
import sys
import os
import math
import sqlite3
import rospy
from geometry_msgs.msg import PoseStamped


class GotoLocation:
    def __init__(self, name):
        rospy.init_node('goto_location', anonymous=True)
        self.name = name
        self.db_path = rospy.get_param(
            '~db_path', os.path.expanduser('~/trolley_fingerprints.db'))

        target = self._lookup_location(name)
        if target is None:
            rospy.logerr("No location named '%s' found in DB", name)
            sys.exit(1)

        x, y, theta = target
        rospy.loginfo("Target: '%s' at (%.2f, %.2f, %.0f deg)",
                      name, x, y, math.degrees(theta))

        # Publisher to move_base_simple/goal
        pub = rospy.Publisher('/move_base_simple/goal', PoseStamped,
                              queue_size=1)
        rospy.sleep(1.0)  # Let publisher connect

        # Wait for at least one subscriber
        waited = 0
        while pub.get_num_connections() == 0 and waited < 5:
            rospy.sleep(0.5)
            waited += 0.5
            rospy.loginfo("Waiting for move_base to subscribe...")

        if pub.get_num_connections() == 0:
            rospy.logerr("No subscribers on /move_base_simple/goal")
            sys.exit(1)

        # Build pose
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = math.sin(theta / 2.0)
        goal.pose.orientation.w = math.cos(theta / 2.0)

        rospy.loginfo("Publishing goal to /move_base_simple/goal...")
        pub.publish(goal)
        rospy.sleep(0.5)  # Make sure it goes through

        rospy.loginfo("Goal sent. Watch the trolley.")
        rospy.loginfo("To cancel, publish to /move_base/cancel.")

    def _lookup_location(self, name):
        conn = sqlite3.connect(self.db_path)
        c = conn.cursor()
        c.execute('SELECT x, y, theta FROM named_locations WHERE name = ?',
                  (name,))
        row = c.fetchone()
        conn.close()
        return row


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: rosrun trolley_bringup goto_location.py <name>")
        sys.exit(1)
    try:
        GotoLocation(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
