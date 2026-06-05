#!/usr/bin/env python
"""
random_explorer.py

Autonomous wandering: picks a safe direction from the laser scan, sends a
goal in that direction to move_base, and repeats. move_base + DWA handles
the actual obstacle avoidance.

Topics / Actions:
  subscribes : /scan         (sensor_msgs/LaserScan)
  uses tf    : odom -> base_link
  actions    : /move_base    (move_base_msgs/MoveBaseAction)

Params:
  ~min_goal_dist   (float, 1.5)   min distance to project goal [m]
  ~max_goal_dist   (float, 3.0)   max distance to project goal [m]
  ~safety_clearance(float, 0.8)   discard directions closer than this [m]
  ~goal_timeout    (float, 30.0)  max time to wait for a goal [s]
  ~rest_time       (float, 1.0)   pause between goals [s]
  ~front_arc_deg   (float, 180.0) only consider directions in this arc
                                  centered on robot heading [deg]
"""
import math
import random

import numpy as np
import rospy
import actionlib
import tf2_ros
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler


class RandomExplorer(object):
    def __init__(self):
        # --- Params ---
        self.min_dist  = float(rospy.get_param('~min_goal_dist',    1.5))
        self.max_dist  = float(rospy.get_param('~max_goal_dist',    3.0))
        self.clearance = float(rospy.get_param('~safety_clearance', 0.8))
        self.goal_timeout = float(rospy.get_param('~goal_timeout', 30.0))
        self.rest_time = float(rospy.get_param('~rest_time',        1.0))
        self.front_arc = math.radians(
            float(rospy.get_param('~front_arc_deg', 180.0))
        )
        # Set to math.pi (3.14159) if laser angles need a 180-deg flip to
        # align with base_link's +x. Set to 0.0 if the TF already handles it.
        self.laser_offset = float(rospy.get_param('~laser_offset_rad', 0.0))

        self.scan = None
        rospy.Subscriber('/scan', LaserScan, self.scan_cb, queue_size=1)

        # TF to read robot's pose in odom
        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf)

        # Connect to move_base action server
        rospy.loginfo("random_explorer: waiting for move_base action server...")
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo("random_explorer: connected to move_base.")

        # Need a scan before doing anything
        rospy.loginfo("random_explorer: waiting for first /scan ...")
        while self.scan is None and not rospy.is_shutdown():
            rospy.sleep(0.2)
        rospy.loginfo("random_explorer: ready. min=%.1f max=%.1f clr=%.1f",
                      self.min_dist, self.max_dist, self.clearance)

    def scan_cb(self, msg):
        self.scan = msg

    def get_robot_pose_in_odom(self):
        """Returns (x, y, yaw) in the odom frame, or None on TF failure."""
        try:
            t = self.tf_buf.lookup_transform(
                'odom', 'base_link', rospy.Time(0), rospy.Duration(0.5))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF lookup failed: %s", e)
            return None
        q = t.transform.rotation
        # yaw from quaternion (planar)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return t.transform.translation.x, t.transform.translation.y, yaw

    def pick_safe_direction(self):
        """Return (relative_angle_to_robot, projected_distance) for the
        best direction, or None if nothing is safe."""
        s = self.scan
        ranges = np.array(s.ranges, dtype=np.float32)
        # Replace NaN/inf with a large finite distance so we PREFER unknown space
        # (laser sees nothing -> probably wide open)
        large = float(s.range_max if s.range_max > 0 else 10.0)
        ranges = np.where(np.isfinite(ranges) & (ranges > s.range_min),
                          ranges, large)

        # Angles for each beam, in the laser frame.
        # With the 180-deg-yaw TF you set up, the laser frame's "0 deg" points
        # *backward* in base_link, so we have to compensate. We want the goal
        # in the BASE_LINK frame's forward direction. The simplest robust
        # approach: convert each beam to a unit vector in base_link by rotating
        # by pi (since laser is rotated 180 deg relative to base_link).
        angles_laser = s.angle_min + np.arange(len(ranges)) * s.angle_increment
        angles_base  = angles_laser + self.laser_offset
        # wrap to [-pi, pi]
        angles_base  = np.arctan2(np.sin(angles_base), np.cos(angles_base))

        # Mask: only beams within front_arc, and only beams with > clearance free
        half_arc = self.front_arc / 2.0
        mask = (np.abs(angles_base) <= half_arc) & (ranges > self.clearance)
        if not np.any(mask):
            return None

        # Score: pick the direction with the most clearance. To avoid always
        # going straight, take the top-N candidates and pick one randomly.
        candidate_idx = np.where(mask)[0]
        candidate_ranges = ranges[candidate_idx]
        # Top 20% of free directions
        N = max(1, int(0.2 * len(candidate_idx)))
        top = candidate_idx[np.argsort(-candidate_ranges)[:N]]
        chosen = random.choice(top.tolist())

        ang  = float(angles_base[chosen])
        dist = float(ranges[chosen])
        # Project goal: clip to [min_dist, max_dist] but never closer than
        # (range - safety_padding)
        safe_dist = min(self.max_dist, max(self.min_dist, dist * 0.6))
        return ang, safe_dist

    def send_goal(self, x, y, yaw):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'odom'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation = Quaternion(*q)

        rospy.loginfo("random_explorer: -> goal (%.2f, %.2f, yaw=%.2f)",
                      x, y, yaw)
        self.client.send_goal(goal)
        ok = self.client.wait_for_result(rospy.Duration(self.goal_timeout))
        if not ok:
            rospy.logwarn("random_explorer: goal timed out, cancelling.")
            self.client.cancel_goal()
            return False
        state = self.client.get_state()
        # 3 = SUCCEEDED
        rospy.loginfo("random_explorer: goal finished, state=%d", state)
        return state == 3

    def spin(self):
        while not rospy.is_shutdown():
            pose = self.get_robot_pose_in_odom()
            if pose is None:
                rospy.sleep(0.5)
                continue
            x0, y0, yaw0 = pose

            pick = self.pick_safe_direction()
            if pick is None:
                rospy.logwarn("random_explorer: no safe direction. "
                              "Backing up briefly...")
                # Try a small reverse goal — DWA will likely refuse if obstacles
                # are behind, but worth trying to escape.
                bx = x0 - 0.5 * math.cos(yaw0)
                by = y0 - 0.5 * math.sin(yaw0)
                self.send_goal(bx, by, yaw0)
                rospy.sleep(self.rest_time)
                continue

            rel_ang, dist = pick
            goal_yaw = yaw0 + rel_ang
            gx = x0 + dist * math.cos(goal_yaw)
            gy = y0 + dist * math.sin(goal_yaw)

            self.send_goal(gx, gy, goal_yaw)
            rospy.sleep(self.rest_time)


if __name__ == '__main__':
    rospy.init_node('random_explorer')
    try:
        RandomExplorer().spin()
    except rospy.ROSInterruptException:
        pass
