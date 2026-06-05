#!/usr/bin/env python
"""
encoder_to_odom.py

Subscribes to wheel encoder tick counts from the Arduino and publishes:
  - nav_msgs/Odometry on /odom
  - tf: odom -> base_link  (only if ~publish_tf is true)

Expected encoder topics from rosserial:
  /left_ticks   (std_msgs/Int32 or Int64)   cumulative ticks, left wheel
  /right_ticks  (std_msgs/Int32 or Int64)   cumulative ticks, right wheel

If your Arduino publishes different topic names or a single combined message,
adjust the subscribers below.

Parameters (all private, ~):
  ticks_per_meter (float)  encoder ticks per meter of wheel travel
  track_width     (float)  distance between left/right wheel centers [m]
  publish_rate    (float)  Hz
  base_frame      (str)    child frame for odom msg & TF
  odom_frame      (str)    parent frame for odom msg & TF
  invert_left     (bool)   negate left tick delta
  invert_right    (bool)   negate right tick delta
  publish_tf      (bool)   if true, broadcast odom->base_link TF
"""
import math
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from std_msgs.msg import Int32, Int64

class EncoderToOdom(object):
    def __init__(self):
        # --- Parameters ---
        self.ticks_per_meter = float(rospy.get_param('~ticks_per_meter', 4848.0))
        self.track_width     = float(rospy.get_param('~track_width', 0.46))
        self.publish_rate    = float(rospy.get_param('~publish_rate', 30.0))
        self.base_frame      = rospy.get_param('~base_frame', 'base_link')
        self.odom_frame      = rospy.get_param('~odom_frame', 'odom')
        self.invert_left     = bool(rospy.get_param('~invert_left', False))
        self.invert_right    = bool(rospy.get_param('~invert_right', True))
        self.publish_tf      = bool(rospy.get_param('~publish_tf', False))

        # --- State ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vth = 0.0

        self.left_ticks = None   # cumulative, last received
        self.right_ticks = None
        self.last_left = None    # last value used in delta computation
        self.last_right = None
        self.last_time = rospy.Time.now()

        # --- Publishers / Subscribers ---
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=20)
        if self.publish_tf:
            self.tf_br = tf.TransformBroadcaster()
            rospy.loginfo("encoder_to_odom: publishing TF %s -> %s",
                          self.odom_frame, self.base_frame)
        else:
            self.tf_br = None
            rospy.loginfo("encoder_to_odom: TF publishing DISABLED")

        # Try Int32 first; if Arduino sends Int64 it'll still work via duck-typing
        # because both have .data. We just need to listen for the right topic.
        rospy.Subscriber('left_ticks',  Int32, self.left_cb,  queue_size=50)
        rospy.Subscriber('right_ticks', Int32, self.right_cb, queue_size=50)

        rospy.loginfo("encoder_to_odom: ticks/m=%.1f track=%.3f rate=%.1f Hz "
                      "invert L=%s R=%s",
                      self.ticks_per_meter, self.track_width, self.publish_rate,
                      self.invert_left, self.invert_right)

    def left_cb(self, msg):
        v = -msg.data if self.invert_left else msg.data
        self.left_ticks = v

    def right_cb(self, msg):
        v = -msg.data if self.invert_right else msg.data
        self.right_ticks = v

    def spin(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()

    def step(self):
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        if dt <= 0.0:
            return

        # Wait until we've received at least one tick reading from each wheel
        if self.left_ticks is None or self.right_ticks is None:
            self.last_time = now
            return

        # Initialise baseline on first valid pair so we don't compute a huge
        # delta from "no previous reading" to the cumulative count.
        if self.last_left is None:
            self.last_left = self.left_ticks
            self.last_right = self.right_ticks
            self.last_time = now
            return

        d_left_ticks  = self.left_ticks  - self.last_left
        d_right_ticks = self.right_ticks - self.last_right
        self.last_left  = self.left_ticks
        self.last_right = self.right_ticks

        d_left  = d_left_ticks  / self.ticks_per_meter
        d_right = d_right_ticks / self.ticks_per_meter

        d_center = 0.5 * (d_left + d_right)
        d_theta  = (d_right - d_left) / self.track_width

        # Integrate using midpoint heading
        mid_theta = self.theta + 0.5 * d_theta
        self.x += d_center * math.cos(mid_theta)
        self.y += d_center * math.sin(mid_theta)
        self.theta += d_theta
        # Wrap theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self.vx  = d_center / dt
        self.vth = d_theta  / dt
        self.last_time = now

        self.publish(now)

    def publish(self, stamp):
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, self.theta)

        # --- TF ---
        if self.tf_br is not None:
            self.tf_br.sendTransform(
                (self.x, self.y, 0.0),
                q,
                stamp,
                self.base_frame,
                self.odom_frame,
            )

        # --- Odometry message ---
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*q)

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vth

        # Diagonal covariance: small but non-zero so EKF/AMCL is happy
        odom.pose.covariance = [
            0.01, 0,    0,    0,    0,    0,
            0,    0.01, 0,    0,    0,    0,
            0,    0,    1e6,  0,    0,    0,
            0,    0,    0,    1e6,  0,    0,
            0,    0,    0,    0,    1e6,  0,
            0,    0,    0,    0,    0,    0.05,
        ]
        odom.twist.covariance = [
            0.01, 0,    0,    0,    0,    0,
            0,    0.01, 0,    0,    0,    0,
            0,    0,    1e6,  0,    0,    0,
            0,    0,    0,    1e6,  0,    0,
            0,    0,    0,    0,    1e6,  0,
            0,    0,    0,    0,    0,    0.05,
        ]

        self.odom_pub.publish(odom)


if __name__ == '__main__':
    rospy.init_node('encoder_to_odom')
    EncoderToOdom().spin()
