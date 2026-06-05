#!/usr/bin/env python3
"""
LiDAR follower with teach-and-track mode and velocity prediction.

Subscribes:
  /scan  (sensor_msgs/LaserScan)
  /follower/command (std_msgs/String): commands "teach", "stop", "go"
Publishes:
  /cmd_vel_in (geometry_msgs/Twist)
  /follower/status (std_msgs/String): current state

States:
  IDLE       - waiting, no motion
  TEACHING   - capture next closest object in front as target
  FOLLOWING  - actively tracking target
"""
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String


STATE_IDLE = "IDLE"
STATE_TEACHING = "TEACHING"
STATE_FOLLOWING = "FOLLOWING"


class LidarFollower:
    def __init__(self):
        rospy.init_node('lidar_follower')

        # Search params
        self.search_angle = float(rospy.get_param('~search_angle', math.radians(45)))
        self.min_range = float(rospy.get_param('~min_range', 0.30))
        self.max_range = float(rospy.get_param('~max_range', 3.0))

        # Following params
        self.follow_distance = float(rospy.get_param('~follow_distance', 0.5))
        self.distance_tolerance = float(rospy.get_param('~distance_tolerance', 0.05))
        self.safety_distance = float(rospy.get_param('~safety_distance', 0.3))

        # Target tracking gates
        self.max_distance_change = float(rospy.get_param('~max_distance_change', 0.8))
        self.max_bearing_change = float(rospy.get_param('~max_bearing_change', 1.0))

        # Speed control
        self.max_lin_vel = float(rospy.get_param('~max_lin_vel', 0.5))
        self.max_ang_vel = float(rospy.get_param('~max_ang_vel', 1.5))
        self.lin_gain = float(rospy.get_param('~lin_gain', 1.2))
        self.ang_gain = float(rospy.get_param('~ang_gain', 2.0))

        # Lost target timeout
        self.lost_timeout = float(rospy.get_param('~lost_timeout', 2.5))

        # LiDAR mounting offset (radians)
        # Your LiDAR is mounted rotated relative to base_link forward.
        # Earlier calibration showed +1.7284 rad (~99 deg) offset.
        # If using a different physical setup, adjust accordingly.
        # Default math.pi (180 deg) for "LiDAR facing backwards" setups.
        self.lidar_yaw_offset = float(rospy.get_param('~lidar_yaw_offset', math.pi))

        # State
        self.state = STATE_IDLE
        self.target_distance = None
        self.target_bearing = None
        self.target_velocity_dist = 0.0
        self.target_velocity_bearing = 0.0
        self.last_seen = rospy.Time.now()
        self.last_update_time = rospy.Time.now()

        # ROS interfaces
        self.cmd_pub = rospy.Publisher('/cmd_vel_in', Twist, queue_size=1)
        self.status_pub = rospy.Publisher('/follower/status', String, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self._scan_cb, queue_size=1)
        rospy.Subscriber('/follower/command', String, self._cmd_cb, queue_size=5)

        rospy.Timer(rospy.Duration(0.5), self._broadcast_status)

        rospy.loginfo("LiDAR follower started.")
        rospy.loginfo("  Follow distance: %.2f m", self.follow_distance)
        rospy.loginfo("  Search arc: +/- %.0f deg", math.degrees(self.search_angle))
        rospy.loginfo("  LiDAR yaw offset: %.3f rad (%.0f deg)",
                      self.lidar_yaw_offset, math.degrees(self.lidar_yaw_offset))
        rospy.loginfo("  Max speeds: %.2f m/s, %.2f rad/s",
                      self.max_lin_vel, self.max_ang_vel)
        rospy.loginfo("State: IDLE. Send 'teach' to learn a target.")

        rospy.spin()

    def _cmd_cb(self, msg):
        cmd = msg.data.lower().strip()
        if cmd == "teach":
            rospy.loginfo("Received TEACH command")
            self.state = STATE_TEACHING
        elif cmd == "stop":
            rospy.loginfo("Received STOP command")
            self.state = STATE_IDLE
            self.target_distance = None
            self.target_bearing = None
            self.target_velocity_dist = 0.0
            self.target_velocity_bearing = 0.0
            self._publish_stop()
        elif cmd == "go":
            if self.target_distance is not None:
                rospy.loginfo("Received GO - resuming follow")
                self.state = STATE_FOLLOWING
            else:
                rospy.logwarn("Cannot GO - no target taught yet.")
        else:
            rospy.logwarn("Unknown command: '%s'. Use: teach, stop, go", cmd)

    def _broadcast_status(self, _evt):
        msg = String()
        if self.state == STATE_FOLLOWING and self.target_distance is not None:
            msg.data = "%s | target: %.2fm at %.0fdeg" % (
                self.state, self.target_distance,
                math.degrees(self.target_bearing))
        else:
            msg.data = self.state
        self.status_pub.publish(msg)

    def _scan_cb(self, scan):
        if self.state == STATE_IDLE:
            return

        if self.state == STATE_TEACHING:
            self._teach(scan)
            return

        self._follow(scan)

    def _teach(self, scan):
        """Capture closest object in forward arc as target."""
        dist, bearing = self._find_closest_in_arc(scan)
        if dist is None:
            rospy.logwarn("TEACH: no valid target in forward arc. Try again.")
            self.state = STATE_IDLE
            return

        self.target_distance = dist
        self.target_bearing = bearing
        self.follow_distance = dist
        self.target_velocity_dist = 0.0
        self.target_velocity_bearing = 0.0
        self.last_seen = rospy.Time.now()
        self.last_update_time = rospy.Time.now()
        self.state = STATE_FOLLOWING

        rospy.loginfo("TAUGHT target: %.2fm at %.0fdeg. Now FOLLOWING.",
                      dist, math.degrees(bearing))
        rospy.loginfo("Follow distance auto-set to %.2fm", dist)

    def _follow(self, scan):
        """Track previously-taught target with velocity prediction."""
        now = rospy.Time.now()
        dt = (now - self.last_update_time).to_sec()
        if dt < 0.001:
            dt = 0.05

        # Predict where target should be now
        predicted_dist = self.target_distance + self.target_velocity_dist * dt
        predicted_bearing = self._wrap(
            self.target_bearing + self.target_velocity_bearing * dt)

        # Search near predicted position
        dist, bearing = self._find_near_predicted(
            scan, predicted_dist, predicted_bearing)

        if dist is None:
            elapsed_lost = (now - self.last_seen).to_sec()
            if elapsed_lost > self.lost_timeout:
                rospy.logwarn_throttle(2.0,
                    "Lost target for %.1fs. Stopping." % elapsed_lost)
                self.state = STATE_IDLE
                self._publish_stop()
            else:
                # Briefly lost - keep moving toward predicted position
                lin_vel, ang_vel = self._compute_velocity(
                    predicted_dist, predicted_bearing)
                lin_vel *= 0.6
                ang_vel *= 0.7
                twist = Twist()
                twist.linear.x = lin_vel
                twist.angular.z = ang_vel
                self.cmd_pub.publish(twist)
            return

        # Update target velocity (smoothed)
        if dt > 0.001:
            new_v_dist = (dist - self.target_distance) / dt
            new_v_bearing = self._wrap(bearing - self.target_bearing) / dt
            alpha = 0.4
            self.target_velocity_dist = (
                alpha * new_v_dist + (1 - alpha) * self.target_velocity_dist)
            self.target_velocity_bearing = (
                alpha * new_v_bearing + (1 - alpha) * self.target_velocity_bearing)

        # Update target position
        self.target_distance = dist
        self.target_bearing = bearing
        self.last_seen = now
        self.last_update_time = now

        # Compute velocity
        lin_vel, ang_vel = self._compute_velocity(dist, bearing)
        twist = Twist()
        twist.linear.x = lin_vel
        twist.angular.z = ang_vel
        self.cmd_pub.publish(twist)

    def _find_closest_in_arc(self, scan):
        """Closest point within forward arc (in trolley frame)."""
        closest_dist = float('inf')
        closest_bearing = 0.0
        found = False

        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r):
                continue
            if r < self.min_range or r > self.max_range:
                continue

            raw_bearing = scan.angle_min + i * scan.angle_increment
            # Apply LiDAR mounting offset to convert to trolley frame
            bearing = self._wrap(raw_bearing + self.lidar_yaw_offset)

            if abs(bearing) > self.search_angle:
                continue

            if r < closest_dist:
                closest_dist = r
                closest_bearing = bearing
                found = True

        if not found:
            return None, None
        return closest_dist, closest_bearing

    def _find_near_predicted(self, scan, predicted_dist, predicted_bearing):
        """Find closest candidate near predicted target position."""
        best_dist = None
        best_bearing = None
        best_score = float('inf')

        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r):
                continue
            if r < self.min_range or r > self.max_range:
                continue

            raw_bearing = scan.angle_min + i * scan.angle_increment
            bearing = self._wrap(raw_bearing + self.lidar_yaw_offset)

            d_dist = abs(r - predicted_dist)
            if d_dist > self.max_distance_change:
                continue

            d_bearing = abs(self._wrap(bearing - predicted_bearing))
            if d_bearing > self.max_bearing_change:
                continue

            score = d_dist + 0.3 * d_bearing
            if score < best_score:
                best_score = score
                best_dist = r
                best_bearing = bearing

        return best_dist, best_bearing

    def _compute_velocity(self, distance, bearing):
        """Generate cmd_vel to maintain follow_distance while facing target."""
        # Emergency stop if too close
        if distance < self.safety_distance:
            return 0.0, 0.0

        # Angular: face the target
        ang_vel = self.ang_gain * bearing
        ang_vel = max(-self.max_ang_vel, min(self.max_ang_vel, ang_vel))

        # Linear: maintain follow distance
        distance_error = distance - self.follow_distance
        if abs(distance_error) < self.distance_tolerance:
            lin_vel = 0.0
        else:
            lin_vel = self.lin_gain * distance_error
            lin_vel = max(-self.max_lin_vel, min(self.max_lin_vel, lin_vel))

        # Reduce forward speed when making sharp turns
        if abs(bearing) > math.radians(45):
            lin_vel *= 0.4
        elif abs(bearing) > math.radians(25):
            lin_vel *= 0.7

        return lin_vel, ang_vel

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())

    @staticmethod
    def _wrap(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


if __name__ == '__main__':
    try:
        LidarFollower()
    except rospy.ROSInterruptException:
        pass
