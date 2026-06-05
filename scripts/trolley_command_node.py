#!/usr/bin/env python3
"""
Trolley Command Node - MQTT to ROS bridge
"""
import json
import math
import threading
import rospy
import paho.mqtt.client as mqtt

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from actionlib_msgs.msg import GoalID


class TrolleyCommandNode:
    def __init__(self):
        rospy.init_node('trolley_command_node')

        self.mqtt_host = rospy.get_param('~mqtt_host', 'localhost')
        self.mqtt_port = int(rospy.get_param('~mqtt_port', 1883))
        self.cmd_topic    = rospy.get_param('~cmd_topic',   'trolley/command')
        self.scan_topic   = rospy.get_param('~scan_topic',  'trolley/wifi_scan')
        self.status_topic = rospy.get_param('~status_topic','trolley/status')
        self.status_period = float(rospy.get_param('~status_period', 2.0))

        rospy.loginfo("trolley_command_node params:")
        rospy.loginfo("  mqtt_host: %s:%d", self.mqtt_host, self.mqtt_port)
        rospy.loginfo("  command topic: %s",  self.cmd_topic)
        rospy.loginfo("  scan topic:    %s",  self.scan_topic)
        rospy.loginfo("  status topic:  %s",  self.status_topic)

        self.state = 'IDLE'
        self.last_resolved_xy = None
        self.last_goal_xy = None
        self.current_pose = None
        self.lock = threading.RLock()

        self.scan_query_pub = rospy.Publisher('/wifi_resolver/query', String, queue_size=1)
        self.move_base_goal_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)
        self.move_base_cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        rospy.Subscriber('/wifi_resolver/result', String, self._on_resolver_result, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self._on_odom, queue_size=1)
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self._on_move_base_result, queue_size=1)

        self.mqtt_client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id='trolley_command_node')
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_message = self._on_mqtt_message
        try:
            self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, keepalive=30)
        except Exception as e:
            rospy.logerr("MQTT connect failed: %s", str(e))
            raise SystemExit(1)

        # Drive MQTT network loop from a ROS timer (single-threaded, reliable)
        # Run MQTT loop in a dedicated daemon thread
        # Use paho's built-in network thread (loop_start)
        self.mqtt_client.loop_start()
        # Periodic status publisher (every 2s)
        self.status_timer = rospy.Timer(rospy.Duration(self.status_period), self._publish_status)

        rospy.loginfo("trolley_command_node ready. Current state: %s", self.state)

    def _on_mqtt_connect(self, client, userdata, flags, reason_code, properties):
        rospy.loginfo("MQTT connected (reason_code=%s)", str(reason_code))
        client.subscribe(self.cmd_topic)
        client.subscribe(self.scan_topic)
        rospy.loginfo("Subscribed to %s and %s", self.cmd_topic, self.scan_topic)

    def _on_mqtt_message(self, client, userdata, msg):
        topic = msg.topic
        try:
            payload = msg.payload.decode('utf-8').strip()
        except Exception:
            rospy.logwarn("Non-UTF8 MQTT payload on %s", topic)
            return

        rospy.loginfo("MQTT MSG on %s: %s", topic, payload[:200])

        if topic == self.cmd_topic:
            self._handle_command(payload)
        elif topic == self.scan_topic:
            self._handle_wifi_scan(payload)
        else:
            rospy.logwarn("Unknown MQTT topic: %s", topic)

    def _handle_command(self, payload):
        cmd = payload.lower()
        if payload.startswith('{'):
            try:
                obj = json.loads(payload)
                cmd = obj.get('cmd', '').lower()
            except Exception:
                pass

        rospy.loginfo("Received command: %s", cmd)

        with self.lock:
            if cmd in ('idle', 'stop'):
                self._set_state('IDLE')
                self._cancel_goal()
                self._publish_zero_velocity()
            elif cmd == 'find_me':
                self._set_state('FIND_ME')
                self._publish_status_now('Waiting for WiFi scan')
            elif cmd == 'follow_me':
                self._set_state('FOLLOW_ME')
                self._publish_status_now('Waiting for WiFi scans')
            else:
                rospy.logwarn("Unknown command: %s", cmd)
                self._publish_status_now('Unknown command: ' + cmd)

    def _handle_wifi_scan(self, payload):
        try:
            obj = json.loads(payload)
        except Exception as e:
            rospy.logwarn("Bad WiFi scan JSON: %s", str(e))
            return

        if 'scan' not in obj:
            rospy.logwarn("WiFi scan missing 'scan' field")
            return

        rospy.loginfo("Forwarding WiFi scan with %d APs to resolver", len(obj['scan']))
        self.scan_query_pub.publish(String(data=json.dumps(obj)))

    def _on_resolver_result(self, msg):
        try:
            res = json.loads(msg.data)
        except Exception:
            rospy.logwarn("Bad resolver result")
            return

        if 'error' in res:
            rospy.logwarn("Resolver error: %s", res['error'])
            self._publish_status_now('Resolver error: ' + res['error'])
            return

        x, y = res['x'], res['y']
        conf = res.get('confidence', 0.0)
        rospy.loginfo("Resolver returned (%.2f, %.2f) conf=%.2f", x, y, conf)

        with self.lock:
            self.last_resolved_xy = (x, y, conf)
            if self.state in ('FIND_ME', 'FOLLOW_ME'):
                self._send_goal(x, y)

    def _on_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        with self.lock:
            self.current_pose = (x, y, yaw)

    def _on_move_base_result(self, msg):
        status = msg.status.status
        STATUS_MAP = {0:'PENDING',1:'ACTIVE',2:'PREEMPTED',3:'SUCCEEDED',
                      4:'ABORTED',5:'REJECTED',6:'PREEMPTING',7:'RECALLING',
                      8:'RECALLED',9:'LOST'}
        status_str = STATUS_MAP.get(status, 'UNKNOWN')
        rospy.loginfo("move_base result: %s", status_str)
        if status == 3:
            self._publish_status_now('Goal reached')
        elif status in (4, 5):
            self._publish_status_now('Goal failed: ' + status_str)

    def _set_state(self, new_state):
        if self.state != new_state:
            rospy.loginfo("State change: %s -> %s", self.state, new_state)
            self.state = new_state

    def _send_goal(self, x, y):
        rospy.loginfo("Sending move_base goal: (%.2f, %.2f)", x, y)
        goal = MoveBaseActionGoal()
        goal.header.stamp = rospy.Time.now()
        goal.goal_id.stamp = rospy.Time.now()
        goal.goal_id.id = 'trolley_cmd_' + str(rospy.Time.now().to_nsec())
        goal.goal.target_pose.header.frame_id = 'map'
        goal.goal.target_pose.header.stamp = rospy.Time.now()
        goal.goal.target_pose.pose.position.x = x
        goal.goal.target_pose.pose.position.y = y
        goal.goal.target_pose.pose.orientation.w = 1.0
        self.move_base_goal_pub.publish(goal)
        self.last_goal_xy = (x, y)
        self._publish_status_now('Navigating to (%.2f, %.2f)' % (x, y))

    def _cancel_goal(self):
        cancel = GoalID()
        self.move_base_cancel_pub.publish(cancel)
        self.last_goal_xy = None

    def _publish_zero_velocity(self):
        twist = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.05)

    def _publish_status(self, event):
        self._publish_status_now()

    def _publish_status_now(self, message=None):
        with self.lock:
            status = {
                'state': self.state,
                'message': message or '',
            }
            if self.current_pose is not None:
                status['robot_pose'] = {
                    'x': round(self.current_pose[0], 3),
                    'y': round(self.current_pose[1], 3),
                    'yaw_deg': round(math.degrees(self.current_pose[2]), 1),
                }
            if self.last_resolved_xy is not None:
                status['last_resolved'] = {
                    'x': round(self.last_resolved_xy[0], 3),
                    'y': round(self.last_resolved_xy[1], 3),
                    'confidence': round(self.last_resolved_xy[2], 3),
                }
            if self.last_goal_xy is not None:
                status['current_goal'] = {
                    'x': round(self.last_goal_xy[0], 3),
                    'y': round(self.last_goal_xy[1], 3),
                }
        try:
            self.mqtt_client.publish(self.status_topic, json.dumps(status), qos=0)
        except Exception as e:
            rospy.logwarn("MQTT publish failed: %s", str(e))


if __name__ == '__main__':
    try:
        TrolleyCommandNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
