#!/usr/bin/env python3
"""
Record the current robot pose as a named location.

Usage: rosrun trolley_bringup record_location.py <name>
Example: rosrun trolley_bringup record_location.py A
"""
import sys
import os
from datetime import datetime
import math
import sqlite3
import subprocess
import re
import rospy
import tf2_ros


class LocationRecorder:
    def __init__(self, name):
        rospy.init_node('record_location', anonymous=True)
        self.name = name
        self.db_path = rospy.get_param(
            '~db_path', os.path.expanduser('~/trolley_fingerprints.db'))
        self.wifi_interface = rospy.get_param('~wifi_interface', 'wlan0')

        # Get pose from TF (map -> base_link)
        self.pose = self._get_pose_from_tf()
        if self.pose is None:
            rospy.logerr("Could not get pose from TF. "
                         "Is hector_mapping running?")
            sys.exit(1)

        rospy.loginfo("Recording location '%s' at (%.2f, %.2f)",
                      self.name, self.pose[0], self.pose[1])

        # Take a WiFi snapshot
        fp_id = self._record_wifi_snapshot()

        # Save to named_locations
        conn = sqlite3.connect(self.db_path)
        c = conn.cursor()
        c.execute('''
            INSERT OR REPLACE INTO named_locations
            (name, x, y, theta, fingerprint_id)
            VALUES (?, ?, ?, ?, ?)
        ''', (self.name, self.pose[0], self.pose[1], self.pose[2], fp_id))
        conn.commit()
        conn.close()

        rospy.loginfo("Saved location '%s' to DB. fingerprint_id=%s",
                      self.name, fp_id)

    def _get_pose_from_tf(self):
        rospy.loginfo("Waiting for map->base_link TF...")
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        rospy.sleep(1.0)

        timeout = rospy.Time.now() + rospy.Duration(10.0)
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            try:
                tf = tf_buffer.lookup_transform(
                    'map', 'base_link', rospy.Time(0), rospy.Duration(0.5))
                x = tf.transform.translation.x
                y = tf.transform.translation.y
                q = tf.transform.rotation
                yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y),
                                 1.0 - 2.0*(q.y*q.y + q.z*q.z))
                rospy.loginfo("Got pose: (%.2f, %.2f, %.0f deg)",
                              x, y, math.degrees(yaw))
                return (x, y, yaw)
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                    tf2_ros.ConnectivityException):
                if rospy.Time.now() > timeout:
                    rospy.logerr("TF lookup failed - no map->base_link transform")
                    return None
                rate.sleep()
        return None

    def _record_wifi_snapshot(self):
        try:
            out = subprocess.check_output(
                ['sudo', 'iwlist', self.wifi_interface, 'scan'],
                stderr=subprocess.DEVNULL, timeout=8
            ).decode('utf-8', errors='ignore')
        except Exception as e:
            rospy.logwarn("WiFi scan failed: %s", str(e))
            return None

        readings = []
        for cell in out.split('Cell ')[1:]:
            b = re.search(r'Address: ([0-9A-Fa-f:]{17})', cell)
            r = re.search(r'Signal level=(-?\d+)', cell)
            if b and r:
                readings.append((b.group(1), int(r.group(1))))

        if not readings:
            rospy.logwarn("No WiFi readings captured")
            return None

        conn = sqlite3.connect(self.db_path)
        c = conn.cursor()
        ts = datetime.now().isoformat()
        c.execute(
            'INSERT INTO fingerprints (x, y, theta, timestamp) VALUES (?, ?, ?, ?)',
            (self.pose[0], self.pose[1], self.pose[2], ts))
        fp_id = c.lastrowid
        for bssid, rssi in readings:
            c.execute('INSERT INTO readings (fingerprint_id, bssid, rssi) '
                      'VALUES (?, ?, ?)', (fp_id, bssid, rssi))
        conn.commit()
        conn.close()

        rospy.loginfo("Captured WiFi fingerprint #%d with %d APs",
                      fp_id, len(readings))
        return fp_id


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: rosrun trolley_bringup record_location.py <name>")
        sys.exit(1)
    try:
        LocationRecorder(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
