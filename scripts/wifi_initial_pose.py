#!/usr/bin/env python
"""
wifi_initial_pose.py

Two behaviors:
  1. At startup, scan once and publish /initialpose (same as before).
  2. Stay alive and offer a service /relocalize that re-scans WiFi
     and republishes /initialpose. Useful when the robot gets lost or
     when you move it manually and want it to find itself again.

Trigger from CLI:
    rosservice call /relocalize
"""
import math, os, re, subprocess, sqlite3
import numpy as np
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import quaternion_from_euler


def scan_wifi(iface):
    out = subprocess.check_output(['sudo', 'iwlist', iface, 'scan'],
                                  stderr=subprocess.DEVNULL).decode('utf-8', 'ignore')
    cells = re.split(r'Cell \d+ - ', out)[1:]
    scan = {}
    for c in cells:
        m_bssid = re.search(r'Address: ([0-9A-F:]{17})', c)
        m_sig = re.search(r'Signal level=(-?\d+)', c)
        if m_bssid and m_sig:
            scan[m_bssid.group(1).upper()] = int(m_sig.group(1))
    return scan

def load_fingerprints(db_path):
    con = sqlite3.connect(db_path)
    cur = con.cursor()
    fps = cur.execute('SELECT point_id, x, y FROM phone_points').fetchall()
    result = []
    for fid, x, y in fps:
        readings = dict(cur.execute(
            'SELECT bssid, rssi FROM phone_reading WHERE point_id=?', (fid,)
        ).fetchall())
        readings = {b.upper(): r for b, r in readings.items()}
        result.append((fid, x, y, 0.0, readings))
    con.close()
    return result

def similarity(live, fp):
    keys = set(live) | set(fp)
    if not keys:
        return -1e9
    d2 = 0.0
    for k in keys:
        a = live.get(k, -100)
        b = fp.get(k, -100)
        d2 += (a - b) ** 2
    return -math.sqrt(d2 / len(keys))


def best_pose(live_scan, fingerprints, k=3):
    scored = sorted(((similarity(live_scan, fp[4]), fp) for fp in fingerprints),
                    key=lambda t: -t[0])[:k]
    rospy.loginfo("Top matches:")
    for s, fp in scored:
        rospy.loginfo("  id=%d (x=%.2f y=%.2f th=%.2f) score=%.2f",
                      fp[0], fp[1], fp[2], fp[3], s)
    scores = np.array([s for s, _ in scored])
    weights = np.exp((scores - scores.max()) / 5.0)
    weights /= weights.sum()
    x = sum(w * fp[1] for w, (_, fp) in zip(weights, scored))
    y = sum(w * fp[2] for w, (_, fp) in zip(weights, scored))
    cx = sum(w * math.cos(fp[3]) for w, (_, fp) in zip(weights, scored))
    sy = sum(w * math.sin(fp[3]) for w, (_, fp) in zip(weights, scored))
    th = math.atan2(sy, cx)
    return x, y, th, scored[0][0]


class WifiLocalizer(object):
    def __init__(self):
        self.db_path = rospy.get_param('~db_path',
                                       os.path.expanduser('~/trolley_fingerprints.db'))
        self.iface = rospy.get_param('~wifi_interface', 'wlan0')
        self.k = int(rospy.get_param('~k', 3))
        self.publish_on_startup = bool(rospy.get_param('~publish_on_startup', True))

        rospy.loginfo("Loading fingerprint DB from %s", self.db_path)
        self.fingerprints = load_fingerprints(self.db_path)
        rospy.loginfo("Loaded %d fingerprints", len(self.fingerprints))

        self.pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped,
                                   queue_size=1, latch=True)
        self.srv = rospy.Service('relocalize', Trigger, self.handle_relocalize)
        rospy.loginfo("Service /relocalize ready. "
                      "Call: rosservice call /relocalize")

        if self.publish_on_startup:
            rospy.sleep(2.0)
            ok, msg = self.do_localize()
            rospy.loginfo("Startup relocalize: %s (%s)", ok, msg)

    def do_localize(self):
        try:
            rospy.loginfo("Scanning WiFi on %s ...", self.iface)
            live = scan_wifi(self.iface)
            rospy.loginfo("Saw %d APs", len(live))
            if len(live) == 0:
                return False, "No APs seen"

            x, y, th, best_score = best_pose(live, self.fingerprints, k=self.k)
            rospy.loginfo("Estimated pose: x=%.2f y=%.2f th=%.2f (best=%.2f)",
                          x, y, th, best_score)

            rms = -best_score
            pos_var = max(0.25, (rms / 10.0) ** 2)
            yaw_var = max(0.25, (rms / 20.0) ** 2)

            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = rospy.Time.now()
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            q = quaternion_from_euler(0, 0, th)
            msg.pose.pose.orientation.x = q[0]
            msg.pose.pose.orientation.y = q[1]
            msg.pose.pose.orientation.z = q[2]
            msg.pose.pose.orientation.w = q[3]
            cov = [0.0] * 36
            cov[0] = pos_var
            cov[7] = pos_var
            cov[35] = yaw_var
            msg.pose.covariance = cov

            self.pub.publish(msg)
            return True, "x=%.2f y=%.2f th=%.2f score=%.2f" % (x, y, th, best_score)

        except subprocess.CalledProcessError as e:
            return False, "iwlist failed: %s" % e
        except Exception as e:
            return False, "%s" % e

    def handle_relocalize(self, req):
        ok, msg = self.do_localize()
        return TriggerResponse(success=ok, message=msg)


if __name__ == '__main__':
    rospy.init_node('wifi_initial_pose')
    WifiLocalizer()
    rospy.spin()
