#!/usr/bin/env python3
"""
WiFi Position Resolver
======================
Given a WiFi scan (list of BSSID+RSSI), returns most likely (x, y) on the map
using k-nearest-neighbor matching against the fingerprint database.

Subscribes:  /wifi_resolver/query  (std_msgs/String, JSON: {"scan": [...]})
Publishes:   /wifi_resolver/result (std_msgs/String, JSON: {"x":..,"y":..,...})
"""

import os
import math
import sqlite3
import json
import rospy
from std_msgs.msg import String


class WiFiResolver:
    def __init__(self):
        rospy.init_node('wifi_resolver')

        self.db_path = rospy.get_param(
            '~db_path', os.path.expanduser('~/trolley_fingerprints.db'))
        self.k = int(rospy.get_param('~k', 5))
        self.missing_penalty = int(rospy.get_param('~missing_penalty', -100))
        self.min_common_aps = int(rospy.get_param('~min_common_aps', 3))

        rospy.loginfo("wifi_resolver params:")
        rospy.loginfo("  db_path          = %s", self.db_path)
        rospy.loginfo("  k                = %d", self.k)
        rospy.loginfo("  missing_penalty  = %d dBm", self.missing_penalty)
        rospy.loginfo("  min_common_aps   = %d", self.min_common_aps)

        if not os.path.isfile(self.db_path):
            rospy.logerr("Fingerprint DB not found: %s", self.db_path)
            raise SystemExit(1)

        self._load_fingerprints()

        rospy.Subscriber('/wifi_resolver/query', String, self._query_cb,
                         queue_size=1)
        self.pub = rospy.Publisher('/wifi_resolver/result', String,
                                   queue_size=1)

        rospy.loginfo("wifi_resolver ready. Loaded %d fingerprints, %d APs.",
                      len(self.fingerprints), len(self.all_bssids))

    def _load_fingerprints(self):
        conn = sqlite3.connect(self.db_path)
        c = conn.cursor()
        c.execute('SELECT id, x, y, theta FROM fingerprints')
        fps = {row[0]: {'x': row[1], 'y': row[2], 'theta': row[3], 'rssi': {}}
               for row in c.fetchall()}
        c.execute('SELECT fingerprint_id, bssid, rssi FROM readings')
        for fp_id, bssid, rssi in c.fetchall():
            if fp_id in fps:
                fps[fp_id]['rssi'][bssid] = rssi
        conn.close()

        self.fingerprints = [v for v in fps.values() if v['rssi']]
        self.all_bssids = set()
        for fp in self.fingerprints:
            self.all_bssids.update(fp['rssi'].keys())

    def _scan_distance(self, query_rssi, fp_rssi):
        all_bssids = set(query_rssi.keys()) | set(fp_rssi.keys())
        common = set(query_rssi.keys()) & set(fp_rssi.keys())
        if len(common) < self.min_common_aps:
            return None

        sum_sq = 0.0
        for bssid in all_bssids:
            q = query_rssi.get(bssid, self.missing_penalty)
            f = fp_rssi.get(bssid, self.missing_penalty)
            sum_sq += (q - f) ** 2
        return math.sqrt(sum_sq)

    def _resolve(self, query_rssi):
        scored = []
        for fp in self.fingerprints:
            d = self._scan_distance(query_rssi, fp['rssi'])
            if d is not None:
                scored.append((d, fp))

        if not scored:
            return None

        scored.sort(key=lambda x: x[0])
        nearest = scored[:self.k]

        eps = 1e-6
        total_w = 0.0
        x_sum = 0.0
        y_sum = 0.0
        for d, fp in nearest:
            w = 1.0 / (d + eps)
            x_sum += fp['x'] * w
            y_sum += fp['y'] * w
            total_w += w

        x = x_sum / total_w
        y = y_sum / total_w
        mean_d = sum(d for d, _ in nearest) / len(nearest)
        conf = 1.0 / (1.0 + mean_d / 50.0)

        return {
            'x': x,
            'y': y,
            'confidence': conf,
            'neighbors_used': len(nearest),
            'mean_distance': mean_d,
        }

    def _query_cb(self, msg):
        try:
            req = json.loads(msg.data)
            scan = req.get('scan', [])
            query_rssi = {item['bssid']: item['rssi'] for item in scan}
        except Exception as e:
            rospy.logwarn("Bad query: %s", str(e))
            self.pub.publish(String(data=json.dumps({'error': 'bad_request'})))
            return

        if not query_rssi:
            self.pub.publish(String(data=json.dumps({'error': 'empty_scan'})))
            return

        result = self._resolve(query_rssi)
        if result is None:
            rospy.logwarn("Could not resolve scan: not enough common APs")
            self.pub.publish(String(data=json.dumps(
                {'error': 'no_match', 'min_common_aps': self.min_common_aps})))
            return

        rospy.loginfo("Resolved to (%.2f, %.2f) conf=%.2f (mean_dist=%.1f)",
                      result['x'], result['y'], result['confidence'],
                      result['mean_distance'])
        self.pub.publish(String(data=json.dumps(result)))


if __name__ == '__main__':
    try:
        WiFiResolver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
