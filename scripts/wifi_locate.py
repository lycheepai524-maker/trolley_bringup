#!/usr/bin/env python3
"""
Test client for the WiFi resolver.
Scans WiFi locally on the Pi, sends to /wifi_resolver/query, prints result.
"""
import json
import re
import subprocess
import rospy
from std_msgs.msg import String


def scan_wifi(interface='wlan0'):
    try:
        out = subprocess.check_output(
            ['sudo', 'iwlist', interface, 'scan'],
            stderr=subprocess.DEVNULL, timeout=8
        ).decode('utf-8', errors='ignore')
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
        rospy.logerr("WiFi scan failed: %s", str(e))
        return []

    cells = out.split('Cell ')
    readings = []
    for cell in cells[1:]:
        bssid_m = re.search(r'Address: ([0-9A-Fa-f:]{17})', cell)
        rssi_m  = re.search(r'Signal level=(-?\d+)', cell)
        if bssid_m and rssi_m:
            readings.append({'bssid': bssid_m.group(1),
                             'rssi':  int(rssi_m.group(1))})
    return readings


def main():
    rospy.init_node('wifi_locate', anonymous=True)
    pub = rospy.Publisher('/wifi_resolver/query', String, queue_size=1)
    result_holder = {'msg': None}

    def cb(msg):
        result_holder['msg'] = msg.data

    rospy.Subscriber('/wifi_resolver/result', String, cb, queue_size=1)
    rospy.sleep(1.0)

    readings = scan_wifi()
    if not readings:
        rospy.logerr("No WiFi readings collected. Aborting.")
        return
    rospy.loginfo("Scanned %d APs locally.", len(readings))

    pub.publish(String(data=json.dumps({'scan': readings})))

    timeout = rospy.Time.now() + rospy.Duration(5.0)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if result_holder['msg'] is not None:
            break
        if rospy.Time.now() > timeout:
            rospy.logerr("No reply from resolver within 5 seconds.")
            return
        rate.sleep()

    print("\n=== RESOLVER RESPONSE ===")
    res = json.loads(result_holder['msg'])
    if 'error' in res:
        print(f"ERROR: {res['error']}")
    else:
        print(f"Position: ({res['x']:.2f}, {res['y']:.2f})")
        print(f"Confidence: {res['confidence']:.3f}")
        print(f"Neighbors used: {res['neighbors_used']}")
        print(f"Mean RSSI distance: {res['mean_distance']:.1f}")
    print("=========================\n")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
