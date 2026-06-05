#!/usr/bin/env python3
"""
Tick Calibration Script
=======================
Push the trolley a known distance in a straight line and compute
ticks_per_meter empirically.

Usage:
  1. Place trolley behind a start line (mark on floor with tape)
  2. Mark a finish line exactly 2.0 m forward
  3. Run this script
  4. When prompted, push the trolley straight from start to finish
  5. Press Enter when finished
  6. Copy the printed ticks_per_meter into trolley_bringup.launch
"""

import rospy
from std_msgs.msg import Int32

# Distance you'll push the trolley (in meters). Change if you measured a
# different distance.
DISTANCE_M = 2.0

class TickCalibrator:
    def __init__(self):
        rospy.init_node('calibrate_ticks', anonymous=True)
        self.left = None
        self.right = None
        rospy.Subscriber('/left_ticks', Int32, self._left_cb)
        rospy.Subscriber('/right_ticks', Int32, self._right_cb)

    def _left_cb(self, msg):
        self.left = msg.data

    def _right_cb(self, msg):
        self.right = msg.data

    def wait_for_data(self, timeout=5.0):
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.left is not None and self.right is not None:
                return True
            if (rospy.Time.now() - start).to_sec() > timeout:
                return False
            rate.sleep()
        return False

def main():
    cal = TickCalibrator()
    print("Waiting for tick data on /left_ticks and /right_ticks...")
    if not cal.wait_for_data():
        print("ERROR: never received ticks. Is rosserial running?")
        return

    print("\n----------------------------------------------------")
    print("  TICK CALIBRATION")
    print("----------------------------------------------------")
    print(f"  Distance to push:    {DISTANCE_M:.2f} m")
    print(f"  Current left_ticks:  {cal.left}")
    print(f"  Current right_ticks: {cal.right}")
    print("----------------------------------------------------")
    input("Place trolley at START line. Press ENTER when ready...")

    left_start  = cal.left
    right_start = cal.right
    print(f"\nStarting ticks: L={left_start}  R={right_start}")
    print(f"Now PUSH the trolley exactly {DISTANCE_M:.2f} m straight forward.")
    input("Press ENTER when you've reached the FINISH line...")

    left_end  = cal.left
    right_end = cal.right

    d_left  = left_end  - left_start
    d_right = right_end - right_start
    d_avg   = (d_left + d_right) / 2.0

    print("\n----------------------------------------------------")
    print("  RESULTS")
    print("----------------------------------------------------")
    print(f"  Left  ticks delta:  {d_left}")
    print(f"  Right ticks delta:  {d_right}")
    print(f"  Average:            {d_avg:.1f}")
    print(f"  Distance:           {DISTANCE_M:.2f} m")
    print("----------------------------------------------------")

    if d_avg <= 0:
        print("ERROR: ticks did not increase. Check direction or invert_*"
              " params. Did you push forward?")
        return

    tpm = d_avg / DISTANCE_M
    print(f"\n  >>>  ticks_per_meter = {tpm:.1f}  <<<\n")

    # Sanity: warn if left and right disagree by more than 10%
    if abs(d_left - d_right) / max(abs(d_avg), 1) > 0.10:
        print("  WARNING: Left and right disagree by more than 10%.")
        print("  Possible causes:")
        print("    - Trolley didn't go straight")
        print("    - Wheel slip or different wheel diameters")
        print("    - One encoder is missing ticks")
        print("  Re-run a few times and average if results vary.\n")

    print("Open trolley_bringup.launch and set:")
    print(f"  <param name=\"ticks_per_meter\" value=\"{tpm:.1f}\"/>")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
