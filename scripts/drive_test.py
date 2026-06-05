#!/usr/bin/env python3
"""
Drive Test
==========
Drives the trolley forward at a target velocity for a target time,
then stops cleanly.

Usage:
  rosrun trolley_bringup drive_test.py [duration_sec] [velocity_mps]
  e.g.:
  rosrun trolley_bringup drive_test.py 5.0 0.3
"""
import sys
import rospy
from geometry_msgs.msg import Twist


def main():
    duration = 5.0
    velocity = 0.3
    if len(sys.argv) > 1:
        duration = float(sys.argv[1])
    if len(sys.argv) > 2:
        velocity = float(sys.argv[2])

    rospy.init_node('drive_test', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.sleep(0.5)  # let publisher establish

    rospy.loginfo(f"Driving forward at {velocity} m/s for {duration} s")
    rospy.loginfo(f"Expected travel: ~{velocity * duration:.2f} m")

    twist = Twist()
    twist.linear.x = velocity

    rate = rospy.Rate(20)  # 20 Hz, well above watchdog rate
    t_start = rospy.Time.now()

    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - t_start).to_sec()
        if elapsed >= duration:
            break
        pub.publish(twist)
        rate.sleep()

    # Send explicit stop
    twist.linear.x = 0.0
    for _ in range(20):
        pub.publish(twist)
        rate.sleep()

    rospy.loginfo("Drive test complete. Motors stopped.")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
