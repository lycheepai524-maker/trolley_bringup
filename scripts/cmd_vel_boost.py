#!/usr/bin/env python
"""
cmd_vel_boost.py

Sits between move_base and the Arduino. Solves the motor-deadband problem:
DWA wants to send commands like 0.15 m/s but the motors only respond above
~0.5 m/s. This node boosts small non-zero commands up to the minimum that
actually produces motion.

Topics:
  in:  cmd_vel_in  (geometry_msgs/Twist)  — from move_base
  out: cmd_vel_out (geometry_msgs/Twist)  — to Arduino

Params (all private, ~):
  min_linear        (float, 0.5)   minimum speed that produces motion [m/s]
  min_angular       (float, 2.2)   minimum angular speed that turns wheels [rad/s]
  linear_deadband   (float, 0.02)  if |v| < this, treat as zero (no boost)
  angular_deadband  (float, 0.05)  if |w| < this, treat as zero (no boost)
"""
import rospy
from geometry_msgs.msg import Twist


class CmdVelBoost(object):
    def __init__(self):
        self.min_lin = float(rospy.get_param('~min_linear',  0.5))
        self.min_ang = float(rospy.get_param('~min_angular', 2.2))
        self.lin_db  = float(rospy.get_param('~linear_deadband',  0.02))
        self.ang_db  = float(rospy.get_param('~angular_deadband', 0.05))

        self.pub = rospy.Publisher('cmd_vel_out', Twist, queue_size=10)
        rospy.Subscriber('cmd_vel_in', Twist, self.cb, queue_size=10)
        rospy.loginfo("cmd_vel_boost: min_lin=%.2f min_ang=%.2f "
                      "deadband (lin=%.3f ang=%.3f)",
                      self.min_lin, self.min_ang, self.lin_db, self.ang_db)

    def cb(self, msg):
        out = Twist()
        v = msg.linear.x
        w = msg.angular.z

        # Linear: keep zero as zero; boost anything in (deadband, min_lin) up to min_lin
        if abs(v) < self.lin_db:
            out.linear.x = 0.0
        elif abs(v) < self.min_lin:
            out.linear.x = self.min_lin if v > 0 else -self.min_lin
        else:
            out.linear.x = v

        # Angular: same logic
        if abs(w) < self.ang_db:
            out.angular.z = 0.0
        elif abs(w) < self.min_ang:
            out.angular.z = self.min_ang if w > 0 else -self.min_ang
        else:
            out.angular.z = w

        self.pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('cmd_vel_boost')
    CmdVelBoost()
    rospy.spin()
