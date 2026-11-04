#!/usr/bin/env python3
"""
status_publisher.py
====================
Publishes food_robot_msgs/RobotStatus at 1 Hz.

Subscribers:
  /food_robot/compartment_status (std_msgs/String)  -> updates compartment fields
  /food_robot/robot_state        (std_msgs/String)  -> updates robot state field

Publishers:
  /food_robot/status (food_robot_msgs/RobotStatus)  @ 1 Hz

Battery simulation: starts at 100%, drains 0.01%/s, floor at 0.
"""

import rospy
from std_msgs.msg import String
from food_robot_msgs.msg import RobotStatus


class StatusPublisher:
    def __init__(self):
        rospy.init_node('status_publisher', anonymous=False)

        # Internal state
        self.battery_percent   = 100.0
        self.compartment_status = "locked"
        self.robot_state        = "IDLE"
        self._start_time        = rospy.Time.now()

        # Publisher
        self.status_pub = rospy.Publisher(
            '/food_robot/status', RobotStatus, queue_size=10)

        # Subscribers
        rospy.Subscriber('/food_robot/compartment_status',
                         String, self._compartment_cb)
        rospy.Subscriber('/food_robot/robot_state',
                         String, self._state_cb)

        # Timer: 1 Hz publish + battery drain
        self._timer = rospy.Timer(rospy.Duration(1.0), self._publish_cb)

        rospy.loginfo("[StatusPublisher] Node started — publishing at 1 Hz")

    def _compartment_cb(self, msg):
        self.compartment_status = msg.data

    def _state_cb(self, msg):
        self.robot_state = msg.data

    def _publish_cb(self, event):
        # Drain battery 0.01% per second
        self.battery_percent = max(0.0, self.battery_percent - 0.01)

        status = RobotStatus()
        status.header.stamp    = rospy.Time.now()
        status.current_task    = ("delivery"
                                  if self.robot_state not in ("IDLE", "")
                                  else "idle")
        status.battery_percent  = self.battery_percent
        status.compartment_locked = (self.compartment_status == "locked")
        status.compartment_status = self.compartment_status
        status.robot_state      = self.robot_state

        self.status_pub.publish(status)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = StatusPublisher()
    node.run()
