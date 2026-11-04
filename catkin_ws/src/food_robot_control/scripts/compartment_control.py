#!/usr/bin/env python3
"""
compartment_control.py
========================
ROS node that simulates a food compartment lid servo.

Services:
  /compartment/lock   (std_srvs/Trigger) -> locks the lid
  /compartment/unlock (std_srvs/Trigger) -> unlocks the lid

Publishers:
  /food_robot/compartment_status  (std_msgs/String)  "locked" | "unlocked"
  /food_robot/compartment_locked  (std_msgs/Bool)
"""

import rospy
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, TriggerResponse


class CompartmentController:
    def __init__(self):
        rospy.init_node('compartment_control', anonymous=False)

        self.compartment_locked = True  # Start locked (safe default)

        # Publishers
        self.status_pub = rospy.Publisher(
            '/food_robot/compartment_status', String, queue_size=10, latch=True)
        self.locked_pub = rospy.Publisher(
            '/food_robot/compartment_locked', Bool, queue_size=10, latch=True)

        # Services
        rospy.Service('/compartment/lock',   Trigger, self.lock_callback)
        rospy.Service('/compartment/unlock', Trigger, self.unlock_callback)

        # Publish initial state
        self._publish_state()

        rospy.loginfo("[CompartmentControl] Node started. Compartment is LOCKED.")

    def _publish_state(self):
        status_str = "locked" if self.compartment_locked else "unlocked"
        self.status_pub.publish(String(data=status_str))
        self.locked_pub.publish(Bool(data=self.compartment_locked))

    def lock_callback(self, req):
        self.compartment_locked = True
        self._publish_state()
        rospy.loginfo("[CompartmentControl] 🔒 Compartment LOCKED")
        return TriggerResponse(success=True, message="Compartment locked")

    def unlock_callback(self, req):
        self.compartment_locked = False
        self._publish_state()
        rospy.loginfo("[CompartmentControl] 🔓 Compartment UNLOCKED")
        return TriggerResponse(success=True, message="Compartment unlocked")

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = CompartmentController()
    node.run()
