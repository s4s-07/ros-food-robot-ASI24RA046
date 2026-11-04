#!/usr/bin/env python3
"""
navigate_action_server.py  (Basic version — Module 2 baseline)
==============================================================
Simple action server that simulates navigation by sleeping
proportional to the Euclidean distance to the destination.

Action: /navigate_to_goal (food_robot_msgs/NavigateToGoalAction)
"""

import math
import rospy
import actionlib
from food_robot_msgs.msg import (NavigateToGoalAction,
                                  NavigateToGoalFeedback,
                                  NavigateToGoalResult)

KNOWN_DESTINATIONS = {
    'Table1':    (3.0,  2.0),
    'Table2':    (3.0,  4.5),
    'Table3':    (3.0,  7.0),
    'Table4':    (7.0,  2.0),
    'Table5':    (7.0,  4.5),
    'Table6':    (7.0,  7.0),
    'FrontDesk': (-7.0, -5.5),
    'Kitchen':   (9.0,  0.5),
    'Room204':   (9.0,  3.5),
}

ROBOT_SPEED = 0.3  # m/s (simulated)


class BasicNavigationServer:
    def __init__(self):
        rospy.init_node('navigate_action_server', anonymous=False)

        self._as = actionlib.SimpleActionServer(
            '/navigate_to_goal',
            NavigateToGoalAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._as.start()
        rospy.loginfo("[BasicNavServer] Action server started on /navigate_to_goal")

    def execute_cb(self, goal):
        dest     = goal.destination_name
        order_id = goal.order_id
        rospy.loginfo("[BasicNavServer] Goal: dest='%s', order='%s'", dest, order_id)

        result   = NavigateToGoalResult()
        feedback = NavigateToGoalFeedback()

        if dest not in KNOWN_DESTINATIONS:
            result.success = False
            result.message = "Unknown destination: {}".format(dest)
            self._as.set_aborted(result)
            return

        x, y = KNOWN_DESTINATIONS[dest]
        total_dist = math.sqrt(x**2 + y**2)
        total_steps = max(5, int(total_dist / ROBOT_SPEED / 0.5))  # 0.5 s per step

        rospy.loginfo("[BasicNavServer] Distance=%.2fm, Steps=%d", total_dist, total_steps)

        for i in range(total_steps):
            # Preemption check
            if self._as.is_preempt_requested():
                rospy.logwarn("[BasicNavServer] Preempted!")
                result.success = False
                result.message = "Preempted"
                result.total_distance = total_dist * (i / total_steps)
                self._as.set_preempted(result)
                return

            progress = float(i) / total_steps
            dist_remaining = total_dist * (1.0 - progress)
            eta = dist_remaining / ROBOT_SPEED

            feedback.distance_remaining = dist_remaining
            feedback.eta_seconds        = eta
            feedback.status_text        = "Navigating to {}... {:.0f}%".format(
                dest, progress * 100)
            feedback.current_state      = "NAVIGATING"
            self._as.publish_feedback(feedback)

            rospy.sleep(0.5)

        # Success
        result.success        = True
        result.message        = "Arrived at {}".format(dest)
        result.total_distance = total_dist
        self._as.set_succeeded(result)
        rospy.loginfo("[BasicNavServer] Arrived at '%s'", dest)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = BasicNavigationServer()
    node.run()
