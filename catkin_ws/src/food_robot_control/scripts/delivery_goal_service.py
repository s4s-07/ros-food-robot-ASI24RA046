#!/usr/bin/env python3
"""
delivery_goal_service.py
=========================
ROS service server for /food_robot/set_delivery_goal.

Validates destination, then forwards to the navigate_action_server_v2
action server as a NavigateToGoal goal.
"""

import rospy
import actionlib
from food_robot_msgs.srv import SetDeliveryGoal, SetDeliveryGoalResponse
from food_robot_msgs.msg import NavigateToGoalAction, NavigateToGoalGoal

# Known cafeteria destinations with (x, y) coordinates
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


class DeliveryGoalService:
    def __init__(self):
        rospy.init_node('delivery_goal_service', anonymous=False)

        # Action client (connects to navigate_action_server_v2)
        self._action_client = actionlib.SimpleActionClient(
            '/navigate_to_goal_v2', NavigateToGoalAction)

        rospy.loginfo("[DeliveryGoalSvc] Waiting for action server...")
        server_ready = self._action_client.wait_for_server(
            timeout=rospy.Duration(10.0))
        if server_ready:
            rospy.loginfo("[DeliveryGoalSvc] Action server connected.")
        else:
            rospy.logwarn("[DeliveryGoalSvc] Action server NOT available — "
                          "service will respond with error if called.")

        # Service server
        rospy.Service('/food_robot/set_delivery_goal',
                      SetDeliveryGoal, self._handle_request)

        rospy.loginfo("[DeliveryGoalSvc] Service /food_robot/set_delivery_goal ready.")
        rospy.loginfo("[DeliveryGoalSvc] Valid destinations: %s",
                      ', '.join(sorted(KNOWN_DESTINATIONS.keys())))

    def _handle_request(self, req):
        dest     = req.destination_name.strip()
        order_id = req.order_id.strip()

        rospy.loginfo("[DeliveryGoalSvc] Received: dest='%s', order='%s'",
                      dest, order_id)

        # Validate destination
        if dest not in KNOWN_DESTINATIONS:
            msg = ("Unknown destination '{}'. Valid options: {}".format(
                dest, ', '.join(sorted(KNOWN_DESTINATIONS.keys()))))
            rospy.logwarn("[DeliveryGoalSvc] %s", msg)
            return SetDeliveryGoalResponse(accepted=False, message=msg)

        # Check action server is available
        if not self._action_client.wait_for_server(rospy.Duration(3.0)):
            msg = "Action server /navigate_to_goal_v2 not available."
            rospy.logerr("[DeliveryGoalSvc] %s", msg)
            return SetDeliveryGoalResponse(accepted=False, message=msg)

        # Send goal
        goal = NavigateToGoalGoal(
            destination_name=dest,
            order_id=order_id
        )
        self._action_client.send_goal(goal)

        msg = "Delivery accepted — navigating to '{}' for order '{}'".format(
            dest, order_id)
        rospy.loginfo("[DeliveryGoalSvc] %s", msg)
        return SetDeliveryGoalResponse(accepted=True, message=msg)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = DeliveryGoalService()
    node.run()
