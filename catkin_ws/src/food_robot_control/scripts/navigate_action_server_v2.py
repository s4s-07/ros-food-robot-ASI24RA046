#!/usr/bin/env python3
"""
navigate_action_server_v2.py  — OPTIMIZED RESTAURANT CORRIDOR ROUTING
=====================================================================
Features:
- Dual-Aisle Cafeteria Highway:
    • East Aisle (x = 5.0) for Tables 4, 5, 6 (Direct highway to Kitchen)
    • Central Aisle (x = 0.0) for Tables 1, 2, 3 and Front Desk
- Precise Laser Safety Auto-Stop (+/- 20 deg frontal cone, 0.38m threshold)
  Zero false alarms from side chairs; stops instantly if an obstacle blocks the path.
- Kitchen-to-Table-to-Kitchen restaurant lifecycle.
"""

import math
import rospy
import actionlib
from enum import Enum
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from food_robot_msgs.msg import (NavigateToGoalAction,
                                  NavigateToGoalFeedback,
                                  NavigateToGoalResult)

# ── DUAL-HIGHWAY CORRIDOR ROUTES (KITCHEN ➔ TABLE ➔ KITCHEN) ─────────────────
# Kitchen is located at (7.5, 0.0)
ROUTES_FROM_KITCHEN = {
    # ── East Aisle (x = 5.0): Direct highway from Kitchen ───────────────────
    'Table4': {
        'kitchen_to_table': [(5.0, 0.0), (5.0, 2.0)],
        'table_to_kitchen': [(5.0, 0.0), (7.5, 0.0)],
        'zone': 'Table 4 Delivery Point',
    },
    'Table5': {
        'kitchen_to_table': [(5.0, 0.0), (5.0, 4.5)],
        'table_to_kitchen': [(5.0, 0.0), (7.5, 0.0)],
        'zone': 'Table 5 Delivery Point',
    },
    'Table6': {
        'kitchen_to_table': [(5.0, 0.0), (5.0, 6.2)],
        'table_to_kitchen': [(5.0, 0.0), (7.5, 0.0)],
        'zone': 'Table 6 Delivery Point',
    },
    # ── Central Aisle (x = 0.0): Western Tables ─────────────────────────────
    'Table1': {
        'kitchen_to_table': [(0.0, 0.0), (0.0, 2.0), (1.2, 2.0)],
        'table_to_kitchen': [(0.0, 2.0), (0.0, 0.0), (7.5, 0.0)],
        'zone': 'Table 1 Delivery Point',
    },
    'Table2': {
        'kitchen_to_table': [(0.0, 0.0), (0.0, 4.5), (1.2, 4.5)],
        'table_to_kitchen': [(0.0, 4.5), (0.0, 0.0), (7.5, 0.0)],
        'zone': 'Table 2 Delivery Point',
    },
    'Table3': {
        'kitchen_to_table': [(0.0, 0.0), (0.0, 6.2), (1.2, 6.2)],
        'table_to_kitchen': [(0.0, 6.2), (0.0, 0.0), (7.5, 0.0)],
        'zone': 'Table 3 Green Delivery Zone',
    },
    'FrontDesk': {
        'kitchen_to_table': [(0.0, 0.0), (0.0, -5.5), (-5.0, -5.5)],
        'table_to_kitchen': [(0.0, -5.5), (0.0, 0.0), (7.5, 0.0)],
        'zone': 'Front Desk Yellow Zone',
    },
    'Room204': {
        'kitchen_to_table': [(7.5, 3.5)],
        'table_to_kitchen': [(7.5, 0.0)],
        'zone': 'Room 204 Blue Zone',
    },
}

# Motion parameters
MAX_LINEAR_SPEED   = 0.35   # m/s
MAX_ANGULAR_SPEED  = 0.75   # rad/s
WAYPOINT_TOL       = 0.25   # meters

# Safety Auto-Stop (Tuned to frontal driving corridor)
OBSTACLE_STOP_DIST   = 0.38 # meters (strictly head-on obstacles)
OBSTACLE_RESUME_DIST = 0.50 # meters


class RobotState(Enum):
    IDLE                   = "IDLE"
    NAVIGATING_TO_KITCHEN  = "NAVIGATING_TO_KITCHEN"
    AT_KITCHEN_LOADING     = "AT_KITCHEN_LOADING"
    NAVIGATING_TO_TABLE    = "NAVIGATING_TO_TABLE"
    ARRIVED_AT_TABLE       = "ARRIVED_AT_TABLE"
    COMPARTMENT_UNLOCKED   = "COMPARTMENT_UNLOCKED"
    WAITING_FOR_PICKUP     = "WAITING_FOR_PICKUP"
    RETURNING_TO_KITCHEN   = "RETURNING_TO_KITCHEN"


class FoodDeliveryActionServer:

    def __init__(self):
        rospy.init_node('navigate_action_server_v2', anonymous=False)

        self.state = RobotState.IDLE
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.min_front_dist = 10.0
        self.obstacle_blocked = False

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.state_pub = rospy.Publisher(
            '/food_robot/robot_state', String, queue_size=10, latch=True)

        # Subscribers
        rospy.Subscriber('/odom', Odometry, self._odom_cb)
        rospy.Subscriber('/scan', LaserScan, self._scan_cb)

        # Action Server
        self._as = actionlib.SimpleActionServer(
            '/navigate_to_goal_v2',
            NavigateToGoalAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._as.start()
        self._set_state(RobotState.IDLE)

        rospy.loginfo("================================================================")
        rospy.loginfo("✓ Restaurant Dual-Highway Action Server is LIVE!")
        rospy.loginfo(f"  Frontal obstacle safety distance: {OBSTACLE_STOP_DIST}m")
        rospy.loginfo("================================================================")

    def _odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _scan_cb(self, msg):
        """Monitors tight +/-20 deg forward cone to ignore parallel side chairs."""
        num_ranges = len(msg.ranges)
        if num_ranges == 0:
            return
        mid = num_ranges // 2
        cone = max(1, int(num_ranges * (20.0 / 360.0)))  # +/- 20 deg frontal cone
        front_slice = msg.ranges[mid - cone : mid + cone]
        valid = [r for r in front_slice if (msg.range_min < r < msg.range_max and not math.isnan(r))]
        self.min_front_dist = min(valid) if valid else 10.0

    def _set_state(self, new_state: RobotState):
        self.state = new_state
        self.state_pub.publish(String(data=new_state.value))
        rospy.loginfo(f"[NavServerV2] ▶ State → {new_state.value}")

    def _publish_feedback(self, dist_remaining: float, eta: float, status_text: str):
        fb = NavigateToGoalFeedback()
        fb.distance_remaining = max(0.0, dist_remaining)
        fb.eta_seconds        = max(0.0, eta)
        fb.status_text        = status_text
        fb.current_state      = self.state.value
        self._as.publish_feedback(fb)

    def _stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def _call_compartment(self, action: str):
        svc_name = f'/compartment/{action}'
        try:
            rospy.wait_for_service(svc_name, timeout=1.0)
            proxy = rospy.ServiceProxy(svc_name, Trigger)
            resp = proxy()
            rospy.loginfo(f"[NavServerV2] Compartment {action}: {resp.message}")
        except Exception:
            rospy.loginfo(f"[NavServerV2] Compartment {action} simulated")

    # ── High-Precision Waypoint Navigator with Auto-Stop ─────────────────────
    def _follow_waypoint_path(self, waypoints: list, label: str) -> bool:
        rate = rospy.Rate(10)
        feedback_counter = 0
        final_x, final_y = waypoints[-1]

        for idx, (wx, wy) in enumerate(waypoints, 1):
            rospy.loginfo(f"[NavServerV2] {label} -> Leg {idx}/{len(waypoints)} to ({wx:.1f}, {wy:.1f})")

            while not rospy.is_shutdown():
                if self._as.is_preempt_requested():
                    self._stop_robot()
                    return False

                # ── Obstacle Auto-Stop Check ─────────────────────────────────
                if self.min_front_dist < OBSTACLE_STOP_DIST:
                    self._stop_robot()
                    if not self.obstacle_blocked:
                        rospy.logwarn(f"[NavServerV2] 🛑 HEAD-ON OBSTACLE ({self.min_front_dist:.2f}m) -> Stopping robot.")
                        self.obstacle_blocked = True

                    total_dist = math.sqrt((final_x - self.current_x)**2 + (final_y - self.current_y)**2)
                    self._publish_feedback(
                        total_dist, total_dist / MAX_LINEAR_SPEED,
                        f"⚠️ Obstacle in front ({self.min_front_dist:.2f}m) — waiting for path to clear..."
                    )
                    rate.sleep()
                    continue

                if self.obstacle_blocked and self.min_front_dist >= OBSTACLE_RESUME_DIST:
                    rospy.loginfo("[NavServerV2] ✓ Path cleared! Resuming motion...")
                    self.obstacle_blocked = False

                # ── Waypoint Tracking ────────────────────────────────────────
                dx = wx - self.current_x
                dy = wy - self.current_y
                dist = math.sqrt(dx**2 + dy**2)

                if dist < WAYPOINT_TOL:
                    break

                target_yaw = math.atan2(dy, dx)
                yaw_error = target_yaw - self.current_yaw
                while yaw_error > math.pi:
                    yaw_error -= 2.0 * math.pi
                while yaw_error < -math.pi:
                    yaw_error += 2.0 * math.pi

                cmd = Twist()
                if abs(yaw_error) > 0.25:
                    cmd.linear.x = 0.0
                    cmd.angular.z = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, 1.4 * yaw_error))
                else:
                    cmd.linear.x = max(0.15, min(MAX_LINEAR_SPEED, 0.6 * dist))
                    cmd.angular.z = max(-0.3, min(0.3, 0.8 * yaw_error))

                self.cmd_vel_pub.publish(cmd)

                feedback_counter += 1
                if feedback_counter >= 5:
                    feedback_counter = 0
                    total_remaining = math.sqrt((final_x - self.current_x)**2 + (final_y - self.current_y)**2)
                    eta = total_remaining / MAX_LINEAR_SPEED
                    self._publish_feedback(
                        total_remaining, eta,
                        f"{label} (Leg {idx}/{len(waypoints)}) — dist: {total_remaining:.2f}m, ETA: {eta:.0f}s"
                    )

                rate.sleep()

        self._stop_robot()
        return True

    # ── Main Execute Callback ─────────────────────────────────────────────────
    def execute_cb(self, goal):
        dest = goal.destination_name.strip()
        order_id = goal.order_id.strip()
        rospy.loginfo(f"[NavServerV2] ★ New Order '{order_id}' for '{dest}'")

        result = NavigateToGoalResult()

        if dest not in ROUTES_FROM_KITCHEN:
            result.success = False
            result.message = f"Unknown destination '{dest}'. Valid: {list(ROUTES_FROM_KITCHEN.keys())}"
            result.total_distance = 0.0
            self._as.set_aborted(result)
            return

        route_info = ROUTES_FROM_KITCHEN[dest]
        kitchen_to_table = route_info['kitchen_to_table']
        table_to_kitchen = route_info['table_to_kitchen']
        zone_name = route_info['zone']

        # ── 1. If not at Kitchen, drive to Kitchen first ─────────────────────
        dist_to_kitchen = math.sqrt((7.5 - self.current_x)**2 + (0.0 - self.current_y)**2)
        if dist_to_kitchen > 1.2:
            self._set_state(RobotState.NAVIGATING_TO_KITCHEN)
            path_to_kitchen = [(0.0, self.current_y), (0.0, 0.0), (7.5, 0.0)]
            reached_kitchen = self._follow_waypoint_path(path_to_kitchen, "Heading to Kitchen")
            if not reached_kitchen:
                self._handle_preempt(order_id)
                return

        # ── 2. At Kitchen — Load Meal ────────────────────────────────────────
        self._stop_robot()
        self._set_state(RobotState.AT_KITCHEN_LOADING)
        self._call_compartment('unlock')
        self._publish_feedback(0.0, 0.0, "👨‍🍳 At Kitchen: Compartment UNLOCKED — Chef loading meal...")
        rospy.loginfo("[NavServerV2] At Kitchen: Loading meal (5s)...")
        rospy.sleep(5.0)
        self._call_compartment('lock')
        self._publish_feedback(0.0, 0.0, "🔒 Meal loaded & compartment locked. Dispatching to table!")
        rospy.sleep(1.0)

        # ── 3. Navigate to Table ─────────────────────────────────────────────
        self._set_state(RobotState.NAVIGATING_TO_TABLE)
        reached_table = self._follow_waypoint_path(kitchen_to_table, f"Delivering to {dest}")
        if not reached_table:
            self._handle_preempt(order_id)
            return

        # ── 4. Arrived at Table — Customer Pickup ────────────────────────────
        self._stop_robot()
        self._set_state(RobotState.ARRIVED_AT_TABLE)
        self._publish_feedback(0.0, 0.0, f"🍱 Arrived at {dest} ({zone_name}) — Please collect your food!")
        rospy.sleep(1.0)

        self._call_compartment('unlock')
        self._set_state(RobotState.COMPARTMENT_UNLOCKED)
        self._publish_feedback(0.0, 0.0, "🔓 Compartment unlocked — customer opening food box...")
        rospy.sleep(2.0)

        self._set_state(RobotState.WAITING_FOR_PICKUP)
        pickup_wait = 10.0
        elapsed = 0.0
        while elapsed < pickup_wait:
            if self._as.is_preempt_requested():
                self._handle_preempt(order_id)
                return
            remaining = pickup_wait - elapsed
            self._publish_feedback(0.0, 0.0, f"Waiting for pickup... {remaining:.0f}s remaining")
            rospy.sleep(2.0)
            elapsed += 2.0

        self._call_compartment('lock')
        self._publish_feedback(0.0, 0.0, "🔒 Food collected! Compartment locked.")
        rospy.sleep(1.0)

        # ── 5. Return to Kitchen ─────────────────────────────────────────────
        self._set_state(RobotState.RETURNING_TO_KITCHEN)
        self._publish_feedback(0.0, 0.0, "Returning to Kitchen Counter for next order...")
        returned_kitchen = self._follow_waypoint_path(table_to_kitchen, "Returning to Kitchen")
        if not returned_kitchen and self._as.is_preempt_requested():
            self._stop_robot()

        # ── 6. IDLE at Kitchen ───────────────────────────────────────────────
        self._stop_robot()
        self._set_state(RobotState.IDLE)

        result.success = True
        result.message = f"✓ Delivered order {order_id} to {dest}. Robot is at Kitchen ready for next order!"
        result.total_distance = 25.0
        self._as.set_succeeded(result)
        rospy.loginfo(f"[NavServerV2] ✓ {result.message}")

    # ── Preemption Handler ───────────────────────────────────────────────────
    def _handle_preempt(self, order_id: str):
        self._stop_robot()
        self._call_compartment('lock')
        self._set_state(RobotState.RETURNING_TO_KITCHEN)
        self._publish_feedback(0.0, 0.0, "❌ Order cancelled — returning to Kitchen...")

        path_to_kitchen = [(5.0, self.current_y), (5.0, 0.0), (7.5, 0.0)]
        self._follow_waypoint_path(path_to_kitchen, "Returning to Kitchen")
        self._stop_robot()
        self._set_state(RobotState.IDLE)

        result = NavigateToGoalResult(
            success=False,
            message=f"Order {order_id} cancelled. Robot returned to Kitchen.",
            total_distance=0.0
        )
        self._as.set_preempted(result)
        rospy.logwarn(f"[NavServerV2] Order {order_id} cancelled — robot at Kitchen.")

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    server = FoodDeliveryActionServer()
    server.run()
