#!/usr/bin/env python3
"""
send_goal_example.py  — INTERACTIVE RESTAURANT DELIVERY DISPATCHER
==================================================================
Prompts the operator to select a table, dispatches the robot to Kitchen
to pick up food, delivers to the table, and returns to Kitchen for the next order!

Usage:
  rosrun food_robot_bringup send_goal_example.py
  (or with direct args: rosrun food_robot_bringup send_goal_example.py Table3 order_001)
"""

import sys
import rospy
import actionlib
from food_robot_msgs.msg import NavigateToGoalAction, NavigateToGoalGoal

# ── ANSI Terminal Colors ─────────────────────────────────────────────────────
RED     = '\033[91m'
GREEN   = '\033[92m'
YELLOW  = '\033[93m'
BLUE    = '\033[94m'
MAGENTA = '\033[95m'
CYAN    = '\033[96m'
BOLD    = '\033[1m'
RESET   = '\033[0m'

TABLE_OPTIONS = {
    '1': 'Table1',
    '2': 'Table2',
    '3': 'Table3',
    '4': 'Table4',
    '5': 'Table5',
    '6': 'Table6',
    '7': 'Room204',
    '8': 'FrontDesk',
}

STATE_COLORS = {
    'NAVIGATING_TO_KITCHEN': CYAN,
    'AT_KITCHEN_LOADING':    MAGENTA,
    'NAVIGATING_TO_TABLE':   BLUE,
    'ARRIVED_AT_TABLE':      GREEN,
    'COMPARTMENT_UNLOCKED':  GREEN,
    'WAITING_FOR_PICKUP':    YELLOW,
    'RETURNING_TO_KITCHEN':  CYAN,
    'IDLE':                  RESET,
}


def feedback_cb(feedback):
    state = feedback.current_state
    color = STATE_COLORS.get(state, RESET)
    print(f"  {color}[{state}]{RESET}  "
          f"dist: {feedback.distance_remaining:.2f}m  ETA: {feedback.eta_seconds:.0f}s  "
          f"→ {feedback.status_text}")


def print_banner():
    print(f"\n{BOLD}{CYAN}╔══════════════════════════════════════════════════════╗{RESET}")
    print(f"{BOLD}{CYAN}║    🍱 RESTAURANT FOOD DELIVERY ROBOT DISPATCHER     ║{RESET}")
    print(f"{BOLD}{CYAN}╚══════════════════════════════════════════════════════╝{RESET}\n")


def print_menu():
    print(f"{BOLD}Select destination table for food delivery:{RESET}")
    print(f"  {CYAN}[1]{RESET} Table 1        {CYAN}[2]{RESET} Table 2        {GREEN}[3] Table 3 (Green Zone){RESET}")
    print(f"  {CYAN}[4]{RESET} Table 4        {CYAN}[5]{RESET} Table 5        {CYAN}[6]{RESET} Table 6")
    print(f"  {BLUE}[7]{RESET} Room 204 (VIP) {YELLOW}[8]{RESET} Front Desk     {RED}[q]{RESET} Quit / Exit\n")


def dispatch_order(client, destination, order_id):
    print(f"\n{BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━{RESET}")
    print(f"🚀 {BOLD}Dispatching Order:{RESET} {CYAN}{order_id}{RESET} ➔ Destination: {GREEN}{BOLD}{destination}{RESET}")
    print(f"   {YELLOW}Workflow:{RESET} Kitchen (Load) ➔ {destination} (Serve) ➔ Kitchen (Return)")
    print(f"   (Press {YELLOW}Ctrl+C{RESET} to cancel order)\n")

    goal = NavigateToGoalGoal(destination_name=destination, order_id=order_id)
    client.send_goal(goal, feedback_cb=feedback_cb)

    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        print(f"\n{YELLOW}⚡ Order cancelled by operator! Robot returning to Kitchen...{RESET}")
        client.cancel_goal()
        client.wait_for_result(rospy.Duration(15.0))

    result = client.get_result()
    print()
    if result and result.success:
        print(f"{GREEN}{BOLD}✓ ORDER COMPLETE!{RESET}")
        print(f"  {result.message}")
    else:
        print(f"{RED}{BOLD}✗ Order ended or cancelled.{RESET}")
        if result:
            print(f"  {result.message}")
    print(f"{BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━{RESET}\n")


def main():
    rospy.init_node('food_delivery_dispatcher', anonymous=True)
    print_banner()

    print("Connecting to robot action server (/navigate_to_goal_v2)...")
    client = actionlib.SimpleActionClient('/navigate_to_goal_v2', NavigateToGoalAction)
    if not client.wait_for_server(rospy.Duration(10.0)):
        print(f"{RED}✗ Action server not available!{RESET}")
        print("  Make sure navigate_action_server_v2.py is running.")
        sys.exit(1)
    print(f"{GREEN}✓ Connected to Food Delivery Robot!{RESET}\n")

    # If args were passed via CLI: rosrun food_robot_bringup send_goal_example.py Table3 order_001
    if len(sys.argv) >= 3:
        dest = sys.argv[1]
        order_id = sys.argv[2]
        dispatch_order(client, dest, order_id)
        return

    order_counter = 1

    # Interactive loop
    while not rospy.is_shutdown():
        print_menu()
        try:
            choice = input(f"{BOLD}👉 Which table to serve? (1-8 or table name, 'q' to quit): {RESET}").strip()
        except (KeyboardInterrupt, EOFError):
            print(f"\n{YELLOW}Exiting dispatcher.{RESET}")
            break

        if choice.lower() in ('q', 'quit', 'exit'):
            print(f"{YELLOW}Exiting dispatcher. Have a great day!{RESET}")
            break

        # Map choice 1-8 or direct name
        target_table = TABLE_OPTIONS.get(choice, choice)

        if target_table not in ('Table1', 'Table2', 'Table3', 'Table4', 'Table5', 'Table6', 'Room204', 'FrontDesk'):
            print(f"{RED}Invalid selection '{choice}'. Please select 1-8 or valid table name.{RESET}\n")
            continue

        order_id = f"ORDER_{order_counter:03d}"
        order_counter += 1

        dispatch_order(client, target_table, order_id)


if __name__ == '__main__':
    main()
