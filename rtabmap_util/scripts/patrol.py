#!/usr/bin/env python3
import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rtabmap_msgs.msg import Goal


class PatrolNode(Node):
    def __init__(self, waypoints):
        super().__init__('patrol')

        # --- Parameters ---
        self.declare_parameter('time', 1.0)
        self.declare_parameter('frame_id', '')
        self.waiting_time = self.get_parameter('time').value
        self.frame_id = self.get_parameter('frame_id').value

        # --- Variables ---
        self.waypoints = waypoints
        self.current_index = 0

        # --- Publisher & Subscriber ---
        self.pub = self.create_publisher(Goal, 'rtabmap/goal_node', 10)
        self.sub = self.create_subscription(Bool, 'rtabmap/goal_reached', self.callback, 10)

        self.get_logger().info(f"Waypoints: {self.waypoints}")
        self.get_logger().info(f"Waiting time: {self.waiting_time:.1f} sec")
        self.get_logger().info(f"Publishing goals on: {self.pub.topic_name}")
        self.get_logger().info(f"Receiving goal status on: {self.sub.topic_name}")

        # Delay before sending first goal (ensure discovery)
        time.sleep(1.0)

        # Send first goal
        self.send_goal()

    def callback(self, msg: Bool):
        """Called when goal_reached is received."""
        if msg.data:
            self.get_logger().info(
                f"Goal '{self.waypoints[self.current_index]}' reached! "
                f"Publishing next goal in {self.waiting_time:.1f} sec..."
            )
        else:
            self.get_logger().info(
                f"Goal '{self.waypoints[self.current_index]}' failed! "
                f"Publishing next goal in {self.waiting_time:.1f} sec..."
            )

        # Move to next waypoint
        self.current_index = (self.current_index + 1) % len(self.waypoints)

        # Wait before sending next goal
        time.sleep(self.waiting_time)
        self.send_goal()

    def send_goal(self):
        """Send current goal to RTAB-Map."""
        waypoint = self.waypoints[self.current_index]
        msg = Goal()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.frame_id = self.frame_id

        # Check if waypoint is a node id (int) or a label (string)
        try:
            msg.node_id = int(waypoint)
            msg.node_label = ""
        except ValueError:
            msg.node_id = 0
            msg.node_label = waypoint

        self.get_logger().info(
            f"Publishing goal '{waypoint}' ({self.current_index + 1}/{len(self.waypoints)})"
        )
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    # Extract waypoints from command-line args
    if len(sys.argv) < 3:
        print(
            "Usage: patrol.py waypointA waypointB waypointC ... "
            "[--ros-args -p time:=1.0 -p frame_id:=base_footprint]"
        )
        return

    waypoints = [x for x in sys.argv[1:] if not x.startswith('--') and not x.startswith('_')]
    node = PatrolNode(waypoints)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()