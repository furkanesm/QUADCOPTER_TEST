#!/usr/bin/env python3
import os
import json
import math
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import Waypoint
from s500_mission_fsm.mavros_interface import MavrosInterface
from std_msgs.msg import String

class RouteAdapterNode(Node):
    """
    Reads route_results.json, applies mock GSD to convert pixel to NED,
    and pushes them to Ardupilot via MAVROS WaypointPush service.
    """
    def __init__(self):
        super().__init__('route_adapter_node')
        
        # ROS Parameters
        self.declare_parameter('route_file_path', 'companion/offline_planning_results/route_results.json')
        self.declare_parameter('mock_gsd_m_per_px', 0.05) # MUST BE CALIBRATED LATER
        self.declare_parameter('autopilot_type', 'ardupilot')
        self.declare_parameter('altitude_m', 33.0)

        self.route_file = self.get_parameter('route_file_path').value
        self.gsd = self.get_parameter('mock_gsd_m_per_px').value
        self.altitude = self.get_parameter('altitude_m').value
        
        self.mavros = MavrosInterface(self, autopilot_type=self.get_parameter('autopilot_type').value)
        self.statustext_pub = self.create_publisher(String, '/mavros/statustext/send', 10)
        
        self.timer = self.create_timer(2.0, self.check_and_push_route)
        self.route_pushed = False

        self.get_logger().info(f"RouteAdapterNode started. Mock GSD: {self.gsd} m/px (TODO: Calibrate)")

    def check_and_push_route(self):
        if self.route_pushed:
            return

        if not os.path.exists(self.route_file):
            return

        try:
            with open(self.route_file, 'r') as f:
                data = json.load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to read {self.route_file}: {e}")
            return
            
        if data.get('status') != "SUCCESS":
            self.get_logger().warn("Route planning did not succeed. Waiting...")
            return

        # Assuming the JSON contains a "waypoints" array. If not, use start and goal as a fallback mock route.
        waypoints_px = data.get('waypoints', [])
        if not waypoints_px:
            self.get_logger().warn("No explicit 'waypoints' found in json, using 'start' and 'goal'")
            waypoints_px = [data.get('start', [0,0]), data.get('goal', [0,0])]

        start_px_x, start_px_y = waypoints_px[0]

        mission_items = []
        for i, (px, py) in enumerate(waypoints_px):
            # Translate pixel to ENU (East-North-Up) using mock GSD
            # Pixel origin is top-left, we want relative to start pixel.
            dx_px = px - start_px_x
            dy_px = -(py - start_px_y) # Invert Y for North
            
            x_m = dx_px * self.gsd # East
            y_m = dy_px * self.gsd # North
            
            wp = Waypoint()
            wp.frame = Waypoint.FRAME_LOCAL_ENU # MAV_FRAME_LOCAL_ENU (10)
            wp.command = 16 # MAV_CMD_NAV_WAYPOINT
            wp.is_current = (i == 0)
            wp.autocontinue = True
            
            # Param 1-4
            wp.param1 = 0.0 # Hold time
            wp.param2 = 1.0 # Accept Radius
            wp.param3 = 0.0 # Pass Radius
            wp.param4 = math.nan # Yaw
            
            # X, Y, Z
            wp.x_lat = x_m # X in localized ENU
            wp.y_long = y_m # Y in localized ENU
            wp.z_alt = self.altitude # Z Up
            
            mission_items.append(wp)

        # Notify via statustext (MAVLink msg 253 equivalent over ROS)
        min_dist = data.get('min_obs_dist', -1)
        msg = String()
        msg.data = f"AStar path applied. {len(mission_items)} WP. min_dist: {min_dist}px SAFE"
        self.statustext_pub.publish(msg)
        
        success = self.mavros.push_mission(mission_items)
        if success:
            self.get_logger().info(f"Pushed {len(mission_items)} waypoints successfully.")
            self.route_pushed = True
            self.timer.cancel() # Done pushing this offline route
        else:
            self.get_logger().error("Failed to push waypoints.")

def main(args=None):
    rclpy.init(args=args)
    node = RouteAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
