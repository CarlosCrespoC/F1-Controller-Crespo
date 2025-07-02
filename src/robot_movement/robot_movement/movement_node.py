import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import math

class ModifiedFSMController(Node):
    def __init__(self):
        super().__init__('modified_fsm_controller_node')
        
        # --- Publisher and Subscriber Setup ---
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.subscriber = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)

        # --- Finite State Machine (FSM) Definition ---
        self.state = 'ADVANCE'
        
        # --- Trajectory and Control Parameters ---
        self.target_points = [
            (9.0, 0.0),      # Starting point (near target 1)
            (9.4, 8.0),      # Corner 1
            (-13.0, 8.75),  # Corner 2
            (-13.50, 0.51),  # Corner 3
            (0.0, 0.0)       # Final point (back to origin)
        ]
        self.target_index = 0
        self.tolerance = 0.5
        self.desired_turn_angle = math.radians(90)
        
        # --- Variables to store the robot's pose ---
        self.current_pose = None
        self.current_yaw = None
        self.turn_start_yaw = None
        
        self.get_logger().info('FSM node with separated conditionals has been initialized.')

    def odom_callback(self, msg):
        """
        Main callback that executes with each odometry message.
        The logic is structured into separate conditionals for each state.
        """
        # Step 1: Always update the robot's position and orientation.
        self._update_odometry(msg)
        
        # If we don't have odometry data yet, do nothing.
        if self.current_pose is None:
            return
            
        # --- SEQUENCE OF SEPARATE CONDITIONALS ---
        
        # Conditional 1: Is the current state ADVANCE?
        if self.state == 'ADVANCE':
            self._handle_advance_state()

        # Conditional 2: Is the current state TURN?
        if self.state == 'TURN':
            self._handle_turn_state()

        # Conditional 3: Is the current state STOP?
        if self.state == 'STOP':
            self._handle_stop_state()

    def _handle_advance_state(self):
        """Manages the logic when the robot is in the ADVANCE state."""
        current_target = self.target_points[self.target_index]
        distance_to_target = math.hypot(current_target[0] - self.current_pose[0], current_target[1] - self.current_pose[1])
        
        # Condition to keep advancing if the target has not yet been reached.
        if distance_to_target >= self.tolerance:
            self.get_logger().info(f'Advancing towards {current_target} | Remaining distance: {distance_to_target:.2f} m')
            self._publish_command(speed=1.5, steering_angle=0.0)
        
        # Condition to change state if the target has been reached.
        if distance_to_target < self.tolerance:
            self.get_logger().info(f'Target point {current_target} reached.')
            self._transition_from_advancing()

    def _transition_from_advancing(self):
        """Decides which state to transition to after reaching a target point."""
        # Condition: Is this the last point in the trajectory?
        if self.target_index == len(self.target_points) - 1:
            self.get_logger().info('Final trajectory point reached. Changing to STOP state.')
            self.state = 'STOP'
        
        # Condition: Is this not the last point in the trajectory?
        if self.target_index < len(self.target_points) - 1:
            self.get_logger().info('Initiating preparation for the turn.')
            self.state = 'TURN'
            self.turn_start_yaw = self.current_yaw

    def _handle_turn_state(self):
        """Manages the logic when the robot is in the TURN state."""
        if self.turn_start_yaw is None:
            return

        angle_difference = self._normalize_angle(self.current_yaw - self.turn_start_yaw)
        
        # Condition to keep turning if the desired angle has not been reached.
        if abs(angle_difference) < (self.desired_turn_angle - 0.05): # Small margin
            self.get_logger().info(f'Turning... Current angle: {math.degrees(angle_difference):.2f}°')
            self._publish_command(speed=0.5, steering_angle=0.3)
            
        # Condition to change state if the turn is complete.
        if abs(angle_difference) >= (self.desired_turn_angle - 0.05):
            self.get_logger().info('Turn completed.')
            self._transition_from_turning()

    def _transition_from_turning(self):
        """
        Decides which state to transition to after completing a turn.
        CORRECTED: After a turn, the robot should always switch to the ADVANCE
        state to head towards the next point.
        """
        self.target_index += 1
        self.turn_start_yaw = None  # Reset the starting yaw

        # Check if we are still within the bounds of the points list
        if self.target_index < len(self.target_points):
            next_target = self.target_points[self.target_index]
            self.get_logger().info(f'Changing to ADVANCE state. Next target: {next_target}')
            self.state = 'ADVANCE'
        else:
            # If for some reason we exceed the index, stop for safety.
            self.get_logger().warn('Target index exceeded list of points. Stopping the robot.')
            self.state = 'STOP'

    def _handle_stop_state(self):
        """Manages the logic to stop the robot. Publishes once and then remains idle."""
        self.get_logger().info('Robot stopped at its final position.', throttle_duration_sec=5)
        self._publish_command(speed=0.0, steering_angle=0.0)

    def _update_odometry(self, msg):
        """Extracts and updates the position (x, y) and orientation (yaw) from the odometry message."""
        pos = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.current_pose = (pos.x, pos.y)

        # Calculate yaw from quaternion
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y ** 2 + orientation.z ** 2)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
    def _publish_command(self, speed, steering_angle):
        """Creates and publishes an AckermannDriveStamped message."""
        msg = AckermannDriveStamped()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering_angle)
        self.publisher.publish(msg)

    def _normalize_angle(self, angle):
        """Normalizes an angle to the range [-π, π]."""
        while angle > math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

def main(args=None):
    """Main function to initialize the ROS2 node."""
    rclpy.init(args=args)
    node = ModifiedFSMController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down the node. Stopping the robot.')
        node._publish_command(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()