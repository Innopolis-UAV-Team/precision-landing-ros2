"""Main precision lander node implementation."""
import math
from typing import Optional, Dict, Any
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import threading  # Add threading module

from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool

from .constants import LandingState, YawControlStrategy, DEFAULT_PARAMS
from .pid_controller import PIDController
from .state_machine import PrecisionLandingStateMachine, LandingStateProcessor
from .coordinate_transforms import (
    normalize_angle, quaternion_to_euler, calculate_distance_2d, limit_velocity
)


class PrecisionLanderNode(Node):
    """ROS2 node for precision landing with MAVROS."""
    
    def __init__(self):
        super().__init__('precision_lander_node')
        
        # Initialize counters for debugging
        self.pose_callback_count = 0
        self.detection_callback_count = 0
        self.control_callback_count = 0
        
        # Initialize state variables
        self.current_pose: Optional[PoseStamped] = None
        self.target_pose: Optional[PoseStamped] = None
        self.mavros_state: Optional[State] = None
        
        # Declare and load parameters
        self._declare_parameters()
        
        # Initialize PID controllers
        self._init_pid_controllers()
        self.get_logger().info("PID controllers initialized with parameters")
        
        # Initialize QoS profiles
        self.mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Initialize subscribers, publishers, and services
        self._init_subscribers()
        self._init_publishers()
        self._init_service_clients()
        
        # Initialize state machine
        self.state_machine = PrecisionLandingStateMachine()
        self.state_processor = LandingStateProcessor()
        
        # Create timers
        control_frequency = self.get_parameter('control_frequency').value
        self.control_timer = self.create_timer(
            1.0 / control_frequency, 
            self.control_loop_callback
        )
        
        # Stats timer
        self.stats_timer = self.create_timer(5.0, self.print_stats)
        
        self.get_logger().info("Precision Lander Node initialized successfully")
        
        self.initializing_done = False  # Prevent OFFBOARD switch during startup
        # print control_frequency value
        self.get_logger().info(f"Control frequency set to {control_frequency}Hz")



        self.last_cmd_vel = TwistStamped()  # Store the last velocity command

    def vel_publish_callback(self):
        """Continuously publish the last commanded velocity to maintain OFFBOARD mode."""
        if self.last_cmd_vel:
            self.last_cmd_vel.header.stamp = self.get_clock().now().to_msg()
            self.cmd_vel_pub.publish(self.last_cmd_vel)
            self.get_logger().info(f"Published velocity: linear=({self.last_cmd_vel.twist.linear.x:.3f}, "
                                    f"{self.last_cmd_vel.twist.linear.y:.3f}, {self.last_cmd_vel.twist.linear.z:.3f}), "
                                    f"angular=({self.last_cmd_vel.twist.angular.z:.3f})")
        # print self.last_cmd_vel
        else:
            
            self.get_logger().info("No last command velocity set, publishing zero velocity")

        
    def print_stats(self):
        """Print node statistics."""
        self.get_logger().info("=== NODE STATISTICS ===")
        self.get_logger().info(
            f"Callbacks - Pose: {self.pose_callback_count}, "
            f"Detection: {self.detection_callback_count}, "
            f"Control: {self.control_callback_count}"
        )
        
        current_state = self.state_machine.get_state()
        self.get_logger().info(f"Current state: {current_state}")
        
        if self.current_pose:
            pos = self.current_pose.pose.position
            self.get_logger().info(f"Current position: [{pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}]")
        
        if self.target_pose:
            target_pos = self.target_pose.pose.position
            self.get_logger().info(f"Target position: [{target_pos.x:.2f}, {target_pos.y:.2f}, {target_pos.z:.2f}]")
        
        if self.mavros_state:
            self.get_logger().info(f"MAVROS - Connected: {self.mavros_state.connected}, "
                                 f"Armed: {self.mavros_state.armed}, Mode: {self.mavros_state.mode}")

    def _declare_parameters(self):
        """Declare all ROS2 parameters with default values."""
        for param_name, default_value in DEFAULT_PARAMS.items():
            if isinstance(default_value, str):
                descriptor = ParameterDescriptor(description=f"Parameter {param_name}")
                self.declare_parameter(param_name, default_value, descriptor)
            else:
                descriptor = ParameterDescriptor(description=f"Parameter {param_name}")
                self.declare_parameter(param_name, float(default_value), descriptor)
            
    def _init_pid_controllers(self):
        """Initialize PID controllers with parameters."""
        # X axis PID
        self.pid_x = PIDController(
            kp=self.get_parameter('pid_x_kp').value,
            ki=self.get_parameter('pid_x_ki').value,
            kd=self.get_parameter('pid_x_kd').value,
            max_integral=self.get_parameter('pid_x_max_integral').value,
            max_output=self.get_parameter('pid_x_max_output').value
        )
        
        # Y axis PID
        self.pid_y = PIDController(
            kp=self.get_parameter('pid_y_kp').value,
            ki=self.get_parameter('pid_y_ki').value,
            kd=self.get_parameter('pid_y_kd').value,
            max_integral=self.get_parameter('pid_y_max_integral').value,
            max_output=self.get_parameter('pid_y_max_output').value
        )
        
        # Z axis PID
        self.pid_z = PIDController(
            kp=self.get_parameter('pid_z_kp').value,
            ki=self.get_parameter('pid_z_ki').value,
            kd=self.get_parameter('pid_z_kd').value,
            max_integral=self.get_parameter('pid_z_max_integral').value,
            max_output=self.get_parameter('pid_z_max_output').value
        )
        
        # Yaw PID (if parameters exist)
        if self.has_parameter('pid_yaw_kp'):
            self.pid_yaw = PIDController(
                kp=self.get_parameter('pid_yaw_kp').value,
                ki=self.get_parameter('pid_yaw_ki').value,
                kd=self.get_parameter('pid_yaw_kd').value,
                max_integral=self.get_parameter('pid_yaw_max_integral').value,
                max_output=self.get_parameter('pid_yaw_max_output').value
            )
        else:
            self.pid_yaw = PIDController(0.5, 0.0, 0.1, 1.0, 1.0)
        
    def _init_subscribers(self):
        """Initialize subscribers."""
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            self.mavros_qos
        )
        
        self.target_sub = self.create_subscription(
            PoseStamped,
            '/mavros/landing_target/pose',
            self.target_callback,
            self.sensor_qos
        )
        
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            self.mavros_qos
        )

        self.get_logger().info("Subscribers initialized: Pose, Target, State")
        
    def _init_publishers(self):
        """Initialize publishers."""
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            10
        )
        self.get_logger().info("Publisher initialized: cmd_vel")
        
    def _init_service_clients(self):
        """Initialize service clients."""
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.get_logger().info("Service clients initialized: SetMode, Arming")

    def pose_callback(self, msg: PoseStamped):
        """Handle drone pose updates."""
        try:
            self.current_pose = msg
            self.pose_callback_count += 1
            
            # Log every 100 calls (5 seconds at 20Hz)
            if self.pose_callback_count % 100 == 0:
                self.get_logger().debug(f"Pose callback #{self.pose_callback_count}")
                
        except Exception as e:
            self.get_logger().error(f"Error in pose_callback: {e}")
        
    def target_callback(self, msg: PoseStamped):
        """Handle landing target pose updates."""
        try:
            self.target_pose = msg
            self.detection_callback_count += 1
            
            # Update state machine if initialized
            if hasattr(self, 'state_machine') and self.state_machine:
                self.state_machine.update_target_position(
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z
                )
                            
        except Exception as e:
            self.get_logger().error(f"Error in target_callback: {e}")

    def state_callback(self, msg: State):
        """Handle MAVROS state updates."""
        try:
            # Check if the drone exited OFFBOARD mode
            if self.mavros_state and self.mavros_state.mode == "OFFBOARD" and msg.mode != "OFFBOARD":
                self.get_logger().warn("Drone exited OFFBOARD mode. Resetting initialization.")
                self._reset_initialization()

            self.mavros_state = msg
        except Exception as e:
            self.get_logger().error(f"Error in state_callback: {e}")

    def _reset_initialization(self):
        """Reset initialization and state machine."""
        self.get_logger().info("Resetting state machine to INITIALIZING state.")
        self.state_machine.set_state(LandingState.INITIALIZING)
        self.initializing_done = False
        self._publish_zero_velocity()
        
    def control_loop_callback(self):
        """Main control loop callback."""
        self.control_callback_count += 1
        self.vel_publish_callback()
        self.get_logger().info(f"Control callback #{self.control_callback_count}")

        if not self._has_required_data():
            # Log only every 100 calls (5 seconds at 20Hz)
            if self.control_callback_count % 100 == 0:
                self.get_logger().warn("Waiting for required data: pose, state")
            self._publish_zero_velocity()  # Ensure velocity commands are published
            return

        # Check for manual mode override
        if (self.mavros_state and
            self.mavros_state.mode not in ["OFFBOARD", "AUTO.LAND", "AUTO.PRECLAND"] and
            self.state_machine.get_state() not in [LandingState.INITIALIZING, LandingState.IDLE]):
            self.get_logger().info("Manual flight mode override detected, reverting to IDLE")
            self.state_machine.set_state(LandingState.IDLE)
            self._publish_zero_velocity()  # Ensure velocity commands are published
            return

        # Get current state
        current_state = self.state_machine.get_state()

        # Log state every 200 calls (10 seconds at 20Hz)
        if self.control_callback_count % 200 == 0:
            self.get_logger().info(f"Current state: {current_state}")

        # Handle states
        if current_state == LandingState.INITIALIZING:
            self._handle_initializing_state()
        elif current_state == LandingState.IDLE:
            self._handle_idle_state()
        elif current_state == LandingState.SEARCHING:
            self._handle_searching_state()
        elif current_state == LandingState.CENTERING:
            self._handle_centering_state()
        elif current_state == LandingState.DESCENDING:
            self._handle_descending_state()
        elif current_state == LandingState.LANDING:
            self._handle_landing_state()
        elif current_state == LandingState.LANDED:
            self._handle_landed_state()
        else:
            self.get_logger().warn(f"Unknown state: {current_state}")
            self._publish_zero_velocity()  # Ensure velocity commands are published

    def _handle_initializing_state(self):
        """Handle INITIALIZING state."""
        if self.mavros_state and self.mavros_state.connected:
            self.get_logger().info("MAVROS connected, transitioning to IDLE")
            self.state_machine.set_state(LandingState.IDLE)
            self.initializing_done = True  # Mark initialization as complete
        self._publish_zero_velocity()

    def _handle_idle_state(self):
        """Handle IDLE state."""
        # Check if drone is ready for landing
        if (self.mavros_state and
            self.mavros_state.armed and
            self.mavros_state.mode in ["OFFBOARD", "AUTO.LAND", "AUTO.PRECLAND"]):
            self.get_logger().info("Drone armed and in a valid mode for landing, starting target search")
            self.state_machine.set_state(LandingState.SEARCHING)
            # switch to OFFBOARD mode if not already
            self._switch_to_offboard_mode()
            
        # Log status every 200 calls
        if self.control_callback_count % 200 == 0:
            armed_status = "armed" if (self.mavros_state and self.mavros_state.armed) else "disarmed"
            mode_status = self.mavros_state.mode if self.mavros_state else "unknown"
            self.get_logger().info(f"Waiting for OFFBOARD mode. Current: {mode_status}, {armed_status}")

    def _handle_searching_state(self):
        """Handle SEARCHING state."""
        if self.target_pose is not None:
            # Target found, transition to centering
            self.get_logger().info("Target detected, transitioning to CENTERING")
            self.state_machine.set_state(LandingState.CENTERING)
            self._switch_to_offboard_mode()

        # Continue searching, hold position
        self._publish_zero_velocity()
        
        # Log search every 200 calls
        if self.control_callback_count % 200 == 0:
            self.get_logger().info("Searching for landing target...")

    def _handle_centering_state(self):
        """Handle CENTERING state."""
        if self.target_pose is None:
            # Target lost, return to searching
            self.get_logger().warn("Target lost, returning to SEARCHING")
            self.state_machine.set_state(LandingState.SEARCHING)
            return

        # Calculate error in X and Y
        error_x = self.target_pose.pose.position.x - self.current_pose.pose.position.x
        error_y = self.target_pose.pose.position.y - self.current_pose.pose.position.y
        distance_to_target = math.sqrt(error_x**2 + error_y**2)

        # Check if close enough to center
        landing_threshold = self.get_parameter('landing_threshold').value
        if distance_to_target < landing_threshold:
            self.get_logger().info(f"Centered over target (distance: {distance_to_target:.3f}), starting descent")
            self.state_machine.set_state(LandingState.DESCENDING)
            return

        # Execute centering control
        self._execute_centering_control(error_x, error_y)

    def _handle_descending_state(self):
        """Handle DESCENDING state."""
        if self.target_pose is None:
            # Target lost during descent
            self.get_logger().warn("Target lost during descent, returning to SEARCHING")
            self.state_machine.set_state(LandingState.SEARCHING)
            return

        # Check altitude for landing completion
        current_altitude = self.current_pose.pose.position.z
        if current_altitude < 0.2:  # Close to ground
            self.get_logger().info("Near ground, transitioning to LANDING")
            self.state_machine.set_state(LandingState.LANDING)
            return

        # Continue descent with position correction
        self._execute_descending_control()

    def _handle_landing_state(self):
        """Handle LANDING state."""
        # Slow descent until ground contact
        final_descent_speed = self.get_parameter('final_descent_speed').value
        
        cmd_vel = TwistStamped()
        cmd_vel.header.stamp = self.get_clock().now().to_msg()
        cmd_vel.header.frame_id = "base_link"
        
        cmd_vel.twist.linear.z = -final_descent_speed  # Slow descent
        cmd_vel.twist.linear.x = 0.0
        cmd_vel.twist.linear.y = 0.0
        cmd_vel.twist.angular.z = 0.0
        
        # self.cmd_vel_pub.publish(cmd_vel)
        self.last_cmd_vel = cmd_vel  # Update last commanded velocity


        # Check ground contact
        if self.current_pose.pose.position.z < 0.1:
            self.get_logger().info("Landing completed")
            self.state_machine.set_state(LandingState.LANDED)

    def _handle_landed_state(self):
        """Handle LANDED state."""
        # Stop motors or hold position
        self._publish_zero_velocity()
        
        # Log completion every 200 calls
        if self.control_callback_count % 200 == 0:
            self.get_logger().info("Landing completed successfully")

    def _execute_centering_control(self, error_x: float, error_y: float):
        """Execute centering control using PID."""
        # Calculate PID commands
        dt = 1.0 / self.get_parameter('control_frequency').value
        
        vel_x = self.pid_x.compute(error_x, dt)
        vel_y = self.pid_y.compute(error_y, dt)
        
        # Limit velocities
        max_vel_xy = self.get_parameter('max_velocity_xy').value
        vel_x = max(-max_vel_xy, min(max_vel_xy, vel_x))
        vel_y = max(-max_vel_xy, min(max_vel_xy, vel_y))
        
        # Publish command
        cmd_vel = TwistStamped()
        cmd_vel.header.stamp = self.get_clock().now().to_msg()
        cmd_vel.header.frame_id = "base_link"
        
        cmd_vel.twist.linear.x = vel_x
        cmd_vel.twist.linear.y = vel_y
        cmd_vel.twist.linear.z = 0.0  # Hold altitude
        cmd_vel.twist.angular.z = 0.0
        
        # self.cmd_vel_pub.publish(cmd_vel)
        self.last_cmd_vel = cmd_vel  # Update last commanded velocity
        
        # Log commands every 100 calls
        # if self.control_callback_count % 100 == 0:
        self.get_logger().info(f"Centering: error=({error_x:.3f}, {error_y:.3f}), vel=({vel_x:.3f}, {vel_y:.3f})")

    def _execute_descending_control(self):
        """Execute descending control with position correction."""
        if self.target_pose is None:
            return
            
        # Position correction during descent
        error_x = self.target_pose.pose.position.x - self.current_pose.pose.position.x
        error_y = self.target_pose.pose.position.y - self.current_pose.pose.position.y
        
        dt = 1.0 / self.get_parameter('control_frequency').value
        vel_x = self.pid_x.compute(error_x, dt) * 0.5  # Reduced correction
        vel_y = self.pid_y.compute(error_y, dt) * 0.5
        
        # Descent speed
        final_descent_speed = self.get_parameter('final_descent_speed').value
        vel_z = -final_descent_speed * 2.0  # Faster than final descent
        
        cmd_vel = TwistStamped()
        cmd_vel.header.stamp = self.get_clock().now().to_msg()
        cmd_vel.header.frame_id = "base_link"
        
        cmd_vel.twist.linear.x = vel_x
        cmd_vel.twist.linear.y = vel_y
        cmd_vel.twist.linear.z = vel_z
        cmd_vel.twist.angular.z = 0.0
        
        # self.cmd_vel_pub.publish(cmd_vel)
        self.last_cmd_vel = cmd_vel  # Update last commanded velocity
        
        # Log descent every 50 calls
        if self.control_callback_count % 50 == 0:
            altitude = self.current_pose.pose.position.z
            self.get_logger().info(f"Descending: altitude={altitude:.3f}m, vel=({vel_x:.3f}, {vel_y:.3f}, {vel_z:.3f})")

    def _publish_zero_velocity(self):
        """Publish zero velocity command and update the last command."""
        self.last_cmd_vel = TwistStamped()
        self.last_cmd_vel.header.stamp = self.get_clock().now().to_msg()
        self.last_cmd_vel.header.frame_id = "base_link"
        self.last_cmd_vel.twist.linear.x = 0.0
        self.last_cmd_vel.twist.linear.y = 0.0
        self.last_cmd_vel.twist.linear.z = 0.0
        self.last_cmd_vel.twist.angular.z = 0.0
        # self.cmd_vel_pub.publish(self.last_cmd_vel)        

        self.get_logger().debug("Published zero velocity command")

    def _has_required_data(self) -> bool:
        """Check if we have required data for control."""
        return (self.current_pose is not None and 
                self.mavros_state is not None)
    
    def _switch_to_offboard_mode(self):
        """Switch flight mode to OFFBOARD in a separate thread."""
        self.get_logger().info("Switching flight mode to OFFBOARD in a separate thread...")
        thread = threading.Thread(target=self._switch_to_offboard_mode_thread)
        thread.start()

    def _switch_to_offboard_mode_thread(self):
        """Threaded function to switch flight mode to OFFBOARD."""
        # Check if MAVROS state is available
        if not self.mavros_state:
            self.get_logger().error("MAVROS state is not available. Cannot switch to OFFBOARD mode.")
            return

        # Ensure the drone is armed
        if not self.mavros_state.armed:
            self.get_logger().error("Drone is not armed. Cannot switch to OFFBOARD mode.")
            return

        # Ensure valid velocity commands are being published
        if not self.last_cmd_vel:
            self.get_logger().error("No velocity commands are being published. Cannot switch to OFFBOARD mode.")
            return

        # Wait for the SetMode service to become available
        if not self.set_mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("SetMode service not available. Cannot switch to OFFBOARD mode.")
            return

        # Attempt to switch to OFFBOARD mode
        self.get_logger().info("Attempting to switch to OFFBOARD mode...")
        req = SetMode.Request()
        req.custom_mode = "OFFBOARD"
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        # Check the result of the service call
        if future.result() and future.result().mode_sent:
            self.get_logger().info("Flight mode set to OFFBOARD successfully.")
        else:
            self.get_logger().error("Failed to set flight mode to OFFBOARD. Check conditions and try again.")


def main(args=None):
    """Main function."""
    import logging
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)
    
    logger.info("=== MAIN FUNCTION STARTED ===")
    
    try:
        rclpy.init(args=args)
        logger.info("RCLPy initialized successfully")
        
        precision_lander_node = PrecisionLanderNode()
        precision_lander_node.get_logger().info("Node created successfully")
        
        precision_lander_node.get_logger().info("Starting spin...")
        rclpy.spin(precision_lander_node)
        
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received")
    except Exception as e:
        logger.error(f"Error in main: {e}")
        import traceback
        logger.error(traceback.format_exc())
    finally:
        logger.info("Cleaning up...")
        if 'precision_lander_node' in locals():
            precision_lander_node.destroy_node()
        rclpy.shutdown()
        logger.info("=== MAIN FUNCTION ENDED ===")


if __name__ == '__main__':
    main()