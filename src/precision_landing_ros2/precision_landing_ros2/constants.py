"""Constants and default parameters for precision landing."""
from enum import Enum


class LandingState(Enum):
    """States for the precision landing state machine."""
    INITIALIZING = "initializing"
    IDLE = "idle"
    SEARCHING = "searching"
    CENTERING = "centering"
    DESCENDING = "descending"
    LANDING = "landing"
    LANDED = "landed"
    EMERGENCY = "emergency"


class YawControlStrategy(Enum):
    """Yaw control strategies."""
    FIXED = "fixed"
    FOLLOW_TARGET = "follow_target"
    WIND_ALIGNED = "wind_aligned"


# Default parameters for the precision landing system
DEFAULT_PARAMS = {
    # PID controller parameters for X axis
    'pid_xy_kp': 0.5,
    'pid_xy_ki': 0.1,
    'pid_xy_kd': 0.05,
    'pid_xy_max_integral': 1.0,
    'pid_xy_max_output': 2.0,

    
    # PID controller parameters for Z axis
    'pid_z_kp': 0.8,
    'pid_z_ki': 0.15,
    'pid_z_kd': 0.1,
    'pid_z_max_integral': 0.5,
    'pid_z_max_output': 1.5,
    
    # PID controller parameters for Yaw axis
    'pid_yaw_kp': 0.5,
    'pid_yaw_ki': 0.0,
    'pid_yaw_kd': 0.1,
    'pid_yaw_max_integral': 1.0,
    'pid_yaw_max_output': 1.0,
    
    # Maximum velocities
    'max_velocity_xy': 2.0,
    'max_velocity_z': 1.0,

    # Landing parameters
    'centering_threshold': 0.5,  # Distance to start searching for the landing target
    'landing_threshold': 0.1,
    'yaw_threshold': 5.0,  # Yaw angle threshold for landing in degrees
    'search_altitude': 2.0,  # Altitude to start centering and descending
    'landing_speed': 0.3,
    'final_landing_speed': 0.5,

    # Detection parameters
    'detection_timeout': 5.0,
    'min_detection_confidence': 0.7,

    # AUTO.MISSION intercept parameters
    'auto_mission_remaining_waypoints': 3.0,
    'auto_mission_min_reached_wp': 1.0,
    
    # Frame names
    'world_frame': "map",
    'body_frame': "base_link",
    
    # Control frequency
    'control_frequency': 20.0,
    
    # Safety parameters
    'max_tilt_angle': 0.5,
    'emergency_altitude': 5.0,
}
