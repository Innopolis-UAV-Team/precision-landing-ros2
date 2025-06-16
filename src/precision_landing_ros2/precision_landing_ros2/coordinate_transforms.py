"""Coordinate transformation utilities."""
import math
from typing import Tuple
from geometry_msgs.msg import Quaternion, Vector3


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi] range.
    
    Args:
        angle: Input angle in radians
        
    Returns:
        Normalized angle in radians
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_euler(q: Quaternion) -> Tuple[float, float, float]:
    """Convert quaternion to Euler angles (roll, pitch, yaw).
    
    Args:
        q: Quaternion message
        
    Returns:
        Tuple of (roll, pitch, yaw) in radians
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)
        
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert Euler angles to quaternion.
    
    Args:
        roll: Roll angle in radians
        pitch: Pitch angle in radians
        yaw: Yaw angle in radians
        
    Returns:
        Quaternion message
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    
    return q


def calculate_distance_2d(x1: float, y1: float, x2: float, y2: float) -> float:
    """Calculate 2D Euclidean distance.
    
    Args:
        x1, y1: First point coordinates
        x2, y2: Second point coordinates
        
    Returns:
        Distance between points
    """
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


def limit_velocity(vx: float, vy: float, vz: float, 
                  max_vx: float, max_vy: float, max_vz: float) -> Tuple[float, float, float]:
    """Limit velocity components to maximum values.
    
    Args:
        vx, vy, vz: Velocity components
        max_vx, max_vy, max_vz: Maximum velocity limits
        
    Returns:
        Limited velocity components
    """
    vx = max(-max_vx, min(max_vx, vx))
    vy = max(-max_vy, min(max_vy, vy))
    vz = max(-max_vz, min(max_vz, vz))
    return vx, vy, vz