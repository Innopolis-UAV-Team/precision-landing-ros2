"""PID Controller implementation for precision landing."""
import time
from typing import Optional


class PIDController:
    """PID Controller class."""
    
    def __init__(self, kp: float = 0.0, ki: float = 0.0, kd: float = 0.0,
                 max_integral: float = float('inf'), max_output: float = float('inf')):
        """Initialize PID controller.
        
        Args:
            kp: Proportional gain
            ki: Integral gain  
            kd: Derivative gain
            max_integral: Maximum integral value (anti-windup)
            max_output: Maximum output value
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral = max_integral
        self.max_output = max_output
        
        self.integral = 0.0
        self.previous_error: Optional[float] = None
        self.previous_time: Optional[float] = None
        
    def update_gains(self, kp: float, ki: float, kd: float,
                    max_integral: float, max_output: float):
        """Update PID gains and limits."""
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral = max_integral
        self.max_output = max_output
        
    def compute(self, error: float, current_time: Optional[float] = None) -> float:
        """Compute PID output.
        
        Args:
            error: Current error value
            current_time: Current time (if None, uses system time)
            
        Returns:
            PID output value
        """
        if current_time is None:
            current_time = time.time()
            
        # Calculate time delta
        if self.previous_time is None:
            dt = 0.0
        else:
            dt = current_time - self.previous_time
            
        # Avoid division by zero
        if dt <= 0.0:
            dt = 0.001
            
        # Proportional term
        proportional = self.kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = max(-self.max_integral, min(self.max_integral, self.integral))
        integral = self.ki * self.integral
        
        # Derivative term
        if self.previous_error is None:
            derivative = 0.0
        else:
            derivative = self.kd * (error - self.previous_error) / dt
            
        # Compute output with saturation
        output = proportional + integral + derivative
        output = max(-self.max_output, min(self.max_output, output))
        
        # Store values for next iteration
        self.previous_error = error
        self.previous_time = current_time
        
        return output
        
    def reset(self):
        """Reset PID controller state."""
        self.integral = 0.0
        self.previous_error = None
        self.previous_time = None