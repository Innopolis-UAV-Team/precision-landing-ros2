"""State machine for precision landing."""
from typing import Optional
from .constants import LandingState


class PrecisionLandingStateMachine:
    """State machine for managing precision landing states."""
    
    def __init__(self, logger=None):
        """Initialize state machine.
        
        Args:
            logger: ROS logger instance (optional)
        """
        self.current_state = LandingState.INITIALIZING
        self.previous_state = None
        self.target_x = None
        self.target_y = None
        self.logger = logger
        
        if self.logger:
            self.logger.info("State machine initialized")
    
    def get_state(self) -> LandingState:
        """Get current state."""
        return self.current_state
    
    def set_state(self, new_state: LandingState):
        """Set new state."""
        if new_state != self.current_state:
            self.previous_state = self.current_state
            self.current_state = new_state
            
            if self.logger:
                self.logger.info(f"State transition: {self.previous_state} -> {self.current_state}")
    
    def update_target_position(self, x: float, y: float):
        """Update target position."""
        self.target_x = x
        self.target_y = y
        
        if self.logger:
            self.logger.debug(f"Target position updated: ({x:.3f}, {y:.3f})")
    
    def get_target_position(self) -> tuple:
        """Get target position."""
        return (self.target_x, self.target_y)
    
    def reset(self):
        """Reset state machine to initial state."""
        self.current_state = LandingState.INITIALIZING
        self.previous_state = None
        self.target_x = None
        self.target_y = None
        
        if self.logger:
            self.logger.info("State machine reset")


class LandingStateProcessor:
    """Process landing state transitions and validations."""
    
    def __init__(self, logger=None):
        """Initialize state processor.
        
        Args:
            logger: ROS logger instance (optional)
        """
        self.logger = logger
        self.state_timeouts = {
            LandingState.SEARCHING: 30.0,  # 30 seconds to find target
            LandingState.CENTERING: 15.0,  # 15 seconds to center
            LandingState.DESCENDING: 20.0, # 20 seconds to descend
            LandingState.LANDING: 10.0,    # 10 seconds final landing
        }
        self.state_entry_times = {}
        
    def is_state_timeout(self, state: LandingState, current_time: float) -> bool:
        """Check if state has timed out."""
        if state not in self.state_timeouts:
            return False
            
        if state not in self.state_entry_times:
            self.state_entry_times[state] = current_time
            return False
            
        elapsed = current_time - self.state_entry_times[state]
        timeout = self.state_timeouts[state]
        
        if elapsed > timeout:
            if self.logger:
                self.logger.warn(f"State {state} timed out after {elapsed:.1f}s")
            return True
            
        return False
    
    def reset_state_timer(self, state: LandingState, current_time: float):
        """Reset timer for a specific state."""
        self.state_entry_times[state] = current_time
        
    def validate_state_transition(self, from_state: LandingState, to_state: LandingState) -> bool:
        """Validate if state transition is allowed."""
        # Define allowed transitions
        allowed_transitions = {
            LandingState.INITIALIZING: [LandingState.IDLE, LandingState.EMERGENCY],
            LandingState.IDLE: [LandingState.SEARCHING, LandingState.EMERGENCY],
            LandingState.SEARCHING: [LandingState.CENTERING, LandingState.IDLE, LandingState.EMERGENCY],
            LandingState.CENTERING: [LandingState.DESCENDING, LandingState.SEARCHING, LandingState.EMERGENCY],
            LandingState.DESCENDING: [LandingState.LANDING, LandingState.CENTERING, LandingState.EMERGENCY],
            LandingState.LANDING: [LandingState.LANDED, LandingState.EMERGENCY],
            LandingState.LANDED: [LandingState.IDLE, LandingState.EMERGENCY],
            LandingState.EMERGENCY: [LandingState.IDLE],
        }
        
        if from_state in allowed_transitions:
            is_allowed = to_state in allowed_transitions[from_state]
            
            if not is_allowed and self.logger:
                self.logger.warn(f"Invalid state transition: {from_state} -> {to_state}")
                
            return is_allowed
        
        return False