import rospy
import time


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)
# WIP
class PID:
    def __init__(self, kp, ki, kd, dt=0, max_error=2, name="", debug=False):
        self.debug = debug
        self.name = name
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.error = 0
        self.error_sum = 0
        self.error_diff = 0
        self.last_error = 0
        self.max_error = max_error

    
    def update(self, error):
        self.error = error
        self.error_sum += error
        self.error_diff = error - self.last_error
        self.last_error = error
        if(self.max_error != 0):
            self.error_sum = clamp(self.error_sum, -self.max_error, self.max_error)
        pid_val = self.kp * self.error + self.ki * self.error_sum + self.kd * self.error_diff
        pid_val = clamp(pid_val, -2, 2)
        if self.debug:
            rospy.logdebug("PID {}: error: {}, error_sum: {}, pid_val: {}".format(self.name, self.error, self.error_sum, pid_val))
        return pid_val
    

    
    def reset(self):
        self.error = 0
        self.error_sum = 0
        self.error_diff = 0
        self.last_error = 0
        
class RobustPID:
    def __init__(self, kp, ki, kd, max_integral=2.0, max_output=2.0, 
                 name="", debug=False, derivative_filter_alpha=0.1):
        self.debug = debug
        self.name = name
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        # State variables
        self.error = 0
        self.error_sum = 0
        self.error_diff = 0
        self.last_error = 0
        self.filtered_derivative = 0
        
        # Limits
        self.max_integral = max_integral
        self.max_output = max_output
        
        # Time tracking
        self.last_time = time.time()
        
        # Derivative filter to reduce noise
        self.derivative_filter_alpha = derivative_filter_alpha
        
        # For derivative on measurement (optional improvement)
        self.last_measurement = None
    
    def update(self, error, measurement=None):
        current_time = time.time()
        dt = current_time - self.last_time
        
        # Avoid division by zero and handle first call
        if dt <= 0 or dt > 1.0:  # Reset if dt is too large (system pause)
            dt = 0.02  # Default 50Hz control loop
        
        self.error = error
        
        # Proportional term
        p_term = self.kp * self.error
        
        # Integral term with windup protection
        self.error_sum += error * dt
        self.error_sum = clamp(self.error_sum, -self.max_integral, self.max_integral)
        i_term = self.ki * self.error_sum
        
        # Derivative term with filtering
        if measurement is not None and self.last_measurement is not None:
            # Derivative on measurement (reduces derivative kick)
            raw_derivative = -(measurement - self.last_measurement) / dt
        else:
            # Derivative on error
            raw_derivative = (error - self.last_error) / dt
        
        # Low-pass filter for derivative
        self.filtered_derivative = (self.derivative_filter_alpha * raw_derivative + 
                                   (1 - self.derivative_filter_alpha) * self.filtered_derivative)
        
        d_term = self.kd * self.filtered_derivative
        
        # Calculate PID output
        pid_val = p_term + i_term + d_term
        pid_val = clamp(pid_val, -self.max_output, self.max_output)
        
        # Update state for next iteration
        self.last_error = error
        self.last_time = current_time
        if measurement is not None:
            self.last_measurement = measurement
        
        if self.debug:
            rospy.logdebug("PID {}: P={:.3f} I={:.3f} D={:.3f} Total={:.3f} dt={:.3f}".format(
                self.name, p_term, i_term, d_term, pid_val, dt))
        
        return pid_val
    
    def reset(self):
        self.error = 0
        self.error_sum = 0
        self.error_diff = 0
        self.last_error = 0
        self.filtered_derivative = 0
        self.last_time = time.time()
        self.last_measurement = None
