import rospy


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)
# WIP
class PID:
    def __init__(self, kp, ki, kd, dt=0, max_error=2, max_val=1, name="", max_sum = -1):
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
        self.max_val = max_val
        self.max_sum = max_sum

    
    def update(self, error):
        self.error = error
        self.error_sum += error
        self.error_diff = error - self.last_error
        self.last_error = error
        if self.max_sum != -1:
            self.error_sum = clamp(self.error_sum, -self.max_sum, self.max_sum)
        if(self.max_error != 0):
            self.error_sum = clamp(self.error_sum, -self.max_error, self.max_error)
        pid_val = self.kp * self.error + self.ki * self.error_sum + self.kd * self.error_diff
        pid_val = clamp(pid_val, -self.max_val, self.max_val)
        # rospy.logdebug("PID {}: error: {}, error_sum: {}, pid_val: {}".format(self.name, self.error, self.error_sum, pid_val))
        return pid_val
    

    
    def reset(self):
        self.error = 0
        self.error_sum = 0
        self.error_diff = 0
        self.last_error = 0


# if __name__ == "__main__":
#     r = rospy.init_node("pid")
#     x = PID(0.2,0,0)
#     y = PID(0.2,0,0)
#     z = PID(0.2,0,0)
#     yaw = PID(0.2,0,0)
