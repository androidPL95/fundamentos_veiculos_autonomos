import numpy as np

class PID:
    def __init__(self, Kp, Ki, Kd, Ts):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Ts = Ts

        self.integral = 0
        self.prev_error = 0

    def update(self, setpoint, measurement):
        error = setpoint - measurement

        # Integral
        self.integral += error * self.Ts

        # Derivada
        derivative = (error - self.prev_error) / self.Ts

        # PID
        u = (
            self.Kp * error +
            self.Ki * self.integral +
            self.Kd * derivative
        )

        self.prev_error = error

        return u