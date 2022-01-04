import time


class PIDController:

    def __init__(self, target_pos):
        self.target_pos = target_pos
        self.Kp = 5462
        self.Ki = 7016
        self.Kd = 2037

        self.currentTime = time.time()
        self.lastTime = self.currentTime
        self.lastError = 0.0

        self.integralTermSum = 0.0

        self.bias = 0.0

        return

    def reset(self):
        return

    # TODO: Complete your PID control within this function. At the moment, it holds
    #      only the bias. Your final solution must use the error between the
    #      target_pos and the ball position, plus the PID gains. You cannot
    #      use the bias in your final answer.
    def get_fan_rpm(self, vertical_ball_position):
        error = self.target_pos - vertical_ball_position

        self.currentTime = time.time()
        deltaTime = self.currentTime - self.lastTime
        deltaError = error - self.lastError

        # deltaTimeCumulative = self.timer - self.lastTime
        # deltaTimeInstant = .3
        # print(deltaTimeCumulative)

        KpTerm = self.Kp * error

        self.integralTermSum += error * deltaTime
        # discrete approximation of derivative

        KdTerm = deltaError / .01

        self.lastTime = self.currentTime
        self.lastError = error

        output = KpTerm + (self.integralTermSum * self.Ki) + (KdTerm * self.Kd)

        return output
