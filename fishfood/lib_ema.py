class EMA():
# exponential moving average filter
# instances have their own alpha

    def __init__(self, alpha_arg):
        self.alpha = alpha_arg
        self.initialized = 0
        self.S = 0

    def update_ema(self, Y):
        if (self.initialized == 0):
            self.S = 1000
            self.initialized = 1
        else:
            self.S = self.alpha * Y + (1.0 - self.alpha) * self.S
        return self.S

    def get_ema(self):
        return self.S

    def reset_ema(self):
        initialized = 0
