class EMA():

    """Exponential moving average filter. EMA objects have their own weighing parameter alpha.
    
    Attributes:
        alpha (float): Weighing between previous value and current reading, [0,1]
        initialized (bool): Filter status
        S (float): Filtered value
    """

    def __init__(self, alpha_arg):
        """A Filter object with a specified alpha-value is created
        
        Args:
            alpha_arg (float): Weighing between previous value and current reading, [0,1]
        """
        self.alpha = alpha_arg
        self.initialized = False
        self.S = 0

    def update_ema(self, Y):
        """Updates filtered value with current readings
        
        Args:
            Y (float): Current sensor reading
        
        Returns:
            float: EMA-filtered value
        """
        if (self.initialized == False):
            self.S = 1000
            self.initialized = True
        else:
            self.S = self.alpha * Y + (1.0 - self.alpha) * self.S
        return self.S

    def get_ema(self):
        """Returns filtered value without update
        
        Returns:
            float: EMA-filtered value
        """
        return self.S

    def reset_ema(self):
        """Resets filter and "deletes history"
        """
        initialized = False
