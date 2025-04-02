class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0):
        """
        Initialize the PID controller with gain parameters and the desired setpoint.

        Parameters:
        kp: Proportional gain
        ki: Integral gain
        kd: Derivative gain
        setpoint: Target value for the system
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint

        # Internal states
        self.prev_error = 0
        self.integral = 0

    def compute(self, measured_value, dt):
        """
        Compute the control signal based on the current error.

        Parameters:
        measured_value: The current system measurement
        dt: Time step since the last control signal update

        Returns:
        control_signal: The computed output to correct the system
        """
        error = self.setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        # PID formula
        control_signal = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        # Update previous error
        self.prev_error = error

        return control_signal


# Example: Using the PIDController for UAV altitude control
if __name__ == "__main__":
    import time

    # Initialize PID controller for altitude control
    target_altitude = 100  # Target altitude in meters
    pid = PIDController(kp=2.0, ki=1.0, kd=0.5, setpoint=target_altitude)

    current_altitude = 90  # Current altitude in meters (example)
    dt = 0.1  # Time step in seconds

    for i in range(50):  # Simulate 50 iterations of control
        control_signal = pid.compute(current_altitude, dt)
        
        # Simulate the UAV system's response (simplified here for demonstration)
        current_altitude += 0.1 * control_signal  # Example response factor
        
        print(f"Time: {i*dt:.2f}s, Altitude: {current_altitude:.2f}m, Control Signal: {control_signal:.2f}")
        time.sleep(dt)
