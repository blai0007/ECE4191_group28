class PIController:
    def __init__(self, Kp, Ki):
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.integral = 0  # Accumulated error (integral term)
        self.previous_error = 0  # Previous error (for calculating integral)

    def compute(self, error, dt):
        # Proportional
        p = self.Kp * error

        # Integral 
        self.integral += error * dt
        i = self.Ki * self.integral

        output = p + i
        return output
    
def motor_setpoint(setpoint, actual, dt):
    error = setpoint - actual
    # Convert to RPM
    error = error / (900/60)

    controller = pi_controller.compute(error, dt)

    return controller


# Initialize the PI controller with gains
pi_controller = PIController(kp=0.5, ki=0.1)

while True:
    # Setpoint
    expected_rpm = 100 # EXPECTED SPEED OF MOTOR
    expected_tick_per_sec = expected_rpm * (900/60)

    en1_ticks = 100
    en2_ticks = 120
    dt = 0.1
    # Ticks per second
    en1_speed = en1_ticks / dt
    en2_speed = en2_ticks / dt

    # Compute new speed clamped from 0-100
    m1_speed = max(0, min(100, motor_setpoint(expected_tick_per_sec, en1_speed, dt)))
    m2_speed = max(0, min(100, motor_setpoint(expected_tick_per_sec, en2_speed, dt)))

    # HERE YOU WOULD SET MOTOR FUNCTION
    # set_motor(in1_left, in2_left, motor_num=0, direction=1, speed=set_speed(m1_speed))
    # set_motor(in1_right, in2_right, motor_num=1, direction=1, speed=set_speed(m2_speed))



