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
    
    def motor_setpoint(self, expected, actual, dt):
        error = expected - actual
        # Convert to RPM
        # error = error / (900/60)
        print(f"rotation error = {error}")
        controller = self.compute(error, dt) / ((170 * 10/12 * 1) * 900 / dt*60)
        if controller > 1:
            controller = 1
        elif controller < 0:
            controller = 0
        return controller * 100


# Initialize the PI controller with gains
# pi_controller = PIController(Kp=10, Ki=0.1)

# while True:
#     # Setpoint
#     expected_rpm = 100 # EXPECTED SPEED OF MOTOR
#     expected_tick_per_sec = expected_rpm * (900/60)

#     en1_ticks = 100
#     en2_ticks = 120
#     dt = 0.1
#     # Ticks per second
#     en1_speed = en1_ticks / dt
#     en2_speed = en2_ticks / dt

#     # Compute new speed clamped from 0-100
#     m1_speed = max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, en1_speed, dt)))
#     m2_speed = max(0, min(100, pi_controller.motor_setpoint(expected_tick_per_sec, en2_speed, dt)))

    # HERE YOU WOULD SET MOTOR FUNCTION
    # set_motor(in1_left, in2_left, motor_num=0, direction=1, speed=set_speed(m1_speed))
    # set_motor(in1_right, in2_right, motor_num=1, direction=1, speed=set_speed(m2_speed))



