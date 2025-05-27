# This program's purpose is to control servos using rqt reconfigure.
# A good equation to know here is duty_cycle = (pulse_width / period) * 100.
# The code below assumes a standard period of 20,000 μs (microseconds) and a standard frequency of 50 Hz.

# Written by Jack Frings '26

import rclpy
from rclpy.node import Node 
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange

import RPi.GPIO as GPIO

class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('servo_controller')

        self.log = self.get_logger()

        # GPIO Setup 
        self.PIN = 13 
        self.angle = 90.0 # neutral position

        # Tell the RPi that we are referring to GPIOs by GPIO number
        # This means we are NOT referring to them by pin number.
        # eg. gpio4 is pin #7, and we reference it as 4.
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.PIN, GPIO.OUT)

        self.pwm = GPIO.PWM(self.PIN, 50) # 50 Hz 
        self.pwm.start(0)

        # Signal Parameter
        descriptor_bounds = FloatingPointRange()
        descriptor_bounds.from_value = 0
        descriptor_bounds.to_value = 180
        descriptor_bounds.step = 0.1
        angle_descriptor = ParameterDescriptor(floating_point_range = [descriptor_bounds])

        self.declare_parameter('angle', self.angle, angle_descriptor)
        self.create_timer(0.1, self.update_parameters)

    def angle_to_duty_cycle(self, angle):
        return 5 + (angle / 180.0) * 5.0

    def update_parameters(self):
        # If signal parameter is changed, then change the pwm signal being sent to self.PIN
        change = (self.angle != self.get_parameter('angle').value)
        if change:
            self.angle = self.get_parameter('angle').value 
            self.pwm.ChangeDutyCycle(self.angle_to_duty_cycle(self.angle))

def main(args=None):
    rclpy.init(args=args)

    servo_controller = ServoControllerNode()
    rclpy.spin(servo_controller)

    servo_controller.pwm.stop()
    GPIO.cleanup()

    servo_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
