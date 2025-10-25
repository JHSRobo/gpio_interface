# This program's purpose is to control servos using rqt reconfigure.
# A good equation to know here is duty_cycle = (pulse_width / period) * 100.
# The code below assumes a standard period of 20,000 μs (microseconds) and a standard frequency of 50 Hz.
# Written by Jack Frings '26

import rclpy
from rclpy.node import Node 
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange
import time

from gpiozero import Servo
from gpiozero.pins.lgpio import LGPIOFactory

class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('servo_controller')
        self.log = self.get_logger()

        # GPIO Setup
        self.factory = LGPIOFactory()
        self.PIN = 12
        self.angle = 180  # Initial angle

        # Declare angle parameter with bounds
        descriptor_bounds = IntegerRange()
        descriptor_bounds.from_value = 120
        descriptor_bounds.to_value = 240
        descriptor_bounds.step = 1
        angle_descriptor = ParameterDescriptor(integer_range=[descriptor_bounds])

        self.declare_parameter('angle', self.angle, angle_descriptor)
        self.create_timer(0.1, self.update_parameters)

    def update_parameters(self):
        new_angle = self.get_parameter('angle').value

        # If signal parameter is changed, then change the pwm signal being sent to self.PIN
        if self.angle != new_angle:
            self.angle = new_angle

            # Connects the servo if a change needs to be done
            servo = Servo(self.PIN, pin_factory=self.factory,
                          min_pulse_width=0.0005, max_pulse_width=0.0025)

            pulse_width = (self.angle / 360) * (2500 - 500) + 500
            servo_value = (pulse_width - 1500) / 1000
            servo.value = servo_value

            time.sleep(1)  # Allow time for movement

            # These lines make sure that the servo disconnects after updating its parameters
            servo.close()

def main(args=None):
    rclpy.init(args=args)

    servo_controller = ServoControllerNode()
    rclpy.spin(servo_controller)

    servo_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
