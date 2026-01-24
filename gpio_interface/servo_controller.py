# This program's purpose is to control servos using rqt reconfigure.
# A good equation to know here is duty_cycle = (pulse_width / period) * 100.
# The code below assumes a standard period of 20,000 μs (microseconds) and a standard frequency of 50 Hz.

# Written by Jack Frings '26

import rclpy
from rclpy.node import Node 
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange
import time

from gpiozero import AngularServo
from gpiozero.pins.lgpio import LGPIOFactory

class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('servo_controller')

        self.log = self.get_logger()

        # GPIO and Servo Setup 
        self.factory = LGPIOFactory()
        self.PIN = 13
        self.servo = AngularServo(self.PIN,
            pin_factory=self.factory,
            min_angle=-90,
            max_angle=90,                               
            min_pulse_width=0.0005,
            max_pulse_width=0.0025)

        # Set Initial Servo Angle
        self.angle = 0
        self.servo.angle = self.angle
        
        # Allow time to move before closing the servo
        time.sleep(1)
        self.servo.close()

        # Create Signal Parameter
        descriptor_bounds = IntegerRange(from_value=-60, to_value=90, step=1)
        angle_descriptor = ParameterDescriptor(integer_range = [descriptor_bounds])
        self.declare_parameter('angle', self.angle, angle_descriptor)
        
        # Every 0.1 seconds, check for change to 'angle' parameter
        self.create_timer(0.1, self.update_parameters)

    def update_parameters(self):
        # If signal parameter is changed, then change the pwm signal being sent to self.PIN
        change = (self.angle != self.get_parameter('angle').value)
        if change:
            # Retrieve new angle and update self.angle to match
            self.angle = self.get_parameter('angle').value 
            # Recreate Servo Object
            self.servo = AngularServo(self.PIN,
                pin_factory=self.factory,
                min_angle=-90,
                max_angle=90,                               
                min_pulse_width=0.0005,
                max_pulse_width=0.0025)
            
            # Set the Servo angle to updated angle
            self.servo.angle = self.angle
            
            time.sleep(1)
            self.servo.close()

def main(args=None):
    rclpy.init(args=args)

    servo_controller = ServoControllerNode()
    rclpy.spin(servo_controller)

    servo_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
