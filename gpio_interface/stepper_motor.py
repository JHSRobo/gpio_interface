# This program's purpose is to control a stepper motor from joystick input.
# This code is written to avoid using time.sleep() because of ROS's incompatibility with time.sleep(). Instead, we separate different signals sent to the stepper motor using the built-in create_timer() method.

# Written by Jack Frings '26

import rclpy
from rclpy.node import Node 
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange

import RPi.GPIO as GPIO

class StepperMotorNode(Node):

    def __init__(self):
        super().__init__('stepper_motor')

        self.log  = self.get_logger()

        # Constants
        self.DIR = 20   # Direction GPIO Pin
        self.STEP = 21  # Step GPIO Pin
        self.CW = 1     # Clockwise Rotation
        self.CCW = 0    # Counterclockwise Rotation
        self.SPR = 200   # Steps per Revolution (360 / 1.8)    
      
        # Variables
        self.cur_direction = None
        self.speed = 4
        self.delay = 0.0026 / self.speed 
        self.cur_signal = GPIO.HIGH

        # GPIO Setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(DIR, GPIO.OUT)
        GPIO.setup(STEP, GPIO.OUT)

        # Subscriber to receive input from joystick
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Timer to communicate with the stepper motor
        self.timer = self.create_timer(delay, self.spin_motor)

        # Speed Parameter 
        speed_bounds = FloatingPointRange()
        speed_bounds.from_value = 1
        speed_bounds.to_value = 10
        speed_bounds.step = 1
        speed_descriptor = ParameterDescriptor(floating_point_range = [speed_bounds])

        self.declare_parameter('speed', self.speed, speed_descriptor)
        self.add_on_set_parameters_callback(self.update_parameters)

        def update_parameters(self):
            change = (self.speed != self.get_parameter('speed').value) 
            if change:
                self.speed = self.get_parameter('speed').value 
                self.delay = 0.0026 / self.speed 
                self.destroy_timer(self.timer)
                self.create_timer(self.delay, self.spin_motor)

            return SetParametersResult(successful=True)

        def joy_callback(self, joy):
            if joy.axes[4]:
                self.cur_direction = self.CW
            elif joy.axes[5]:
                self.cur_direction = self.CCW
            else:
                self.cur_direction = None

            if self.cur_direction != None:
                GPIO.output(self.DIR, self.cur_direction)

        def spin_motor(self):
            if self.cur_direction != None:

                GPIO.output(STEP, self.cur_signal)

                if self.cur_signal == GPIO.HIGH:
                    self.cur_signal = GPIO.LOW
                else:
                    self.cur_signal = GPIO.HIGH

def main(args=None):
    rclpy.init(args=args)

    stepper_motor = StepperMotorNode()

    rclpy.spin(stepper_motor)

    stepper_motor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
