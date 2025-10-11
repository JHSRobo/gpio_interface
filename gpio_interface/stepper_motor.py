# This program's purpose is to control a stepper motor from joystick input.
# This code is written to avoid using time.sleep() because of ROS's incompatibility with time.sleep(). Instead, we separate different signals sent to the stepper motor using the built-in create_timer() method.

# Written by Jack Frings '26

import rclpy
from rclpy.node import Node 
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange
from sensor_msgs.msg import Joy

from gpiozero import OutputDevice
from gpiozero.pins.lgpio import LGPIOFactory


class StepperMotorNode(Node):

    def __init__(self):
        super().__init__('stepper_motor')

        self.log  = self.get_logger()
        self.pin_factory = LGPIOFactory()

        # Constants
        self.DIR = 20   # Direction GPIO Pin
        self.STEP = 21  # Step GPIO Pin
        self.CW = 1     # Clockwise Rotation
        self.CCW = 0    # Counterclockwise Rotation
      
        # Variables
        self.cur_direction = None
        self.speed = 8
        self.delay = 0.0026 / self.speed 
        self.cur_signal_on = True

        # GPIO Setup
        self.dir_pin = OutputDevice(self.DIR)
        self.step_pin = OutputDevice(self.STEP)

        # Subscriber to receive input from joystick
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Timer to communicate with the stepper motor
        self.timer = self.create_timer(self.delay, self.spin_motor)

        # Speed Parameter 
        speed_bounds = FloatingPointRange()
        speed_bounds.from_value = 1.0
        speed_bounds.to_value = 10.0
        speed_bounds.step = 1.0
        speed_descriptor = ParameterDescriptor(floating_point_range = [speed_bounds])

        self.declare_parameter('speed', self.speed, speed_descriptor)
        self.create_timer(0.1, self.update_parameters)

    def update_parameters(self):
        change = (self.speed != self.get_parameter('speed').value) 
        if change:
            # If speed has a new value, delete the current stepper motor timer and make a new timer with the new delay resulting from the new speed.
            self.speed = self.get_parameter('speed').value 
            self.delay = 0.0026 / self.speed 
            self.log.info(str(self.delay))
            self.destroy_timer(self.timer)
            self.timer = self.create_timer(self.delay, self.spin_motor)

    def joy_callback(self, joy):
        # Check if the pilot is trying to spin the stepper motor clockwise, counterclockwise, or not at all.
        if joy.axes[6] == 1:
            self.cur_direction = self.CW
        elif joy.axes[6] == -1:
            self.cur_direction = self.CCW
        else:
            self.cur_direction = None

        # Change the direction pin according to the new direction value
        if self.cur_direction != None:
            if self.cur_direction == self.CW:
                self.dir_pin.on()
            else:
                self.dir_pin.off()

    def spin_motor(self):
        # If the pilot is trying to rotate the stepper motor
        if self.cur_direction != None:
            
            # Output GPIO.HIGH and GPIO.LOW at alternating intervals to the STEP pin.
            if self.cur_signal_on:
                self.step_pin.on()
            else:
                self.step_pin.off()
            self.cur_signal_on = not self.cur_signal_on

def main(args=None):
    rclpy.init(args=args)

    stepper_motor = StepperMotorNode()

    rclpy.spin(stepper_motor)

    stepper_motor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
