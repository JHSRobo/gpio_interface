# This program's purpose is to control a MOSFET transistor on the SuperHAt. 
# From the software side, this just means turning the transistor on and off by either sending power to a GPIO pin or not sending power. 

# Written by Jack Frings '26

import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Joy

import RPi.GPIO as GPIO

class MOSFETNode(Node):
    def __init__(self):
        super().__init__('mosfet_node')

        # Logger 
        self.log = self.get_logger()

        # GPIO Setup
        self.PIN = 15
        self.cur_signal = GPIO.LOW

        # Tell the RPi that we are referring to GPIOs by GPIO number
        # This means we are NOT referring to them by pin number.
        # eg. gpio4 is pin #7, and we reference it as 4.
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.PIN, GPIO.OUT)

        # Initially send the pin no power
        GPIO.output(self.PIN, self.cur_signal)
        
        # Variable to keep track of if the button on joystick has been let go before next press  
        self.cached_input = False

        # Subscriber to receive input from joystick
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

    def joy_callback(self, joy):
        # If button is pressed after not being pressed
        if joy.buttons[2] and not self.cached_input:
            # Swap the current signal being sent to the GPIO pin 
            if self.cur_signal == GPIO.LOW:
                self.cur_signal = GPIO.HIGH
            else:
                self.cur_signal = GPIO.LOW

            # Send the new GPIO signal to the pin 
            GPIO.output(self.PIN, self.cur_signal)

        self.cached_input = joy.buttons[2]

def main(args=None):
    rclpy.init(args=args)

    mosfet_node = MOSFETNode()

    # Runs the program until shutdown
    rclpy.spin(mosfet_node)

    # Stop sending signals to GPIO
    GPIO.cleanup()

    # On shutdown, kills the node
    mosfet_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
