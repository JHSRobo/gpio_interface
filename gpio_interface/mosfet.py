# This program's purpose is to control a MOSFET transistor on the SuperHAt. 
# From the software side, this just means turning the transistor on and off by either sending power to a GPIO pin or not sending power. 

# Written by Jack Frings '26

import rclpy
from rclpy.node import Node 
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy

from gpiozero import OutputDevice
from gpiozero.pins.lgpio import LGPIOFactory


# Top Left: 11
# Top Right: 16
# Bottom Left: 17
# Bottom Right: 18

class MOSFETNode(Node):
    def __init__(self):
        super().__init__('mosfet_node')

        # Logger 
        self.log = self.get_logger()
        self.pin_factory = LGPIOFactory()

        # GPIO Pin to turn on and off
        self.PIN = 16

        # GPIO Pin Parameter
        bounds = IntegerRange()
        bounds.from_value = 1
        bounds.to_value = 4 
        bounds.step = 1
        descriptor = ParameterDescriptor(integer_range = [bounds])
        self.declare_parameter("PIN", 2, descriptor)

        self.device = OutputDevice(self.PIN)

        # Vartiable deciding whether the current GPIO Pin should be on or off
        self.cur_signal_on = False

        # Initially send the pin no power
        self.device.off()
        
        # Variable to keep track of if the button on joystick has been let go before next press  
        self.cached_input = False

        # If the thrusters are disabled, don't run the pump
        self.thrusters_enabled = False

        # Subscriber to receive input from joystick
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.thrusters_enabled_sub = self.create_subscription(Bool, 'thruster_status', self.thrusters_enabled_callback, 10)

        self.create_timer(0.1, self.update_parameters)

    def update_parameters(self):
        # If the current pin is not the same as the current parameter
        if self.PIN != self.corresponding_pin(self.get_parameter("PIN").value):
            self.device.off()
            self.PIN = self.corresponding_pin(self.get_parameter("PIN").value)
            self.device = OutputDevice(self.PIN)
            self.log.info("New MOSFET GPIO Pin: " + str(self.PIN))

    def corresponding_pin(self, num):
        pins = [11, 16, 17, 18]
        return pins[num-1]

    def thrusters_enabled_callback(self, msg):
        self.thrusters_enabled = msg.data

    def joy_callback(self, joy):
        # If button is pressed after not being pressed
        if joy.buttons[2] and not self.cached_input:
            # Swap the current signal being sent to the GPIO pin 
            self.cur_signal_on = not self.cur_signal_on

            # Send the new GPIO signal to the pin 
            if self.thrusters_enabled:
                if self.cur_signal_on:
                    self.device.on()
                else:
                    self.device.off()
            else:
                self.device.off()

        self.cached_input = joy.buttons[2]

def main(args=None):
    rclpy.init(args=args)

    mosfet_node = MOSFETNode()

    # Runs the program until shutdown
    rclpy.spin(mosfet_node)

    # On shutdown, kills the node
    mosfet_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
