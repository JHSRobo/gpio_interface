import rclpy
from rclpy.node import Node
import RPi.GPIO
from std_msgs.msg import Bool

class GPIOController(Node):
    def __init__(self):
        super().__init__('gpio_control')

        # Quick reference for logging
        self.log = self.get_logger()

        # Tell the RPi that we are referring to GPIOs by GPIO number
        # This means we are NOT referring to them by pin number.
        # eg. gpio4 is pin #7, and we reference it as 4.
        RPi.GPIO.setmode(RPi.GPIO.BCM)
        RPi.GPIO.setwarnings(False)

        self.gpio_nums = [ 16, 17, 18, 19, 22, 25 ]

        self.gpio_status = { i: False for i in self.gpio_nums }

        # Define Parameters, prepare & shut off all pins
        for i in self.gpio_nums:
            self.declare_parameter(f"gpio{i}", False)
            RPi.GPIO.setup(i, RPi.GPIO.OUT)
            RPi.GPIO.output(i, 0)

        # Timer, to always check for parameter updates
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        for i in self.gpio_nums:
            parameter_val = self.get_parameter(f"gpio{i}").value
            if self.gpio_status[i] is not parameter_val:
                self.gpio_status[i] = parameter_val
                if parameter_val: 
                    RPi.GPIO.output(i, 1)
                    self.log.info(f"Turned pin {i} on")
                else: 
                    RPi.GPIO.output(i,0)
                    self.log.info(f"Turned pin {i} off")

def main():
    rclpy.init()

    gpio = GPIOController()

    # Runs the program until shutdown is recieved
    rclpy.spin(gpio)

    # On shutdown, kill node and cleanup pins
    RPi.GPIO.cleanup()
    gpio.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
