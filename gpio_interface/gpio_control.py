import rclpy
from rclpy.node import Node
from gpiozero import OutputDevice
from std_msgs.msg import Bool
from gpiozero.pins.lgpio import LGPIOFactory

class GPIOController(Node):
    def __init__(self):
        super().__init__('gpio_control')

        # Quick reference for logging
        self.log = self.get_logger()
        self.pin_factory = LGPIOFactory()

        self.gpio_nums = [ 16, 17, 18, 19, 22, 25 ]

        self.gpios = {i: OutputDevice(i) for i in self.gpio_nums}
        self.gpio_status = { i: False for i in self.gpio_nums }

        # Define Parameters, prepare & shut off all pins
        for i in self.gpio_nums:
            self.declare_parameter(f"gpio{i}", False)
            self.gpios[i].off()

        # Timer, to always check for parameter updates
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        for i in self.gpio_nums:
            parameter_val = self.get_parameter(f"gpio{i}").value
            if self.gpio_status[i] is not parameter_val:
                self.gpio_status[i] = parameter_val
                if parameter_val: 
                    self.gpios[i].on()
                    self.log.info(f"Turned pin {i} on")
                else: 
                    self.gpios[i].off()
                    self.log.info(f"Turned pin {i} off")

def main():
    rclpy.init()

    gpio = GPIOController()

    # Runs the program until shutdown is recieved
    rclpy.spin(gpio)

    # On shutdown, kill node and cleanup pins
    gpio.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
