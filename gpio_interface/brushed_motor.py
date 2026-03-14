



import rclpy
from rclpy.node import Node
from gpiozero import Motor
from rcl_interfaces.msg import SetParametersResult

class BrushedMotorNode(Node):

    def __init__(self):

        super().__init__('brushed_motor')

        self.log = self.get_logger()

        self.motor = Motor(forward=17, backward=18) # change as needed
        self.motor.stop()

        self.declare_parameter('brushed_motor_speed', 0.0)
        self.add_on_set_parameters_callback(self.motor_callback)

    def motor_callback(self, params):
        for param in params:
            if param.name == "brushed_motor_speed":
                command = max(-1.0, min(1.0, param.value))
                if command > 0:
                    self.motor.forward(command)
                elif command < 0:
                    self.motor.backward(abs(command))
                else:
                    self.motor.stop()
        return SetParametersResult(successful=True)

def main():
    rclpy.init()
    node = BrushedMotorNode()
    rclpy.spin(node)

    node.motor.stop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
