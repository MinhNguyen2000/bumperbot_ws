import rclpy
from rclpy.node import Node                                         # To create a new ROS2 node
from rcl_interfaces.msg import SetParametersResult                  # To create a standard ROS2 response when a parameter is changed
from rclpy.parameter import Parameter                               # To access the type of a ROS parameter

class SimpleParameter(Node):
    def __init__(self):                                             # Constructor of the node
        super().__init__("simple_parameter")                        # Node name

        self.declare_parameter("simple_int_param", 23)              # Function inherited from the Node class to define the name of the variable
        self.declare_parameter("simple_string_param", "Minh")

        self.add_on_set_parameters_callback(self.paramChangeCallback) # Whenever a parameter is change during node runtime, the callback function is executed

    def paramChangeCallback(self, params):
        result = SetParametersResult()

        for param in params:                                        # Loop through all of the parameters that have changed
            if param.name == "simple_int_param" and param.type_ == Parameter.Type.INTEGER:
                self.get_logger().info("Param simple_int_param changed! New value is %d" % param.value)
                result.successful = True

            if param.name == "simple_string_param" and param.type_ == Parameter.Type.STRING:
                self.get_logger().info("Param simple_string_param changed! New value is %s" % param.value)
                result.successful = True

        return result
    
def main():
    rclpy.init()
    simple_parameter = SimpleParameter()
    rclpy.spin(simple_parameter)

    simple_parameter.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()