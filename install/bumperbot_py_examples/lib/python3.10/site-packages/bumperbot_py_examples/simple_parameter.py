import rclpy
from rclpy.node import Node                                         # To create a new ROS2 node
from rcl_interfaces.msg import SetParametersResult                  # To create a standard ROS2 response when a parameter is changed
from rclpy.parameter import Parameter                               # To access the type of a ROS parameter

class SimpleParameter(Node):
    def __init__(self):                                             # Constructor of the node
        super().__init__("simple_parameter")                        # Node name

        # Declare parameters and default value w/ a function inherited from the Node class
        self.declare_parameter("simple_int_param", 23)              
        self.declare_parameter("simple_string_param", "Minh")

        # Define a callback function when parameters are changed at runtime
        self.add_on_set_parameters_callback(self.paramChangeCallback) 

    def paramChangeCallback(self, params):
        result = SetParametersResult()

        # Loop through all of the parameters that have changed
        for param in params:                                        
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