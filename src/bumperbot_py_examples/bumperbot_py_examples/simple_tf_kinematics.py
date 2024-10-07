import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from bumperbot_msgs.srv import GetTransform

class SimpleTFKinematics(Node):
    def __init__(self):
        super().__init__("simple_tf_kinematics")

        self.static_tf_broadcaster_ = StaticTransformBroadcaster(self)
        self.dynamic_tf_broadcaster_ = TransformBroadcaster(self)

        # Documentation of TransformStamped() message "https://docs.ros2.org/galactic/api/geometry_msgs/msg/TransformStamped.html"
        self.static_transform_stamped_ = TransformStamped()
        self.dynamic_transform_stamped_ = TransformStamped()

        self.x_increment_ = 0.05
        self.last_x_ = 0.0

        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_,self)

        # ============ STATIC TRANSFORMATION ============
        self.static_transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.static_transform_stamped_.header.frame_id = "bumperbot_base"
        self.static_transform_stamped_.child_frame_id = "bumperbot_top"

        # Translation of 30 cm upwards
        self.static_transform_stamped_.transform.translation.x = 0.0
        self.static_transform_stamped_.transform.translation.y = 0.0
        self.static_transform_stamped_.transform.translation.z = 0.3

        # No rotation (quaternion: (0, 0, 0, 1))
        self.static_transform_stamped_.transform.rotation.x = 0.0
        self.static_transform_stamped_.transform.rotation.y = 0.0
        self.static_transform_stamped_.transform.rotation.z = 0.0
        self.static_transform_stamped_.transform.rotation.w = 1.0

        # Broadcast the transform
        self.static_tf_broadcaster_.sendTransform(self.static_transform_stamped_)

        # Log the publication of the transform
        self.get_logger().info("Publishing static trasnform between %s and %s" % 
                               (self.static_transform_stamped_.header.frame_id, self.static_transform_stamped_.child_frame_id))
        
        # Timer for dynamic transformation publication frequency
        self.timer_ = self.create_timer(0.1, self.timerCallback)

        # Server to provide the transformation matrix between two frames
        self.get_transform_srv_ = self.create_service(GetTransform, "get_transform",self.getTransformCallback)

    def timerCallback(self):
        self.dynamic_transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.dynamic_transform_stamped_.header.frame_id = "odom"
        self.dynamic_transform_stamped_.child_frame_id = "bumperbot_base"
        
        # ============ DYNAMIC TRANSFORMATION ============
        # Translation of x_increment_ in the x-axis whenever the callback function is called
        self.dynamic_transform_stamped_.transform.translation.x = self.last_x_ + self.x_increment_
        self.dynamic_transform_stamped_.transform.translation.y = 0.0
        self.dynamic_transform_stamped_.transform.translation.z = 0.0

        # No rotation (quaternion: (0, 0, 0, 1))
        self.dynamic_transform_stamped_.transform.rotation.x = 0.0
        self.dynamic_transform_stamped_.transform.rotation.y = 0.0
        self.dynamic_transform_stamped_.transform.rotation.z = 0.0
        self.dynamic_transform_stamped_.transform.rotation.w = 1.0

        self.dynamic_tf_broadcaster_.sendTransform(self.dynamic_transform_stamped_)

        self.last_x_ = self.dynamic_transform_stamped_.transform.translation.x

    def getTransformCallback(self,req,res):
        self.get_logger().info("Requested trasnform between %s and %s " % (req.frame_id, req.child_frame_id))
        requested_transform = TransformStamped()
        
        try: 
            requested_transform = self.tf_buffer_.lookup_transform(req.frame_id, req.child_frame_id, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().info("An error occured while transforming %s and %s" % (req.frame_id, req.child_frame_id))
            res.success = False
            return res
        
        res.transform = requested_transform
        res.success = True
        return res
    
def main():
    rclpy.init()
    simple_tf_kinematics = SimpleTFKinematics()
    rclpy.spin(simple_tf_kinematics)
    simple_tf_kinematics.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()