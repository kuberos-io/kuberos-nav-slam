import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import PoseStamped


class TfToPose(Node):

    def __init__(self):
        super().__init__('tf_to_pose')

        self.parent_frame_id = None
        self.child_frame_id = None

        self.params()

        timer_cb_group = MutuallyExclusiveCallbackGroup()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        timer_period = 0.1
        self._timer = self.create_timer(
            timer_period,
            self.timer_callback, callback_group=timer_cb_group)

        self.pose_pub = self.create_publisher(PoseStamped, 'tf_as_pose', 10)

    def params(self):
        """handle ROS parameters and store them as class variables
        :returns: -

        """
        self.declare_parameter(
            'parent_frame_id',
            'map',
            descriptor=ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter(
            'child_frame_id',
            'base_link',
            descriptor=ParameterDescriptor(dynamic_typing=True))

        self.parent_frame_id = self.get_parameter_or(
            'parent_frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter_or(
            'child_frame_id').get_parameter_value().string_value

    def timer_callback(self):
        """timer callback funtion
        :returns: -

        """
        try:
            t = self.tf_buffer.lookup_transform(
                self.parent_frame_id,
                self.child_frame_id,
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform {self.parent_frame_id} to {self.child_frame_id}: {ex}')
            return

        pose = PoseStamped()
        pose.header = t.header
        pose.header.frame_id = self.parent_frame_id

        pose.pose.position.x = t.transform.translation.x
        pose.pose.position.y = t.transform.translation.y
        pose.pose.orientation.x = t.transform.rotation.x
        pose.pose.orientation.y = t.transform.rotation.y
        pose.pose.orientation.z = t.transform.rotation.z
        pose.pose.orientation.w = t.transform.rotation.w

        self.pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    try:
        tf_to_pose_node = TfToPose()
    except ValueError as e:
        print(f"Error while initializing: {e}")
        return 1

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(tf_to_pose_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        tf_to_pose_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
