import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

from transforms3d.taitbryan import euler2quat


class PoseToStaticTF(Node):

    def __init__(self):
        super().__init__('pose_to_static_tf')

        self.parent_frame_id = None
        self.child_frame_id = None
        self.pose_as_str = None

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.params()

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.make_transform()

    def params(self):
        """handle ROS parameters and store them as class variables
        :returns: -

        """
        self.declare_parameter(
            'x',
            0.0,
            descriptor=ParameterDescriptor(dynamic_typing=True))

        self.declare_parameter(
            'y',
            0.0,
            descriptor=ParameterDescriptor(dynamic_typing=True))

        self.declare_parameter(
            'yaw',
            0.0,
            descriptor=ParameterDescriptor(dynamic_typing=True))

        self.declare_parameter(
            'parent_frame_id',
            'world',
            descriptor=ParameterDescriptor(dynamic_typing=True))

        self.declare_parameter(
            'child_frame_id',
            'map',
            descriptor=ParameterDescriptor(dynamic_typing=True))

        self.declare_parameter(
            'pose_as_str',
            '',
            descriptor=ParameterDescriptor(dynamic_typing=True))

        self.x = self.get_parameter_or('x').value
        self.y = self.get_parameter_or('y').value
        self.yaw = self.get_parameter_or('yaw').value

        self.parent_frame_id = self.get_parameter_or(
            'parent_frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter_or(
            'child_frame_id').get_parameter_value().string_value
        self.pose_as_str = self.get_parameter_or(
            'pose_as_str').get_parameter_value().string_value

        self._logger.error(f'pose as string {self.pose_as_str}')

        if self.pose_as_str:
            pose_vals = self.pose_as_str.split(',')
            if len(pose_vals) == 3:
                self.x = float(pose_vals[0])
                self.y = float(pose_vals[1])
                self.yaw = float(pose_vals[2])

    def make_transform(self):
        """make static transform from pose
        :returns: -

        """
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame_id
        t.child_frame_id = self.child_frame_id

        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = float(0.0)

        q = euler2quat(self.yaw, 0., 0.)
        t.transform.rotation.w = q[0]
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]

        self.tf_static_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    try:
        pose_to_static_tf_node = PoseToStaticTF()
    except ValueError as e:
        print(f"Error while initializing: {e}")
        return 1

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(pose_to_static_tf_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        pose_to_static_tf_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
