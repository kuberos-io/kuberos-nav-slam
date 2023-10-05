import rclpy
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf2_ros.transform_listener import TransformListener

from tf2_ros.buffer import Buffer
import tf2_ros
from tf2_ros import TransformException

from transforms3d.taitbryan import euler2quat


class WorldToMapTF(Node):

    def __init__(self):
        super().__init__('WorldToMapTF')

        self.tmp_buffer = Buffer()
        
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_timer(1./10., self.timer_callback)
        self.buffer = Buffer()
        self.listener = TransformListener(buffer=self.buffer, node=self)#, spin_thread=False)

    def timer_callback(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'gt_openni_camera'
        t.child_frame_id = 'openni_camera'
        t.transform.translation.x = 0.
        t.transform.translation.y = 0.
        t.transform.translation.z = 0.

        q = euler2quat(0., 0., 0.)
        t.transform.rotation.w = q[0]
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        world_to_cam = None
        cam_to_map = None
        
        try:
            world_to_cam = self.buffer.lookup_transform(target_frame="gt_openni_camera", source_frame="world", time=rclpy.time.Time())
            world_to_cam.header.frame_id = "openni_camera"
            cam_to_map = self.buffer.lookup_transform(target_frame="openni_camera", source_frame="map", time=rclpy.time.Time() )
          
            self.tmp_buffer.set_transform(transform=world_to_cam, authority="default_authority")
            self.tmp_buffer.set_transform(transform=cam_to_map, authority="default_authority")
            
            if self.tmp_buffer.can_transform(target_frame="map", source_frame="world", time=rclpy.time.Time()):
                stamped_transform = self.tmp_buffer.lookup_transform(target_frame="world", source_frame="map", time=rclpy.time.Time())
                self.tf_broadcaster.sendTransform(stamped_transform)

        except (tf2_ros.LookupException, TransformException):
            pass

def main(args=None):
    rclpy.init(args=args)

    try:
        node = WorldToMapTF()
    except ValueError as e:
        print(f"Error while initializing: {e}")
        return 1
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
