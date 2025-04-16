import rclpy
from packaging import version
import numpy as np

import rerun as rr
assert version.parse(rr.__version__) >= version.parse("0.22.0")

from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, PointCloud, CompressedImage
from geometry_msgs.msg import PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber

from .MessageFactory import from_stamped_pose, from_image, from_pointcloud, from_compressed_image


class MACVO_visualizer_Node(Node):
    NODENAME = "macvo_visualizer"
    TIMELINE = "ros_time_ns"
    
    def __init__(self, imageL_topic: str | None, imageR_topic: str | None, pose_topic: str, map_topic: str | None = None):
        super().__init__(self.NODENAME)
        rr.init(self.NODENAME, spawn=True)
        rr.set_time_sequence(self.TIMELINE, 0)
        rr.log("/log", rr.TextLog("Rerun session initialized"))
        rr.log("/", rr.ViewCoordinates(xyz=rr.ViewCoordinates.FRD), static=True)
        
        self.sync_stereo = None

        if imageL_topic is not None and imageR_topic is not None:
            self.imageL_sub = Subscriber(self, CompressedImage, imageL_topic + "/compressed", qos_profile=1)
            self.imageR_sub = Subscriber(self, CompressedImage, imageR_topic + "/compressed", qos_profile=1)
            self.sync_stereo = ApproximateTimeSynchronizer(
                [self.imageL_sub, self.imageR_sub], queue_size=2, slop=0.1
            )
            self.sync_stereo.registerCallback(self.receive_stereo_frame)
        elif imageL_topic is not None:
            self.imageL_sub = self.create_subscription(
                CompressedImage, imageL_topic + "/compressed",
                callback=self.receive_compact_frame, qos_profile=1
            )

        self.pose_sub   = self.create_subscription(
            PoseStamped, pose_topic,
            callback=self.receive_pose, qos_profile=1
        )
        self.prev_position = None
        
        if map_topic is not None:
            self.map_sub = self.create_subscription(
                PointCloud, map_topic,
                callback=self.receive_map, qos_profile=1
            )
    
    def receive_compact_frame(self, msg: Image) -> None:
        rr.set_time_nanos(self.TIMELINE, Time.from_msg(msg.header.stamp).nanoseconds)
        image = from_compressed_image(msg)[..., :3][:, :, ::-1].astype(np.uint8)
        self.get_logger().info(f"Receive: {image.shape}")
        rr.log("/world/drone/cam/imgL", rr.Image(image), static=True)
    
    def receive_stereo_frame(self, msg_L: Image, msg_R: Image) -> None:
        rr.set_time_nanos(self.TIMELINE, Time.from_msg(msg_L.header.stamp).nanoseconds)
        
        imageL = from_image(msg_L)[..., :3][:, :, ::-1].astype(np.uint8)
        imageR = from_image(msg_R)[..., :3][:, :, ::-1].astype(np.uint8)
        self.get_logger().info(f"Receive: Left={imageL.shape}, Right={imageR.shape}")

        rr.log("/world/drone/cam/imgL", rr.Image(imageL), static=True)
        rr.log("/world/drone/cam/imgR", rr.Image(imageR), static=True)

    def receive_pose(self, pose: PoseStamped) -> None:
        pp_pose, _, time = from_stamped_pose(pose)
        if self.prev_position is None: self.prev_position = pp_pose.translation()

        rr.set_time_nanos(self.TIMELINE, Time.from_msg(time).nanoseconds)
        rr.log("/world/drone/cam/imgL", rr.Transform3D(
            translation=pp_pose.translation().squeeze().numpy(),
            rotation=rr.datatypes.Quaternion(xyzw=pp_pose.rotation().squeeze().numpy()),
            axis_length=1.0
        ))
        rr.log("/world/trajectory", rr.LineStrips3D([self.prev_position.numpy().tolist(), pp_pose.translation().numpy().tolist()]))
        self.prev_position = pp_pose.translation()

    def receive_map(self, map: PointCloud) -> None:
        position, color, _, time = from_pointcloud(map)
        rr.set_time_nanos(self.TIMELINE, Time.from_msg(time).nanoseconds)
        rr.log("/world/map", rr.Points3D(position.numpy(), colors=color.int().numpy()))


def main():
    rclpy.init()
    
    node = MACVO_visualizer_Node(
        imageL_topic="/zed/zed_node/left/image_rect_color",
        imageR_topic=None,
        # imageR_topic="/zed/zed_node/right/image_rect_color",
        pose_topic="/macvo/pose",
        map_topic ="/macvo/map"
    )
    print('MACVO Visualization Node created.')
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

