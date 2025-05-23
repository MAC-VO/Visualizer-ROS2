import rclpy
from packaging import version

import rerun as rr
assert version.parse(rr.__version__) >= version.parse("0.22.0")

from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, PointCloud
from geometry_msgs.msg import PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber

from .bridge import from_stamped_pose, from_image, from_pointcloud


class MACVO_visualizer_Node(Node):
    NODENAME = "macvo_visualizer"
    TIMELINE = "ros_time_ns"
    
    def __init__(self, image_topic: tuple[str, str] | tuple[str, None] | None, pose_topic: str, map_topic: str | None = None, track_topic: str | None = None):
        super().__init__(self.NODENAME)
        rr.init(self.NODENAME)
        rr.connect_tcp()
        # rr.set_time_sequence(self.TIMELINE, 0)
        rr.log("/log", rr.TextLog("Rerun session initialized"))
        rr.log("/", rr.ViewCoordinates(xyz=rr.ViewCoordinates.FRD), static=True)
        
        match image_topic:
            case None:
                pass
            case (imageL_topic, None):
                imageL_topic, _ = image_topic
                self.imageL_sub = self.create_subscription(
                    Image, imageL_topic, callback=self.receive_mono, qos_profile=1
                )
                
            case (imageL_topic, imageR_topic):
                imageL_topic, imageR_topic = image_topic

                self.imageL_sub = Subscriber(self, Image, imageL_topic, qos_profile=1)
                self.imageR_sub = Subscriber(self, Image, imageR_topic, qos_profile=1)
                self.sync_stereo = ApproximateTimeSynchronizer(
                    [self.imageL_sub, self.imageR_sub], queue_size=2, slop=0.1
                )
                self.sync_stereo.registerCallback(self.receive_frame)

        self.pose_sub   = self.create_subscription(
            PoseStamped, pose_topic,
            callback=self.receive_pose, qos_profile=1
        )
        self.prev_position = None
        
        if map_topic is not None:
            self.map_sub = self.create_subscription(
                PointCloud, map_topic,
                callback=self.receive_map_constructor("/world/map"), qos_profile=1
            )
        
        if track_topic is not None:
            self.track_sub = self.create_subscription(
                PointCloud, track_topic, callback=self.receive_map_constructor("/world/vo"), qos_profile=1
            )
    
    def receive_mono(self, msg_L: Image) -> None:
        # rr.set_time_nanos(self.TIMELINE, Time.from_msg(msg_L.header.stamp).nanoseconds)
        
        imageL = from_image(msg_L)[..., :3][..., ::-1]
        self.get_logger().info(f"Receive: Left={imageL.shape}, Right=None")

        rr.log("/world/drone/cam/imgL", rr.Image(imageL).compress(), static=False)
    
    def receive_frame(self, msg_L: Image, msg_R: Image) -> None:
        # rr.set_time_nanos(self.TIMELINE, Time.from_msg(msg_L.header.stamp).nanoseconds)
        
        imageL = from_image(msg_L)[..., :3][..., ::-1]
        imageR = from_image(msg_R)[..., :3][..., ::-1]
        self.get_logger().info(f"Receive: Left={imageL.shape}, Right={imageR.shape}")

        rr.log("/world/drone/cam/imgL", rr.Image(imageL), static=False)
        rr.log("/world/drone/cam/imgR", rr.Image(imageR), static=False)

    def receive_pose(self, pose: PoseStamped) -> None:
        pp_pose, _, time = from_stamped_pose(pose)
        if self.prev_position is None: self.prev_position = pp_pose.translation()

        # rr.set_time_nanos(self.TIMELINE, Time.from_msg(time).nanoseconds)
        rr.log("/world/drone/cam/imgL", rr.Transform3D(
            translation=pp_pose.translation().squeeze().numpy(),
            rotation=rr.datatypes.Quaternion(xyzw=pp_pose.rotation().squeeze().numpy()),
            axis_length=1.0
        ))
        rr.log("/world/trajectory", rr.LineStrips3D([self.prev_position.numpy().tolist(), pp_pose.translation().numpy().tolist()]))
        self.prev_position = pp_pose.translation()

    def receive_map_constructor(self, rr_path: str):
        def receive_map(map: PointCloud) -> None:
            position, color, _, time = from_pointcloud(map)
            
            if color.size(0) < 10:
                print(color.size)
                return
            
            # rr.set_time_nanos(self.TIMELINE, Time.from_msg(time).nanoseconds)
            rr.log(rr_path, rr.Points3D(position.numpy(), colors=color.int().numpy()))
        return receive_map


def main():
    rclpy.init()
    
    node = MACVO_visualizer_Node(
        image_topic=(
            "/zed/zed_node/left/image_rect_color",
            None
        ),
        pose_topic="/macvo/pose",
        map_topic ="/macvo/map",
        track_topic="/macvo/track"
    )
    print('MACSLAM Visualization Node created.')
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

