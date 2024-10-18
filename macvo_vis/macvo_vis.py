import rclpy
import rerun as rr

from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, PointCloud
from geometry_msgs.msg import PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber

from pathlib import Path

from .MessageFactory import to_stamped_pose, from_image, to_pointcloud


class MACVO_visualizer_Node(Node):
    NODENAME = "macvo_visualizer"
    TIMELINE = "ros_time_ns"
    
    def __init__(self, imageL_topic: str, imageR_topic: str):
        super().__init__(self.NODENAME)
        rr.init(self.NODENAME)
        rr.set_time_sequence(self.TIMELINE, 0)
        rr.log("/log", rr.TextLog("Rerun session initialized"))
        
        self.imageL_sub = Subscriber(self, Image, imageL_topic, qos_profile=10)
        self.imageR_sub = Subscriber(self, Image, imageR_topic, qos_profile=10)
        
        self.sync_stereo = ApproximateTimeSynchronizer(
            [self.imageL_sub, self.imageR_sub], queue_size=2, slop=0.1
        )
        self.sync_stereo.registerCallback(self.receive_frame)
        
    
    def receive_frame(self, msg_L: Image, msg_R: Image) -> None:
        rr.set_time_nanos(self.TIMELINE, Time.from_msg(msg_L.header.stamp).nanoseconds)
        
        imageL = from_image(msg_L)
        imageR = from_image(msg_R)
        self.get_logger().info(f"Receive: Left={imageL.shape}, Right={imageR.shape}")

        rr.log("/world/drone/cam/imgL", rr.Image(imageL).compress())
        rr.log("/world/drone/cam/imgR", rr.Image(imageR).compress())


def main():
    rclpy.init()
    
    node = MACVO_visualizer_Node(
        imageL_topic="/zed/zed_node/rgb/image_rect_color",
        imageR_topic="/zed/zed_node/right/image_rect_color",
    )
    print('MACVO Node created.')
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
