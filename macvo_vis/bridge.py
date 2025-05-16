import std_msgs.msg as std_msgs
import sensor_msgs.msg as sensor_msgs
import geometry_msgs.msg as geometry_msgs
from builtin_interfaces.msg import Time

import torch
import pypose as pp
import numpy as np

_name_to_dtypes = {
    "rgb8":    (np.uint8,  3),
    "rgba8":   (np.uint8,  4),
    "rgb16":   (np.uint16, 3),
    "rgba16":  (np.uint16, 4),
    "bgr8":    (np.uint8,  3),
    "bgra8":   (np.uint8,  4),
    "bgr16":   (np.uint16, 3),
    "bgra16":  (np.uint16, 4),
    "mono8":   (np.uint8,  1),
    "mono16":  (np.uint16, 1),

    # for bayer image (based on cv_bridge.cpp)
    "bayer_rggb8":      (np.uint8,  1),
    "bayer_bggr8":      (np.uint8,  1),
    "bayer_gbrg8":      (np.uint8,  1),
    "bayer_grbg8":      (np.uint8,  1),
    "bayer_rggb16":     (np.uint16, 1),
    "bayer_bggr16":     (np.uint16, 1),
    "bayer_gbrg16":     (np.uint16, 1),
    "bayer_grbg16":     (np.uint16, 1),

    # OpenCV CvMat types
    "8UC1":    (np.uint8,   1),
    "8UC2":    (np.uint8,   2),
    "8UC3":    (np.uint8,   3),
    "8UC4":    (np.uint8,   4),
    "8SC1":    (np.int8,    1),
    "8SC2":    (np.int8,    2),
    "8SC3":    (np.int8,    3),
    "8SC4":    (np.int8,    4),
    "16UC1":   (np.uint16,   1),
    "16UC2":   (np.uint16,   2),
    "16UC3":   (np.uint16,   3),
    "16UC4":   (np.uint16,   4),
    "16SC1":   (np.int16,  1),
    "16SC2":   (np.int16,  2),
    "16SC3":   (np.int16,  3),
    "16SC4":   (np.int16,  4),
    "32SC1":   (np.int32,   1),
    "32SC2":   (np.int32,   2),
    "32SC3":   (np.int32,   3),
    "32SC4":   (np.int32,   4),
    "32FC1":   (np.float32, 1),
    "32FC2":   (np.float32, 2),
    "32FC3":   (np.float32, 3),
    "32FC4":   (np.float32, 4),
    "64FC1":   (np.float64, 1),
    "64FC2":   (np.float64, 2),
    "64FC3":   (np.float64, 3),
    "64FC4":   (np.float64, 4)
}


def to_stamped_pose(pose: pp.LieTensor | torch.Tensor, frame_id: str, time: Time) -> geometry_msgs.PoseStamped:
    pose_ = pose.detach().cpu()
    out_msg                 = geometry_msgs.PoseStamped()
    out_msg.header          = std_msgs.Header()
    out_msg.header.stamp    = time
    out_msg.header.frame_id = frame_id
    
    out_msg.pose.position.x = pose_[0].item()
    out_msg.pose.position.y = pose_[1].item()
    out_msg.pose.position.z = pose_[2].item()
    
    out_msg.pose.orientation.x = pose_[3].item()
    out_msg.pose.orientation.y = pose_[4].item()
    out_msg.pose.orientation.z = pose_[5].item()
    out_msg.pose.orientation.w = pose_[6].item()
    return out_msg


def from_stamped_pose(msg: geometry_msgs.PoseStamped) -> tuple[pp.LieTensor, str, Time]:
    pose = pp.SE3(torch.tensor([
        msg.pose.position.x,
        msg.pose.position.y,
        msg.pose.position.z,

        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w
    ]))
    return pose, msg.header.frame_id, msg.header.stamp


def from_image(msg: sensor_msgs.Image) -> np.ndarray:
    if msg.encoding not in _name_to_dtypes:
        raise KeyError(f"Unsupported image encoding {msg.encoding}")
    
    dtype_name, channel = _name_to_dtypes[msg.encoding]
    dtype = np.dtype(dtype_name)
    dtype = dtype.newbyteorder('>' if msg.is_bigendian else '<')
    shape = (msg.height, msg.width, channel)
    
    data = np.frombuffer(msg.data, dtype=dtype).reshape(shape)
    data.strides = (msg.step, dtype.itemsize * channel, dtype.itemsize)
    return data

def from_compressed_image(msg: sensor_msgs.CompressedImage) -> np.ndarray:
    from PIL import Image
    import io
    data = io.BytesIO(msg.data)
    img = Image.open(data)
    return np.array(img)


def to_pointcloud(position: torch.Tensor, keypoints: torch.Tensor, frame_id: str, time: Time) -> sensor_msgs.PointCloud:
    """
    position    should be a Nx3 pytorch Tensor (dtype=float)
    keypoints   should be a Nx2 pytorch Tensor (dtype=float)
    """
    assert position.size(0) == keypoints.size(0)
    
    out_msg     = sensor_msgs.PointCloud()
    position_   = position.detach().cpu().numpy()
    keypoints_  = keypoints.detach().cpu().numpy()
    
    out_msg.header = std_msgs.Header()
    out_msg.header.stamp    = time
    out_msg.header.frame_id = frame_id
    
    out_msg.points = [
        geometry_msgs.Point32(x=float(position_[pt_idx, 0]), y=float(position_[pt_idx, 1]), z=float(position_[pt_idx, 2]))
        for pt_idx in range(position.size(0))
    ]
    out_msg.channels = [
        sensor_msgs.ChannelFloat32(
            name="kp_u", values=keypoints_[..., 0].tolist()
        ),
        sensor_msgs.ChannelFloat32(
            name="kp_v", values=keypoints_[..., 1].tolist()
        ),
    ]
    
    return out_msg


def from_pointcloud(msg: sensor_msgs.PointCloud) -> tuple[torch.Tensor, torch.Tensor, str, Time]:
    """
    Returns
        position    a Nx3 pytorch Tensor (dtype=float)
        color       a Nx3 pytorch Tensor (dtype=uint8)
        frame_id
        stamp       Time stamp for the point cloud
    """
    position = torch.tensor([[pt.x, pt.y, pt.z] for pt in msg.points])
    ch = {c.name: np.asarray(c.values) for c in msg.channels}
    color    = torch.tensor(np.stack([ch["r"], ch["g"], ch["b"]], axis=1).astype(np.uint8))

    return position, color, msg.header.frame_id, msg.header.stamp
