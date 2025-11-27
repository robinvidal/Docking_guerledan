"""
Utilitaires pour transformations TF2 dans ROS2.
"""

import numpy as np
from geometry_msgs.msg import TransformStamped, Pose, Point, Quaternion
from typing import Tuple
import math


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """
    Convertit angles d'Euler en quaternion.
    
    Args:
        roll: Roulis (rad)
        pitch: Tangage (rad)
        yaw: Lacet (rad)
    
    Returns:
        Quaternion ROS
    
    Examples:
        >>> q = euler_to_quaternion(0.0, 0.0, np.pi/2)
        >>> assert abs(q.z - 0.707) < 0.01
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy

    return q


def quaternion_to_euler(q: Quaternion) -> Tuple[float, float, float]:
    """
    Convertit quaternion en angles d'Euler.
    
    Args:
        q: Quaternion ROS
    
    Returns:
        (roll, pitch, yaw) en radians
    
    Examples:
        >>> q = Quaternion(x=0.0, y=0.0, z=0.707, w=0.707)
        >>> r, p, y = quaternion_to_euler(q)
        >>> assert abs(y - np.pi/2) < 0.01
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Gimbal lock
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def create_transform(x: float, y: float, z: float,
                     roll: float, pitch: float, yaw: float,
                     parent_frame: str, child_frame: str,
                     timestamp) -> TransformStamped:
    """
    Crée un message TransformStamped.
    
    Args:
        x, y, z: Translation (m)
        roll, pitch, yaw: Rotation (rad)
        parent_frame: Nom du repère parent
        child_frame: Nom du repère enfant
        timestamp: Timestamp ROS
    
    Returns:
        TransformStamped prêt à broadcaster
    
    Examples:
        >>> from rclpy.time import Time
        >>> t = create_transform(1.0, 2.0, 0.0, 0.0, 0.0, np.pi/4,
        ...                      'base_link', 'cage', Time().to_msg())
        >>> assert t.transform.translation.x == 1.0
    """
    t = TransformStamped()
    t.header.stamp = timestamp
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame
    
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    
    q = euler_to_quaternion(roll, pitch, yaw)
    t.transform.rotation = q
    
    return t


def pose_from_xyyaw(x: float, y: float, yaw: float, z: float = 0.0) -> Pose:
    """
    Crée un message Pose à partir de coordonnées 2D + yaw.
    
    Args:
        x, y: Position 2D (m)
        yaw: Orientation (rad)
        z: Altitude (m, défaut 0)
    
    Returns:
        Pose ROS
    
    Examples:
        >>> pose = pose_from_xyyaw(5.0, 10.0, np.pi/2)
        >>> assert pose.position.x == 5.0
    """
    pose = Pose()
    pose.position = Point(x=x, y=y, z=z)
    pose.orientation = euler_to_quaternion(0.0, 0.0, yaw)
    return pose


def transform_point(point: Tuple[float, float, float],
                   transform: TransformStamped) -> Tuple[float, float, float]:
    """
    Applique une transformation à un point 3D.
    
    Args:
        point: (x, y, z) dans repère source
        transform: Transformation à appliquer
    
    Returns:
        (x', y', z') dans repère cible
    
    Examples:
        >>> from rclpy.time import Time
        >>> tf = create_transform(1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        ...                       'A', 'B', Time().to_msg())
        >>> p_transformed = transform_point((2.0, 3.0, 4.0), tf)
        >>> assert p_transformed[0] == 3.0  # Translation x
    """
    # Extraction translation
    tx = transform.transform.translation.x
    ty = transform.transform.translation.y
    tz = transform.transform.translation.z
    
    # Extraction rotation (quaternion -> matrice simplifiée pour exemple)
    q = transform.transform.rotation
    roll, pitch, yaw = quaternion_to_euler(q)
    
    # Application rotation (seulement yaw pour simplifier, full 3D nécessite matrice)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    
    x_rot = point[0] * cos_yaw - point[1] * sin_yaw
    y_rot = point[0] * sin_yaw + point[1] * cos_yaw
    z_rot = point[2]  # Pas de rotation en z pour yaw seul
    
    # Application translation
    x_final = x_rot + tx
    y_final = y_rot + ty
    z_final = z_rot + tz
    
    return x_final, y_final, z_final
