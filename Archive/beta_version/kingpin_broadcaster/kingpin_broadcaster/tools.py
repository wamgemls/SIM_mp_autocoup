import numpy as np
from typing import Iterable, Optional, Tuple
from squaternion import Quaternion
from geometry_msgs.msg import (PointStamped, Pose, PoseStamped,
                               PoseWithCovarianceStamped, TransformStamped,
                               Vector3Stamped)

def _build_affine(
        rotation: Optional[Iterable] = None,
        translation: Optional[Iterable] = None) -> np.ndarray:
    """
    Build an affine matrix from a quaternion and a translation.
    :param rotation: The quaternion as [w, x, y, z]
    :param translation: The translation as [x, y, z]
    :returns: The quaternion and the translation array
    """
    affine = np.eye(4)
    if rotation is not None:
        affine[:3, :3] = _get_mat_from_quat(np.asarray(rotation))
    if translation is not None:
        affine[:3, 3] = np.asarray(translation)
    return affine


def _transform_to_affine(transform: TransformStamped) -> np.ndarray:
    """
    Convert a `TransformStamped` to a affine matrix.
    :param transform: The transform that should be converted
    :returns: The affine transform
    """
    transform = transform.transform
    transform_rotation_matrix = [
        transform.rotation.w,
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z
    ]
    transform_translation = [
        transform.translation.x,
        transform.translation.y,
        transform.translation.z
    ]
    return _build_affine(transform_rotation_matrix, transform_translation)


def _get_mat_from_quat(quaternion: np.ndarray) -> np.ndarray:
    """
    Convert a quaternion to a rotation matrix.
    This method is based on quat2mat from https://github.com
    f185e866ecccb66c545559bc9f2e19cb5025e0ab/transforms3d/quaternions.py#L101 ,
    since that library is not available via rosdep.
    :param quaternion: A numpy array containing the w, x, y, and z components of the quaternion
    :returns: The rotation matrix
    """
    Nq = np.sum(np.square(quaternion))
    if Nq < np.finfo(np.float64).eps:
        return np.eye(3)

    XYZ = quaternion[1:] * 2.0 / Nq
    wXYZ = XYZ * quaternion[0]
    xXYZ = XYZ * quaternion[1]
    yYZ = XYZ[1:] * quaternion[2]
    zZ = XYZ[2] * quaternion[3]

    return np.array(
        [[1.0-(yYZ[0]+zZ), xXYZ[1]-wXYZ[2], xXYZ[2]+wXYZ[1]],
         [xXYZ[1]+wXYZ[2], 1.0-(xXYZ[0]+zZ), yYZ[1]-wXYZ[0]],
         [xXYZ[2]-wXYZ[1], yYZ[1]+wXYZ[0], 1.0-(xXYZ[0]+yYZ[0])]])


def _get_quat_from_mat(rot_mat: np.ndarray) -> np.ndarray:
    """
    Convert a rotation matrix to a quaternion.
    This method is a copy of mat2quat from https://github.com
    f185e866ecccb66c545559bc9f2e19cb5025e0ab/transforms3d/quaternions.py#L150 ,
    since that library is not available via rosdep.
    Method from
    Bar-Itzhack, Itzhack Y. (2000), "New method for extracting the
    quaternion from a rotation matrix", AIAA Journal of Guidance,
    Control and Dynamics 23(6):1085-1087 (Engineering Note), ISSN
    0731-5090
    :param rot_mat: A roatation matrix
    :returns: An quaternion
    """
    # Decompose rotation matrix
    Qxx, Qyx, Qzx, Qxy, Qyy, Qzy, Qxz, Qyz, Qzz = rot_mat.flat
    # Create matrix
    K = np.array([
        [Qxx - Qyy - Qzz, 0,               0,               0],
        [Qyx + Qxy,       Qyy - Qxx - Qzz, 0,               0],
        [Qzx + Qxz,       Qzy + Qyz,       Qzz - Qxx - Qyy, 0],
        [Qyz - Qzy,       Qzx - Qxz,       Qxy - Qyx,       Qxx + Qyy + Qzz]]
    ) / 3.0
    vals, vecs = np.linalg.eigh(K)
    # Select largest eigenvector and reorder to w,x,y,z
    q = vecs[[3, 0, 1, 2], np.argmax(vals)]
    # Invert quaternion if w is negative (results in positive w)
    if q[0] < 0:
        q *= -1
    return q


def _decompose_affine(affine: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Decompose an affine transformation into a quaternion and the translation.
    :param affine: The affine transformation matrix
    :returns: The quaternion and the translation array
    """
    return _get_quat_from_mat(affine[:3, :3]), affine[:3, 3]


# PointStamped
def do_transform_point(
        point: PointStamped,
        transform: TransformStamped) -> PointStamped:
    """
    Transform a `PointStamped` using a given `TransformStamped`.
    :param point: The point
    :param transform: The transform
    :returns: The transformed point
    """
    _, point = _decompose_affine(
        np.matmul(
            _transform_to_affine(transform),
            _build_affine(translation=[
                point.point.x,
                point.point.y,
                point.point.z
            ])))

    res = PointStamped()
    res.point.x = point[0]
    res.point.y = point[1]
    res.point.z = point[2]
    res.header = transform.header
    return res



# Vector3Stamped
def do_transform_vector3(
        vector3: Vector3Stamped,
        transform: TransformStamped) -> Vector3Stamped:
    """
    Transform a `Vector3Stamped` using a given `TransformStamped`.
    :param vector3: The vector3
    :param transform: The transform
    :returns: The transformed vector3
    """
    transform.transform.translation.x = 0.0
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.0
    _, point = _decompose_affine(
        np.matmul(
            _transform_to_affine(transform),
            _build_affine(translation=[
                vector3.vector.x,
                vector3.vector.y,
                vector3.vector.z
            ])))
    res = Vector3Stamped()
    res.vector.x = point[0]
    res.vector.y = point[1]
    res.vector.z = point[2]
    res.header = transform.header
    return res



# Pose
def do_transform_pose(
        pose: Pose,
        transform: TransformStamped) -> Pose:
    """
    Transform a `Pose` using a given `TransformStamped`.
    This method is used to share the tranformation done in
    `do_transform_pose_stamped()` and `do_transform_pose_with_covariance_stamped()`
    :param pose: The pose
    :param transform: The transform
    :returns: The transformed pose
    """
    quaternion, point = _decompose_affine(
        np.matmul(
            _transform_to_affine(transform),
            _build_affine(
                translation=[
                    pose.position.x,
                    pose.position.y,
                    pose.position.z
                ],
                rotation=[
                    pose.orientation.w,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z])))
    res = Pose()
    res.position.x = point[0]
    res.position.y = point[1]
    res.position.z = point[2]
    res.orientation.w = quaternion[0]
    res.orientation.x = quaternion[1]
    res.orientation.y = quaternion[2]
    res.orientation.z = quaternion[3]
    return res


# PoseStamped
def do_transform_pose_stamped(
        pose: PoseStamped,
        transform: TransformStamped) -> PoseStamped:
    """
    Transform a `PoseStamped` using a given `TransformStamped`.
    :param pose: The stamped pose
    :param transform: The transform
    :returns: The transformed pose stamped
    """
    res = PoseStamped()
    res.pose = do_transform_pose(pose.pose, transform)
    res.header = transform.header
    return res

def noise_pose_stamped(input_pose: PoseStamped) -> PoseStamped:

    res = PoseStamped()
    res.header = input_pose.header
    
    res.pose.position.x = input_pose.pose.position.x + np.random.normal(0,0.03)
    res.pose.position.y = input_pose.pose.position.y + np.random.normal(0,0.03)
    q_in = Quaternion(  input_pose.pose.orientation.w,
                        input_pose.pose.orientation.x,
                        input_pose.pose.orientation.y,
                        input_pose.pose.orientation.z
                        )
    roll, pitch, yaw = q_in.to_euler()
    yaw += np.random.normal(0,0.001)
    q_out = Quaternion.from_euler(roll,pitch,yaw)
    
    res.pose.orientation.w = q_out.w
    res.pose.orientation.x = q_out.x
    res.pose.orientation.y = q_out.y
    res.pose.orientation.z = q_out.z

    return res