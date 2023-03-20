import numpy as np
from squaternion import Quaternion
from geometry_msgs.msg import (PointStamped, Pose, PoseStamped,
                               PoseWithCovarianceStamped, TransformStamped,
                               Vector3Stamped)

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
    yaw += np.random.normal(0,0.005)
    q_out = Quaternion.from_euler(roll,pitch,yaw)
    
    res.pose.orientation.w = q_out.w
    res.pose.orientation.x = q_out.x
    res.pose.orientation.y = q_out.y
    res.pose.orientation.z = q_out.z

    return res