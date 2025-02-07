from scipy.spatial.transform import Rotation as R

def euler_to_quat(roll, pitch, yaw):
    quat = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
    return quat

print(euler_to_quat(0, 0, 0))
