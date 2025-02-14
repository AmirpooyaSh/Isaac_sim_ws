from scipy.spatial.transform import Rotation as R
import numpy as np

# def euler_to_quat(roll, pitch, yaw):
#     quat = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
#     return quat

# print(euler_to_quat(0, 0, 0))

def divide_portions(x, y):
    if y == 0:
        return "Error: Division by zero is not allowed"
    
    portions = x // y  # Number of full portions
    remainder = x % y  # Remaining part
    
    return portions, remainder

# Example usage
print(np.tan(0))