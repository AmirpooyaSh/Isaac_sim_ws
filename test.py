from scipy.spatial.transform import Rotation as R

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
x = 2
y = 0.3
result = divide_portions(x, y)
print(f"{x} can be divided into {result[0]} portions of length {y}, with a remainder of {result[1]}")