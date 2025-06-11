import numpy as np

def stud_corners_quat(center, quat, length, width, height):
    w, x, y, z = quat
    n = np.sqrt(w*w + x*x + y*y + z*z)
    w, x, y, z = w/n, x/n, y/n, z/n
    
    R = np.array([
        [1 - 2*(y*y + z*z),  2*(x*y - z*w),      2*(x*z + y*w)],
        [2*(x*y + z*w),      1 - 2*(x*x + z*z),  2*(y*z - x*w)],
        [2*(x*z - y*w),      2*(y*z + x*w),      1 - 2*(x*x + y*y)]
    ])
    
    L_local = np.array([0, 1, 0])
    W_local = np.array([0, 0, 1])
    H_local = np.array([1, 0, 0])
    
    L_hat = R @ L_local
    W_hat = R @ W_local
    H_hat = R @ H_local
    
    vL = 0.5 * length * L_hat
    vW = 0.5 * width  * W_hat
    vH = 0.5 * height * H_hat
    
    c = np.asarray(center)
    corners = [c + sL*vL + sW*vW + sH*vH
               for sL in (-1, 1)
               for sW in (-1, 1)
               for sH in (-1, 1)]
    return np.vstack(corners)

ev = np.sqrt(2)/2
center     = (3.63075, 1.48443, 0.94819)
quat       = [ev, 0, ev, 0]
length     = 3.6576
thickness  = 0.04
height     = 0.1016

corners = stud_corners_quat(center, quat, length, thickness, height)
corners