from ...postypes.SixDPos import SixDPos
from ...postypes.configuration import configuration
import numpy as np


class FwKinematics:
    
    def get_fw_kinematics(self, config: configuration) -> SixDPos:
        # Taking joint angles
        theta1, theta2, theta3, theta4, theta5, theta6 = config
        
        # dh_table: theta, alpha, d, r
        dh_table = [
            (0, np.pi, 0, 0),
            (theta1, np.pi/2, -0.645, 0.330),
            (theta2, 0, 0, 1.150),
            (theta3 - np.pi/2, np.pi/2, 0, 0.115),
            (theta4, -np.pi/2, -1.220, 0),
            (theta5, np.pi/2, 0, 0),
            (theta6 + np.pi, np.pi, -0.215, 0)
        ]

        # Multiplying all transformation matrices
        T_total = np.eye(4)
        for theta, alpha, d, r in dh_table:
            # DH transformation matrix calculation 
            ct, st = np.cos(theta), np.sin(theta)
            ca, sa = np.cos(alpha), np.sin(alpha)
            
            T = np.array([
                [ct, -st * ca,  st * sa, r * ct],
                [st,  ct * ca, -ct * sa, r * st],
                [0,   sa,       ca,      d],
                [0,   0,        0,       1]
            ])
            T_total = T_total @ T
        
        # Getting zeroes for low values
        T_total[np.abs(T_total) < 1e-10] = 0.0
        
        print(f"Transformation Matrix:\n{T_total}\n")

        # Getting x, y, z
        x, y, z = T_total[:3, 3]
        print(f"Position: x={x:.3f} m, y={y:.3f} m, z={z:.3f} m")
            
        R = T_total[:3, :3]
        sin_pitch = np.clip(-R[2, 0], -1.0, 1.0)  # Clipping to valid range
        pitch = np.arcsin(sin_pitch)  # B (around Y axis)
        
        # Checking gimbal lock
        cos_pitch = np.cos(pitch)
        if abs(cos_pitch) > 1e-6:
            # Normal case
            yaw = np.arctan2(R[1, 0], R[0, 0])   # A (around Z axis)
            roll = np.arctan2(R[2, 1], R[2, 2])  # C (around X axis)
        else:
            # Gimbal lock case
            yaw = np.arctan2(-R[0, 1], R[1, 1])
            roll = 0.0

        print(f"Orientation: A={np.degrees(yaw):.3f}°, B={np.degrees(pitch):.3f}°, C={np.degrees(roll):.3f}°\n")

        return SixDPos(x, y, z, yaw, pitch, roll)