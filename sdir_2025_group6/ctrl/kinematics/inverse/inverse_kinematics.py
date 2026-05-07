from ...postypes.configuration import configuration
import numpy as np
import math

class InvKinematics:

    def __init__(self):
        # Robot arm dimensions
        self.l1 = 0.645
        self.m  = 0.330
        self.l2 = 1.150
        self.n  = 0.115
        self.l3 = 1.220
        self.d6 = 0.215

        # Joint limits for each joint
        self.joint_limits = [
            (-185.0, 185.0),  # theta1 range
            (-140.0,  -5.0),  # theta2 range
            (-120.0, 168.0),  # theta3 range
            (-350.0, 350.0),  # theta4 range
            (-125.0, 125.0),  # theta5 range
            (-350.0, 350.0),  # theta6 range
        ]

    def get_inv_kinematics(self, pos):
        # Get position and orientation
        X, Y, Z, A, B, C = pos.get_position()
        A_deg, B_deg, C_deg = math.degrees(A), math.degrees(B), math.degrees(C)

        # Rpy_to_Rot
        r, p, y = math.radians(C_deg), math.radians(B_deg), math.radians(A_deg)
        Rx = np.array([[1.0, 0.0, 0.0],
                       [0.0, math.cos(r), -math.sin(r)],
                       [0.0, math.sin(r),  math.cos(r)]])
        Ry = np.array([[ math.cos(p), 0.0, math.sin(p)],
                       [0.0, 1.0, 0.0],
                       [-math.sin(p), 0.0, math.cos(p)]])
        Rz = np.array([[math.cos(y), -math.sin(y), 0.0],
                       [math.sin(y),  math.cos(y), 0.0],
                       [0.0, 0.0, 1.0]])
        R_target = Rz @ Ry @ Rx

        # Wrist center position
        ae_vec = R_target[:, 2]
        wx = X - self.d6 * ae_vec[0]
        wy = Y - self.d6 * ae_vec[1]
        wz = Z - self.d6 * ae_vec[2]

        # Solve_theta123
        solutions_123 = []
        ab = math.sqrt(wx*wx + wy*wy)
        h = wz - self.l1
        l4 = math.sqrt(self.l3*self.l3 + self.n*self.n)

        # Handle singularity at base
        if ab < 1e-9:
            theta1_list = [0.0]
            config_list = [0]
        else:
            theta1_forward = -math.atan2(wy, wx)
            theta1_backward = theta1_forward + math.pi
            theta1_list = [theta1_forward, theta1_backward]
            config_list = [0, 1]

        # Loop through each base configuration
        for idx, theta1 in zip(config_list, theta1_list):
            r1 = (ab - self.m) if idx == 0 else (ab + self.m)
            
            if abs(r1) < 1e-6:
                continue
            
            p = math.sqrt(r1*r1 + h*h)
            if p < 1e-9:
                continue
            
            # Law of cosines for theta3
            cos_b3 = np.clip((p*p - self.l2*self.l2 - l4*l4) / (-2*self.l2*l4), -1.0, 1.0)
            
            if abs(abs(cos_b3) - 1.0) < 1e-6:  # Elbow singularity
                continue

            beta3 = math.acos(cos_b3)
            beta4 = math.atan2(self.n, self.l3)
            beta5 = math.pi - beta3

            # Both elbow configurations
            for elbow in [0, 1]:
                theta3 = beta4 + beta5 if (idx == 0 and elbow == 0) or (idx == 1 and elbow == 1) else beta4 - beta5

                # Calculate theta2
                beta1 = math.atan2(h, r1)
                cos_b2 = np.clip((l4*l4 - self.l2*self.l2 - p*p) / (-2*self.l2*p), -1.0, 1.0)
                beta2 = math.acos(cos_b2)

                if idx == 0:
                    theta2 = -(beta1 + beta2) if elbow == 0 else beta2 - beta1
                else:
                    theta2 = -(math.pi - (beta1 + beta2)) if elbow == 0 else -(math.pi - (beta1 - beta2))

                solutions_123.append((theta1, theta2, theta3))

        # Process all solutions
        configs = []
        seen_solutions = set()

        for t1r, t2r, t3r in solutions_123:
            t1d, t2d, t3d = math.degrees(t1r), math.degrees(t2r), math.degrees(t3r)

            # Theta1 with offsets to handle wraparound
            for t1_offset in [0, 360, -360]:
                t1d_var = t1d + t1_offset
                
                # Check if joints 1-3 are within limits
                if not (self.joint_limits[0][0] <= t1d_var <= self.joint_limits[0][1] and
                        self.joint_limits[1][0] <= t2d <= self.joint_limits[1][1] and
                        self.joint_limits[2][0] <= t3d <= self.joint_limits[2][1]):
                    continue

                # Solve_theta456
                dh_params = [(0, 180, 0, 0), (t1d_var, 90, -645, 330), (t2d, 0, 0, 1150),
                             (t3d - 90, 90, 0, 115), (0, 0, 0, 0)]

                T03 = np.eye(4)
                for theta, alpha, d, r in dh_params:
                    thetaR, alphaR = math.radians(theta), math.radians(alpha)
                    ct, st = np.cos(thetaR), np.sin(thetaR)
                    ca, sa = np.cos(alphaR), np.sin(alphaR)
                    T = np.array([[ct, -st*ca,  st*sa, r*ct],
                                  [st,  ct*ca, -ct*sa, r*st],
                                  [0,   sa,     ca,    d],
                                  [0,   0,      0,     1]])
                    T03 = T03 @ T

                R03 = T03[0:3, 0:3]
                Ry_corr = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
                R6_3 = R03.T @ R_target @ Ry_corr
                R6_3[np.abs(R6_3) < 1e-10] = 0.0

                # Calculate theta5
                sin_t5 = math.sqrt(R6_3[0, 2]**2 + R6_3[1, 2]**2)
                cos_t5 = R6_3[2, 2]

                wrist_solutions = []

                # Wrist singularity: theta5 ≈ 0
                if sin_t5 < 1e-6:
                    theta5 = math.atan2(sin_t5, cos_t5)
                    theta4, theta6 = 0.0, math.atan2(R6_3[1, 0], R6_3[0, 0])
                    wrist_solutions.append((math.degrees(theta4), math.degrees(theta5), math.degrees(theta6)))
                
                # Wrist singularity: theta5 ≈ 180
                elif abs(cos_t5 + 1.0) < 1e-6:
                    theta5 = math.atan2(sin_t5, cos_t5)
                    theta4, theta6 = 0.0, math.atan2(-R6_3[1, 0], -R6_3[0, 0])
                    wrist_solutions.append((math.degrees(theta4), math.degrees(theta5), math.degrees(theta6)))
                
                # Normal case
                else:
                    for flip in [1, -1]:
                        theta5 = math.atan2(flip * sin_t5, cos_t5)
                        
                        if theta5 >= 0:
                            theta4_base = math.atan2(R6_3[1, 2], R6_3[0, 2])
                            theta6_base = math.atan2(R6_3[2, 1], -R6_3[2, 0])
                        else:
                            theta4_base = math.atan2(-R6_3[1, 2], -R6_3[0, 2])
                            theta6_base = math.atan2(-R6_3[2, 1], R6_3[2, 0])

                        t5 = math.degrees(theta5)

                        # Try offsets for theta4 and theta6 (wider range for ±350° limits)
                        for t4_offset in [0, 360, -360]:
                            t4_candidate = math.degrees(theta4_base) + t4_offset
                            if not (self.joint_limits[3][0] <= t4_candidate <= self.joint_limits[3][1]):
                                continue
                            
                            for t6_offset in [0, 360, -360]:
                                t6_candidate = math.degrees(theta6_base) + t6_offset
                                if (self.joint_limits[4][0] <= t5 <= self.joint_limits[4][1] and
                                    self.joint_limits[5][0] <= t6_candidate <= self.joint_limits[5][1]):
                                    wrist_solutions.append((t4_candidate, t5, t6_candidate))

                # Remove duplicates and add valid configurations
                for t4d, t5d, t6d in wrist_solutions:
                    key = (round(t1d_var, 6), round(t2d, 6), round(t3d, 6),
                           round(t4d, 6), round(t5d, 6), round(t6d, 6))

                    if key in seen_solutions:
                        continue

                    seen_solutions.add(key)
                    cfg = configuration([math.radians(t1d_var), math.radians(t2d), math.radians(t3d),
                                        math.radians(t4d), math.radians(t5d), math.radians(t6d)])
                    configs.append(cfg)

        # Print results
        print(f"For inverse kinematics we found {len(configs)} solutions")
        for idx, cfg in enumerate(configs):
            print(f"\n Solution {idx + 1}")
            print(f"θ1 = {math.degrees(cfg[0]):>8.3f}°")
            print(f"θ2 = {math.degrees(cfg[1]):>8.3f}°")
            print(f"θ3 = {math.degrees(cfg[2]):>8.3f}°")
            print(f"θ4 = {math.degrees(cfg[3]):>8.3f}°")
            print(f"θ5 = {math.degrees(cfg[4]):>8.3f}°")
            print(f"θ6 = {math.degrees(cfg[5]):>8.3f}°")
        
        

        return configs