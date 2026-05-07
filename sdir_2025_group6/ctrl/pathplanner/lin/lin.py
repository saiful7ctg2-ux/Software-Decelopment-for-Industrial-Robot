import numpy as np
import math
from ...postypes.trajectory import trajectory
from ...postypes.configuration import configuration
from ...postypes.SixDPos import SixDPos
from ...kinematics.direct.fw_kinematics import FwKinematics
from ...kinematics.inverse.inverse_kinematics import InvKinematics


class Lin:
    def __init__(self):
        # Motion parameters
        self.v_max = 2.0       # m/s
        self.b_max = 2.3       # m/s²
        self.T_IPO = 0.005     # s

        # Kinematics solvers
        self.fk = FwKinematics()
        self.ik = InvKinematics()

    def get_lin_trajectory(self, start, end) -> trajectory:

        traj = trajectory()
      
        # Convert inputs to Cartesian
     
        start_cfg_original = start if isinstance(start, configuration) else None

        if isinstance(start, SixDPos):
            start_pos = start
        elif isinstance(start, configuration):
            start_pos = self.fk.get_fw_kinematics(start)
        elif isinstance(start, (list, tuple)) and len(start) == 6:
            start_pos = SixDPos(*start)
       
        else:
            raise TypeError("Unsupported start type")

        if isinstance(end, SixDPos):
            end_pos = end
        elif isinstance(end, configuration):
            end_pos = self.fk.get_fw_kinematics(end)
        elif isinstance(end, (list, tuple)) and len(end) == 6:
            end_pos = SixDPos(*end)

        
        else:
            raise TypeError("Unsupported end type")

       
        # Extract positions
      
        x1, y1, z1, A1, B1, C1 = start_pos.get_position()
        x2, y2, z2, A2, B2, C2 = end_pos.get_position()

        p1 = np.array([x1, y1, z1])  
        p2 = np.array([x2, y2, z2]) 
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        dz = p2[2] - p1[2]

        # Square each difference
        dx2 = dx ** 2
        dy2 = dy ** 2
        dz2 = dz ** 2

        # Sum of squares
        sum_sq = dx2 + dy2 + dz2

        # Square root to get Euclidean distance
        dist_m = math.sqrt(sum_sq)
      
        # Velocity profile selection
       
        v_limit = math.sqrt(dist_m * self.b_max)

        if self.v_max > v_limit:
            # Triangular
            v = v_limit
            t_b_ideal = v / self.b_max
            t_b = math.ceil(t_b_ideal / self.T_IPO) * self.T_IPO
            t_v = t_b
            t_e = t_b + t_v
        else:
            # Trapezoidal
            v = self.v_max
            t_b_ideal = v / self.b_max
            t_v_ideal = dist_m / v
            t_b = math.ceil(t_b_ideal / self.T_IPO) * self.T_IPO
            t_v = math.ceil(t_v_ideal / self.T_IPO) * self.T_IPO
            t_e = t_v + t_b

        v_m = dist_m / t_v
        b_m = v_m / t_b

        # Trajectory generation
        
        t = 0.0
        prev_cfg = start_cfg_original
        points = []

        while t <= t_e + self.T_IPO / 2.0:

            if t <= t_b:
                s = 0.5 * b_m * t ** 2
            elif t <= t_v:
                s = v_m * t - 0.5 * v_m ** 2 / b_m
            else:
                s = v_m * t_v - 0.5 * b_m * (t_e - t) ** 2

            s_norm = min(max(s / dist_m, 0.0), 1.0)

            p = p1 + s_norm * (p2 - p1)

            # Orientation kept constant
            pos = SixDPos(
                p[0], p[1], p[2], A1, B1, C1
            )

            # Inverse kinematics
           
            ik_solutions = self.ik.get_inv_kinematics(pos)

            if ik_solutions:
                if prev_cfg is None:
                    cfg = ik_solutions[0]
                else:
                    # Select closest solution
                    min_dist = float('inf')
                    best_cfg = ik_solutions[0]
                    prev_joints = prev_cfg.get_configuration()

                    for sol in ik_solutions:
                        joints = sol.get_configuration()
                        d = sum((joints[i] - prev_joints[i]) ** 2 for i in range(6))
                        d = math.sqrt(d)
                        if d < min_dist:
                            min_dist = d
                            best_cfg = sol

                    cfg = best_cfg

                points.append(cfg)
                prev_cfg = cfg

            else:
                print(f"Warning: No IK solution at t={t:.3f}s")

            t += self.T_IPO

        traj.set_trajectory(points)
        return traj
