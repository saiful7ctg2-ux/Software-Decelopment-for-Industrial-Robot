import numpy as np
import math
from ...postypes.trajectory import trajectory
from ...postypes.configuration import configuration
from ...postypes.SixDPos import SixDPos
from ...kinematics.direct.fw_kinematics import FwKinematics
from ...kinematics.inverse.inverse_kinematics import InvKinematics


class Circular:
    def __init__(self):
        # Motion parameters 
        self.v_max = 2.0       
        self.a_max = 2.3       
        self.T_IPO = 0.005    

        # Kinematics solvers
        self.fk = FwKinematics()
        self.ik = InvKinematics()

    def get_circular_trajectory(self, start, via, end) -> trajectory:
        
        
        # Convert to SixDPos 
        def to_sixdpos(obj):
            
            if isinstance(obj, SixDPos):
                return obj
            elif isinstance(obj, configuration):
                return self.fk.get_fw_kinematics(obj)
            elif isinstance(obj, (list, tuple)) and len(obj) == 6:
                return SixDPos(*obj)
            else:
                raise TypeError(f"Unsupported type: {type(obj)}")
        
        #  Closest IK Solution 
        def select_closest_solution(solutions, prev_cfg):
            
            min_dist = float('inf')
            best_cfg = solutions[0]
            prev_joints = prev_cfg.get_configuration()
            
            for sol in solutions:
                joints = sol.get_configuration()
                # Euclidean distance in joint space
                dist_squared = sum((joints[i] - prev_joints[i])**2 for i in range(6))
                dist = math.sqrt(dist_squared)
                
                if dist < min_dist:
                    min_dist = dist
                    best_cfg = sol
            
            return best_cfg
        
        # Initialize Trajectory
        traj = trajectory()
        
        # Store original start configuration if provided
        start_cfg_original = start if isinstance(start, configuration) else None
        
        # Convert Inputs to Cartesian Positions
        P_st = to_sixdpos(start)
        P_h = to_sixdpos(via)
        P_z = to_sixdpos(end)
        
        # Extract positions and orientations
        x_st, y_st, z_st, A_st, B_st, C_st = P_st.get_position()
        x_h, y_h, z_h, _, _, _ = P_h.get_position()
        x_z, y_z, z_z, _, _, _ = P_z.get_position()
        
        # Position vectors 
        p_st = np.array([x_st, y_st, z_st])
        p_h = np.array([x_h, y_h, z_h])
        p_z = np.array([x_z, y_z, z_z])
        
        print("=== Circular Interpolation (Lecture Method) ===")
        print(f"Start point P_st: {p_st}")
        print(f"Via point P_h: {p_h}")
        print(f"End point P_z: {p_z}")
        
        # Circle Parameters 
        
        v_stz = p_z - p_st  
        v_sth = p_h - p_st  
        
        # Check if points are collinear
        cross_check = np.cross(v_stz, v_sth)
        if np.linalg.norm(cross_check) < 1e-6:
            print("Error: Points are collinear, cannot form a circle")
            
            from ...pathplanner.lin.lin import Lin
            lin = Lin()
            return lin.get_lin_trajectory(start, end)
        
        
   
        xC = v_stz / np.linalg.norm(v_stz)
        
        zC = np.cross(xC, v_sth)
        zC = zC / np.linalg.norm(zC)
        
        yC = np.cross(zC, xC)
        
        A_0C = np.column_stack([xC, yC, zC])
        
        # Transform vectors into KC frame
        A_0C_T = A_0C.T
        v_stz_C = A_0C_T @ v_stz
        v_sth_C = A_0C_T @ v_sth
        
        # Extract helper values 
        d = v_stz_C[0]    
        Hx = v_sth_C[0]   
        Hy = v_sth_C[1]   
        
        # Calculate circle center in KC frame 
        Mx = d / 2.0
        My = (Hx**2 + Hy**2 - Hx*d) / (2.0 * Hy)
        center_C = np.array([Mx, My, 0.0])
        
        # Compute radius 
        R = math.sqrt(Mx**2 + My**2)
        
        # Compute central angle
        phi_Z = 2.0 * math.pi - 2.0 * math.atan2(Mx, My)
        
        print(f"Circle radius R: {R:.6f} m")
        print(f"Central angle phi_Z: {math.degrees(phi_Z):.3f}°")
        print(f"Circle center (KC frame): {center_C}")
        
        # Calculate Arc Length
        arc_length = R * phi_Z
        print(f"Arc length: {arc_length:.6f} m")
        
        #  Check Maximum Velocity 
        
        v_limit = math.sqrt(arc_length * self.a_max)
        
        if self.v_max > v_limit:
            v_max_used = v_limit
            print(f"Velocity limited to {v_max_used:.3f} m/s (triangular profile)")
        else:
            v_max_used = self.v_max
            print(f"Using maximum velocity {v_max_used:.3f} m/s (trapezoidal profile)")
        
        #  Calculate Timing Parameters
        t_acc_ideal = v_max_used / self.a_max
        t_acc = math.ceil(t_acc_ideal / self.T_IPO) * self.T_IPO
        
        
        t_const_ideal = arc_length / v_max_used
        t_const = math.ceil(t_const_ideal / self.T_IPO) * self.T_IPO
        
       
        t_total = t_acc + t_const
        
        # Adjust Velocity and Acceleration for Consistency 
        v_max_adj = arc_length / t_const
        a_max_adj = v_max_adj / t_acc
        
        print(f"Adjusted velocity: {v_max_adj:.3f} m/s")
        print(f"Adjusted acceleration: {a_max_adj:.3f} m/s²")
        print(f"Acceleration time: {t_acc:.3f} s")
        print(f"Constant velocity time: {t_const:.3f} s")
        print(f"Total time: {t_total:.3f} s")
        
        #  Generate Trajectory Points
        t = 0.0
        prev_cfg = start_cfg_original
        points = []
        
        # Calculate distance covered during acceleration phase
        distance_acc_phase = 0.5 * a_max_adj * t_acc**2
        
        # Helper angle for position calculation 
        phi_M = math.atan2(My, Mx)
        
        while t <= t_total + self.T_IPO / 2.0:
            #  Calculate Motion Profile 
            if t <= t_acc:
               
                a = a_max_adj
                v = a_max_adj * t
                s = 0.5 * a_max_adj * t**2
                
            elif t <= t_const:
                
                a = 0.0
                v = v_max_adj
                s = distance_acc_phase + v_max_adj * (t - t_acc)
                
            else:
                
                time_in_decel = t - t_const
                a = -a_max_adj
                v = v_max_adj - a_max_adj * time_in_decel
                s = arc_length - 0.5 * a_max_adj * (t_total - t)**2
            
            # Clamp s to valid range
            s = max(0.0, min(s, arc_length))
            
            #  Calculate Position on Circular Arc 
           
            phi = s / R
            
            # Compute local KC coordinates
            
            sin_half_phi = math.sin(phi / 2.0)
            pCx = 2.0 * R * sin_half_phi * math.sin(phi / 2.0 - phi_M)
            pCy = 2.0 * R * sin_half_phi * math.cos(phi / 2.0 - phi_M)
            pCz = 0.0
            
            pC = np.array([pCx, pCy, pCz])
            
            
            pos_world = p_st + A_0C @ pC
            
            # Keep orientation fixed at start point 
            A_t = A_st
            B_t = B_st
            C_t = C_st
            
            # Create SixDPos object
            pos_sixd = SixDPos(pos_world[0], pos_world[1], pos_world[2], A_t, B_t, C_t)
            
            #  Solve Inverse Kinematics
            ik_solutions = self.ik.get_inv_kinematics(pos_sixd)
            
            if ik_solutions:
                if prev_cfg is None:
                    
                    cfg = ik_solutions[0]
                else:
                   
                    cfg = select_closest_solution(ik_solutions, prev_cfg)
                
                points.append(cfg)
                prev_cfg = cfg
            else:
                print(f"Warning: No IK solution at t={t:.3f}s, phi={math.degrees(phi):.1f}°")
            
           
            t += self.T_IPO
        
        # Return Trajectory 
        traj.set_trajectory(points)
        print(f"Generated {len(points)} trajectory points")
        print("=" * 50)
        return traj