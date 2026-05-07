from ...postypes.trajectory import trajectory
from ...postypes.configuration import configuration
import numpy as np
import math

class Ptp:
    
    def __init__(self):
        self.num_joints = 6

        # Maximum joint velocities (rad/s)
        self.max_velocities = [
            np.radians(120),
            np.radians(115),
            np.radians(120),
            np.radians(190),
            np.radians(180),
            np.radians(260)
        ]

        # Maximum joint accelerations (rad/s^2)
        self.max_accelerations = [np.radians(500)] * 6

        self.T_IPO = 0.05  

   
    # PTP sync
    
    def get_ptp_trajectory(self, start_cfg, end_cfg):
       
        traj = trajectory()

        theta_start = np.array(start_cfg.get_configuration())
        theta_end   = np.array(end_cfg.get_configuration())
        delta_theta = theta_end - theta_start

        # Input validation
        if len(theta_start) != self.num_joints or len(theta_end) != self.num_joints:
            raise ValueError(f"Configuration must have {self.num_joints} joints")
       
        # asynchronous motion time per joint
       
        joint_times = []

        for i in range(self.num_joints):
            s_ep = abs(delta_theta[i])
            v_m  = self.max_velocities[i]
            b_m  = self.max_accelerations[i]

            if s_ep < 1e-9:
                joint_times.append(0.0)
                continue

            # check for trapezoidal or triangular
            if s_ep > v_m**2 / b_m:
                # Trapezoidal
                t_b = v_m / b_m
                t_const = (s_ep / v_m) - (v_m / b_m)
                t_e = 2 * t_b + t_const
            else:
                # Triangular
                v_peak = math.sqrt(s_ep * b_m)
                t_e = 2 * v_peak / b_m

            joint_times.append(t_e)

        
        # lead joint time
        
        t_e_max = max(joint_times)

        # if no movement needed
        if t_e_max < 1e-9:
            traj.add_configuration(configuration(theta_start.tolist()))
            return traj

       
        # interpolation correction
        
        steps = int(np.ceil(t_e_max / self.T_IPO))
        t_e_sync = steps * self.T_IPO

       
        # synchronized trapezoidal profiles
        
        joint_profiles = []

        for i in range(self.num_joints):
            s_ep = abs(delta_theta[i])
            b_m  = self.max_accelerations[i]

            if s_ep < 1e-9:
                joint_profiles.append({
                    'v_m': 0.0,
                    'b_m': 0.0,
                    't_b': 0.0,
                    't_v': t_e_sync,
                    't_e': t_e_sync,
                    's_ep': 0.0
                })
                continue

            # Solve quadratic equation for synchronized velocity:
            A = 1.0
            B = -(t_e_sync * b_m)
            C = s_ep * b_m

            disc = B * B - 4 * A * C
            
            v_m = (-B - math.sqrt(disc)) / (2 * A)

            # Round acceleration time to nearest sample
            t_b = v_m / b_m
            t_b_rounded = round(t_b / self.T_IPO) * self.T_IPO
            
            # Recalculate v_m based on rounded t_b
            v_m = b_m * t_b_rounded

            # Calculate braking start time
            t_const = max(0.0, t_e_sync - 2 * t_b_rounded)
            t_v = t_b_rounded + t_const
            
            # Calculate actual distance covered with rounded times
            s_accel = 0.5 * b_m * t_b_rounded * t_b_rounded
            s_const = v_m * t_const
            s_decel = v_m * t_b_rounded - 0.5 * b_m * t_b_rounded * t_b_rounded
            s_actual = s_accel + s_const + s_decel
            
            # correct for rounding error
            scale_factor = s_ep / s_actual if s_actual > 1e-9 else 1.0

            joint_profiles.append({
                'v_m': v_m,
                'b_m': b_m,
                't_b': t_b_rounded,
                't_v': t_v,
                't_e': t_e_sync,
                's_ep': s_ep,
                'scale_factor': scale_factor
            })
        
        # Generate trajectory samples
        
        num_steps = steps

        for k in range(num_steps + 1):
            t = k * self.T_IPO
            current_cfg = []

            for i in range(self.num_joints):
                s_ep = abs(delta_theta[i])

                if s_ep < 1e-9:
                    current_cfg.append(theta_start[i])
                    continue

                prof = joint_profiles[i]
                v_m, b_m = prof['v_m'], prof['b_m']
                t_b, t_v, t_e = prof['t_b'], prof['t_v'], prof['t_e']
                scale_factor = prof['scale_factor']

                direction = 1.0 if delta_theta[i] > 0 else -1.0

                # Compute position based on phase
                if t <= t_b:
                    # Acceleration phase
                    s_t = 0.5 * b_m * t * t

                elif t <= t_v:
                    # Constant velocity phase
                    s_b = 0.5 * b_m * t_b * t_b
                    s_v = v_m * (t - t_b)
                    s_t = s_b + s_v

                elif t <= t_e:
                    # Deceleration phase
                    t_prime = t - t_v
                    s_b = 0.5 * b_m * t_b * t_b
                    s_v = v_m * (t_v - t_b)
                    s_d = v_m * t_prime - 0.5 * b_m * t_prime * t_prime
                    s_t = s_b + s_v + s_d

                else:
                    # use exact target
                    s_t = s_ep

                # Apply scale factor to correct rounding errors
                s_t = s_t * scale_factor
                
                # Clamp to not exceed target distance
                s_t = min(s_t, s_ep)

                theta_t = theta_start[i] + direction * s_t
                current_cfg.append(theta_t)

            traj.add_configuration(configuration(current_cfg))

        # Ensure final position
        all_configs = traj.get_all_configuration()
        if len(all_configs) > 0:
            # modify the last configuration object
            all_configs[-1].set_configuration(theta_end.tolist())
        
        return traj

   
    # PTP async
    
    def get_ptp_async_trajectory(self, start_cfg, end_cfg):
       
        traj = trajectory()

        theta_start = np.array(start_cfg.get_configuration())
        theta_end   = np.array(end_cfg.get_configuration())
        delta_theta = theta_end - theta_start

        # Input validation
        if len(theta_start) != self.num_joints or len(theta_end) != self.num_joints:
            raise ValueError(f"Configuration must have {self.num_joints} joints")

        joint_profiles = []
        joint_times = []
       
        # Compute trapezoidal or triangular profile
       
        for i in range(self.num_joints):
            s_ep = abs(delta_theta[i])
            v_m  = self.max_velocities[i]
            b_m  = self.max_accelerations[i]

            if s_ep < 1e-9:
                joint_profiles.append({
                    'type': 'stationary',
                    'v_m': 0.0,
                    'b_m': 0.0,
                    't_b': 0.0,
                    't_v': 0.0,
                    't_e': 0.0,
                    's_ep': 0.0
                })
                joint_times.append(0.0)
                continue

            # Check if trapezoidal or triangular
            if s_ep > v_m**2 / b_m:
                # Trapezoidal
                t_b = v_m / b_m
                t_const = (s_ep / v_m) - (v_m / b_m)
                t_v = t_b + t_const
                t_e = t_v + t_b
                profile_type = 'trapezoidal'
            else:
                # Triangular
                v_m = math.sqrt(s_ep * b_m)
                t_b = v_m / b_m
                t_v = t_b  # No constant velocity phase
                t_e = 2 * t_b
                profile_type = 'triangular'

            # Round t_b to nearest sample
            t_b = round(t_b / self.T_IPO) * self.T_IPO
            v_m = b_m * t_b
            
            # Round t_e up to next sample
            t_e = np.ceil(t_e / self.T_IPO) * self.T_IPO
            
            # Recalculate t_v for consistency
            t_v = t_e - t_b

            joint_profiles.append({
                'type': profile_type,
                'v_m': v_m,
                'b_m': b_m,
                't_b': t_b,
                't_v': t_v,
                't_e': t_e,
                's_ep': s_ep
            })

            joint_times.append(t_e)

        # maximum motion time (for trajectory length)
       
        t_e_max = max(joint_times) if joint_times else 0.0

        # no movement
        if t_e_max < 1e-9:
            traj.add_configuration(configuration(theta_start.tolist()))
            return traj

        
        # Generate trajectory samples
       
        num_steps = int(np.ceil(t_e_max / self.T_IPO))

        for k in range(num_steps + 1):
            t = k * self.T_IPO
            current_cfg = []

            for i in range(self.num_joints):
                prof = joint_profiles[i]

                # Stationary joint or finished joint
                if prof['type'] == 'stationary' or t >= prof['t_e']:
                    current_cfg.append(theta_end[i])
                    continue

                s_ep = prof['s_ep']
                v_m, b_m = prof['v_m'], prof['b_m']
                t_b, t_v, t_e = prof['t_b'], prof['t_v'], prof['t_e']

                direction = 1.0 if delta_theta[i] > 0 else -1.0

                # Compute position based on phase
                if t <= t_b:
                    # Acceleration phase
                    s_t = 0.5 * b_m * t * t

                elif t <= t_v:
                    # Constant velocity phase
                    s_b = 0.5 * b_m * t_b * t_b
                    s_v = v_m * (t - t_b)
                    s_t = s_b + s_v

                else:
                    # Deceleration phase
                    t_prime = t - t_v
                    s_b = 0.5 * b_m * t_b * t_b
                    s_v = v_m * (t_v - t_b)
                    s_d = v_m * t_prime - 0.5 * b_m * t_prime * t_prime
                    s_t = s_b + s_v + s_d

                # Clamp to not exceed target distance
                s_t = min(s_t, s_ep)

                theta_t = theta_start[i] + direction * s_t
                current_cfg.append(theta_t)

            traj.add_configuration(configuration(current_cfg))

        # exact final position
        all_configs = traj.get_all_configuration()
        if len(all_configs) > 0:
            # modify the last configuration object
            all_configs[-1].set_configuration(theta_end.tolist())
        
        return traj