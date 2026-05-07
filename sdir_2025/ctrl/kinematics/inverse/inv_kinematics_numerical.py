from ...postypes.configuration import configuration
from ...postypes.SixDPos import SixDPos
import numpy as np
import math

class InvKinematicsNumerical:
    
    
    def __init__(self):
        # Robot arm dimensions
        self.l1 = 0.645
        self.m = 0.330
        self.l2 = 1.150
        self.n = 0.115
        self.l3 = 1.220
        self.d6 = 0.215

        # Joint limits (in degrees)
        self.joint_limits = [
            (-185.0, 185.0),  # theta1 range
            (-140.0, -5.0),   # theta2 range
            (-120.0, 168.0),  # theta3 range
            (-350.0, 350.0),  # theta4 range
            (-125.0, 125.0),  # theta5 range
            (-350.0, 350.0),  # theta6 range
        ]
        
        # Convergence parameters
        self.max_iterations = 500
        self.position_tolerance = 1e-5
        self.orientation_tolerance = 1e-4
        self.damping_factor = 0.01
        
    def forward_kinematics(self, theta):
        
        theta1, theta2, theta3, theta4, theta5, theta6 = theta
        
        # DH table: (theta, alpha, d, r)
        dh_table = [
            (0, np.pi, 0, 0),
            (theta1, np.pi/2, -self.l1, self.m),
            (theta2, 0, 0, self.l2),
            (theta3 - np.pi/2, np.pi/2, 0, self.n),
            (theta4, -np.pi/2, -self.l3, 0),
            (theta5, np.pi/2, 0, 0),
            (theta6 + np.pi, np.pi, -self.d6, 0)
        ]
        
        # transformation matrix
        T_total = np.eye(4)
        for theta_val, alpha, d, r in dh_table:
            cos_theta = np.cos(theta_val)
            sin_theta = np.sin(theta_val)
            cos_alpha = np.cos(alpha)
            sin_alpha = np.sin(alpha)
            
            T = np.array([
                [cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, r * cos_theta],
                [sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, r * sin_theta],
                [0, sin_alpha, cos_alpha, d],
                [0, 0, 0, 1]
            ])
            T_total = T_total @ T
        
        # Extract position
        x = T_total[0, 3]
        y = T_total[1, 3]
        z = T_total[2, 3]
        
        # Extract orientation (ZYX Euler angles)
        R = T_total[:3, :3]
        sin_pitch = np.clip(-R[2, 0], -1.0, 1.0)
        pitch = np.arcsin(sin_pitch)  # B
        
        cos_pitch = np.cos(pitch)
        if abs(cos_pitch) > 1e-6:
            yaw = np.arctan2(R[1, 0], R[0, 0])    # A
            roll = np.arctan2(R[2, 1], R[2, 2])   # C
        else:
            # Gimbal lock case
            yaw = np.arctan2(-R[0, 1], R[1, 1])
            roll = 0.0
        
        return np.array([x, y, z, yaw, pitch, roll])
    
    def calculate_jacobian(self, theta):
       
        epsilon = 1e-6
        jacobian = np.zeros((6, 6))
        
        # Get current end-effector pose
        current_pose = self.forward_kinematics(theta)
        
        # Calculate each column of Jacobian
        for joint_idx in range(6):
            # Perturb this joint slightly
            theta_perturbed = theta.copy()
            theta_perturbed[joint_idx] += epsilon
            
            # Calculate new pose
            perturbed_pose = self.forward_kinematics(theta_perturbed)
            
            # Derivative = (change in output) / (change in input)
            jacobian[:, joint_idx] = (perturbed_pose - current_pose) / epsilon
        
        return jacobian
    
    def wrap_angle_to_limits(self, angle_deg, joint_index):
       
        # Try different wrappings
        candidates = [
            angle_deg,
            angle_deg + 360,
            angle_deg - 360
        ]
        
        lower_limit = self.joint_limits[joint_index][0]
        upper_limit = self.joint_limits[joint_index][1]
        
        for candidate in candidates:
            if lower_limit <= candidate <= upper_limit:
                return candidate
        
        # If none work, clamp to limits
        return np.clip(angle_deg, lower_limit, upper_limit)
    
    def solve_from_initial_guess(self, target_pose, initial_guess_deg):
       
        # Convert to radians
        theta = np.radians(initial_guess_deg)
        
        # Newton-Raphson iteration
        for iteration in range(self.max_iterations):
            # Step 1: Calculate current pose
            current_pose = self.forward_kinematics(theta)
            
            # Step 2: Calculate error
            error = current_pose - target_pose
            
            # Step 3: Check if we've converged
            position_error = np.linalg.norm(error[:3])
            orientation_error = np.linalg.norm(error[3:])
            
            if position_error < self.position_tolerance and orientation_error < self.orientation_tolerance:
                # Success and Check if within joint limits
                theta_deg = np.degrees(theta)
                within_limits = True
                
                for i in range(6):
                    if not (self.joint_limits[i][0] <= theta_deg[i] <= self.joint_limits[i][1]):
                        within_limits = False
                        break
                
                if within_limits:
                    return True, theta, np.linalg.norm(error)
            
            # Step 4: Calculate Jacobian
            J = self.calculate_jacobian(theta)
            
            # Step 5: Solve for joint update using damped least squares
            # Δθ = -J^T * (J*J^T + λI)^(-1) * error
            JJT = J @ J.T
            damping = self.damping_factor * np.eye(6)
            
            try:
                delta_theta = -J.T @ np.linalg.solve(JJT + damping, error)
            except np.linalg.LinAlgError:
                # Matrix is singular, can't solve
                return False, theta, np.linalg.norm(error)
            
            # Step 6: Update joint angles
            theta = theta + delta_theta
            
            # Step 7: Apply joint limits with wrapping
            theta_deg = np.degrees(theta)
            for i in range(6):
                theta_deg[i] = self.wrap_angle_to_limits(theta_deg[i], i)
            theta = np.radians(theta_deg)
        
        # Did not converge within max iterations
        return False, theta, np.linalg.norm(error)
    
    def generate_initial_guesses(self):
       
        guesses = []
        
        # Strategy 1: Systematic exploration of configuration space
        # Based on typical robot poses
        
        # Base angles (like theta1 forward/backward)
        base_angles = [0, 90, -90, 180, -180]
        
        # Shoulder angles (theta2 range is -140 to -5)
        shoulder_angles = [-90, -70, -45, -20, -10]
        
        # Elbow angles (theta3 range is -120 to 168)
        elbow_angles = [0, 45, 90, -45, -90]
        
        # Create combinations for joints 1-3
        for theta1 in base_angles[:3]:  # Limit combinations
            for theta2 in shoulder_angles[:3]:
                for theta3 in elbow_angles[:3]:
                    guesses.append([theta1, theta2, theta3, 0, 0, 0])
        
        # Strategy 2: Add wrist variations to some base configurations
        wrist_configs = [
            [0, 0, 0],
            [90, 0, 0],
            [-90, 0, 0],
            [0, 90, 0],
            [0, -90, 0],
            [0, 0, 90],
            [0, 0, -90]
        ]
        
        for wrist in wrist_configs:
            guesses.append([0, -90, 45, wrist[0], wrist[1], wrist[2]])
            guesses.append([90, -70, 0, wrist[0], wrist[1], wrist[2]])
        
        # Strategy 3: Random exploration for better coverage
        np.random.seed(42)  # For reproducibility
        for _ in range(10):
            random_guess = []
            for i in range(6):
                low, high = self.joint_limits[i]
                angle = np.random.uniform(low, high)
                random_guess.append(angle)
            guesses.append(random_guess)
        
        return guesses
    
    def get_inv_kinematics(self, pos: SixDPos):
        
       
        # Extract target pose
        X, Y, Z, A, B, C = pos.get_position()
        target_pose = np.array([X, Y, Z, A, B, C])
        
        print(f"NUMERICAL IK (Jacobian Newton-Raphson Method)")
        print(f"Target Position: x={X:.3f}m, y={Y:.3f}m, z={Z:.3f}m")
        print(f"Target Orientation: A={np.degrees(A):.3f}°, B={np.degrees(B):.3f}°, C={np.degrees(C):.3f}°")
        
        # Generate initial guesses
        initial_guesses = self.generate_initial_guesses()
        print(f"\nTrying {len(initial_guesses)} initial guesses...")
        
        # Store solutions
        solutions = []
        seen_configurations = set()
        
        # Try each initial guess
        for guess_idx, guess in enumerate(initial_guesses):
            converged, final_theta, error = self.solve_from_initial_guess(target_pose, guess)
            
            if converged:
                theta_deg = np.degrees(final_theta)
                
                # Try alternate solutions for theta1, theta5, theta6
                # Like geometric method does with ±360° wrapping
                alternate_solutions = []
                
                # Original solution
                alternate_solutions.append(theta_deg.copy())
                
                # Alternate theta1 (±360°)
                for offset1 in [360, -360]:
                    theta1_alt = self.wrap_angle_to_limits(theta_deg[0] + offset1, 0)
                    if abs(theta1_alt - theta_deg[0]) > 1:
                        alt = theta_deg.copy()
                        alt[0] = theta1_alt
                        alternate_solutions.append(alt)
                
                # Alternate theta4 (±360°)
                for offset4 in [360, -360]:
                    theta4_alt = self.wrap_angle_to_limits(theta_deg[3] + offset4, 3)
                    if abs(theta4_alt - theta_deg[3]) > 1:
                        alt = theta_deg.copy()
                        alt[3] = theta4_alt
                        alternate_solutions.append(alt)
                
                # Alternate theta6 (±360°)
                for offset6 in [360, -360]:
                    theta6_alt = self.wrap_angle_to_limits(theta_deg[5] + offset6, 5)
                    if abs(theta6_alt - theta_deg[5]) > 1:
                        alt = theta_deg.copy()
                        alt[5] = theta6_alt
                        alternate_solutions.append(alt)
                
                # Process all alternate solutions
                for alt_theta_deg in alternate_solutions:
                    # Create unique key for duplicate detection
                    key = tuple(round(angle, 2) for angle in alt_theta_deg)
                    
                    if key not in seen_configurations:
                        # Verify all joints within limits
                        valid = True
                        for i in range(6):
                            if not (self.joint_limits[i][0] <= alt_theta_deg[i] <= self.joint_limits[i][1]):
                                valid = False
                                break
                        
                        if valid:
                            seen_configurations.add(key)
                            
                            # Convert to configuration object
                            alt_theta_rad = [math.radians(angle) for angle in alt_theta_deg]
                            cfg = configuration(alt_theta_rad)
                            solutions.append(cfg)
                            
                            print(f"\nSolution {len(solutions)} (from guess {guess_idx+1}):")
                            print(f"  Initial guess: θ1={guess[0]:>7.2f}°, θ2={guess[1]:>7.2f}°, θ3={guess[2]:>7.2f}°, θ4={guess[3]:>7.2f}°, θ5={guess[4]:>7.2f}°, θ6={guess[5]:>7.2f}°")
                            print(f"  Final solution: θ1={alt_theta_deg[0]:>8.3f}°, θ2={alt_theta_deg[1]:>8.3f}°, θ3={alt_theta_deg[2]:>8.3f}°")
                            print(f"                  θ4={alt_theta_deg[3]:>8.3f}°, θ5={alt_theta_deg[4]:>8.3f}°, θ6={alt_theta_deg[5]:>8.3f}°")
                            print(f"  Final error: {error:.6f}")
        
        print(f"Found {len(solutions)} unique solutions using numerical method")
        
        return solutions