import numpy as np
import math

class NumericalIKSolverStepByStep:
    def __init__(self):
        # Robot arm dimensions
        self.l1 = 0.645
        self.m = 0.330
        self.l2 = 1.150
        self.n = 0.115
        self.l3 = 1.220
        self.d6 = 0.215

        # Joint limits (degrees)
        self.joint_limits = [
            (-185, 185),  # theta1
            (-140, -5),   # theta2
            (-120, 168),  # theta3
            (-350, 350),  # theta4
            (-125, 125),  # theta5
            (-350, 350)   # theta6
        ]

        # Solver parameters
        self.max_iterations = 500
        self.position_tolerance = 1e-5
        self.orientation_tolerance = 1e-4
        self.damping = 0.01

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

        T_total = np.eye(4)
        for theta_i, alpha, d, r in dh_table:
            ct, st = np.cos(theta_i), np.sin(theta_i)
            ca, sa = np.cos(alpha), np.sin(alpha)
            T = np.array([
                [ct, -st * ca, st * sa, r * ct],
                [st, ct * ca, -ct * sa, r * st],
                [0, sa, ca, d],
                [0, 0, 0, 1]
            ])
            T_total = T_total @ T

        x, y, z = T_total[:3, 3]

        R = T_total[:3, :3]
        sin_pitch = np.clip(-R[2, 0], -1, 1)
        pitch = np.arcsin(sin_pitch)
        cos_pitch = np.cos(pitch)
        if abs(cos_pitch) > 1e-6:
            yaw = np.arctan2(R[1, 0], R[0, 0])
            roll = np.arctan2(R[2, 1], R[2, 2])
        else:
            yaw = np.arctan2(-R[0, 1], R[1, 1])
            roll = 0.0

        return np.array([x, y, z, yaw, pitch, roll])

    def compute_jacobian(self, theta):
        eps = 1e-6
        J = np.zeros((6, 6))
        f_current = self.forward_kinematics(theta)

        for j in range(6):
            theta_perturbed = theta.copy()
            theta_perturbed[j] += eps
            f_perturbed = self.forward_kinematics(theta_perturbed)
            J[:, j] = (f_perturbed - f_current) / eps

        return J

    def wrap_angle_to_limits(self, angle_deg, joint_idx):
        low, high = self.joint_limits[joint_idx]
        angle_deg = ((angle_deg + 180) % 360) - 180
        return np.clip(angle_deg, low, high)

    def solve_iteration(self, theta, target):
        current = self.forward_kinematics(theta)
        error = current - target

        pos_error = np.linalg.norm(error[:3])
        ori_error = np.linalg.norm(error[3:])

        J = self.compute_jacobian(theta)
        JJT = J @ J.T
        damping_matrix = self.damping * np.eye(6)
        delta_theta = -J.T @ np.linalg.solve(JJT + damping_matrix, error)

        theta_new = theta + delta_theta
        theta_deg = np.degrees(theta_new)
        for i in range(6):
            theta_deg[i] = self.wrap_angle_to_limits(theta_deg[i], i)
        theta_new = np.radians(theta_deg)

        return theta_new, pos_error, ori_error, delta_theta, J

    def solve_from_initial_guess(self, target, initial_guess_deg):
        theta = np.radians(initial_guess_deg)
        iteration_data = []

        for iteration in range(self.max_iterations):
            theta, pos_error, ori_error, delta_theta, J = self.solve_iteration(theta, target)

            iteration_data.append({
                'iteration': iteration,
                'theta_deg': np.degrees(theta).copy(),
                'position_error': pos_error,
                'orientation_error': ori_error,
                'delta_theta_deg': np.degrees(delta_theta).copy(),
                'jacobian': J.copy()
            })

            if pos_error < self.position_tolerance and ori_error < self.orientation_tolerance:
                break

        return theta, iteration_data

# ====== USAGE EXAMPLE ======
solver = NumericalIKSolverStepByStep()

# Single initial guess (degrees)
initial_guess = [0, -90, 0, 0, 0, 0]

# Target pose: [x, y, z, yaw, pitch, roll] in radians
target_pose = np.array([-1.55, 0, 2.125, 1.571, 0, 0])

theta_final, iter_data = solver.solve_from_initial_guess(target_pose, initial_guess)

# ====== PRINT RESULTS ======
print("\n=== FINAL THETA (deg) ===")
print(np.degrees(theta_final))

print("\n=== ITERATION DATA ===")
for data in iter_data:
    print(f"\n--- Iteration {data['iteration']} ---")
    print(f"Theta (deg): {np.round(data['theta_deg'], 3)}")
    print(f"Position error: {data['position_error']:.6f} m")
    print(f"Orientation error: {data['orientation_error']:.6f} rad")
    print(f"Delta theta (deg): {np.round(data['delta_theta_deg'], 6)}")
    print(f"Jacobian:\n{np.round(data['jacobian'], 4)}")
