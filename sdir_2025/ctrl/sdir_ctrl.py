from typing import List
from .postypes.configuration import configuration
from .postypes.trajectory import trajectory
from .postypes.SixDPos import SixDPos
from .kinematics.direct.fw_kinematics import FwKinematics
from .kinematics.inverse.inverse_kinematics import InvKinematics
from .kinematics.inverse.inv_kinematics_numerical import InvKinematicsNumerical
from .pathplanner.ptp.ptp import Ptp
from .pathplanner.lin.lin import Lin
from .pathplanner.circular.circular import Circular
import math


class SdirCtrl:
    def __init__(self):
        """Initialize with both IK solvers available"""
        pass
        
    def get_pos_from_config(self, config: configuration) -> SixDPos:
        fw_kinematics = FwKinematics()
        new_pos = fw_kinematics.get_fw_kinematics(config)
        return new_pos

    def get_config_from_pos(self, pos: SixDPos, method='geometric') -> List[configuration]:
        """
        Get inverse kinematics solutions
        
        Args:
            pos: Target position (SixDPos object)
            method: 'geometric' or 'numerical' (no 'both' option)
        
        Returns:
            List of configuration solutions
        """
        all_solutions = []
        
        # Geometric method (analytical)
        if method == 'geometric':
            print("\n" + "="*70)
            print("USING GEOMETRIC (ANALYTICAL) METHOD")
            print("="*70)
            inv_kinematics_geo = InvKinematics()
            all_solutions = inv_kinematics_geo.get_inv_kinematics(pos)
        
        # Numerical method (Jacobian Newton-Raphson)
        elif method == 'numerical':
            print("\n" + "="*70)
            print("USING NUMERICAL (JACOBIAN NEWTON-RAPHSON) METHOD")
            print("="*70)
            inv_kinematics_num = InvKinematicsNumerical()
            all_solutions = inv_kinematics_num.get_inv_kinematics(pos)
        
        else:
            raise ValueError(f"Invalid method '{method}'. Use 'geometric' or 'numerical'")
        
        # Print summary
        print("\n" + "="*70)
        print(f"TOTAL SOLUTIONS FOUND: {len(all_solutions)}")
        print("="*70)
        for idx, cfg in enumerate(all_solutions):
            theta_deg = [math.degrees(angle) for angle in cfg]
            print(f"\nFinal Solution {idx + 1}:")
            print(f"  θ1={theta_deg[0]:>8.3f}°, θ2={theta_deg[1]:>8.3f}°, θ3={theta_deg[2]:>8.3f}°")
            print(f"  θ4={theta_deg[3]:>8.3f}°, θ5={theta_deg[4]:>8.3f}°, θ6={theta_deg[5]:>8.3f}°")
        print("="*70 + "\n")
        
        return all_solutions

    def move_robot_ptp(self, start, end) -> trajectory:
        if isinstance(start, configuration) and isinstance(end, configuration):
            ptp = Ptp()
            return ptp.get_ptp_trajectory(start, end)
        else:
            raise ValueError("Invalid argument types for move_robot_ptp")
    
    def move_robot_ptp_async(self, start, end) -> trajectory:
        if isinstance(start, configuration) and isinstance(end, configuration):
            ptp = Ptp()
            return ptp.get_ptp_async_trajectory(start, end)
        else:
            raise ValueError("Invalid argument types for move_robot_ptp")

    def move_robot_lin(self, start, end) -> trajectory:
        if isinstance(start, configuration) and isinstance(end, configuration):
            lin = Lin()
            return lin.get_lin_trajectory(start, end)
        else:
            raise ValueError("Invalid argument types for move_robot_lin")
    
    def move_robot_circular(self, start, via, end) -> trajectory:
        if (isinstance(start, (configuration, SixDPos)) and 
            isinstance(via, (configuration, SixDPos)) and 
            isinstance(end, (configuration, SixDPos))):
            circular = Circular()
            return circular.get_circular_trajectory(start, via, end)
        else:
            raise ValueError("Invalid argument types for move_robot_circular. Expected configuration or SixDPos objects.")