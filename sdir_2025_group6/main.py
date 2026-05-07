import os
import time
from ctrl.sdir_ctrl import SdirCtrl
from ctrl.com.json_handler import JsonHandler
from ctrl.postypes.SixDPos import SixDPos
from ctrl.postypes.configuration import configuration
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


# Define operation modes
class OpMode:
    CFG_2_POS = 0
    POS_2_CFG = 1
    PTP = 2
    PTPSYNC = 3
    LIN = 4
    CIRCULAR = 5 


def initial(): 
    global client, sim, jh
    portNb = 23000  # default value

    try:
        # Initialize the CoppeliaSim remote API client
        client = RemoteAPIClient('127.0.0.1', portNb)
        sim = client.require('sim')
        scene_path = os.path.abspath('C:/Users/Saiful/sdir_2025_group6/sim/sdir.ttt')
        print("Connected to CoppeliaSim")

        # Load the scene
        sim.loadScene(scene_path)

        # Get object handles for the robot joints
        jh = [0] * 6
        for i in range(6):
            Joint = '/KR120_2700_2_joint' + str(i + 1)
            jh[i] = sim.getObject(Joint)

        # Start the simulation
        sim.startSimulation()
        print("Simulation Started")

    except Exception as e:
        print(f"Error during initialization: {str(e)}")
        raise e

    return client


def main():
    global client, sim
    print("This is the entry point of the SDIR programming project")
    ctrl = SdirCtrl()  # Create an instance of SdirCtrl
    c = [0.0] * 6

    try:
        initial()  # Connect to CoppeliaSim
    except Exception as e:
        print(f"Error during initialization: {str(e)}, exiting.")
        return

    running = True
    while running:
        signal_value = sim.getStringSignal("callsignal")

        if signal_value is not None and len(signal_value) > 0:
            json_handler = JsonHandler(signal_value)
            mode = json_handler.get_op_mode()
            ik_method = json_handler.get_ik_method()  # NEW: Get selected IK method

            if mode == OpMode.POS_2_CFG:
                # Inverse Kinematics - use selected method
                pos = SixDPos(json_handler.get_data()[0])
                
                print(f"\n{'='*70}")
                print(f"IK Request - Method: {ik_method.upper()}")
                print(f"{'='*70}")
                
                # Get configurations using selected method
                result_cfg = ctrl.get_config_from_pos(pos, method=ik_method)
                
                # Send results back to GUI with method indicator
                json_return_string = json_handler.get_json_string(result_cfg, ik_method=ik_method)
                sim.setStringSignal("returnsignal", json_return_string)
                scriptHandle = sim.getScriptHandle('Coord_Dialog')
                sim.callScriptFunction("returnSignal", scriptHandle)

            if mode == OpMode.CFG_2_POS:
                # Forward Kinematics
                cfg = configuration(json_handler.get_data()[0])
                return_pos = ctrl.get_pos_from_config(cfg)
                json_return_string = json_handler.get_json_string(return_pos)
                sim.setStringSignal("returnsignal", json_return_string)
                scriptHandle = sim.getScriptHandle('Coord_Dialog')
                sim.callScriptFunction("returnSignal", scriptHandle)

            if mode == OpMode.PTP:
                start_cfg = configuration(json_handler.get_data()[0])
                end_cfg = configuration(json_handler.get_data()[1])
                trajectory = ctrl.move_robot_ptp_async(start_cfg, end_cfg)
                for cur_cfg in trajectory.get_all_configuration():
                    c[0] = float(cur_cfg[0])
                    c[1] = float(cur_cfg[1])
                    c[2] = float(cur_cfg[2])
                    c[3] = float(cur_cfg[3])
                    c[4] = float(cur_cfg[4])
                    c[5] = float(cur_cfg[5])
                    script_handle = sim.getScriptHandle('KR120_2700_2')
                    sim.callScriptFunction("runConfig", script_handle, len(c), c, "", "")
                    time.sleep(0.05)

            if mode == OpMode.PTPSYNC:
                start_cfg = configuration(json_handler.get_data()[0])
                end_cfg = configuration(json_handler.get_data()[1])
                trajectory = ctrl.move_robot_ptp(start_cfg, end_cfg)
                for cur_cfg in trajectory.get_all_configuration():
                    c[0] = float(cur_cfg[0])
                    c[1] = float(cur_cfg[1])
                    c[2] = float(cur_cfg[2])
                    c[3] = float(cur_cfg[3])
                    c[4] = float(cur_cfg[4])
                    c[5] = float(cur_cfg[5])
                    script_handle = sim.getScriptHandle('KR120_2700_2')
                    sim.callScriptFunction("runConfig", script_handle, len(c), c, "", "")
                    time.sleep(0.05)

            if mode == OpMode.LIN:
                start_cfg = configuration(json_handler.get_data()[0])
                end_cfg = configuration(json_handler.get_data()[1])
                trajectory = ctrl.move_robot_lin(start_cfg, end_cfg)
                for cur_cfg in trajectory.get_all_configuration():
                    c[0] = float(cur_cfg[0])
                    c[1] = float(cur_cfg[1])
                    c[2] = float(cur_cfg[2])
                    c[3] = float(cur_cfg[3])
                    c[4] = float(cur_cfg[4])
                    c[5] = float(cur_cfg[5])
                    script_handle = sim.getScriptHandle('KR120_2700_2')
                    sim.callScriptFunction("runConfig", script_handle, len(c), c, "", "")
                    time.sleep(0.05)

            sim.clearStringSignal("callsignal")
            time.sleep(0.05)
            
            if mode == OpMode.CIRCULAR:
                start_pos = SixDPos(json_handler.get_data()[0])
                via_pos = SixDPos(json_handler.get_data()[1])
                end_pos = SixDPos(json_handler.get_data()[2])
                trajectory = ctrl.move_robot_circular(start_pos, via_pos, end_pos)
                for cur_cfg in trajectory.get_all_configuration():
                    c[0] = float(cur_cfg[0])
                    c[1] = float(cur_cfg[1])
                    c[2] = float(cur_cfg[2])
                    c[3] = float(cur_cfg[3])
                    c[4] = float(cur_cfg[4])
                    c[5] = float(cur_cfg[5])
                    script_handle = sim.getScriptHandle('KR120_2700_2')
                    sim.callScriptFunction("runConfig", script_handle, len(c), c, "", "")
                    time.sleep(0.05)


if __name__ == "__main__":
    main()