# SDIR Programming Project

This project is designed for the SDIR programming project using CoppeliaSim and Python. It communicates with CoppeliaSim through the ZMQ Remote API.

## Prerequisites

Make sure you have the following installed:

- **CoppeliaSim** 

- **Python Package**: coppeliasim-zmqremoteapi-client

## Running the Code

- **CoppeliaSim**: Open CoppeliaSim.

- **Run the Python Code**: run the main.py file.

## Usage
The code listens for a signal (callsignal) from CoppeliaSim and performs actions based on the specified operation mode.

Operation Modes:

- **OpMode.POS_2_CFG**: Convert position to configurations using inverse kinematics.
- **OpMode.CFG_2_POS**: Convert a configuration to position using direct kinematics.
- **OpMode.PTP**: Move the robot using Point-to-Point motion.
- **OpMode.PTPSYNC**: Move the robot using Point-to-Point motion with synchronization.
- **OpMode.LIN**: Move the robot using linear motion.

## Additional Information

- The project assumes that the scene file ('**sdir.ttt**') is in '**sim**' folder. Modify the path in main.py if needed.
