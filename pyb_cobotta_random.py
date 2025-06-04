import os
import time
import xml.etree.ElementTree as ET  # To parse limits from URDF

import numpy as np
import pybullet as p
import pybullet_data

# --- Configuration ---

try:
    script_dir = os.path.dirname(__file__) if "__file__" in locals() else os.getcwd()
    urdf_file = os.path.join(script_dir, "cobotta.urdf")
except Exception:
    print("\n--- Failed to load URDF ---")
use_gui = False  # Set to False to run without visualization
save_vid = False
simulation_steps = 2500  # How long to run the simulation


# --- Helper Function to Extract Joint Info ---
def get_joint_info_from_urdf(filename):
    """Parses the URDF file to get movable joint names, limits, and indices."""
    try:
        tree = ET.parse(filename)
        root = tree.getroot()
    except FileNotFoundError:
        print(f"Error: URDF file not found at {filename}")
        return None
    except ET.ParseError:
        print(f"Error: Could not parse URDF file {filename}")
        return None

    movable_joints = {}
    # Iterate through joints in the URDF
    for joint_element in root.findall("joint"):
        joint_name = joint_element.get("name")
        print(f"joint name: {joint_name}")
        joint_type = joint_element.get("type")
        print(f"joint type: {joint_type}")

        # Consider only revolute and prismatic joints that are NOT mimic joints
        if (
            joint_type in ["revolute", "prismatic"]
            and joint_element.find("mimic") is None
        ):
            limit_element = joint_element.find("limit")
            if limit_element is not None:
                try:
                    lower = float(limit_element.get("lower", -np.inf))
                    upper = float(limit_element.get("upper", np.inf))
                    velocity = float(
                        limit_element.get("velocity", 1.0)
                    )  # Default if not specified
                    movable_joints[joint_name] = {
                        "lower": lower,
                        "upper": upper,
                        "velocity": velocity,
                    }
                except (TypeError, ValueError):
                    print(
                        f"Warning: Could not parse limits for joint '{joint_name}'. Skipping."
                    )
            else:
                # Handle continuous joints or joints without explicit limits if needed
                if joint_type == "revolute":
                    print(
                        f"Info: Joint '{joint_name}' is revolute but has no limits tag. Assuming continuous."
                    )
                    # Assign arbitrary wide limits for random generation, or handle appropriately
                    movable_joints[joint_name] = {
                        "lower": -np.pi * 2,
                        "upper": np.pi * 2,
                        "velocity": 1.0,
                    }
                else:  # Prismatic without limits doesn't make much sense for random motion
                    print(
                        f"Warning: Joint '{joint_name}' ({joint_type}) has no limits tag. Skipping."
                    )

    return movable_joints


# --- PyBullet Simulation ---

# Start PyBullet
if use_gui:
    physicsClient = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  # Disable default GUI elements
    # Improve visual fidelity slightly
    # p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
    # p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    if save_vid:
        video_filename = "simulation_cobotta.mp4"
        p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, video_filename)
        print(f"Recording simulation video to {video_filename}")

else:
    physicsClient = p.connect(p.DIRECT)

# Add search path for PyBullet's standard assets (like plane)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set gravity
p.setGravity(0, 0, -9.81)

# Load ground plane
planeId = p.loadURDF("plane.urdf")

# Load the robot URDF
robot_start_pos = [0, 0, 0]
robot_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
print(f"Loading URDF: {urdf_file}")
try:
    robotId = p.loadURDF(
        urdf_file, robot_start_pos, robot_start_orientation, useFixedBase=1
    )
    print(f"Robot loaded with ID: {robotId}")
    # Add this camera configuration:
    p.resetDebugVisualizerCamera(
        cameraDistance=0.8,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=robot_start_pos,
    )
except Exception as e:
    print("\n--- Failed to load URDF ---")
    print(f"Error: {e}")
    print("Possible reasons:")
    print("1. URDF file path is incorrect.")
    print("2. Mesh files (.dae) are missing or not found.")
    print("   - Check if the 'package://' paths need modification (see script notes).")
    print(
        "   - Ensure mesh files exist relative to the URDF or in pybullet's search path."
    )
    print("3. The URDF file has syntax errors not caught visually.")
    p.disconnect()
    exit()


# --- Get Joint Information from PyBullet and URDF ---
num_joints = p.getNumJoints(robotId)
pybullet_joint_indices = []
joint_name_to_info = {}
joint_limits = {}

print("\n--- Robot Joints Found by PyBullet ---")
for i in range(num_joints):
    info = p.getJointInfo(robotId, i)
    joint_id = info[0]
    joint_name = info[1].decode("utf-8")
    joint_type = info[2]
    joint_lower_limit = info[8]
    joint_upper_limit = info[9]
    joint_max_force = info[10]
    joint_max_velocity = info[11]

    # Store info mapping PyBullet index to name and properties
    joint_name_to_info[joint_name] = {
        "id": joint_id,
        "type": joint_type,
        "pybullet_lower": joint_lower_limit,
        "pybullet_upper": joint_upper_limit,
        "max_force": joint_max_force,
        "max_velocity": joint_max_velocity,
    }
    print(
        f"Index: {joint_id}, Name: {joint_name}, Type: {joint_type}, Limits: [{joint_lower_limit:.2f}, {joint_upper_limit:.2f}]"
    )

    # Keep track of non-fixed joints that PyBullet can control
    if joint_type != p.JOINT_FIXED:
        pybullet_joint_indices.append(joint_id)


# --- Get Limits from URDF for better accuracy ---
urdf_joint_limits = get_joint_info_from_urdf(urdf_file)

if urdf_joint_limits:
    print("\n--- Movable Joints Parsed from URDF for Randomization ---")
    # Refine the list of joints to move based on URDF parsing
    movable_joint_indices = []
    for joint_name, limits in urdf_joint_limits.items():
        if joint_name in joint_name_to_info:
            pb_info = joint_name_to_info[joint_name]
            # Use URDF limits as they are often more specific than pybullet's defaults
            joint_limits[pb_info["id"]] = {
                "lower": limits["lower"],
                "upper": limits["upper"],
            }
            movable_joint_indices.append(pb_info["id"])
            print(
                f"Using URDF limits for '{joint_name}' (ID: {pb_info['id']}): [{limits['lower']:.4f}, {limits['upper']:.4f}]"
            )
        else:
            print(
                f"Warning: Joint '{joint_name}' from URDF not found in PyBullet model."
            )
else:
    print(
        "\nWarning: Could not parse URDF for limits. Using PyBullet's reported limits."
    )
    movable_joint_indices = pybullet_joint_indices  # Fallback to all non-fixed joints
    for idx in movable_joint_indices:
        info = p.getJointInfo(robotId, idx)
        joint_limits[idx] = {"lower": info[8], "upper": info[9]}


if not movable_joint_indices:
    print("\nError: No movable joints found to control!")
    p.disconnect()
    exit()

print(f"\nControlling {len(movable_joint_indices)} joints.")

# --- Simulation Loop ---
p.setRealTimeSimulation(0)  # Use stepSimulation, not real-time

target_positions = None
change_target_timer = 0
change_interval = 200  # Change target every N simulation steps (approx)

print("\nStarting simulation. Robot will move to random poses.")
for i in range(simulation_steps):
    # --- Set Random Target ---
    if i % change_interval == 0:
        print(f"Step {i}: Setting new random target positions.")
        target_positions = []
        target_indices = []
        for joint_index in movable_joint_indices:
            if joint_index in joint_limits:
                limits = joint_limits[joint_index]
                # Ensure lower limit is strictly less than upper limit before sampling
                if limits["lower"] < limits["upper"]:
                    random_pos = np.random.uniform(
                        low=limits["lower"], high=limits["upper"]
                    )
                    target_positions.append(random_pos)
                    target_indices.append(joint_index)
                else:
                    # Handle cases where limits might be equal or invalid
                    print(
                        f"Warning: Invalid or zero-range limits for joint {joint_index} [{limits['lower']}, {limits['upper']}]. Using current position."
                    )
                    current_state = p.getJointState(robotId, joint_index)
                    target_positions.append(current_state[0])  # Keep current position
                    target_indices.append(joint_index)

        if target_positions:
            # Method 1: Set positions directly (good for kinematic control)
            # p.setJointMotorControlArray(
            #     bodyUniqueId=robotId,
            #     jointIndices=target_indices,
            #     controlMode=p.POSITION_CONTROL,
            #     targetPositions=target_positions,
            #     # Optionally set forces/velocities if needed
            #     # forces=[joint_name_to_info[p.getJointInfo(robotId, idx)[1].decode('utf-8')]['max_force'] for idx in target_indices],
            #     # targetVelocities=[0.0] * len(target_indices) # Target zero velocity at the target position
            # )

            # Method 2: Using setJointMotorControl2 for individual control (more flexible)
            for idx, pos in zip(target_indices, target_positions):
                max_force = joint_name_to_info[
                    p.getJointInfo(robotId, idx)[1].decode("utf-8")
                ].get("max_force", 50)  # Default force if needed
                p.setJointMotorControl2(
                    bodyIndex=robotId,
                    jointIndex=idx,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=pos,
                    force=max_force,  # Apply sufficient force to reach the target
                    # positionGain=0.03, # Optional: Tune PID gains if needed
                    # velocityGain=1
                )

    # --- Step Simulation ---
    p.stepSimulation()

    # --- Optional: Add Delay ---
    if use_gui:
        time.sleep(
            1.0 / 240.0
        )  # Slow down for visualization (240 Hz is default physics)

print("\nSimulation finished.")

# Disconnect from PyBullet
if use_gui:
    if save_vid:
        p.stopStateLogging(p.STATE_LOGGING_VIDEO_MP4)
p.disconnect()
