import math
import time

import numpy as np
import pybullet as p
import pybullet_data

save_vid = True


def step_simulation(steps):
    """Steps the simulation multiple times."""
    for _ in range(steps):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)


physicsClient = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  # Disable default GUI elements
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # To load URDFs

if save_vid:
    video_filename = "simulation_cobotta2.mp4"
    p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, video_filename)
    print(f"Recording simulation video to {video_filename}")
# Set gravity and simulation parameters
p.setGravity(0, 0, -9.81)
p.setTimeStep(1.0 / 240.0)
p.setRealTimeSimulation(0)  # Ensure we control the simulation steps

# Enable more accurate physics - helps with stability
p.setPhysicsEngineParameter(numSolverIterations=50)  # Increased from default
p.setPhysicsEngineParameter(numSubSteps=4)  # Increased from default

# Load plane
planeId = p.loadURDF("plane.urdf")

# --- Environment Objects ---

# Source Region (Grey)
source_region_half_size = [0.1, 0.1, 0.005]  # x, y, z half extents
source_position = [0.3, -0.1, source_region_half_size[2]]  # Centered at z=0.005
source_visual = p.createVisualShape(
    shapeType=p.GEOM_BOX,
    halfExtents=source_region_half_size,
    rgbaColor=[0.5, 0.5, 0.5, 1],
)
source_collision = p.createCollisionShape(
    shapeType=p.GEOM_BOX, halfExtents=source_region_half_size
)
source_area_id = p.createMultiBody(
    baseMass=0,  # Static object
    baseCollisionShapeIndex=source_collision,
    baseVisualShapeIndex=source_visual,
    basePosition=source_position,
)

# Target Region (Brown)
target_region_half_size = [0.1, 0.1, 0.005]  # Same size as source
target_position = [
    0.3,
    0.2,
    target_region_half_size[2],
]  # Displaced from source
target_visual = p.createVisualShape(
    shapeType=p.GEOM_BOX,
    halfExtents=target_region_half_size,
    rgbaColor=[0.55, 0.27, 0.07, 1],
)  # Brown
target_collision = p.createCollisionShape(
    shapeType=p.GEOM_BOX, halfExtents=target_region_half_size
)
target_area_id = p.createMultiBody(
    baseMass=0,  # Static object
    baseCollisionShapeIndex=target_collision,
    baseVisualShapeIndex=target_visual,
    basePosition=target_position,
)

# Object to Pick (Orange Box)
box_half_size = [0.01, 0.01, 0.01]  # Smaller than gripper opening hopefully
box_initial_position = [
    source_position[0],
    source_position[1],
    source_region_half_size[2] * 2 + box_half_size[2] + 0.01,
]  # Start slightly above source region surface
box_visual = p.createVisualShape(
    shapeType=p.GEOM_BOX, halfExtents=box_half_size, rgbaColor=[1, 0.647, 0, 1]
)  # Orange
box_collision = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=box_half_size)
box_id = p.createMultiBody(
    baseMass=0.1,  # Reduced mass for better stability
    baseCollisionShapeIndex=box_collision,
    baseVisualShapeIndex=box_visual,
    basePosition=box_initial_position,
)

# Set friction for box to prevent it from sliding easily
p.changeDynamics(
    box_id, -1, lateralFriction=1.0, spinningFriction=0.1, rollingFriction=0.1
)

# Allow the box to settle
print("Letting box settle...")
step_simulation(200)  # More steps for better settling
box_settled_position, _ = p.getBasePositionAndOrientation(box_id)
print(f"Box settled at: {box_settled_position}")


# --- Load Robot (Cobotta) ---
robot_start_pos = [0, 0, 0.0]
robot_start_orn = p.getQuaternionFromEuler([0, 0, 0])
try:
    robotId = p.loadURDF(
        "cobotta.urdf", robot_start_pos, robot_start_orn, useFixedBase=True
    )
    print("Loaded cobotta_description/urdf/cobotta.urdf")
except p.error:
    try:
        robotId = p.loadURDF(
            "cobotta.urdf", robot_start_pos, robot_start_orn, useFixedBase=True
        )
        print("Loaded cobotta.urdf")
    except p.error as e:
        print("\nError loading Cobotta URDF!")
        print(
            "Please ensure 'cobotta.urdf' or 'cobotta_description/urdf/cobotta.urdf' exists"
        )
        print("and is accessible in the PyBullet search path.")
        print(f"PyBullet error: {e}")
        p.disconnect()
        exit()


p.resetBasePositionAndOrientation(robotId, robot_start_pos, robot_start_orn)

# --- Robot Configuration ---
numJoints = p.getNumJoints(robotId)
print("-" * 20)
print(f"Number of joints: {numJoints}")
print("Joint Info:")
active_joint_indices = []
for i in range(numJoints):
    info = p.getJointInfo(robotId, i)
    print(
        f"Joint {i}: Name: {info[1].decode('utf-8')}, Type: {info[2]}, Link: {info[12].decode('utf-8')}"
    )
    # Filter out fixed joints
    if info[2] != p.JOINT_FIXED:
        active_joint_indices.append(i)
print(f"Active (non-fixed) joint indices: {active_joint_indices}")
print("-" * 20)

# Check the joint info printout above to find the correct indices
end_effector_link_index = 6  # Often the link index for 'tool0' or similar flange/tip
gripper_joint_indices = [7, 8]  # Often the indices for the finger joints

# Check URDF joint limits or experiment
gripper_open_positions = [0.04, 0.04]  # Target positions for open fingers
gripper_closed_positions = [
    0.01,
    0.01,
]  # Slightly open to match box size (prevent pushing)
gripper_force = 20  # Reduced force to avoid excessive impacts

# Improve the gripper dynamics
for joint_idx in gripper_joint_indices:
    p.changeDynamics(
        robotId,
        joint_idx,
        lateralFriction=1.0,
        spinningFriction=0.1,
        rollingFriction=0.1,
        contactStiffness=10000,
        contactDamping=100,
    )

# Initial robot pose (optional, move to a neutral starting position)
initial_joint_angles = [0] * len(active_joint_indices)  # Adjust as needed
p.setJointMotorControlArray(
    robotId,
    jointIndices=active_joint_indices,
    controlMode=p.POSITION_CONTROL,
    targetPositions=initial_joint_angles,
)
step_simulation(100)  # Let robot settle into initial pose

# Set camera view
p.resetDebugVisualizerCamera(
    cameraDistance=1.0,
    cameraYaw=50,
    cameraPitch=-35,
    cameraTargetPosition=[0.3, 0.05, 0.1],  # Focus on the workspace
)

# --- Robot Control Functions ---

# Define a standard orientation (e.g., gripper pointing straight down)
# Tool frame's Z should point down world Z. Often requires rotating pi around Y or X.
down_orientation_euler = [0, math.pi, 0]  # Rotate 180 degrees around Y
down_orientation_quat = p.getQuaternionFromEuler(down_orientation_euler)


def move_arm(
    robot_id,
    target_position,
    target_orientation=down_orientation_quat,
    simulation_steps=120,
    max_velocity=0.5,
):
    """Moves the robot's end effector to a target position and orientation using IK."""
    print(
        f"Moving arm to Position: {target_position}, Orientation: {target_orientation}"
    )

    # Calculate inverse kinematics for all active joints
    # Use current joint poses as the initial guess for IK
    current_joint_poses = [
        p.getJointState(robot_id, i)[0] for i in active_joint_indices
    ]

    joint_poses = p.calculateInverseKinematics(
        robot_id,
        end_effector_link_index,
        target_position,
        targetOrientation=target_orientation,
        currentPositions=current_joint_poses,  # Providing current pose helps convergence
        maxNumIterations=100,
        residualThreshold=1e-5,  # More precise
    )

    # Get more accurate end effector position for distance calculation
    current_ee_pos = p.getLinkState(robot_id, end_effector_link_index)[0]
    distance = np.linalg.norm(np.array(current_ee_pos) - np.array(target_position))

    # Calculate necessary steps based on distance for smooth movement
    # Longer distances need more steps
    adjusted_steps = max(simulation_steps, int(distance / max_velocity * 240))

    # Command the robot joints with position control
    p.setJointMotorControlArray(
        robot_id,
        jointIndices=active_joint_indices[
            : len(joint_poses)
        ],  # Ensure we only control the arm joints IK solves for
        controlMode=p.POSITION_CONTROL,
        targetPositions=joint_poses[: len(active_joint_indices)],
        forces=[50]
        * len(
            joint_poses[: len(active_joint_indices)]
        ),  # Reduced force for smoother motion
    )

    # Simulate for calculated duration to allow the arm to reach the target
    step_simulation(adjusted_steps)

    # Check final position and report
    final_pos = p.getLinkState(robot_id, end_effector_link_index)[0]
    final_distance = np.linalg.norm(np.array(final_pos) - np.array(target_position))
    print(f"Arm reached: {final_pos}, distance to target: {final_distance:.6f}")
    return final_distance < 0.005  # Return True if within 5mm of target


def control_gripper(
    robot_id, target_positions, force=gripper_force, simulation_steps=60
):
    """Controls the gripper joints to target positions."""
    if len(target_positions) != len(gripper_joint_indices):
        print(
            f"Error: Mismatch between target positions ({len(target_positions)}) and gripper joints ({len(gripper_joint_indices)})"
        )
        return

    print(
        f"Setting gripper joints {gripper_joint_indices} to positions {target_positions}"
    )
    p.setJointMotorControlArray(
        robot_id,
        jointIndices=gripper_joint_indices,
        controlMode=p.POSITION_CONTROL,
        targetPositions=target_positions,
        forces=[force] * len(gripper_joint_indices),
    )
    # Simulate to allow gripper to move
    step_simulation(simulation_steps)


def open_gripper(robot_id):
    """Opens the gripper."""
    print("Opening gripper...")
    control_gripper(robot_id, gripper_open_positions)


def close_gripper(robot_id):
    """Closes the gripper."""
    print("Closing gripper...")
    control_gripper(robot_id, gripper_closed_positions)


def is_object_grasped(robot_id, object_id):
    """Check if the object is between gripper fingers."""
    # Get current gripper finger positions
    left_finger_state = p.getJointState(robot_id, gripper_joint_indices[0])
    right_finger_state = p.getJointState(robot_id, gripper_joint_indices[1])

    # Get object position
    obj_pos, obj_orn = p.getBasePositionAndOrientation(object_id)

    # Get end effector position
    ee_pos = p.getLinkState(robot_id, end_effector_link_index)[0]

    # Check distance between object and end effector
    distance = np.linalg.norm(np.array(obj_pos) - np.array(ee_pos))

    # Check if fingers are closed enough but not fully closed (i.e., object is between them)
    gripper_closed = all(
        p.getJointState(robot_id, idx)[0] < gripper_open_positions[i]
        for i, idx in enumerate(gripper_joint_indices)
    )
    gripper_not_fully_closed = any(
        p.getJointState(robot_id, idx)[0] > gripper_closed_positions[i] / 2
        for i, idx in enumerate(gripper_joint_indices)
    )

    # Object should be close to end effector and gripper should be partially closed
    is_grasped = distance < 0.05 and gripper_closed and gripper_not_fully_closed

    print(
        f"Grasp check - Distance: {distance:.4f}, Gripper closed: {gripper_closed}, "
        f"Not fully closed: {gripper_not_fully_closed}, Is grasped: {is_grasped}"
    )

    return is_grasped


# --- Pick and Place Sequence ---

grasp_constraint_id = None  # Variable to hold the grasp constraint

try:
    # 1. Define key positions based on actual box position
    box_pos, box_orn = p.getBasePositionAndOrientation(box_id)  # Use settled position
    pick_approach_height = 0.10  # How far above the object to approach
    pick_height = (
        box_pos[2] + 0.005
    )  # Target Z for grasping (slightly above box center)
    lift_height = 0.15  # How high to lift the object after grasping
    place_target_z = (
        target_position[2] + target_region_half_size[2] + box_half_size[2]
    )  # Z surface of target + box height
    place_approach_height = 0.10  # How far above the placement Z to approach

    pick_approach_pos = [
        box_pos[0],
        box_pos[1],
        box_pos[2] + pick_approach_height,
    ]
    pick_grasp_pos = [box_pos[0], box_pos[1], pick_height]
    pick_lift_pos = [box_pos[0], box_pos[1], box_pos[2] + lift_height]

    place_approach_pos = [
        target_position[0],
        target_position[1],
        place_target_z + place_approach_height,
    ]
    place_down_pos = [
        target_position[0],
        target_position[1],
        place_target_z + 0.005,
    ]  # Place slightly above surface
    place_retreat_pos = [
        target_position[0],
        target_position[1],
        place_target_z + place_approach_height,
    ]  # Same as approach

    # 2. Open Gripper
    open_gripper(robotId)

    # 3. Move to approach position above the object
    move_arm(robotId, pick_approach_pos)

    # 4. Move down to grasping position - slower approach
    move_arm(
        robotId,
        [box_pos[0], box_pos[1], box_pos[2] + 0.05],
        simulation_steps=120,
    )  # Intermediate position
    success = move_arm(
        robotId, pick_grasp_pos, simulation_steps=200
    )  # Very slow final approach

    # Re-check box position as it might have moved slightly
    box_pos, box_orn = p.getBasePositionAndOrientation(box_id)
    print(f"Box position before grasping: {box_pos}")

    # 5. Close Gripper to grasp - slow closing
    close_gripper(robotId)
    step_simulation(30)  # Let physics settle after closing

    # 6. Check if the grasp was successful
    if is_object_grasped(robotId, box_id):
        print("Successfully grasped the object")

        # Create constraint only if grasp was successful
        grasp_constraint_id = p.createConstraint(
            parentBodyUniqueId=robotId,
            parentLinkIndex=end_effector_link_index,
            childBodyUniqueId=box_id,
            childLinkIndex=-1,  # -1 refers to the base of the child body
            jointType=p.JOINT_FIXED,  # Fixed constraint
            jointAxis=[0, 0, 0],  # Not used for JOINT_FIXED
            parentFramePosition=[
                0,
                0,
                0.015,
            ],  # Attach slightly below end effector
            childFramePosition=[
                0,
                0,
                0,
            ],  # Attach at the center of the box base
        )
        print(f"Created constraint ID: {grasp_constraint_id}")
        step_simulation(30)  # Allow constraint to activate

        # 7. Lift the object
        move_arm(robotId, pick_lift_pos)

        # 8. Move to approach position above the target location
        move_arm(robotId, place_approach_pos)

        # 9. Move down to place the object - slow approach
        move_arm(robotId, place_down_pos, simulation_steps=200)  # Slower/shorter move

        # 10. Release the object (remove constraint)
        if grasp_constraint_id is not None:
            print(f"Removing constraint ID: {grasp_constraint_id}")
            p.removeConstraint(grasp_constraint_id)
            grasp_constraint_id = None
        step_simulation(30)  # Allow object to potentially settle slightly

        # 11. Open Gripper
        open_gripper(robotId)
        step_simulation(60)  # Let object settle after release

        # 12. Retreat upwards slowly
        move_arm(robotId, place_retreat_pos, simulation_steps=150)

        # Check final box position
        final_box_pos, _ = p.getBasePositionAndOrientation(box_id)
        print(f"Final box position: {final_box_pos}")

        # 13. Move to a final neutral position (optional)
        move_arm(robotId, [0.2, 0, 0.3])
        print("\nPick and place sequence complete.")
    else:
        print("Failed to grasp the object. Aborting sequence.")
        # Move back up
        move_arm(robotId, pick_approach_pos)

except Exception as e:
    print(f"\nAn error occurred during the pick and place sequence: {e}")
    import traceback

    traceback.print_exc()

finally:
    # Keep simulation running or disconnect
    print("\nSimulation finished. Press Ctrl+C or close window to exit.")
    # Keep running for visual confirmation - step manually
    while p.isConnected():
        # Let user interact or just watch
        keys = p.getKeyboardEvents()
        # Add any key checks if needed, e.g., to reset or exit
        p.stepSimulation()  # Step simulation forward
        time.sleep(1.0 / 240.0)


# Disconnect when done (might not be reached if loop is infinite)
if save_vid:
    p.stopStateLogging(p.STATE_LOGGING_VIDEO_MP4)
p.disconnect()
