#!/usr/bin/env python3
"""
IsaacSim standalone script: KHI Duaro (WD002N) dual-arm SCARA robot + ROS2 bridge.

Loads the Duaro URDF, creates OmniGraph nodes for ROS2 communication
(joint states, TF, clock), sets initial joint positions, and runs the
simulation loop with web streaming for remote visualization.

Usage:
    /isaac-sim/python.sh isaac_sim_duaro.py [--urdf PATH]
"""

import argparse
import math
import os
import signal
import sys

# Ensure ROS2 bridge can find its internal Jazzy libs
os.environ.setdefault("ROS_DISTRO", "jazzy")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
_bridge_lib = "/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib"
if _bridge_lib not in os.environ.get("LD_LIBRARY_PATH", ""):
    os.environ["LD_LIBRARY_PATH"] = _bridge_lib + ":" + os.environ.get("LD_LIBRARY_PATH", "")

# Remove system ROS2 Python 3.12 paths so the bridge's internal
# Python 3.11 rclpy loads instead of the incompatible system one.
sys.path = [p for p in sys.path if "python3.12" not in p]

import numpy as np
from isaacsim import SimulationApp

CONFIG = {
    "renderer": "RaytracedLighting",
    "headless": True,
    "hide_ui": False,
    "width": 1280,
    "height": 720,
    "window_width": 1920,
    "window_height": 1080,
    "display_options": 3286,
}
simulation_app = SimulationApp(CONFIG)

# --- Imports only available after SimulationApp is created ---
import omni.graph.core as og
import omni.kit.commands
import usdrt.Sdf
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils import extensions, stage, viewports
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics

# Enable web streaming (livestream)
simulation_app.set_setting("/app/window/drawMouse", True)
extensions.enable_extension("omni.services.livestream.nvcf")

# Enable ROS2 bridge extension
extensions.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# ============================================================
# Configuration
# ============================================================

parser = argparse.ArgumentParser()
parser.add_argument(
    "--urdf", default="/root/duaro_ws/duaro_isaac.urdf",
    help="Path to the generated Duaro URDF",
)
# Isaac Sim passes extra args; ignore unknown
args, _ = parser.parse_known_args()
URDF_PATH = args.urdf

# 8 active joints: 6 revolute + 2 prismatic (J3)
REVOLUTE_JOINTS = [
    "lower_joint1", "lower_joint2", "lower_joint4",
    "upper_joint1", "upper_joint2", "upper_joint4",
]
PRISMATIC_JOINTS = [
    "lower_joint3",
    "upper_joint3",
]
ALL_ACTIVE_JOINTS = [
    "lower_joint1", "lower_joint2", "lower_joint3", "lower_joint4",
    "upper_joint1", "upper_joint2", "upper_joint3", "upper_joint4",
]

# Home positions from SRDF (ROS1 khi_duaro.srdf)
# lower: [-45°, 45°, 0.09m, 0°]  upper: [45°, -45°, 0.09m, 0°]
INITIAL_JOINT_POS = {
    "lower_joint1": math.radians(-45.0),   # -0.7853 rad
    "lower_joint2": math.radians(45.0),    #  0.7853 rad
    "lower_joint3": 0.09,                  #  0.09 m (prismatic)
    "lower_joint4": 0.0,                   #  0.0 rad
    "upper_joint1": math.radians(45.0),    #  0.7853 rad
    "upper_joint2": math.radians(-45.0),   # -0.7853 rad
    "upper_joint3": 0.09,                  #  0.09 m (prismatic)
    "upper_joint4": 0.0,                   #  0.0 rad
}

# ============================================================
# Scene setup
# ============================================================

simulation_context = SimulationContext(stage_units_in_meters=1.0)

# Camera: Duaro is ~1.07m base (0.82m cabinet + 0.2515m J0) + arms
viewports.set_camera_view(
    eye=np.array([1.8, 1.5, 1.8]),
    target=np.array([0.0, 0.0, 0.9]),
)

# --- Default grid environment ---
def _timeout_handler(signum, frame):
    raise TimeoutError("get_assets_root_path timed out")


try:
    signal.signal(signal.SIGALRM, _timeout_handler)
    signal.alarm(10)
    assets_root_path = get_assets_root_path()
    signal.alarm(0)
    if assets_root_path is not None:
        stage.add_reference_to_stage(
            assets_root_path + "/Isaac/Environments/Grid/default_environment.usd",
            "/environment",
        )
        print(f"Loaded default environment from {assets_root_path}", flush=True)
except Exception as e:
    signal.alarm(0)
    print(f"Warning: Could not load default environment: {e}", flush=True)
    print("Creating local ground plane instead...", flush=True)
    from isaacsim.core.api.objects import GroundPlane
    GroundPlane(prim_path="/environment/groundPlane", z_position=0.0, size=10.0)


# ============================================================
# Import Duaro URDF
# ============================================================

from isaacsim.asset.importer.urdf._urdf import UrdfJointTargetType

status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
import_config.merge_fixed_joints = False
import_config.convex_decomp = False
import_config.import_inertia_tensor = True
import_config.fix_base = True  # Duaro base is fixed to world
import_config.distance_scale = 1.0
import_config.default_drive_type = UrdfJointTargetType.JOINT_DRIVE_POSITION
import_config.default_drive_strength = 5e3
import_config.default_position_drive_damping = 500.0

print(f"Importing Duaro URDF: {URDF_PATH}", flush=True)
status, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=URDF_PATH,
    import_config=import_config,
    get_articulation_root=True,
)
if not status:
    print("ERROR: URDF import failed!", flush=True)
    simulation_app.close()
    sys.exit(1)

print(f"Robot imported at: {prim_path}", flush=True)
ROBOT_ROOT = prim_path.rsplit("/", 1)[0] if "/" in prim_path else prim_path
ARTICULATION_ROOT = prim_path
print(f"Robot root: {ROBOT_ROOT}, Articulation root: {ARTICULATION_ROOT}", flush=True)
simulation_app.update()

# ============================================================
# Physics scene
# ============================================================

usd_stage = omni.usd.get_context().get_stage()

scene = UsdPhysics.Scene.Define(usd_stage, Sdf.Path("/physicsScene"))
scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
scene.CreateGravityMagnitudeAttr().Set(9.81)

physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(usd_stage.GetPrimAtPath("/physicsScene"))
physx_scene_api.CreateEnableCCDAttr(True)
physx_scene_api.CreateEnableStabilizationAttr(True)
physx_scene_api.CreateEnableGPUDynamicsAttr(False)
physx_scene_api.CreateBroadphaseTypeAttr("MBP")
physx_scene_api.CreateSolverTypeAttr("TGS")

# Lighting
distant_light = UsdLux.DistantLight.Define(usd_stage, Sdf.Path("/DistantLight"))
distant_light.CreateIntensityAttr(500)

simulation_app.update()

# ============================================================
# Configure joint drives
# ============================================================

print("Configuring joint drives...", flush=True)

for joint_name in REVOLUTE_JOINTS:
    joint_prim = usd_stage.GetPrimAtPath(f"{ROBOT_ROOT}/joints/{joint_name}")
    if joint_prim.IsValid():
        drive = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
        if drive:
            drive.GetStiffnessAttr().Set(5e3)
            drive.GetDampingAttr().Set(500.0)
            if not drive.GetMaxForceAttr():
                drive.CreateMaxForceAttr(1e3)
            else:
                drive.GetMaxForceAttr().Set(1e3)
            print(f"  Revolute: {joint_name} (stiffness=5e3, damping=500)", flush=True)
    else:
        print(f"  Warning: joint not found: {ROBOT_ROOT}/joints/{joint_name}", flush=True)

for joint_name in PRISMATIC_JOINTS:
    joint_prim = usd_stage.GetPrimAtPath(f"{ROBOT_ROOT}/joints/{joint_name}")
    if joint_prim.IsValid():
        drive = UsdPhysics.DriveAPI.Get(joint_prim, "linear")
        if drive:
            drive.GetStiffnessAttr().Set(1e4)
            drive.GetDampingAttr().Set(1e3)
            if not drive.GetMaxForceAttr():
                drive.CreateMaxForceAttr(1e3)
            else:
                drive.GetMaxForceAttr().Set(1e3)
            print(f"  Prismatic: {joint_name} (stiffness=1e4, damping=1e3)", flush=True)
    else:
        print(f"  Warning: joint not found: {ROBOT_ROOT}/joints/{joint_name}", flush=True)

# ============================================================
# Set initial joint positions via drive targets
# ============================================================

print("Setting initial joint positions...", flush=True)

for joint_name in REVOLUTE_JOINTS:
    pos_rad = INITIAL_JOINT_POS[joint_name]
    joint_prim = usd_stage.GetPrimAtPath(f"{ROBOT_ROOT}/joints/{joint_name}")
    if joint_prim.IsValid():
        drive = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
        if drive:
            # USD uses degrees for angular drives
            drive.GetTargetPositionAttr().Set(math.degrees(pos_rad))

for joint_name in PRISMATIC_JOINTS:
    pos_m = INITIAL_JOINT_POS[joint_name]
    joint_prim = usd_stage.GetPrimAtPath(f"{ROBOT_ROOT}/joints/{joint_name}")
    if joint_prim.IsValid():
        drive = UsdPhysics.DriveAPI.Get(joint_prim, "linear")
        if drive:
            # USD uses cm for linear drives (stage_units_in_meters=1.0 but PhysX linear drives use cm)
            drive.GetTargetPositionAttr().Set(pos_m * 100.0)

for jn, pos in INITIAL_JOINT_POS.items():
    unit = "m" if jn in PRISMATIC_JOINTS else "rad"
    print(f"  {jn}: {pos:.4f} {unit}", flush=True)

simulation_app.update()

# ============================================================
# ROS2 OmniGraph
# ============================================================

try:
    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnImpulseEvent.outputs:execOut", "PublishJointState.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "SubscribeJointState.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "ArticulationController.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "PublishTF.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "PublishClock.inputs:execIn"),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("Context.outputs:context", "PublishTF.inputs:context"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                (
                    "SubscribeJointState.outputs:jointNames",
                    "ArticulationController.inputs:jointNames",
                ),
                (
                    "SubscribeJointState.outputs:positionCommand",
                    "ArticulationController.inputs:positionCommand",
                ),
                (
                    "SubscribeJointState.outputs:velocityCommand",
                    "ArticulationController.inputs:velocityCommand",
                ),
                (
                    "SubscribeJointState.outputs:effortCommand",
                    "ArticulationController.inputs:effortCommand",
                ),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ArticulationController.inputs:robotPath", ARTICULATION_ROOT),
                ("PublishJointState.inputs:topicName", "joint_states_raw"),
                ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(ARTICULATION_ROOT)]),
                ("SubscribeJointState.inputs:topicName", "joint_commands"),
                ("PublishTF.inputs:topicName", "tf"),
                ("PublishTF.inputs:targetPrims", [usdrt.Sdf.Path(ROBOT_ROOT)]),
            ],
        },
    )
    print("OmniGraph created successfully", flush=True)
except Exception as e:
    print(f"ERROR creating OmniGraph: {e}", flush=True)
    import traceback
    traceback.print_exc()

simulation_app.update()

# ============================================================
# Run simulation
# ============================================================

simulation_context.initialize_physics()
simulation_context.play()

# Let the robot settle to initial positions
for _ in range(120):
    simulation_context.step(render=True)

print("=" * 60, flush=True)
print("Duaro (WD002N) simulation running.", flush=True)
print("", flush=True)
print("ROS2 topics:", flush=True)
print("  Publishing:   /joint_states_raw, /tf, /clock", flush=True)
print("  Subscribing:  /joint_commands", flush=True)
print("", flush=True)
print("Active joints (6 revolute + 2 prismatic):", flush=True)
print("  lower_joint1 (rev), lower_joint2 (rev), lower_joint3 (prism), lower_joint4 (rev)", flush=True)
print("  upper_joint1 (rev), upper_joint2 (rev), upper_joint3 (prism), upper_joint4 (rev)", flush=True)
print("", flush=True)
print("Web streaming enabled (livestream). Connect via browser.", flush=True)
print("Press Ctrl+C to stop.", flush=True)
print("=" * 60, flush=True)

while simulation_app.is_running():
    simulation_context.step(render=True)
    og.Controller.set(
        og.Controller.attribute("/ActionGraph/OnImpulseEvent.state:enableImpulse"),
        True,
    )

simulation_context.stop()
simulation_app.close()
