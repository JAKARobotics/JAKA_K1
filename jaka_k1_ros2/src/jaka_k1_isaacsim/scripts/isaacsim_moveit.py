# -*- coding: utf-8 -*-

import os
from pathlib import Path
from isaacsim import SimulationApp

K1_STAGE_PATH = "/World/jaka_k1"
GRAPH_PATH = "/ActionGraph"
CONFIG = {"renderer": "RayTracedLighting", "headless": False}

simulation_app = SimulationApp(CONFIG)

from isaacsim.core.api import SimulationContext
from isaacsim.core.utils import extensions
from omni.usd import get_context
from pxr import Sdf
import omni.graph.core as og

# extensions.enable_extension("isaacsim.code_editor.vscode")
# extensions.enable_extension("omni.kit.debug.vscode")

def _resolve_usd():
    p = os.environ.get("K1_USD_PATH", "")
    if p:
        return p
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory("jaka_k1_description")
        return str(Path(pkg_share) / "k1-de/urdf/jaka_k1/jaka_k1.usd")
    except Exception:
        return str(Path(__file__).resolve().parents[2] /
                   "jaka_k1_description/k1-de/urdf/jaka_k1/jaka_k1.usd")

K1_USD_PATH = _resolve_usd()

print(f"🚀 Isaac Sim starting, loading USD: {K1_USD_PATH}")

if not get_context().open_stage(f"file://{K1_USD_PATH}"):
    raise RuntimeError(f"Failed to open stage: {K1_USD_PATH}")

print("Stage Opened")

k1_prim = get_context().get_stage().GetPrimAtPath(K1_STAGE_PATH)
print("Prim found:", bool(k1_prim) and k1_prim.IsValid())

if not k1_prim or not k1_prim.IsValid():
    raise RuntimeError(f"Prim not found: {K1_STAGE_PATH}")

# list children (quick view of links)
print("Children names under /jaka_k1:")
for c in k1_prim.GetChildren():
    print("  -", c.GetName(), c.GetTypeName())

simulation_context = SimulationContext(stage_units_in_meters=1.0)

# Initialize the stage first, then enable the ROS extensions before starting the simulation.
extensions.enable_extension("isaacsim.ros2.bridge")
extensions.enable_extension("isaacsim.ros2.urdf")
extensions.enable_extension("isaacsim.ros2.tf_viewer")

# The saved graph is loaded before the ROS bridge extension, so its ROS nodes are not registered as OmniGraph nodes. 
# Recreate it now that the bridge is active.
stage = get_context().get_stage()
stage.RemovePrim(GRAPH_PATH)
keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
            ("Context.outputs:context", "PublishClock.inputs:context"),
            ("Context.outputs:context", "PublishJointState.inputs:context"),
            ("Context.outputs:context", "SubscribeJointState.inputs:context"),
            ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
            ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
            ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
            ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
        ],
        keys.SET_VALUES: [
            ("PublishJointState.inputs:targetPrim", [Sdf.Path(f"{K1_STAGE_PATH}/world")]),
            ("PublishJointState.inputs:topicName", "isaac_joint_states"),
            ("SubscribeJointState.inputs:topicName", "isaac_joint_commands"),
            ("ArticulationController.inputs:robotPath", K1_STAGE_PATH),
        ],
    },
)

# Run app update for multiple frames to re-initialize the ROS action graph after setting new prim inputs
simulation_app.update()
simulation_app.update()

simulation_context.play()
simulation_app.update()

while simulation_app.is_running():

    # Run with a fixed step size
    simulation_context.step(render=True)

simulation_context.stop()
simulation_app.close()
