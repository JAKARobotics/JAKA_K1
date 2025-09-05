# -*- coding: utf-8 -*-

import os
from pathlib import Path
from omni.isaac.kit import SimulationApp

K1_STAGE_PATH = "/World/jaka_k1"
GRAPH_PATH = "/ActionGraph"
CONFIG = {"renderer": "RayTracedLighting", "headless": False}

simulation_app = SimulationApp(CONFIG)

from isaacsim.core.api import SimulationContext
from isaacsim.core.utils import extensions
from omni.usd import get_context
from pxr import Usd

extensions.enable_extension("isaacsim.ros2.bridge")
extensions.enable_extension("isaacsim.ros2.urdf")
extensions.enable_extension("isaacsim.ros2.tf_viewer")
extensions.enable_extension("isaacsim.code_editor.vscode")
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

# print(f"Resolved USD path: {K1_USD_PATH}, exists={os.path.exists(K1_USD_PATH)}")

# print(f"🚀 Isaac Sim starting, loading USD: {K1_USD_PATH}")

# k1_stage = get_context().open_stage(K1_USD_PATH)
# # if not isinstance(k1_stage, Usd.Stage):
# #     raise RuntimeError("Failed to open stage.")

# print("Stage Opened")

# k1_prim = get_context().get_stage().GetPrimAtPath(K1_STAGE_PATH)
# print("Prim found:", bool(k1_prim) and k1_prim.IsValid())

# if not k1_prim or not k1_prim.IsValid():
#     raise RuntimeError(f"Prim not found: {K1_STAGE_PATH}")

# # list children (quick view of links)
# print("Children names under /jaka_k1:")
# for c in k1_prim.GetChildren():
#     print("  -", c.GetName(), c.GetTypeName())


# sanity
print(f"Resolved USD path: {K1_USD_PATH}, exists={os.path.exists(K1_USD_PATH)}")
if not os.path.exists(K1_USD_PATH):
    raise RuntimeError(f"USD not found: {K1_USD_PATH}")

# request the stage open (do NOT use file:// prefix here)
ctx = get_context()
ctx.open_stage(K1_USD_PATH)    # non-blocking / asynchronous in many Kit builds


import time 

# poll until the stage is actually available and has the expected root
stage = None
basename = os.path.basename(K1_USD_PATH)
timeout = 10.0   # seconds
t0 = time.time()
while time.time() - t0 < timeout:
    # allow Kit to process events and background loading
    simulation_app.update()   # important: gives the app a chance to finish stage load
    stage = ctx.get_stage()
    if isinstance(stage, Usd.Stage):
        root = stage.GetRootLayer().realPath or ""
        # sometimes realPath may be empty; if so, also check GetSessionLayer etc
        if basename in root or stage.GetSessionLayer().identifier == K1_USD_PATH or stage.GetRootLayer().identifier == K1_USD_PATH:
            break
    time.sleep(0.05)

if not isinstance(stage, Usd.Stage):
    # helpful debug info before failing
    print("DEBUG: ctx.get_stage() ->", ctx.get_stage())
    raise RuntimeError(f"Stage never finished loading in {timeout}s: {K1_USD_PATH!r}")

print("Stage opened successfully:", stage.GetRootLayer().realPath)


simulation_context = SimulationContext(stage_units_in_meters=1.0)

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