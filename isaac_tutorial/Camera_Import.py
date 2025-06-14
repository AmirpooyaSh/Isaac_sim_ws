"""
rgbd_camera_ros_publisher.py
--------------------------------
Programmatically creates a single RGB-D camera in Isaac Sim and publishes
RGB, depth, CameraInfo, TF, and depth-derived point-cloud topics over ROS 1.

Usage
-----
1. Start `roscore` in a separate terminal.
2. Launch Isaac Sim, or run this script head-less with `./python.sh rgbd_camera_ros_publisher.py`.
3. In another terminal check the topics:

       rostopic list
       rostopic hz /rgbd_camera_rgb
       rostopic echo /tf

Prerequisites
-------------
* Isaac Sim ≥ 4.1 with the `omni.isaac.ros_bridge` extension installed
* ROS 1 Noetic (or ROS Melodic) sourced in the same shell
* The environment variable `$ISAACSIM_ROOT` pointing at your Isaac Sim install
"""

# -------------------------------------------------
# 1. Imports & SimulationApp bootstrap
# -------------------------------------------------
import carb
from isaacsim import SimulationApp

CONFIG = {"renderer": "RayTracedLighting", "headless": False}
simulation_app = SimulationApp(CONFIG)          # must be constructed before any omni imports

# -------------------------------------------------
# 2. Omniverse / Isaac-Sim modules
# -------------------------------------------------
import omni
import numpy as np
import omni.graph.core as og
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd

from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import stage, extensions
from omni.isaac.sensor import Camera
import omni.isaac.core.utils.numpy.rotations as rot_utils
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core_nodes.scripts.utils import set_target_prims

# -------------------------------------------------
# 3. Enable ROS bridge *before* using any ROS helpers
# -------------------------------------------------
extensions.enable_extension("omni.isaac.ros_bridge")

# -------------------------------------------------
# 4. Check that roscore is running (optional guard)
# -------------------------------------------------
import rosgraph
if not rosgraph.is_master_online():
    carb.log_error("roscore is not running – please start it before executing this script.")
    simulation_app.close()
    raise SystemExit

# -------------------------------------------------
# 5. Create a simple world + RGB-D camera
# -------------------------------------------------
BACKGROUND_STAGE_PATH = "/background"
BACKGROUND_USD_PATH   = "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"

simulation_context = SimulationContext(stage_units_in_meters=1.0)

# assets_root_path = nucleus.get_assets_root_path()
# if assets_root_path is None:
#     carb.log_error("Could not locate Isaac Sim assets.")
#     simulation_app.close()
#     raise SystemExit

# stage.add_reference_to_stage(assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH)

camera = Camera(
    prim_path="/World/rgbd_camera",
    position=np.array([-3.0, -2.0, 1.0]),       # [m] world frame (Z-up)
    orientation=rot_utils.euler_angles_to_quats(np.array([0, 0, 0]), degrees=True),
    resolution=(640, 480),
    frequency=30                                 # [Hz] acquisition rate
)
camera.initialize()
simulation_app.update()

# -------------------------------------------------
# 6. Helper functions to wire ROS publishers
# -------------------------------------------------
def _set_gate_step(render_product: str, step_size: int, render_var: str):
    """Utility to throttle publishers to (60 / step_size) Hz."""
    gate_path = omni.syntheticdata.SyntheticData._get_node_path(
        render_var + "IsaacSimulationGate", render_product
    )
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)

def publish_camera_tf(cam: Camera):
    """Publish `/tf` frames for the camera (world → camera, and camera → camera_world)."""
    frame = cam.prim_path.split("/")[-1]
    graph_path = "/CameraTFActionGraph"
    if not is_prim_path_valid(graph_path):
        og.Controller.edit(
            {"graph_path": graph_path,
             "evaluator_name": "execution",
             "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION},
            {og.Controller.Keys.CREATE_NODES: [
                 ("OnTick", "omni.graph.action.OnTick"),
                 ("IsaacClock", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                 ("ROSClockPub", "omni.isaac.ros_bridge.ROS1PublishClock")
             ],
             og.Controller.Keys.CONNECT: [
                 ("OnTick.outputs:tick", "ROSClockPub.inputs:execIn"),
                 ("IsaacClock.outputs:simulationTime", "ROSClockPub.inputs:timeStamp"),
             ]}
        )

    og.Controller.edit(
        graph_path,
        {og.Controller.Keys.CREATE_NODES: [
             (f"TF_{frame}",         "omni.isaac.ros_bridge.ROS1PublishTransformTree"),
             (f"TF_{frame}_world",   "omni.isaac.ros_bridge.ROS1PublishRawTransformTree"),
         ],
         og.Controller.Keys.SET_VALUES: [
             (f"TF_{frame}.inputs:topicName", "/tf"),
             (f"TF_{frame}_world.inputs:topicName", "/tf"),
             (f"TF_{frame}_world.inputs:parentFrameId",   frame),
             (f"TF_{frame}_world.inputs:childFrameId",    f"{frame}_world"),
             # static rot: ROS camera → world  (quaternion [w,x,y,z])
             (f"TF_{frame}_world.inputs:rotation", [0.5, -0.5, 0.5, 0.5]),
         ],
         og.Controller.Keys.CONNECT: [
             (f"{graph_path}/OnTick.outputs:tick",        f"TF_{frame}.inputs:execIn"),
             (f"{graph_path}/OnTick.outputs:tick",        f"TF_{frame}_world.inputs:execIn"),
             (f"{graph_path}/IsaacClock.outputs:simulationTime",
                                                      f"TF_{frame}.inputs:timeStamp"),
             (f"{graph_path}/IsaacClock.outputs:simulationTime",
                                                      f"TF_{frame}_world.inputs:timeStamp"),
         ]}
    )
    set_target_prims(f"{graph_path}/TF_{frame}", "inputs:targetPrims", [cam.prim_path])

def _attach_writer(cam: Camera, sensor_type: sd.SensorType, writer_suffix: str,
                   topic: str, freq: int):
    """Generic helper to attach a Replicator ROS1 writer to a render product."""
    render_product = cam._render_product_path
    step_size      = int(60 / freq)
    frame_id       = cam.prim_path.split("/")[-1]

    rv      = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sensor_type.name)
    writer  = rep.writers.get(rv + writer_suffix)
    writer.initialize(
        frameId=frame_id,
        nodeNamespace="",
        queueSize=1,
        topicName=topic
    )
    writer.attach([render_product])
    _set_gate_step(render_product, step_size, rv)

def publish_camera_info(cam: Camera, freq: int = 30):
    render_product = cam._render_product_path
    step_size      = int(60 / freq)
    frame_id       = cam.prim_path.split("/")[-1]

    writer = rep.writers.get("ROS1PublishCameraInfo")
    writer.initialize(frameId=frame_id,
                      nodeNamespace="",
                      queueSize=1,
                      topicName=f"{cam.name}_camera_info",
                      stereoOffset=[0.0, 0.0])
    writer.attach([render_product])
    _set_gate_step(render_product, step_size, "PostProcessDispatch")

def publish_rgb(cam: Camera, freq: int = 30):
    _attach_writer(cam, sd.SensorType.Rgb, "ROS1PublishImage",
                   topic=f"{cam.name}_rgb", freq=freq)

def publish_depth(cam: Camera, freq: int = 30):
    _attach_writer(cam, sd.SensorType.DistanceToImagePlane, "ROS1PublishImage",
                   topic=f"{cam.name}_depth", freq=freq)

def publish_pointcloud(cam: Camera, freq: int = 5):
    _attach_writer(cam, sd.SensorType.DistanceToImagePlane, "ROS1PublishPointCloud",
                   topic=f"{cam.name}_pointcloud", freq=freq)

# -------------------------------------------------
# 7. Wire everything up
# -------------------------------------------------
DESIRED_RATE = 30          # [Hz] for RGB/Depth/CameraInfo
publish_camera_tf(camera)
publish_camera_info(camera, DESIRED_RATE)
publish_rgb(camera, DESIRED_RATE)
publish_depth(camera, DESIRED_RATE)
publish_pointcloud(camera, freq=5)      # lower rate for point-clouds to save bandwidth

# -------------------------------------------------
# 8. Run the simulation
# -------------------------------------------------
simulation_context.initialize_physics()
simulation_context.play()

while simulation_app.is_running():
    simulation_context.step(render=True)

simulation_context.stop()
simulation_app.close()
