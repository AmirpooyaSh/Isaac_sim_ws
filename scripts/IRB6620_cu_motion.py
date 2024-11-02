#
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
# property and proprietary rights in and to this material, related
# documentation and any modifications thereto. Any use, reproduction,
# disclosure or distribution of this material and related documentation
# without an express license agreement from NVIDIA CORPORATION or
# its affiliates is strictly prohibited.
#


try:
    # Third Party
    import isaacsim
except ImportError:
    pass


# Third Party
import torch

a = torch.zeros(
    4, device="cuda:0"
)  # this is necessary to allow isaac sim to use this torch instance
# Third Party
import numpy as np

np.set_printoptions(suppress=True)
# Standard Library

# Standard Library
import argparse

## import curobo:

parser = argparse.ArgumentParser()

parser.add_argument(
    "--headless_mode",
    type=str,
    default=None,
    help="To run headless, use one of [native, websocket], webrtc might not work.",
)

parser.add_argument(
    "--constrain_grasp_approach",
    action="store_true",
    help="When True, approaches grasp with fixed orientation and motion only along z axis.",
    default=False,
)
args = parser.parse_args()

# Third Party
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp(
    {
        "headless": args.headless_mode is not None,
        "width": "1920",
        "height": "1080",
    }
)
# Standard Library
from typing import Optional

# Third Party
import carb
from helper import add_extensions
from omni.isaac.core import World
from omni.isaac.core.controllers import BaseController
from omni.isaac.core.tasks import Stacking as BaseStacking
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.core.utils.string import find_unique_string_name
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.franka import Franka

# CuRobo
from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.sphere_fit import SphereFitType
from curobo.geom.types import WorldConfig, Mesh
from curobo.rollout.rollout_base import Goal
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState
from curobo.types.state import JointState
from curobo.util.usd_helper import UsdHelper
from curobo.util_file import get_robot_configs_path, get_world_configs_path, join_path, load_yaml
from curobo.wrap.reacher.motion_gen import (
    MotionGen,
    MotionGenConfig,
    MotionGenPlanConfig,
    MotionGenResult,
    PoseCostMetric,
)

parser.add_argument("--robot", type=str, default="IRB6620_Config.yaml", help="robot configuration to load")

parser.add_argument(
    "--constrain_grasp_approach",
    action="store_true",
    help="When True, approaches grasp with fixed orientation and motion only along z axis.",
    default=False,
)
args = parser.parse_args()

def add_meshes_to_world():

    MatTable_3Level = Mesh(
        name="MatTable_3Level",
        pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        file_path="/catkin_ws/src/IDC_CuRobo_Project/meshes/MatTable_3Level.stl",
        scale=[0.001, 0.001, 0.001],
    )
    MatTable_6Level = Mesh(
        name="MatTable_6Level",
        pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        file_path="/catkin_ws/src/IDC_CuRobo_Project/meshes/MatTable_6Level.stl",
        scale=[0.001, 0.001, 0.001],
    )
    MatTable_tilted = Mesh(
        name="MatTable_tilted",
        pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        file_path="/catkin_ws/src/IDC_CuRobo_Project/meshes/MatTable_tilted.stl",
        scale=[0.001, 0.001, 0.001],
    )
    NewConvn1_V2 = Mesh(
        name="NewConvn1_V2",
        pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        file_path="/catkin_ws/src/IDC_CuRobo_Project/meshes/NewConvn1_V2.stl",
        scale=[0.001, 0.001, 0.001],
    )
    NewConvn2_V2 = Mesh(
        name="NewConvn2_V2",
        pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        file_path="/catkin_ws/src/IDC_CuRobo_Project/meshes/NewConvn2_V2.stl",
        scale=[0.001, 0.001, 0.001],
    )
    NewSheathingRack = Mesh(
        name="NewSheathingRack",
        pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        file_path="/catkin_ws/src/IDC_CuRobo_Project/meshes/NewSheathingRack.stl",
        scale=[0.001, 0.001, 0.001],
    )

    world_model = WorldConfig(
        mesh=[MatTable_3Level, MatTable_6Level, MatTable_tilted, NewConvn1_V2, NewConvn2_V2, NewSheathingRack],
        cuboid=[],
        capsule=[],
        cylinder=[],
        sphere=[],
    )

    return world_model

class IRB6620(BaseController):
    def __init__(self, 
                 my_world, 
                 name: str = "R1_Controller", 
                 constrain_grasp_approach: bool = False
        ) -> None:
        BaseController.__init__(self, name=name)
        self._save_log = False
        self.my_world = my_world
        self._step_idx = 0
        n_obstacle_cuboids = 20
        n_obstacle_mesh = 2
        # warmup curobo instance
        self.usd_help = UsdHelper()
        self.init_curobo = False
        self.world_file = "IRB6620_Config.yaml"
        self.cmd_js_names = [
            "Joint_1",
            "Joint_2",
            "Joint_3",
            "Joint_4",
            "Joint_5",
            "Joint_6",
            "Joint_8",
        ]

        self.tensor_args = TensorDeviceType()
        robot_path = "/catkin_ws/src/IDC_CuRobo_Project/robot"
        self.robot_cfg = load_yaml(join_path(robot_path, args.robot))["robot_cfg"]
        # self.robot_cfg["kinematics"][
        #     "base_link"
        # ] = "base_link"  # controls which frame the controller is controlling

        # self.robot_cfg["kinematics"][
        #     "ee_link"
        # ] = "tool0"  # controls which frame the controller is controlling

        # self.robot_cfg["kinematics"]["cspace"]["max_acceleration"] = 10.0 # controls how fast robot moves
        self.robot_cfg["kinematics"]["extra_collision_spheres"] = {"attached_object": 100}
        # @self.robot_cfg["kinematics"]["collision_sphere_buffer"] = 0.0

        self._world_model = add_meshes_to_world()
        
        self._world_cfg = WorldConfig(mesh=self._world_model.get_mesh_world().mesh)

        motion_gen_config = MotionGenConfig.load_from_robot_config(
            self.robot_cfg,
            self._world_cfg,
            self.tensor_args,
            trajopt_tsteps=32,
            collision_checker_type=CollisionCheckerType.MESH,
            use_cuda_graph=True,
            interpolation_dt=0.05,
            collision_cache={"obb": n_obstacle_cuboids, "mesh": n_obstacle_mesh},
            store_ik_debug=self._save_log,
            store_trajopt_debug=self._save_log,
        )

        self.motion_gen = MotionGen(motion_gen_config)
        print("IRB6620_R1 warm up ...")
        self.motion_gen.warmup(parallel_finetune=True)
        pose_metric = None
        if constrain_grasp_approach:
            pose_metric = PoseCostMetric.create_grasp_approach_metric(
                offset_position=0.1, tstep_fraction=0.8
            )

        self.plan_config = MotionGenPlanConfig(
            enable_graph=False,
            max_attempts=5,
            enable_graph_attempt=None,
            enable_finetune_trajopt=True,
            partial_ik_opt=False,
            parallel_finetune=True,
            pose_cost_metric=pose_metric,
            time_dilation_factor=0.75,
        )
        self.usd_help.load_stage(self.my_world.stage)
        self.cmd_plan = None
        self.cmd_idx = 0
        self._step_idx = 0
        self.idx_list = None

def main():
    robot_prim_path = "/World/IRB6620/base_link"
    ignore_substring = ["IRB6620", "TargetCube", "material", "Plane"]
    my_world = World(stage_units_in_meters=1.0)
    stage = my_world.stage
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

    

if __name__ == "__main__":
    main()