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

import asyncio


try:
    # Third Party
    import isaacsim
except ImportError:
    pass

import torch

# Simulation App Object
from omni.isaac.kit import SimulationApp


# Standard Library
import argparse

parser = argparse.ArgumentParser()
parser.add_argument(
    "--headless_mode",
    type=str,
    default=None,
    help="To run headless, use one of [native, websocket], webrtc might not work.",
)

args = parser.parse_args()

simulation_app = SimulationApp(
    {
        "headless": args.headless_mode is not None,
        "width": "1920",
        "height": "1080",
    }
)

# Standard Library
from typing import Dict

# Third Party
import carb
import numpy as np
from helper import add_extensions, add_robot_to_scene
from omni.isaac.core import World
from omni.isaac.core.objects import cuboid, sphere

########### OV #################
from omni.isaac.core.utils.types import ArticulationAction

# CuRobo
# from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import WorldConfig, Mesh, Cuboid

# from curobo.curobo_types.base import TensorDeviceType
# from curobo.curobo_types.math import Pose
# from curobo.curobo_types.robot import JointState
# from curobo.curobo_types.state import JointState

from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState
from curobo.types.state import JointState

from curobo.util.logger import log_error, setup_curobo_logger
from curobo.util.usd_helper import UsdHelper
from curobo.util_file import (
    get_assets_path,
    get_filename,
    get_path_of_dir,
    get_robot_configs_path,
    get_world_configs_path,
    join_path,
    load_yaml,
)
from curobo.wrap.reacher.motion_gen import (
    MotionGen,
    MotionGenConfig,
    MotionGenPlanConfig,
    PoseCostMetric,
)

############################################################
########### OV #################;;;;;
from omni.isaac.core.articulations import Articulation
import numpy as np
from typing import List, Tuple

import time



def add_meshes_to_world():

    MatTable_3Level = Mesh(
        name="MatTable_3Level",
        pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        file_path="/home/apshirazi/Isaac_sim_ws/meshes/MatTable_3Level.stl",
        scale=[0.001, 0.001, 0.001],
    )
    MatTable_6Level = Mesh(
        name="MatTable_6Level",
        pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        file_path="/home/apshirazi/Isaac_sim_ws/meshes/MatTable_6Level.stl",
        scale=[0.001, 0.001, 0.001],
    )
    MatTable_tilted = Mesh(
        name="MatTable_tilted",
        pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        file_path="/home/apshirazi/Isaac_sim_ws/meshes/MatTable_tilted.stl",
        scale=[0.001, 0.001, 0.001],
    )
    NewConvn1_V2 = Mesh(
        name="NewConvn1_V2",
        pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        file_path="/home/apshirazi/Isaac_sim_ws/meshes/NewConvn1_V2.stl",
        scale=[0.001, 0.001, 0.001],
    )
    NewConvn2_V2 = Mesh(
        name="NewConvn2_V2",
        pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        file_path="/home/apshirazi/Isaac_sim_ws/meshes/NewConvn2_V2.stl",
        scale=[0.001, 0.001, 0.001],
    )
    NewSheathingRack = Mesh(
        name="NewSheathingRack",
        pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        file_path="/home/apshirazi/Isaac_sim_ws/meshes/NewSheathingRack.stl",
        scale=[0.001, 0.001, 0.001],
    )

    SheathingPlate = Cuboid(
        name="SheathingPlate",
        pose=[-1.8, -2.1, 0.37, 0, 0, 0, 1],
        dims=[3, 1.5, 0.015],
        color=[0.87, 0.72, 0.53, 1]
    )

    world_model = WorldConfig(
        mesh=[MatTable_3Level, MatTable_6Level, MatTable_tilted, NewConvn1_V2, NewConvn2_V2, NewSheathingRack, SheathingPlate.get_mesh()],
        cuboid=[],
        capsule=[],
        cylinder=[],
        sphere=[],
    )

    return world_model

Robot_Conf_Path = "/home/apshirazi/Isaac_sim_ws/robot"
Robot_Conf_Name = "IRB6620_Config.yaml"

class Isaac_Robot(object):
        def __init__(self):
            super(Isaac_Robot, self).__init__()
            print("Isaac Robot Created")

            # Starting Isaac Sim World
            self._my_world = World(stage_units_in_meters=1.0)
            self._stage = self._my_world.stage

            self._xform = self._stage.DefinePrim("/World", "Xform")
            self._stage.SetDefaultPrim(self._xform)
            self._stage.DefinePrim("/curobo", "Xform")
            self._stage = self._my_world.stage

            # Starting CuRobo Logger
            setup_curobo_logger("warn")

            self._usd_help = UsdHelper()
            self._tensor_args = TensorDeviceType()

            # Loading Config.yaml
            self._robot_cfg_path = Robot_Conf_Path
            self._robot_cfg = load_yaml(join_path(self._robot_cfg_path, Robot_Conf_Name))["robot_cfg"]

            # Setting up Default Values
            self._overall_j_names = self._robot_cfg["kinematics"]["cspace"]["joint_names"]
            self._default_ret_cfg = self._robot_cfg["kinematics"]["cspace"]["retract_config"]

            # Adding Robot to Scene
            self._robot, self._robot_prim_path = add_robot_to_scene(self._robot_cfg, self._my_world)

            # Creating Articulation Controller
            self._articulation_ctr = None

            # Creating CuRobo World Model !!!
            self._cu_world_model = self.add_mesh_to_world()

            # Creating CuRobo World Config as "Mesh"
            self._cu_world_cfg = WorldConfig(mesh=self._cu_world_model.mesh)

            # Initializing MotionGen
            self.init_curobo()

            # Starting the Simulation
            add_extensions(simulation_app, args.headless_mode)

            # USD Helper Start
            self._usd_help.load_stage(self._my_world.stage)
            self._usd_help.add_world_to_stage(self._cu_world_cfg, base_frame="/World")

            # These Values will be used to Activate ArticulationController
            self._cmd_plan = None
            self._cmd_idx = 0
            
            # Adding Ground to the Isaac Sim Simulation
            self._my_world.scene.add_default_ground_plane()

            # Simulation Timer
            self._spheres = None
            self._pose_metric = None

            # Done Initiating Isaac Sim !!!         

        def add_mesh_to_world(self):
            MatTable_3Level = Mesh(
                name="MatTable_3Level",
                pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                file_path="/home/apshirazi/Isaac_sim_ws/meshes/MatTable_3Level.stl",
                scale=[0.001, 0.001, 0.001],
            )
            MatTable_6Level = Mesh(
                name="MatTable_6Level",
                pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                file_path="/home/apshirazi/Isaac_sim_ws/meshes/MatTable_6Level.stl",
                scale=[0.001, 0.001, 0.001],
            )
            MatTable_tilted = Mesh(
                name="MatTable_tilted",
                pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                file_path="/home/apshirazi/Isaac_sim_ws/meshes/MatTable_tilted.stl",
                scale=[0.001, 0.001, 0.001],
            )
            NewConvn1_V2 = Mesh(
                name="NewConvn1_V2",
                pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                file_path="/home/apshirazi/Isaac_sim_ws/meshes/NewConvn1_V2.stl",
                scale=[0.001, 0.001, 0.001],
            )
            NewConvn2_V2 = Mesh(
                name="NewConvn2_V2",
                pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                file_path="/home/apshirazi/Isaac_sim_ws/meshes/NewConvn2_V2.stl",
                scale=[0.001, 0.001, 0.001],
            )
            NewSheathingRack = Mesh(
                name="NewSheathingRack",
                pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                file_path="/home/apshirazi/Isaac_sim_ws/meshes/NewSheathingRack.stl",
                scale=[0.001, 0.001, 0.001],
            )

            SheathingPlate = Cuboid(
                name="SheathingPlate",
                pose=[-1.8, -2.1, 0.37, 0, 0, 0, 1],
                dims=[3, 1.5, 0.015],
                color=[0.87, 0.72, 0.53, 1]
            )

            world_model = WorldConfig(
                mesh=[MatTable_3Level, MatTable_6Level, MatTable_tilted, NewConvn1_V2, NewConvn2_V2, NewSheathingRack, SheathingPlate.get_mesh()],
                cuboid=[],
                capsule=[],
                cylinder=[],
                sphere=[],
            )

            return world_model

        def init_curobo(self):
            trajopt_dt = None
            optimize_dt = True
            trajopt_tsteps = 32
            trim_steps = None
            interpolation_dt = 0.05

            self._motion_gen_config = MotionGenConfig.load_from_robot_config(
                self._robot_cfg_path+"/"+Robot_Conf_Name,
                self._cu_world_cfg,
                self._tensor_args,
                collision_checker_type=CollisionCheckerType.MESH,
                num_trajopt_seeds=12,
                num_graph_seeds=12,
                interpolation_dt=interpolation_dt,
                optimize_dt=optimize_dt,
                trajopt_dt=trajopt_dt,
                trajopt_tsteps=trajopt_tsteps,
                trim_steps=trim_steps,
            )

            self._motion_gen = MotionGen(self._motion_gen_config)
            self._motion_gen.warmup(enable_graph=True, warmup_js_trajopt=False)
            print("Curobo is Ready")

            self._plan_config = MotionGenPlanConfig(enable_graph=False,
                                                    enable_graph_attempt=1,
                                                    max_attempts=2,
                                                    enable_finetune_trajopt=False,
                                                    time_dilation_factor=0.5,
                                                    timeout=5.0)

            return
    

        def simulation_exec(self):

            # Simulation Step Counter
            i=0
            self.cmd_idx = 0
            while simulation_app.is_running():
                self._my_world.step(render=True)

                # Check if the Play Botton is Executed or Not !
                if not self._my_world.is_playing():
                    if i % 100 == 0:
                        print("**** Click Play to start simulation *****")
                        i += 1
                    continue
                
                step_index = self._my_world.current_time_step_index

                self._articulation_ctr = self._robot.get_articulation_controller()

                # Bringing Robot Back to Retracted Form (If Simulation is Not Running !!!)
                if step_index < 2:
                    self._my_world.reset()
                    self._robot._articulation_view.initialize()
                    idx_list = [self._robot.get_dof_index(x) for x in self._overall_j_names]
                    self._robot.set_joint_positions(self._default_ret_cfg, idx_list)

                    self._robot._articulation_view.set_max_efforts(
                        values=np.array([5000 for i in range(len(idx_list))]), joint_indices=idx_list
                    )
                if step_index < 20:
                    continue

                if step_index == 50 or step_index % 1000 == 0.0:
                    print("Updating world, reading w.r.t.", self._robot_prim_path)
                    obstacles = self._usd_help.get_obstacles_from_stage(
                        reference_prim_path=self._robot_prim_path,
                        ignore_substring=[
                            self._robot_prim_path,
                            "/World/target",
                            "/World/defaultGroundPlane",
                            "/curobo",
                        ],
                    ).get_collision_check_world()

                    # Update CuRobo World !!!
                    self._motion_gen.update_world(obstacles)
                    print("Updated World")
                    carb.log_info("Synced CuRobo world from stage.")

                # Calculating Paths using CuRobo !!!
                Tar1_Q = self.euler_to_quat(-np.pi / 2, 0, -np.pi / 2)
                _Tar_Home: tuple[np.ndarray, np.ndarray] = (
                    np.array([1.9, 0.43, 1.67]),
                    np.array([Tar1_Q[0], Tar1_Q[1], Tar1_Q[2], Tar1_Q[3]])
                )
                # print(_Tar_Home)

                Tar2_Q = self.euler_to_quat(0, 0, np.pi)
                _Tar_Pick: tuple[np.ndarray, np.ndarray] = (
                    np.array([-0.49, -1.54, 0.44]),
                    np.array([Tar2_Q[0], Tar2_Q[1], Tar2_Q[2], Tar2_Q[3]])
                )
                # print(_Tar_Pick)

                Tar3_Q = self.euler_to_quat(np.pi / 2, 0, np.pi)
                _Tar_Place: tuple[np.ndarray, np.ndarray] = (
                    np.array([0, 1.5, 0.08]),
                    np.array([Tar3_Q[0], Tar3_Q[1], Tar3_Q[2], Tar3_Q[3]])
                )
                # print(_Tar_Place)

                self.cu_calc_plan([-0.49, -1.54, 0.44], [Tar2_Q[0], Tar2_Q[1], Tar2_Q[2], Tar2_Q[3]])
    
                

                # self.cu_calc_plan(_Tar_Place)

                # self.cu_calc_plan(_Tar_Home)
            
            # simulation_app.close()
  
        def cu_calc_plan(self, pose, orientation):

            print("starting execution")
            print(pose)
            print("___________________________________")
            print(orientation)
            print("___________________________________")

            sim_js = self._robot.get_joints_state()
            sim_js_names = self._robot.dof_names
            if np.any(np.isnan(sim_js.positions)):
                log_error("isaac sim has returned NAN joint position values.")

            robot_static = False
            if (np.max(np.abs(sim_js.velocities)) < 0.2):
                robot_static = True
            if (robot_static):
                # Setting Up Current Pose (JointState)
                cu_js = JointState(
                    position=self._tensor_args.to_device(sim_js.positions),
                    velocity=self._tensor_args.to_device(sim_js.velocities),# * 0.0,
                    acceleration=self._tensor_args.to_device(sim_js.velocities) * 0.0,
                    jerk=self._tensor_args.to_device(sim_js.velocities) * 0.0,
                    joint_names=sim_js_names,
                )

                cu_js = cu_js.get_ordered_joint_state(self._motion_gen.kinematics.joint_names)

                # Skipping Sphere Visualization

                # Setting Target Pose (Cartesian Pose of TCP)
                ik_goal = Pose(
                    position=self._tensor_args.to_device(pose),
                    quaternion=self._tensor_args.to_device(orientation),
                )

                self._plan_config.pose_cost_metric = self._pose_metric

                # Returning the Executed Plan
                returning_plan = self._motion_gen.plan_single(cu_js.unsqueeze(0), ik_goal, self._plan_config)

                if returning_plan.success.item():
                    cmd_plan = returning_plan.get_interpolated_plan()
                    cmd_plan = self._motion_gen.get_full_js(cmd_plan)

                    # get only joint names that are in both:
                    idx_list = []
                    common_js_names = []
                    for x in sim_js_names:
                        if x in cmd_plan.joint_names:
                            idx_list.append(self._robot.get_dof_index(x))
                            common_js_names.append(x)

                    cmd_plan = cmd_plan.get_ordered_joint_state(common_js_names)

                    cmd_idx = 0

                    if cmd_plan is not None:
                        cmd_state = cmd_plan[cmd_idx]
                        past_cmd = cmd_state.clone()
                        # get full dof state
                        art_action = ArticulationAction(
                            cmd_state.position.cpu().numpy(),
                            cmd_state.velocity.cpu().numpy(),
                            joint_indices=idx_list,
                        )
                        # set desired joint angles obtained from IK:
                        self._articulation_ctr.apply_action(art_action)

                        cmd_idx += 1
                        for _ in range(2):
                            self._my_world.step(render=False)
                        if cmd_idx >= len(cmd_plan.position):
                            cmd_idx = 0
                            cmd_plan = None
                            past_cmd = None
                    else:
                        carb.log_warn("Plan did not converge to a solution: " + str(returning_plan.status))
                
        def __del__(self):
            print("Simulation Done")
            if not simulation_app.is_running():
                simulation_app.close()
            else:
                print("Close the Simulation Manually")

        def euler_to_quat(self, roll, pitch, yaw):
            cy = np.cos(yaw * 0.5)
            sy = np.sin(yaw * 0.5)
            cp = np.cos(pitch * 0.5)
            sp = np.sin(pitch * 0.5)
            cr = np.cos(roll * 0.5)
            sr = np.sin(roll * 0.5)

            q_w = cr * cp * cy + sr * sp * sy
            q_x = sr * cp * cy - cr * sp * sy
            q_y = cr * sp * cy + sr * cp * sy
            q_z = cr * cp * sy - sr * sp * cy

            return q_x, q_y, q_z, q_w


def main():
    # Creating Isaac Robot Instance
    IDC_Lab = Isaac_Robot()

    # Executing Simulation
    IDC_Lab.simulation_exec()


if __name__ == "__main__":
    main()