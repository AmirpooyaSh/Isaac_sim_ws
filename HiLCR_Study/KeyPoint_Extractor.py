###########################
###########################
### Importing Libraries ###
###########################
###########################

from curobo.wrap.reacher import motion_gen
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

from omni.isaac.core import World
from omni.isaac.core.objects import cuboid, sphere
import numpy as np
import math

# Adding mesh to the world (Standalone Format)
from omni.physx.scripts import utils #It can be used to create Fixed Joints
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot

# This is a CuRobo Library that Converts world_model to USD format for Isaac Sim
from curobo.util.usd_helper import UsdHelper
# This is a CuRobo Library to import Different Object Types to the world model
from curobo.geom.types import WorldConfig, Mesh, Cuboid, Sphere
# This is a CuRobo Library to create Tensors 
from curobo.types.base import TensorDeviceType
# These are CuRobo Libraries to Read/Write Robot Information
from curobo.util_file import (
    get_assets_path,
    get_filename,
    get_path_of_dir,
    get_robot_configs_path,
    get_world_configs_path,
    join_path,
    load_yaml,
)
from curobo.util.logger import log_error, setup_curobo_logger
# MotionGen CuRobo
from curobo.wrap.reacher.motion_gen import (
    MotionGen,
    MotionGenConfig,
    MotionGenPlanConfig,
    MotionGenResult,
    PoseCostMetric,
)
from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import WorldConfig, Mesh, Cuboid, Cylinder, Sphere
from curobo.types.math import Pose
from curobo.types.robot import JointState
from curobo.types.state import JointState
from curobo.geom.sphere_fit import SphereFitType
from curobo.types.robot import RobotConfig
from curobo.util.logger import log_error, log_info, log_warn


import carb

# CuRobo helper library to create scene !!!
from helper import add_extensions, add_robot_to_scene

# ArticulationAction is used to control Robots ArticulationController (You can move them by using apply_action() command)
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.controllers.articulation_controller import ArticulationController
from omni.isaac.core.prims.xform_prim import XFormPrim
import omni.isaac.core.utils.prims as prims_utils
from omni.isaac.core.robots import Robot
from omni.isaac.core.scenes.scene import Scene

# Joint Creation
from omni.physx.scripts import utils

from omni.isaac.dynamic_control import _dynamic_control

# This is used to make VScode understand ArticulationController's type (which is fed from the robot privately)
from typing import cast, List, Tuple, Optional, Any

from omni.isaac.core.utils.extensions import enable_extension
# enable ROS bridge extension
enable_extension("omni.isaac.ros_bridge")

# enable SurfaceGripper extention
enable_extension("omni.isaac.surface_gripper")
# enable measurement extenstion
enable_extension("omni.kit.tool.measure")
simulation_app.update()

# check if rosmaster node is running
# this is to prevent this sample from waiting indefinetly if roscore is not running
# can be removed in regular usage
import rosgraph

if not rosgraph.is_master_online():
    carb.log_error("Please run roscore before executing this script")
    simulation_app.close()
    exit()

import rospy
import sensor_msgs
import sensor_msgs.msg

import omni.graph.core as og
import omni.kit.actions.core

# Xform Creation and Transform
import omni.kit.commands as cmd
# Adding UsdPhysics to Add Mass To the Object
from pxr import Gf, Usd, Sdf, UsdGeom, UsdPhysics, UsdLux

import omni.usd
# Creating SurfaceGripper OmniGraph
import omni.graph.core as og

from omni.isaac.surface_gripper import SurfaceGripper

from omni.kit.commands import execute

import re
import time
import torch
import threading

# Quat Transform
from scipy.spatial.transform import Rotation as R

### ABB RWS Implementation
import abb_motion_program_exec as ABB
from abb_robot_client.rws import JointTarget

# AI Agent Imports
import os, sys, json
from typing import List, Dict
from openai import OpenAI        # pip install openai>=1.14

### RGBD Imports
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd
from omni.isaac.sensor import Camera
import omni.isaac.core.utils.numpy.rotations as rot_utils
from omni.isaac.core_nodes.scripts.utils import set_target_prims
from omni.isaac.core.utils.prims import is_prim_path_valid
from math import tan, radians

import json, io, cv2
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge

from PIL import Image
from io import BytesIO
import base64
import re, ast


###########################
###########################
### Importing Libraries ###
###########################
###########################

############################
############################
### Global Parameters ######
############################
############################

# Setting Controller IP (Virtual Controller as of Now)
client = ABB.MotionProgramExecClient(base_url="http://192.168.1.14")
SIM_TO_CONTROLLER_COMMUNICATION: bool = False

# RATE of PUBLISHING JOINTSTATES
publish_rate = 10.0  # Frequency in Hz

# EV NUMBER FOR QUATERNION ORIENTATION
ev = np.sqrt(2) / 2

# DIRECTORY OF INSTALLATION
INSTALLATION_DIRECTORY: str = "/home/apshirazi"

# Robot Gripper Lengths (In CM) The Most Accuracte Numbers Captured From the CAD Model
ROBOT_1_GRIPPER_LENGTH: float = 0.600202
ROBOT_1_SUCTION_LENGTH: float = 0.549999565226
ROBOT_1_SUCTION_WIDTH: float = 0.359199802554
ROBOT_1_SUCTION_CUP_R: float = 0.035000000104
ROBOT_1_SUCTION_CUP_TO_TOOL_OFFSET: float = 0.03

ROBOT_2_GRIPPER_LENGTH: float = 0.590042
# Robotic Movement Accelaration
MOTION_ACCELERAION_VALUE: float = 1.0
STATION_SPEED: ABB.speeddata = ABB.v1000

SMART_CONV_RANGE_OF_MOTION_J1: float = 4.55
SMART_CONV_RANGE_OF_MOTION_J2: float = 0.5
## Test: Reducing it by 10CM
SMART_CONV_REST_ELEVATION: float = 0.89546
SMART_CONV_MODEL_ON_GROUND_HEIGHT: float = 0.89546
# This shift is required to satisfy the symmetry of the smart conveyor platform
SMART_CONV_X_SHIFT: float = 0.09179

# Smart Material Table ZEORO
SMART_MAT_TABLE: list[float] = [6.444, 4.611, 0.803]
STUD_TO_SAW_OFFSET: float = 0.12093

# Offset Required to Pick Woods From the Table
PICK_OFFSET_FROM_L_CORNER: float = 0.05
PICK_OFFSET_FROM_L_CORNER_AFTER_PASS: float = 0.2
PICK_OFFSET_FROM_W_CORNER: float = 0.0061

#Elevation in which passing TPL is being done
PASSING_ELEVATION: float = 0.85

JACK_PLACEMENT_SIDE_DRAG: float = 0.05
# This represents the angle in which Robot 1 will nail the Jacks to the Kings ! (20 To 45 is a good range)
JACK_SIDE_NAILING_ANGLE: float = 30
JACK_NAILING_OFFSET: float = 0.1

SILL_NAILING_ANGLE: float = 30
SILL_NAILING_OFFSET: float = 0.1

BR_NAILING_ANGLE: float = 45
BR_NAILING_OFFSET: float = 0.1

# This Angle is For the L_U Pass Angle Between Robots 1 and 2
L_U_PASS_ANGLE: float = 30

# Smart Material Table's Maximum Length Capability
SMART_MAT_TABLE_MAX_LENGTH: float = 3.6576


#### SLOPED MATERIAL TABLE
SLOPED_MAT_TABLE: list[float] = [4.009, -2.417, 1.045]
SLOPED_MAT_TABLE_ANGLE: float = 30
# The length offset for picking the wood from the sloped table
PRE_PICK_OFFSET = 0.2

#Nailing Push for 1st/Repetitive/King Nailing
PUSH_TO_NAIL_OFFSET: float = 0.05

#Nailing Push for JACK
PUSH_TO_NAIL_OFFSET_TANGENT: float = 0.01

SLOPED_TABLE_PICK_OFFSET_FROM_L_CORNER: float = 0.17
SLOPED_TABLE_PICK_OFFSET_FROM_W_CORNER: float = 0.0061

### Small Cut Table
# Upon Reloating the Table, The Offset Should be Adjusted Accordingly
SMALL_CUT_TABLE_SAW_POSE: list[float] = [1, 1, 0.5]

# Sheathing Plate Table ( Used to Pick Sheathing Plates )
SHEATHING_PLATE_TABLE_BOTTOM_CENTER: list[float] = [0, -1.2, 0.6]
OFFSET_FROM_SHEATHING_PLATE_TABLE_BOT: float = 0.3

# Bear Loading Pile Stand Information
NUMBER_OF_HEADERS: int = 5
# 2in x 10in 
# 1 Meters Length Timbers Stacked !
RAW_HEADER_DIMENSIONS: list[float] = [1, 0.0508, 0.254]
HEADER_CENTER_COORINATION: list[float] = [-1, 1.5, 0.1]
HEADER_PICK_OFFSET_L: float = 0.02

# 0.5m Is a Suitable Value. Changing it might ruine the Whole Assembly Process
NAILING_CONV_TARGET: float = 0.5

# Variables That Should be Changed For Each Design (For Now)
OVERALL_PANEL_LENGTH: float = SMART_MAT_TABLE_MAX_LENGTH
OVERALL_PANEL_HEIGHT: float = 2.5184
STUD_THICKNESS: float = 0.04
STUD_HEIGHT: float = 0.1016

# LLM Function to be Called
FUNC_ACTION = [
    {
        "type": "function",
        "name": "get_robot_actions",
        "description": (
            "Return an **ordered list of arrays** describing a pick/place sequence.\n"
            "Each inner array MUST be:\n"
            "  [move_or_trigger, robot_id, X, Y, Z, W, Xo, Yo, Zo, tool_name, element_name]\n"
            "Index-by-index requirements:\n"
            "  • 0 → 0 = movement, 1 = trigger\n"
            "  • 1 → robot number\n"
            "  • 2-8 → [X Y Z W Xo Yo Zo] (all zeros if this is a trigger)\n"
            "  • 9-10 → tool_name (e.g., \"tool0\") and element_name (\"None\" for moves)"
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "actions": {
                    "type": "array",
                    "items": {               # inner array schema
                        "type": "array",
                        "minItems": 11,
                        "maxItems": 11,
                        "items": { "anyOf": [
                            {"type": "integer"},
                            {"type": "number"},
                            {"type": "string"}
                        ]}
                    }
                }
            },
            "required": ["actions"],
            "additionalProperties": False
        }
    }
]



############################
############################
### Global Parameters ######
############################
############################

############################
############################
##### Global Objects #######
############################
############################

# Class Robot Gripper
class RobotGripper(object):
    def __init__(self,
                 RobName: str = "IRB6620_R1",
                 ParentLink: str = "Link_7",
                 TCP_Name: str = "T1",
                 C_Pose: List[float] = [0, 0, 0]):
        self.RobName = RobName
        self.ParentLink = ParentLink
        self.TCP_Name = TCP_Name
        self.C_Pose = cast(List[float], C_Pose)

# Class (World Manager)
class WorldManager(object):
    def __init__(self):
        super(WorldManager, self).__init__()
        # Giving the World (Meter Scale)
        self._my_world = World(stage_units_in_meters=1.0)

        self._stage = self._my_world.stage
        self._xform = self._stage.DefinePrim("/World", "Xform")
        self._stage.SetDefaultPrim(self._xform)
        # stage.DefinePrim("/curobo", "Xform")
        self._stage = self._my_world.stage

        # Adding the default Ground to the World
        cast(Scene, self._my_world.scene).add_default_ground_plane()

        # Setting Light
        # distantLight = UsdLux.DistantLight.Get(self._stage, Sdf.Path("/World/lightDistant"))
        # distantLight.GetIntensityAttr().Set(5000.0)  # Set intensity to 5000
        # distantLight.GetColorAttr().Set((1.0, 1.0, 1.0))
        action_registry = omni.kit.actions.core.get_action_registry()
        action = action_registry.get_action("omni.kit.viewport.menubar.lighting", "set_lighting_mode_camera")
        action.execute()
        # sphereLight = UsdLux.SphereLight.Define(self._stage, Sdf.Path("/World/sphereLight"))
        # sphereLight.CreateIntensityAttr(1000.0)
        # sphereLight.CreateRadiusAttr(0.5)

        # Creating USD Helper (CuRobo Object) to handle objects
        self._usd_help = UsdHelper()
        
        # Target Cube For Robotic Movement
        self._target_cube = cuboid.VisualCuboid(
            "/World/target",
            position=np.array([0.5, 0, 3]),
            orientation=np.array([1, 0, 0, 0]),
            color=np.array([1.0, 0, 0]),
            size=0.05,
        )

        # Measurer Cube For Coordination Calculation
        self._measurer_cube = cuboid.VisualCuboid(
            "/World/measurer",
            position=np.array([0.5, 0, 3]),
            orientation=np.array([1, 0, 0, 0]),
            color=np.array([0, 1.0, 0]),
            size=0.05,
        )

        # Smart Conveyor Mover !
        self._conv_cube = cuboid.VisualCuboid(
            "/World/conv_cube",
            position=np.array([0.5, 0, 3]),
            orientation=np.array([1, 0, 0, 0]),
            color=np.array([0, 0, 1.0]),
            size=0.05,
        )

        # ?????
        self._usd_help.load_stage(self._my_world.stage)

        # Adding the required Isaac Sim Extensions to the Simulation (Using CuRobo Helper Library)
        add_extensions(simulation_app, "False")

        # Adding CuRobo World Config to the Stage !!!
        self._curobo_world_cfg = self.init_world_model()
        self._usd_help.add_world_to_stage(self._curobo_world_cfg, base_frame="/World")
    
    def measurement_calculator(self):

        # Creating Target Pose and Orientation
        target_pose = None
        target_orientation = None

        past_pose = None
        past_orientation = None

        while simulation_app.is_running():

            self._my_world.step(render=True)
            
            cube_position, cube_orientation = self._measurer_cube.get_world_pose()   

            if past_pose is None:
                past_pose = cube_position
            if target_pose is None:
                target_pose = cube_position
            if target_orientation is None:
                target_orientation = cube_orientation
            if past_orientation is None:
                past_orientation = cube_orientation

            if (
                (
                    np.linalg.norm(cube_position - target_pose) > 1e-3
                    or np.linalg.norm(cube_orientation - target_orientation) > 1e-3
                )
                and np.linalg.norm(past_pose - cube_position) == 0.0
                and np.linalg.norm(past_orientation - cube_orientation) == 0.0
            ):

                if np.round(cube_position[2], 0) == 500:
                    print("Coordination Calculation Done !")
                    return True
                    
                print(f"Position: {[f'{elem:.3f}' for elem in cube_position]}, Orientation: {[f'{elem:.3f}' for elem in cube_orientation]}")
                target_pose = cube_position
                target_orientation = cube_orientation

            past_pose = cube_position
            past_orientation = cube_orientation

    def world_reset(self):
        self._my_world.reset()
    
    def quat_transfer_world_generator(self, roll, pitch, yaw):
        # Convert degrees to radians
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)

        # Calculate half-angles
        half_roll = roll / 2
        half_pitch = pitch / 2
        half_yaw = yaw / 2

        # Compute cosines and sines of half-angles
        cos_r = math.cos(half_roll)
        sin_r = math.sin(half_roll)
        cos_p = math.cos(half_pitch)
        sin_p = math.sin(half_pitch)
        cos_y = math.cos(half_yaw)
        sin_y = math.sin(half_yaw)

        # Compute quaternion components
        q_w = cos_r * cos_p * cos_y + sin_r * sin_p * sin_y
        q_x = sin_r * cos_p * cos_y - cos_r * sin_p * sin_y
        q_y = cos_r * sin_p * cos_y + sin_r * cos_p * sin_y
        q_z = cos_r * cos_p * sin_y - sin_r * sin_p * cos_y

        return q_w, q_x, q_y, q_z
    
    def init_world_model(self):
        cur_dir = INSTALLATION_DIRECTORY + "/Isaac_sim_ws/"

        # Smart Material Table #1
        Smart_Mat_Table_Quat = self.quat_transfer_world_generator(90, 0, 0)

        Smart_Mat_Table = Mesh(
            name="R2_Smart_Mat_Table",
            pose=[8.0, 5.2, 0, Smart_Mat_Table_Quat[0], 
                                      Smart_Mat_Table_Quat[1],
                                      Smart_Mat_Table_Quat[2],
                                      Smart_Mat_Table_Quat[3]],
            file_path= cur_dir + "smart_table/SM.stl",
            color= [0.2, 0.2, 0.2, 1],
            # Smart_Mat_Supply
            scale=[0.001, 0.001, 0.001],
        )

        # Sloped Table
        Sloped_Table = Mesh(
            name="Sloped_Table",
            pose=[4.0 , -4.0, 0.4, 0, 0, -ev, -ev],
            file_path= cur_dir + "sloped_table/Table.stl",
            color= [0.1, 0.05, 0, 1],
            scale=[0.001, 0.001, 0.001]
        )

        # Small Cutting Table
        Small_Cutting_Table = Mesh(
            name="Small_Cutting_Table",
            pose=[0.2 , 1.2, 0, 0, 0, -ev, -ev],
            file_path= cur_dir + "cutting_table/Small_Cut_Table.stl",
            color= [0.2, 0.2, 0.2, 1],
            scale=[0.001, 0.001, 0.001]
        )
        SMALL_CUT_TABLE_SAW_POSE[0] = Small_Cutting_Table.pose[0]+0.15
        SMALL_CUT_TABLE_SAW_POSE[1] = Small_Cutting_Table.pose[1]
        #Table Height = 0.5
        SMALL_CUT_TABLE_SAW_POSE[2] = Small_Cutting_Table.pose[2]+0.5

        # Ground !
        Cube = Cuboid (
            name="Ground",
            dims=[50, 50, 0.2],
            pose=[0.0, 0.0, -0.1, 1, 0, 0, 0.0],
            color= [0, 0, 0, 0]
        )
        # world_cfg_table = WorldConfig.from_dict(
        #     load_yaml(join_path(get_world_configs_path(), "collision_table.yml"))
        # )
        # world_cfg_table.cuboid[0].pose[2] -= 0.02

        # IDC_Lab = Mesh(
        #     name="idc_lab_model",
        #     pose=[0, 0, 0, 1, 0, 0, 0],
        #     file_path= cur_dir + "lab_model/idc_lab_visualization.stl",
        #     color= [0.1, 0.05, 0, 1],
        #     scale=[0.001, 0.001, 0.001]
        # )


        SheathingTable = Mesh(
            name="Sheathing_Table",
            pose=[0, -2, 0, ev, ev, 0, 0],
            file_path= cur_dir + "sheathing_table/SheathingTable.stl",
            color= [0.1, 0.05, 0, 1],
            scale=[0.001, 0.001, 0.001]
        )

        BearLoadingPileStand = Cuboid (
            name="BearLoading_PileStand",
            dims=[RAW_HEADER_DIMENSIONS[2], 1, 0.1],
            pose=[-1-(RAW_HEADER_DIMENSIONS[2]/2), 1.5, 0.05, 1, 0, 0, 0],
            color= [0.2, 0.2, 0.2, 1]
        )

        # Human_Worker = Mesh(
        #     name="Human_Worker",
        #     pose=[4.2, 1.5, 0, ev, 0, 0, ev],
        #     file_path= cur_dir + "Human_Worker/Worker.stl",
        #     color= [0.1, 0.05, 0, 1],
        #     scale=[0.001, 0.001, 0.001]
        # )


        # Small_Cutting_Table
        world_model = WorldConfig(
            mesh=[Smart_Mat_Table, Sloped_Table, SheathingTable, Small_Cutting_Table],
            cuboid=[Cube, BearLoadingPileStand],
            capsule=[],
            cylinder=[],
            sphere=[],
        )

        return world_model

class CuRoboConv(object):
    def __init__(self,
                 working_world: WorldManager,
                 Conv_Name: str = "Smart_Conveyor",
                 pose: np.array = np.array([0, 0, 0]),
                 input_tool: str = "tool0",
                 w_dir: str = INSTALLATION_DIRECTORY + "/Isaac_sim_ws/smart_conveyor",
                 c_conf_name: str = "Smart_Conveyor.yaml",
                 Gripper_List: Optional[List[RobotGripper]] = None):
        super(CuRoboConv, self).__init__()

        # Reading Robot Configuration (Generated by CuRobo Robot Setup Steps)
        self._robot_cfg_path = w_dir
        self._robot_cfg = load_yaml(join_path(self._robot_cfg_path, c_conf_name))["robot_cfg"]
        self._j_names = self._robot_cfg["kinematics"]["cspace"]["joint_names"]
        self._default_config = self._robot_cfg["kinematics"]["cspace"]["retract_config"]

        # Adding Robot to the Scene (Identifying robot type as Robot (omni.isaac.core.robots Robot))
        self._temp_world_manager = working_world
        self._robot: Robot
        self._robot, self._robot_prim_path = add_robot_to_scene(robot_config=self._robot_cfg,
                                                                robot_name= Conv_Name,
                                                                my_world=self._temp_world_manager._my_world,
                                                                position=pose)
        # Reseting World to see the new robot
        self._temp_world_manager.world_reset()

        # Articulation Controller
        self._articulation_controller: ArticulationController = None

        # Nailing Positions For Side Nailing
        self._nail_poses: List[Tuple[float, float]] = []

        # Nailing Positions For Vertical Nailing
        self._vertical_nail_poses: List[float] = []

        # Disabling Colliders
        # Not Tested ! Might Cause Failures !
        # self._L1_Prim = self._temp_world_manager._stage.GetPrimAtPath("/" + Conv_Name + "/Link_1/collisions")
        # self._Collider_Off = self._L1_Prim.GetAttribute("physics:collisionEnabled").Set(False)

        self._TL_Prim = self._temp_world_manager._stage.GetPrimAtPath("/" + Conv_Name + "/track_link/collisions")
        self._Collider_Off = self._TL_Prim.GetAttribute("physics:collisionEnabled").Set(False)

        self._BL_Prim = self._temp_world_manager._stage.GetPrimAtPath("/" + Conv_Name + "/base_link/collisions")
        self._Collider_Off = self._BL_Prim.GetAttribute("physics:collisionEnabled").Set(False)

        self._L1_Prim = self._temp_world_manager._stage.GetPrimAtPath("/" + Conv_Name + "/Link_1/collisions")
        self._Collider_Off = self._L1_Prim.GetAttribute("physics:collisionEnabled").Set(False)

        self._L2_Prim = self._temp_world_manager._stage.GetPrimAtPath("/" + Conv_Name + "/Link_2/collisions")
        self._Collider_Off = self._L2_Prim.GetAttribute("physics:collisionEnabled").Set(False)

        self._temp_world_manager._my_world.step(render=True)

        #Test
        self._world_updater_counter = 0

    def articulation_controller_init(self, step_index):
        if self._articulation_controller is None:
            self._articulation_controller = cast(ArticulationController, self._robot.get_articulation_controller())
        if step_index < 2:
            self._temp_world_manager._my_world.reset()       
            self._robot._articulation_view.initialize()
            idx_list = [self._robot.get_dof_index(x) for x in self._j_names]
            self._robot.set_joint_positions(self._default_config, idx_list)

            self._robot._articulation_view.set_max_efforts(
                values=np.array([5000 for i in range(len(idx_list))]), joint_indices=idx_list
            )

    def free_conv_movement(self):

        print(self._robot.get_joint_positions())

        current_state = self._robot.get_joint_positions()

        self._temp_world_manager._conv_cube.set_world_pose(position=[2.3, current_state[0]-SMART_CONV_RANGE_OF_MOTION_J1/2, current_state[1]],
                                                           orientation=[1, 0, 0, 0])

        while simulation_app.is_running():

            self._temp_world_manager._my_world.step(render=True)

            current_state = self._robot.get_joint_positions()

            cube_position, cube_orientation = self._temp_world_manager._conv_cube.get_world_pose()

            if cube_position[2] > SMART_CONV_RANGE_OF_MOTION_J2 or cube_position[2] < 0.0:
                current_state = self._robot.get_joint_positions()
                self._temp_world_manager._conv_cube.set_world_pose(position=[2.3, current_state[0]-SMART_CONV_RANGE_OF_MOTION_J1/2, current_state[1]],
                                                        orientation=[1, 0, 0, 0])
                continue
            if np.round(cube_position[2],0) == 1000:
                break
            if cube_position[1]+SMART_CONV_RANGE_OF_MOTION_J1/2 > SMART_CONV_RANGE_OF_MOTION_J1 or cube_position[1]+SMART_CONV_RANGE_OF_MOTION_J1/2 < 0:
                self._temp_world_manager._conv_cube.set_world_pose(position=[2.3, current_state[0]-SMART_CONV_RANGE_OF_MOTION_J1/2, current_state[1]],
                                                        orientation=[1, 0, 0, 0])
                continue

            self.render_exec('Joint_1', cube_position[1]+SMART_CONV_RANGE_OF_MOTION_J1/2)
            self.render_exec('Joint_2', cube_position[2])     

    def render_exec(self,
                    joint_name: str = 'Joint_1',
                    joint_goal: float = 1.5):
        joint_idx = 0
        j_s = self._robot.dof_names
        for joint in j_s:
            if joint == joint_name:
                break
            joint_idx +=1
        
        pose_matrix = self._robot.get_joint_positions()
        pose_descreter = (joint_goal - pose_matrix[joint_idx]) / 100
        while (abs(pose_matrix[joint_idx] - joint_goal) > 2e-4):
            self._temp_world_manager._my_world.step(render=True)
            pose_matrix[joint_idx] += pose_descreter

            art_action = ArticulationAction(
                joint_positions=pose_matrix
            )
            self._articulation_controller.apply_action(art_action)
            time.sleep(0.02)
        
        # TT = time.time()
        # while time.time() - TT <= 2:
        #     self._temp_world_manager._my_world.step(render= True)
        
        # print("Updating world, reading w.r.t.", self._robot_prim_path)
        # obstacles = self._temp_world_manager._usd_help.get_obstacles_from_stage(
        #     reference_prim_path=self._robot_prim_path,
        #     ignore_substring=[
        #         "/World/defaultGroundPlane",
        #         # Other Robot's Prim Path Should also be Ignored !
        #         # This feature is to be developed (MPC)
        #     ],
        # ).get_collision_check_world()

        # Saving the Updated World !
        # file_path = "Full_Conv_World.obj"
        # obstacles.save_world_as_mesh(file_path)

    def attach_object_to_conv(self,
                              obj_name: str = None,
                              Enable_Gravity: bool = True):
        
        # Disabling Gravity Right After Detaching
        Obj_Prim = self._temp_world_manager._stage.GetPrimAtPath("/world/obstacles/" + obj_name)
        Conv_Prim = self._temp_world_manager._stage.GetPrimAtPath("/Smart_Conveyor/Link_2")

        Conv_Collision_Prim = self._temp_world_manager._stage.GetPrimAtPath("/Smart_Conveyor/Link_2/collisions")   

        Conv_Collision_Prim.GetAttribute("physics:collisionEnabled").Set(True)
        self._temp_world_manager._my_world.step(render= True)

        Obj_Prim.GetAttribute("physics:rigidBodyEnabled").Set(True)
        Obj_Prim.GetAttribute("physics:collisionEnabled").Set(True)
        if(Enable_Gravity):
            Obj_Prim.GetAttribute("physxRigidBody:disableGravity").Set(False)
        self._temp_world_manager._my_world.step(render= True)    

        Time = time.time()
        while time.time() - Time <= 0.2:
            self._temp_world_manager._my_world.step(render= True)
        
        # Fix Jointing Stud To Conveyor
        prim = utils.createJoint(self._temp_world_manager._stage, "Fixed", Obj_Prim, Conv_Prim)
        self._temp_world_manager._my_world.step(render= True) 

        #Damper ! (20 is Good !!!)
        Obj_Prim.GetAttribute("physxRigidBody:linearDamping").Set(20)
        Obj_Prim.GetAttribute("physxRigidBody:angularDamping").Set(20)

        if(Enable_Gravity):
            Obj_Prim.GetAttribute("physxRigidBody:disableGravity").Set(True)
        Obj_Prim.GetAttribute("physics:collisionEnabled").Set(False)
        Conv_Collision_Prim.GetAttribute("physics:collisionEnabled").Set(False)
        self._temp_world_manager._my_world.step(render= True)


        
        return prim

class CuRoboRobot(object):
    def __init__(self,
                 working_world: WorldManager,
                 R_Name: str = "IRB6620_R1",
                 pose: np.array = np.array([0, 0, 0]),
                 #This Feature has not been added to CuRobo
                 orientation: np.array = np.array([1, 0, 0, 0]),
                 input_tool: str = "tool0",
                 w_dir: str = INSTALLATION_DIRECTORY + "/Isaac_sim_ws/robot", 
                 r_conf_name: str = "IRB6620_Config.yaml",
                 Gripper_List: Optional[List[RobotGripper]] = None,
                 Cuda_Device: int = 0):
        
        super(CuRoboRobot, self).__init__()

        # Saving Robot's Pose
        self._r_pose = pose

        # Reading Robot Configuration (Generated by CuRobo Robot Setup Steps)
        self._ROS_JS_robot_indicator = R_Name
        self._tensor_args = TensorDeviceType()
        self._robot_cfg_path = w_dir
        self._robot_cfg = load_yaml(join_path(self._robot_cfg_path, r_conf_name))["robot_cfg"]
        self._j_names = self._robot_cfg["kinematics"]["cspace"]["joint_names"]
        self._default_config = self._robot_cfg["kinematics"]["cspace"]["retract_config"]

        
        # Setting up TCP
        self._current_tool = input_tool
        self._robot_cfg["kinematics"]["ee_link"] = self._current_tool

        # Setting up Extra Tool Spheres
        if self._ROS_JS_robot_indicator == "IRB6620_R1":
            self._robot_cfg["kinematics"]["extra_collision_spheres"] = {"tool0": 1, "tool1": 1,}
        if self._ROS_JS_robot_indicator == "IRB6620_R2":
            self._robot_cfg["kinematics"]["extra_collision_spheres"] = {"tool0": 1,}

        # if self._ROS_JS_robot_indicator == "IRB6620_R1":
        #     self._robot_cfg["kinematics"]["extra_collision_spheres"] = {"tool0": 50, "tool1": 100,}
        # if self._ROS_JS_robot_indicator == "IRB6620_R2":
        #     self._robot_cfg["kinematics"]["extra_collision_spheres"] = {"tool0": 50,}

        # Adding Robot to the Scene (Identifying robot type as Robot (omni.isaac.core.robots Robot))
        self._temp_world_manager = working_world
        self._robot: Robot
        self._robot, self._robot_prim_path = add_robot_to_scene(robot_config=self._robot_cfg,
                                                                robot_name= R_Name,
                                                                my_world=self._temp_world_manager._my_world,
                                                                position=pose)
        
        # Reseting World to see the new robot
        self._temp_world_manager.world_reset()

        # Creating SurfaceGripper for each Robot Gripper
        for Gripper in Gripper_List:
            # Creating Prims !
            PrimPathX = "/"+Gripper.RobName+"/"+Gripper.ParentLink+"/"+"GripPosition_"+Gripper.TCP_Name+"_X"
            PrimPathY = "/"+Gripper.RobName+"/"+Gripper.ParentLink+"/"+"GripPosition_"+Gripper.TCP_Name+"_Y"
            PrimPathZ = "/"+Gripper.RobName+"/"+Gripper.ParentLink+"/"+"GripPosition_"+Gripper.TCP_Name+"_Z"
            XformX = self._temp_world_manager._stage.DefinePrim(PrimPathX, "Xform")
            XformY = self._temp_world_manager._stage.DefinePrim(PrimPathY, "Xform")
            XformZ = self._temp_world_manager._stage.DefinePrim(PrimPathZ, "Xform")

            # Creating xyz rpy Attributes !
            XformableX = UsdGeom.Xformable(XformX)
            XformableY = UsdGeom.Xformable(XformY)
            XformableZ = UsdGeom.Xformable(XformZ)

            xyz_attr_X = XformX.CreateAttribute("xyz", Sdf.ValueTypeNames.Float3, True)
            rpy_attr_X = XformX.CreateAttribute("rpy", Sdf.ValueTypeNames.Float3, True)
            xyz_attr_Y = XformY.CreateAttribute("xyz", Sdf.ValueTypeNames.Float3, True)
            rpy_attr_Y = XformY.CreateAttribute("rpy", Sdf.ValueTypeNames.Float3, True)
            xyz_attr_Z = XformZ.CreateAttribute("xyz", Sdf.ValueTypeNames.Float3, True)
            rpy_attr_Z = XformZ.CreateAttribute("rpy", Sdf.ValueTypeNames.Float3, True)

            Grip_Offset_Position = Gf.Vec3d(Gripper.C_Pose[0], Gripper.C_Pose[1], Gripper.C_Pose[2])

            Grip_Offset_Orientation_X = Gf.Vec3d(0, -90, 0)
            Grip_Offset_Orientation_Y = Gf.Vec3d(0, 0, 90)
            Grip_Offset_Orientation_Z = Gf.Vec3d(-90, -180, 0)

            # X
            xyz_attr_X.Set(Grip_Offset_Position)
            translate_op_x = XformableX.AddTranslateOp()
            translate_op_x.Set(Grip_Offset_Position)
            rpy_attr_X.Set(Grip_Offset_Orientation_X)
            rotate_op_x = XformableX.AddRotateXYZOp()
            rotate_op_x.Set(Grip_Offset_Orientation_X)

            # Y
            xyz_attr_Y.Set(Grip_Offset_Position)
            translate_op_y = XformableY.AddTranslateOp()
            translate_op_y.Set(Grip_Offset_Position)
            rpy_attr_Y.Set(Grip_Offset_Orientation_Y)
            rotate_op_y = XformableY.AddRotateXYZOp()
            rotate_op_y.Set(Grip_Offset_Orientation_Y)            

            # Z
            xyz_attr_Z.Set(Grip_Offset_Position)
            translate_op_z = XformableZ.AddTranslateOp()
            translate_op_z.Set(Grip_Offset_Position)
            rpy_attr_Z.Set(Grip_Offset_Orientation_Z)
            rotate_op_z = XformableZ.AddRotateXYZOp()
            rotate_op_z.Set(Grip_Offset_Orientation_Z)

            # Creating SurfaceGripper OmniGraph for Each TCP !!!
            keys = og.Controller.Keys
            (graph_handle, list_of_nodes, _, _) = og.Controller.edit(
                {"graph_path": "/action_graph_"+Gripper.RobName+"_"+Gripper.TCP_Name, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("surfX","omni.isaac.surface_gripper.SurfaceGripper"),
                        ("surfY","omni.isaac.surface_gripper.SurfaceGripper"),
                        ("surfZ","omni.isaac.surface_gripper.SurfaceGripper"),
                        ("close_tick","omni.graph.action.OnImpulseEvent"),
                        ("open_tick","omni.graph.action.OnImpulseEvent"),

                    ],
                    keys.SET_VALUES: [
                        ("surfX.inputs:GripPosition", PrimPathX),
                        ("surfY.inputs:GripPosition", PrimPathY),
                        ("surfZ.inputs:GripPosition", PrimPathZ),
                        ("surfX.inputs:ParentRigidBody", "/"+Gripper.RobName+"/"+Gripper.ParentLink),
                        ("surfY.inputs:ParentRigidBody", "/"+Gripper.RobName+"/"+Gripper.ParentLink),
                        ("surfZ.inputs:ParentRigidBody", "/"+Gripper.RobName+"/"+Gripper.ParentLink),
                        # ("surfX.inputs:DisableGravity", True),
                        # ("surfY.inputs:DisableGravity", True),
                        # ("surfZ.inputs:DisableGravity", True),
                    ],
                    keys.CONNECT: [
                        ("close_tick.outputs:execOut", "surfX.inputs:Close"),
                        ("close_tick.outputs:execOut", "surfY.inputs:Close"),
                        ("close_tick.outputs:execOut", "surfZ.inputs:Close"),
                        ("open_tick.outputs:execOut", "surfX.inputs:Open"),
                        ("open_tick.outputs:execOut", "surfY.inputs:Open"),
                        ("open_tick.outputs:execOut", "surfZ.inputs:Open"),
                    ],
                },
            )

            # Increasing Damping for the Grippers to avoid vibration upon relocating studs
            og.Controller.set(og.Controller.attribute("/action_graph_"+self._ROS_JS_robot_indicator+"_"+Gripper.TCP_Name+"/surfX.inputs:Damping"), 1000000)
            og.Controller.set(og.Controller.attribute("/action_graph_"+self._ROS_JS_robot_indicator+"_"+Gripper.TCP_Name+"/surfY.inputs:Damping"), 1000000)
            og.Controller.set(og.Controller.attribute("/action_graph_"+self._ROS_JS_robot_indicator+"_"+Gripper.TCP_Name+"/surfZ.inputs:Damping"), 1000000)

        self._articulation_controller: ArticulationController = None

        # Creating and Warming Up the MotionGen
        self._r_conf_name = r_conf_name

        # Back To Normal On 4090
        self.motion_gen_warmup()
        # This will indicate if any object is attached to the robot or not
        self._is_obj_attached: bool = False
        # This will be used to prevent the world updater from grasping the attached object's mesh representation as world
        self._attached_obj_prim: str = "/world/obstacles/DummyObstacle"

        # Sphere Generation Requirement
        self._spheres: List[Sphere] = None

        self._spheres = None

        # Creating JointState Publishing Topic:
        self._js_working_name = re.match(r"^(.*_Config)", r_conf_name).group(1)
        self._ros_js_publsiher = self._ROS_JS_robot_indicator + "_joint_state"
        self._js_publisher = rospy.Publisher(self._ros_js_publsiher, sensor_msgs.msg.JointState, queue_size=10)
        self._js_pub_interval: None

        self._computed_path_result: MotionGenResult = None
        self._computed_cmd_plan: JointState = None
        self._computed_idx_list = []

        self._world_updater_counter = 0

        # An Argument to Check if a Robot is At Home Position or Not !
        self._is_at_home: bool = False

        # Deactivating EndEffector's Collision Representation in the Simulation
        # Attaching Surface Grippers to Link 6 rather than Link 7
        # Everything Will Workd ^-^ !!!
        self._EEF_Prim = self._temp_world_manager._stage.GetPrimAtPath("/" + self._ROS_JS_robot_indicator + "/Link_7/collisions")
        self._Collider_Off = self._EEF_Prim.GetAttribute("physics:collisionEnabled").Set(False)

        ## Removing EndEffector's Mass
        self._temp_world_manager._stage.GetPrimAtPath("/" + self._ROS_JS_robot_indicator + "/Link_7").GetAttribute("physics:mass").Set(1e-20)
        self._temp_world_manager._stage.GetPrimAtPath("/" + self._ROS_JS_robot_indicator + "/Link_8").GetAttribute("physics:mass").Set(1e-20)

        # Neeed To Increase Joint Damping To Avoid Shaking within Robot Joints (FIXED IN URDF)
        # for i in range (1, 6):
        #     Joint_Prim = self._temp_world_manager._stage.GetPrimAtPath("/" + self._ROS_JS_robot_indicator + "/Link_" + str(i) + "/Joint_" + str(i+1))
        #     drive = UsdPhysics.DriveAPI.Get(Joint_Prim, "angular")
        #     drive.GetDampingAttr().Set(1e2)
        #     drive.GetStiffnessAttr().Set(1e2)
        #     self._temp_world_manager._my_world.step(render=True)

        # Synchronizer Value For Each Robot
        self._synchronizer: bool = False

    def articulation_controller_init(self, step_index):
        if self._articulation_controller is None:
            self._articulation_controller = cast(ArticulationController, self._robot.get_articulation_controller())
        if step_index < 2:
            self._temp_world_manager._my_world.reset()       
            self._robot._articulation_view.initialize()
            idx_list = [self._robot.get_dof_index(x) for x in self._j_names]
            self._robot.set_joint_positions(self._default_config, idx_list)

            self._robot._articulation_view.set_max_efforts(
                values=np.array([5000 for i in range(len(idx_list))]), joint_indices=idx_list
            )

    def clean_occupied_vram(self):
        torch.cuda.empty_cache()

    def motion_gen_warmup(self,
                          TCP_Name: str = None):
        
        # Default Parameters
        trajopt_dt = None
        optimize_dt = False
        trajopt_tsteps = 32
        trim_steps = None
        max_attempts = 10
        interpolation_dt = 0.05
        enable_finetune_trajopt = False
        # MotionGen StartUp

        self.clean_occupied_vram()

        # This means that there is no need to change the current TCP Config
        if TCP_Name != None:
            self._robot_cfg["kinematics"]["ee_link"] = TCP_Name
        
        P_T: float = 0.0001
        Q_T: float = 0.0001
        C_T: float = 0.0001

        # If NailGun is Used, Thresholds should Drop
        if((self._ROS_JS_robot_indicator == "IRB6620_R1") and (TCP_Name == "tool2" or TCP_Name == "tool3")):
            P_T = 0.05
            Q_T = 0.05
            C_T = 0.05
        
        if((self._ROS_JS_robot_indicator == "IRB6620_R2") and (TCP_Name == "tool1" or TCP_Name == "tool2")):
            P_T = 0.05
            Q_T = 0.05
            C_T = 0.05

        # Creating MotionGenConfig after updating the TCP !
        self._motion_gen_config = MotionGenConfig.load_from_robot_config(
            self._robot_cfg,
            self._temp_world_manager.init_world_model(),
            self._tensor_args,
            collision_checker_type=CollisionCheckerType.MESH,
            num_trajopt_seeds=12,
            num_graph_seeds=12,
            # Error Thresholds !!
            position_threshold= P_T,
            rotation_threshold= Q_T,
            #It Solved Some Errors (Quat)
            cspace_threshold= C_T,
            interpolation_dt=interpolation_dt,
            optimize_dt=optimize_dt,
            trajopt_dt=trajopt_dt,
            trajopt_tsteps=trajopt_tsteps,
            trim_steps=trim_steps,
            # This Cache Value needs to be set PROPERLY
            collision_cache={"obb":100, "mesh": 200}
        )
        
        self._motion_gen = MotionGen(self._motion_gen_config)
        
        print("warming up...")
        #enable_graph=False, warmup_js_trajopt=False
        self._motion_gen.warmup()
        print("Curobo for robot ( " + self._r_conf_name + " ) is ready ... | TCP = " + self._robot_cfg["kinematics"]["ee_link"])

        self._plan_config = MotionGenPlanConfig(enable_graph=True,
                                                enable_graph_attempt=2,
                                                max_attempts=max_attempts,
                                                enable_finetune_trajopt=enable_finetune_trajopt,
                                                time_dilation_factor= MOTION_ACCELERAION_VALUE)

    def restrict_path_plan(self,
                        Position_Restriction: Optional[torch.Tensor] = None,
                        Orientation_Restriction: Optional[torch.Tensor] = None):
        
        # Set default tensors of zeros if inputs are None
        if Orientation_Restriction is None:
            Orientation_Restriction = torch.zeros(3, dtype=torch.float32)
        if Position_Restriction is None:
            Position_Restriction = torch.zeros(3, dtype=torch.float32)
        
        # Combine the two tensors into one
        combined_restriction = torch.cat([Orientation_Restriction, Position_Restriction])

        metrics = PoseCostMetric(hold_partial_pose= True,
                                 hold_vec_weight= combined_restriction)

        self._plan_config = MotionGenPlanConfig(enable_graph=True,
                                                enable_graph_attempt=2,
                                                max_attempts=10,
                                                enable_finetune_trajopt=False,
                                                pose_cost_metric= metrics,
                                                time_dilation_factor= MOTION_ACCELERAION_VALUE)
    
    def release_path_plan_restriction(self):
        # Removing Configs resulting into restrictions
        self._plan_config = MotionGenPlanConfig(enable_graph=True,
                                                enable_graph_attempt=2,
                                                max_attempts=10,
                                                enable_finetune_trajopt=False,
                                                time_dilation_factor= MOTION_ACCELERAION_VALUE)

    def motion_gen_update_world(self, 
                                Removing_Prim_Paths: List[str] = None):

        # 1. Get an Update of the Collision World for the Robot:
        # Ignoring Other Robot's Visual Representation
        # This should be updated for the list of robots !!!!
        Rob_name: str= ""
        if self._ROS_JS_robot_indicator == "IRB6620_R1":
            Rob_name = "IRB6620_R2"
        if self._ROS_JS_robot_indicator == "IRB6620_R2":
            Rob_name = "IRB6620_R1"
        
        print(self._robot_prim_path)

        ignoring_prim_paths = [
            self._robot_prim_path,
            # "/World/defaultGroundPlane",
            "/World/obstacles/R2_Smart_Mat_Table",
            # smart Conveyor's Visual Prims
            "/Smart_Conveyor/base_link/visuals",
            "/Smart_Conveyor/track_link/visuals",
            "/Smart_Conveyor/Link_1/visuals",
            "/Smart_Conveyor/Link_2/visuals",
            # Other Robot's Visual Prims
            "/"+Rob_name+"/base_link/visuals",
            "/"+Rob_name+"/Link_1/visuals",
            "/"+Rob_name+"/Link_2/visuals",
            "/"+Rob_name+"/Link_3/visuals",
            "/"+Rob_name+"/Link_4/visuals",
            "/"+Rob_name+"/Link_5/visuals",
            "/"+Rob_name+"/Link_6/visuals",
            "/"+Rob_name+"/Link_7/visuals",
            "/"+Rob_name+"/Link_8/visuals",
            # Attached Object's Prim (If Any)
            self._attached_obj_prim,
            # Moving Cubes Should also be ignored
            "/World/target",
            "/World/measurer",
            "/World/conv_cube",
            # Human Worker's Visual Model !
            "/World/obstacles/Human_Worker",      
            # Other Robot's Prim Path Should also be Ignored !
            # This feature is to be developed (MPC)
        ]
        # Add collision paths if empty_coll_world is True
        if Removing_Prim_Paths != None:
            for prim in Removing_Prim_Paths:
                ignoring_prim_paths.extend(["/"+prim])
            # We Do this Whenever we want to remove anything from the scene!

            # Alternative Method
            # self._motion_gen.world_model.remove_obstacle("/Smart_Conveyor/Link_1/collisions")
            # self._motion_gen.world_model.remove_obstacle("/Smart_Conveyor/Link_2/collisions")

        self._temp_world_manager._my_world.step(render=True)
        self._motion_gen.clear_world_cache()

        obstacles = self._temp_world_manager._usd_help.get_obstacles_from_stage(
            reference_prim_path=self._robot_prim_path,
            ignore_substring=ignoring_prim_paths,
        ).get_collision_check_world()

        self._motion_gen.update_world(obstacles)

        print("Collision World Updated For Robot "+self._ROS_JS_robot_indicator)
        print("Removed Meshes From the Collision World: ")
        print(Removing_Prim_Paths)
        print("________________________________________________")

    # Free Movement of Robotic Arm with a Cube to Determine Pick and Place Locations (Done)
    def free_TCP_movement(self, 
                          moving_tcp: str = "tool0"):
        # Re-warming up the MotionGen Object with the New Tool to move !
        if moving_tcp != self._robot_cfg["kinematics"]["ee_link"]:
            self.motion_gen_warmup(TCP_Name=moving_tcp)


        # Setting the Initial Pose/Orientation of the Target Cube to the Targetting TCP !!
        dc=_dynamic_control.acquire_dynamic_control_interface()

        object=dc.get_rigid_body("/"+self._ROS_JS_robot_indicator+"/"+moving_tcp)
        object_pose=dc.get_rigid_body_pose(object)

        self._temp_world_manager._target_cube.set_world_pose(position=object_pose.p,
                                                                orientation=[object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]])

        # Creating Target Pose and Orientation
        target_pose = None
        target_orientation = None

        past_pose = None
        past_orientation = None

        # Let the Robot Move Freely
        self.release_path_plan_restriction()

        cmd_plan = None
        # Creating the Loop
        while simulation_app.is_running():

            self._temp_world_manager._my_world.step(render=True)
            
            cube_position, cube_orientation = self._temp_world_manager._target_cube.get_world_pose()
            
            # CUBE POSE - ROBOT POSE
            cube_position[0] -= self._r_pose[0]
            cube_position[1] -= self._r_pose[1]
            cube_position[2] -= self._r_pose[2]

            if past_pose is None:
                past_pose = cube_position
            if target_pose is None:
                target_pose = cube_position
            if target_orientation is None:
                target_orientation = cube_orientation
            if past_orientation is None:
                past_orientation = cube_orientation

            # Running a Plan and Execution Instance Togather
            sim_js = self._robot.get_joints_state()
            sim_js_names = self._robot.dof_names
            if np.any(np.isnan(sim_js.positions)):
                log_error("isaac sim has returned NAN joint position values.")
            cu_js = JointState(
                position=self._tensor_args.to_device(sim_js.positions),
                velocity=self._tensor_args.to_device(sim_js.velocities) * 0.0,
                acceleration=self._tensor_args.to_device(sim_js.velocities) * 0.0,
                jerk=self._tensor_args.to_device(sim_js.velocities) * 0.0,
                joint_names=sim_js_names,
            )
            cu_js = cu_js.get_ordered_joint_state(self._motion_gen.kinematics.joint_names)

            # Checking Rule for New Target Locations
            robot_static = False

            if (np.max(np.abs(sim_js.velocities)) < 0.2):
                robot_static = True
            if (
                (
                    np.linalg.norm(cube_position - target_pose) > 1e-3
                    or np.linalg.norm(cube_orientation - target_orientation) > 1e-3
                )
                and np.linalg.norm(past_pose - cube_position) == 0.0
                and np.linalg.norm(past_orientation - cube_orientation) == 0.0
                and robot_static
            ):
                # Hard Coded Prompts
                if np.round(cube_position[2]+self._r_pose[2],0) == 2000:
                    print("Releasing Restrictions")
                    self.release_path_plan_restriction()
                    # Setting the Initial Pose/Orientation of the Target Cube to the Targetting TCP !!
                    dc=_dynamic_control.acquire_dynamic_control_interface()

                    object=dc.get_rigid_body("/"+self._ROS_JS_robot_indicator+"/"+moving_tcp)
                    object_pose=dc.get_rigid_body_pose(object)

                    self._temp_world_manager._target_cube.set_world_pose(position=object_pose.p,
                                                                            orientation=[object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]])
                if np.round(cube_position[2]+self._r_pose[2],0) == 3000:
                    print("Linear Restriction Activated")
                    # self.restrict_path_plan(Orientation_Restriction= torch.ones(3, dtype=torch.float32))

                    self._plan_config = MotionGenPlanConfig(enable_graph=True,
                                        enable_graph_attempt=2,
                                        max_attempts=10,
                                        enable_finetune_trajopt=False,
                                        pose_cost_metric= PoseCostMetric.create_grasp_approach_metric(offset_position=0.01, tstep_fraction=0.001,linear_axis=0),
                                        time_dilation_factor= MOTION_ACCELERAION_VALUE) 

                    # Setting the Initial Pose/Orientation of the Target Cube to the Targetting TCP !!
                    dc=_dynamic_control.acquire_dynamic_control_interface()

                    object=dc.get_rigid_body("/"+self._ROS_JS_robot_indicator+"/"+moving_tcp)
                    object_pose=dc.get_rigid_body_pose(object)

                    self._temp_world_manager._target_cube.set_world_pose(position=object_pose.p,
                                                                            orientation=[object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]])
                # Z = 1000 : It means that we're done with free movement and we know the coordinations
                # It will break the loop and let the program continue !
                if np.round(cube_position[2]+self._r_pose[2],0) == 1000:
                    print("Free Movement of Robot "+self._ROS_JS_robot_indicator+" is Done !")
                    self._temp_world_manager._target_cube.set_world_pose(position=[2.3, 0, 2],
                                                                         orientation=[1, 0, 0, 0])
                    break
                # Z = 500 : It means that the Attached Object is too Close to Obstacles and we need Accurate Movements
                # To do so, we empty out the robot's Collision world representation
                # It will let CuRobo Plan freely and in a linear way (Conduct Linear Movements)
                if np.round(cube_position[2]+self._r_pose[2],0) == 500:
                    self.motion_gen_update_world(Removing_Prim_Paths=["Smart_Conveyor", "obstacles"])

                    # Setting the Initial Pose/Orientation of the Target Cube to the Targetting TCP !!
                    dc=_dynamic_control.acquire_dynamic_control_interface()

                    object=dc.get_rigid_body("/"+self._ROS_JS_robot_indicator+"/"+moving_tcp)
                    object_pose=dc.get_rigid_body_pose(object)

                    self._temp_world_manager._target_cube.set_world_pose(position=object_pose.p,
                                                                            orientation=[object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]])

                    target_pose = None
                    target_orientation = None
                    past_pose = None
                    past_orientation = None
                    cmd_plan = None
                    # self._motion_gen.detach_object_from_robot("tool0")
                    continue
                # Z = 200 : Bring Back The Whole Collision World
                if np.round(cube_position[2]+self._r_pose[2],0) == 200:
                    self.motion_gen_update_world()

                    # Setting the Initial Pose/Orientation of the Target Cube to the Targetting TCP !!
                    dc=_dynamic_control.acquire_dynamic_control_interface()

                    object=dc.get_rigid_body("/"+self._ROS_JS_robot_indicator+"/"+moving_tcp)
                    object_pose=dc.get_rigid_body_pose(object)

                    self._temp_world_manager._target_cube.set_world_pose(position=object_pose.p,
                                                                            orientation=[object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]])

                    target_pose = None
                    target_orientation = None
                    past_pose = None
                    past_orientation = None
                    cmd_plan = None
                    continue
                # Set EE teleop goals, use cube for simple non-vr init:
                ee_translation_goal = cube_position
                ee_orientation_teleop_goal = cube_orientation

                # compute curobo solution:
                ik_goal = Pose(
                    position=self._tensor_args.to_device(ee_translation_goal),
                    quaternion=self._tensor_args.to_device(ee_orientation_teleop_goal),
                )

                result = self._motion_gen.plan_single(cu_js.unsqueeze(0), ik_goal, self._plan_config)
                succ = result.success.item()

                if succ:
                    ## Announcing the Target Location:
                    print(self._ROS_JS_robot_indicator+" | "+self._robot_cfg["kinematics"]["ee_link"])
                    Modified_Pose = [cube_position[0]+self._r_pose[0],
                                     cube_position[1]+self._r_pose[1],
                                     cube_position[2]+self._r_pose[2]]
                    # print(f"Reached Pose: {Modified_Pose}, Reached Orientation: {cube_orientation}")
                    # Print with 3 decimals
                    print(f"Reached Pose: {[f'{elem:.2f}' for elem in Modified_Pose]}, Reached Orientation: {[cube_orientation]}")

                    ##

                    cmd_plan = result.get_interpolated_plan()
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

                else:
                    carb.log_warn("Plan did not converge to a solution: " + str(result.status))
                target_pose = cube_position
                target_orientation = cube_orientation

            past_pose = cube_position
            past_orientation = cube_orientation
            if cmd_plan is not None:
                cmd_state = cmd_plan[cmd_idx]
                # get full dof state
                art_action = ArticulationAction(
                    cmd_state.position.cpu().numpy(),
                    cmd_state.velocity.cpu().numpy(),
                    joint_indices=idx_list,
                )

                # set desired joint angles obtained from IK:
                self._articulation_controller.apply_action(art_action)

                # Publishing To ROS
                self.ros_js_publisher()

                cmd_idx += 1
                for _ in range(2):
                    self._temp_world_manager._my_world.step(render=False)
                if cmd_idx >= len(cmd_plan.position):
                    cmd_idx = 0
                    cmd_plan = None

    def plan(self,
                        tcp_name: str = "tool1",
                        target_pose: np.array = [0, 0, 0],
                        target_orientation: np.array = [1, 0, 0, 0],
                        update_world_needed: bool = True,
                        removing_primitives: List[str] = None,
                        linear_restriction: torch.tensor = None,
                        orientational_restriction: torch.tensor = None,
                        direct_pose_cost: PoseCostMetric = None,
                        synchronization_required: bool = True):

        if synchronization_required and self._synchronizer == False:
            if SIM_TO_CONTROLLER_COMMUNICATION:
                try:
                    mechunit_name = "ROB_1" if self._ROS_JS_robot_indicator == "IRB6620_R1" else "ROB_2"
                    Actual_JS: JointTarget = client.abb_client.get_jointtarget(mechunit=mechunit_name)
                    if(Actual_JS != None):
                        print("Joint Values are Captured from Robot "+ self._ROS_JS_robot_indicator)
                        Is_Synced = self.move_to_js(Customized_JS=[math.radians(j) for j in Actual_JS.robax])
                        if Is_Synced:
                            print("Station and Simulation are Synced Now")
                            self._synchronizer = True
                        else:
                            print("WARNING: SIMULATION IS DESYNCHRONIZED")
                        print("__________________________________________________")
                except Exception as e:
                    print(f"Failed to get joint target: {e}")
                    print("WARNING: SIMULATION IS DESYNCHRONIZED")
                    print("__________________________________________________")

        # Updating MotionGenConfig if there is any new TCP Being Used
        if tcp_name != self._robot_cfg["kinematics"]["ee_link"]:
            self.motion_gen_warmup(TCP_Name=tcp_name)

        # A New Approach to Avoid CUDA Memory Occupation:
        if(update_world_needed):
            self.motion_gen_update_world(Removing_Prim_Paths=removing_primitives)

        # This is Used if a Direct Pose Cost Metric Was Defined !!!!
        if (direct_pose_cost == None):
            # Adding Path Planning Restrictions if provided
            if (linear_restriction != None) or (orientational_restriction != None) :
                self.restrict_path_plan(Position_Restriction= linear_restriction,
                                        Orientation_Restriction= orientational_restriction)
                
            if (linear_restriction == None) and (orientational_restriction == None):
                self.release_path_plan_restriction()
        else:
            self._plan_config = MotionGenPlanConfig(enable_graph=True,
                                                    enable_graph_attempt=2,
                                                    max_attempts=10,
                                                    enable_finetune_trajopt=False,
                                                    pose_cost_metric= direct_pose_cost,
                                                    time_dilation_factor= MOTION_ACCELERAION_VALUE) 

        result: MotionGenResult = None
        succ = None
        # Start the timer
        TimeOut_Timer = time.time()

        # Planning Announcement
        print("Robot "+self._ROS_JS_robot_indicator+" started to find a path for "+tcp_name+" to coords: ")
        print(f"Pose (X,Y,Z): {target_pose}, Orientation (W,X,Y,Z): {target_orientation}")

        # Giving a 10-second timer to solve IK
        while (time.time() - TimeOut_Timer <= 2):
            # Render
            self._temp_world_manager._my_world.step(render=True)
            # 2. Getting Current JS
            sim_js = self._robot.get_joints_state()
            sim_js_names = self._robot.dof_names

            cu_js = JointState(
                position=self._tensor_args.to_device(sim_js.positions),
                velocity=self._tensor_args.to_device(sim_js.velocities) * 0.0,
                acceleration=self._tensor_args.to_device(sim_js.velocities) * 0.0,
                jerk=self._tensor_args.to_device(sim_js.velocities) * 0.0,
                joint_names=sim_js_names,
            )
            cu_js = cu_js.get_ordered_joint_state(self._motion_gen.kinematics.joint_names)

            # 3. Check for a Solution
            # Updating Target Pose with Respect To the Robot's Location
            Adjusted_Target_Pose: np.array = [target_pose[0]-self._r_pose[0], target_pose[1]-self._r_pose[1], target_pose[2]-self._r_pose[2]]
            # target_pose[0] -= self._r_pose[0]
            # target_pose[1] -= self._r_pose[1]
            # ### Testing ????? Tired_Bear_Test
            # target_pose[2] -= self._r_pose[2]
            # # Z Does not need to be updated
            ik_goal = Pose(
                position=self._tensor_args.to_device(Adjusted_Target_Pose),
                quaternion=self._tensor_args.to_device(target_orientation),
            )
            result = self._motion_gen.plan_single(cu_js.unsqueeze(0), ik_goal, self._plan_config)
            succ = result.success.item()

            # Adding the solution to Robot Object
            self._computed_path_result = result

            if succ and np.max(np.abs(sim_js.velocities)) < 0.2:
                print("Solution Found")
                print("Execution in Progress")
                print("_____________________________________")

                # If the Plan is Successful, Update the Robot's Pose (It's not at home anymore)
                self._is_at_home = False

                # Clear the cache after each iteration to avoid memory buildupworld_reset
                self._computed_cmd_plan = self._computed_path_result.get_interpolated_plan()
                self._computed_cmd_plan = self._motion_gen.get_full_js(self._computed_cmd_plan)
                self._computed_idx_list = []
                common_js_names = []
                sim_js_names = self._robot.dof_names

                for x in sim_js_names:
                    if x in self._computed_cmd_plan.joint_names:
                        self._computed_idx_list.append(self._robot.get_dof_index(x))
                        common_js_names.append(x)
                self._computed_cmd_plan = self._computed_cmd_plan.get_ordered_joint_state(common_js_names)

                print(result.position_error)
                print(result.rotation_error)

                return True
            print(result.status)
            
        carb.log_warn("Plan did not converge to a solution: " + str(result.status))
        # No IK could solve this movement within 10 sec
        return False

    def render_exec(self,
                    renderInstance: bool = True,
                    # Optional To Show Spheres For Traj (True)
                    Show_Sphere: Optional[bool] = True,
                    is_synchronizer: bool = False):

        if not self._computed_path_result.success.item():
            print("Path was not Generated")
        else:
            cmd_plan = self._computed_path_result.get_interpolated_plan()
            cmd_plan = self._motion_gen.get_full_js(cmd_plan)
            # get only joint names that are in both:
            idx_list = []
            common_js_names = []
            sim_js_names = self._robot.dof_names

            for x in sim_js_names:
                if x in cmd_plan.joint_names:
                    idx_list.append(self._robot.get_dof_index(x))
                    common_js_names.append(x)
            cmd_plan = cmd_plan.get_ordered_joint_state(common_js_names)
            cmd_idx = 0

            # Adding the RWS Client to Publish the JS Array !
            my_tool = ABB.tooldata(True, ABB.pose([0, 0, 0.1], [1, 0, 0, 0]), ABB.loaddata(0.001, [0, 0, 0.001], [1, 0, 0, 0], 0, 0, 0))
            # Create a motion program
            mp = ABB.MotionProgram(tool=my_tool)

            while cmd_idx < len(cmd_plan.position):
                if renderInstance:
                    self._temp_world_manager._my_world.step(render=True)

                    # Visualizing Spheres
                    if Show_Sphere == True and self._temp_world_manager._my_world.current_time_step_index % 2 == 0:
                        # Getting Robot JointState
                        sim_js = self._robot.get_joints_state()
                        sending_state = self._tensor_args.to_device(sim_js.positions)

                        sph_list = self._motion_gen.kinematics.get_robot_as_spheres(sending_state)

                        # Check if the Sphere Representation is Generated Previously
                        spheres: List[sphere.VisualSphere] = []

                        # create spheres:
                        s: Sphere

                        # Appending the Created Spheres
                        for si, s in enumerate(sph_list[0]):
                            # We should Update Sphere Position with the Robot's Position (It's not 0, 0, 0 always)
                            updated_s_pose = [s.position[0]+self._r_pose[0],
                                                s.position[1]+self._r_pose[1],
                                                s.position[2]+self._r_pose[2]]
                            sp = sphere.VisualSphere(
                                prim_path="/curobo/"+self._ROS_JS_robot_indicator+"/sph" + str(si),
                                position=np.ravel(updated_s_pose),
                                radius=float(s.radius),
                                color=np.array([0, 0.8, 0.2]),
                            )
                            spheres.append(sp)
                        # If Sphere Representation Exists, Update it !
                        # else:
                        #     for si, s in enumerate(sph_list[0]):
                        #         if not np.isnan(s.position[0]):
                        #             # Updating Sphere Pose to the Robot's Pose
                        #             updated_s_pose = [s.position[0]+self._r_pose[0],
                        #                 s.position[1]+self._r_pose[1],
                        #                 s.position[2]+self._r_pose[2]]
                        #             spheres[si].set_world_pose(position=np.ravel(updated_s_pose))
                        #             spheres[si].set_radius(float(s.radius))

                cmd_state = cmd_plan[cmd_idx]

                # get full Truedof state
                art_action = ArticulationAction(
                    cmd_state.position.cpu().numpy(),
                    cmd_state.velocity.cpu().numpy(),
                    joint_indices=idx_list,
                )

                # if cmd_idx == 0 or cmd_idx % 10 == 0 or cmd_idx == len(cmd_plan.position)-1:
                # Creating RWS Position
                Cur_Pose: JointTarget = ABB.jointtarget(np.degrees(cmd_state.position.cpu().numpy()), [0]*6)

                if cmd_idx == len(cmd_plan.position)-1:
                    mp.MoveAbsJ(Cur_Pose, STATION_SPEED, ABB.fine)
                else:
                    mp.MoveAbsJ(Cur_Pose, STATION_SPEED, ABB.z100)
                # set desired joint angles obtained from IK:
                self._articulation_controller.apply_action(art_action)

                # Publishing To ROS
                self.ros_js_publisher()

                cmd_idx += 1
                if renderInstance:
                    for _ in range(2):
                        self._temp_world_manager._my_world.step(render=False)
            # If the synchronizing plan is being transfered, it cause the robot to Flip !
            if is_synchronizer == False:
                if SIM_TO_CONTROLLER_COMMUNICATION:
                    Rob_Task = "T_ROB1" if self._ROS_JS_robot_indicator == "IRB6620_R1" else "T_ROB2"
                    while client.is_motion_program_running():
                        self._temp_world_manager._my_world.step(render=True)

                    log_results = client.execute_motion_program(mp, task= Rob_Task, wait= False)

                    # Used to visualize RAPID MODULES created by the library
                    # RAPID_Results = mp.get_program_rapid()
                    # print(RAPID_Results)

                    # Waiting for the Robotic Station To Finish The Movement
                    while client.is_motion_program_running():
                        self._temp_world_manager._my_world.step(render=True)

            
            # input("Press Enter to continue...")

        # Cleaning out !
        self._computed_path_result = None
        self._computed_cmd_plan = None
        self._computed_idx_list = []

    def move_to_home(self,
                     if_show_spheres: bool = False,
                     Customized_JS: List[float] = [0, -0.5, 0.5, 0, 0, 0],
                     removing_primitives= []):

        # If Robot is Already at Home Position
        if self._is_at_home == True:
            print("Robot " + self._ROS_JS_robot_indicator + " is Already at Home Position")
            return True

        self.motion_gen_update_world(Removing_Prim_Paths= removing_primitives)

        self.release_path_plan_restriction()

        result: MotionGenResult = None
        succ = None
        # Start the timer
        TimeOut_Timer = time.time()

        # Giving a 10-second timer to solve IK
        while (time.time() - TimeOut_Timer <= 10):
            # Render
            self._temp_world_manager._my_world.step(render=True)
            # 2. Getting Current JS
            sim_js = self._robot.get_joints_state()
            sim_js_names = self._robot.dof_names

            cu_js = JointState(
                position=self._tensor_args.to_device(sim_js.positions),
                velocity=self._tensor_args.to_device(sim_js.velocities) * 0.0,
                acceleration=self._tensor_args.to_device(sim_js.velocities) * 0.0,
                jerk=self._tensor_args.to_device(sim_js.velocities) * 0.0,
                joint_names=sim_js_names,
            )
            cu_js = cu_js.get_ordered_joint_state(self._motion_gen.kinematics.joint_names)

            # Home Position is Defined as all 0
            Home_Loc = torch.tensor(Customized_JS, dtype=torch.float32).unsqueeze(0).cuda()
            home_state = JointState.from_position(
                    Home_Loc,
                    joint_names=[
                        "Joint_1",
                        "Joint_2",
                        "Joint_3",
                        "Joint_4",
                        "Joint_5",
                        "Joint_6",
                        ],
                )
            # A Planning Config For Getting Move to Home (With Trajectory Optimization being disabled)
            Home_Planner = MotionGenPlanConfig(time_dilation_factor= MOTION_ACCELERAION_VALUE)
            
            self._motion_gen.reset_seed()
            result = self._motion_gen.plan_single_js(cu_js.unsqueeze(0), home_state, Home_Planner)
            succ = result.success.item()


            # Adding the solution to Robot Object
            self._computed_path_result = result

            if succ and np.max(np.abs(sim_js.velocities)) < 0.2:
                print("Robot Movement To Home Position")
                print("_____________________________________")
                # Clear the cache after each iteration to avoid memory buildupworld_reset
                self._computed_cmd_plan = self._computed_path_result.get_interpolated_plan()
                self._computed_cmd_plan = self._motion_gen.get_full_js(self._computed_cmd_plan)
                self._computed_idx_list = []
                common_js_names = []
                sim_js_names = self._robot.dof_names

                for x in sim_js_names:
                    if x in self._computed_cmd_plan.joint_names:
                        self._computed_idx_list.append(self._robot.get_dof_index(x))
                        common_js_names.append(x)
                self._computed_cmd_plan = self._computed_cmd_plan.get_ordered_joint_state(common_js_names)

                # Execution
                self.render_exec(renderInstance= True, Show_Sphere= if_show_spheres)
                self._is_at_home = True
                return True
            print(result.status)
            
        carb.log_warn("Plan did not converge to a solution: " + str(result.status))
        # No IK could solve this movement within 10 sec
        return False

    # Used to Synchronize Simulation with the Actual Robots
    def move_to_js(self,
                     if_show_spheres: bool = False,
                     Customized_JS: List[float] = [0, 0, 0, 0, 0, 0]):

        self.motion_gen_update_world()

        self.release_path_plan_restriction()

        result: MotionGenResult = None
        succ = None
        # Start the timer
        TimeOut_Timer = time.time()

        # Giving a 10-second timer to solve IK
        while (time.time() - TimeOut_Timer <= 10):
            # Render
            self._temp_world_manager._my_world.step(render=True)
            # 2. Getting Current JS
            sim_js = self._robot.get_joints_state()
            sim_js_names = self._robot.dof_names

            cu_js = JointState(
                position=self._tensor_args.to_device(sim_js.positions),
                velocity=self._tensor_args.to_device(sim_js.velocities) * 0.0,
                acceleration=self._tensor_args.to_device(sim_js.velocities) * 0.0,
                jerk=self._tensor_args.to_device(sim_js.velocities) * 0.0,
                joint_names=sim_js_names,
            )
            cu_js = cu_js.get_ordered_joint_state(self._motion_gen.kinematics.joint_names)

            # Home Position is Defined as all 0
            Home_Loc = torch.tensor(Customized_JS, dtype=torch.float32).unsqueeze(0).cuda()
            home_state = JointState.from_position(
                    Home_Loc,
                    joint_names=[
                        "Joint_1",
                        "Joint_2",
                        "Joint_3",
                        "Joint_4",
                        "Joint_5",
                        "Joint_6",
                        ],
                )
            # A Planning Config For Getting Move to Home (With Trajectory Optimization being disabled)
            Home_Planner = MotionGenPlanConfig(time_dilation_factor= MOTION_ACCELERAION_VALUE)
            
            self._motion_gen.reset_seed()
            result = self._motion_gen.plan_single_js(cu_js.unsqueeze(0), home_state, Home_Planner)
            succ = result.success.item()


            # Adding the solution to Robot Object
            self._computed_path_result = result

            if succ and np.max(np.abs(sim_js.velocities)) < 0.2:
                # Clear the cache after each iteration to avoid memory buildupworld_reset
                self._computed_cmd_plan = self._computed_path_result.get_interpolated_plan()
                self._computed_cmd_plan = self._motion_gen.get_full_js(self._computed_cmd_plan)
                self._computed_idx_list = []
                common_js_names = []
                sim_js_names = self._robot.dof_names

                for x in sim_js_names:
                    if x in self._computed_cmd_plan.joint_names:
                        self._computed_idx_list.append(self._robot.get_dof_index(x))
                        common_js_names.append(x)
                self._computed_cmd_plan = self._computed_cmd_plan.get_ordered_joint_state(common_js_names)

                # Execution
                self.render_exec(renderInstance= True, Show_Sphere= if_show_spheres, is_synchronizer= True)
                self._is_at_home = True
                return True
            print(result.status)
            
        carb.log_warn("Plan did not converge to a solution: " + str(result.status))
        # No IK could solve this movement within 10 sec
        return False

    # Attaching Object (CuRobo Logic) => Debug Purposes (No Need To be Implemented)
    def IDC_Attach_Object_To_Robot(self,
                                    joint_state: JointState,
                                    object_names: List[str],
                                    attaching_link_name: str = 'tool0',
                                    sphere_number: int = 100,
                                    surface_sphere_radius: float = 0.05,
                                    sphere_fit_type: SphereFitType = SphereFitType.VOXEL_VOLUME_SAMPLE_SURFACE,
                                    voxelize_method: str = "ray",
                                    world_object_pose_offset: Optional[Pose] = None,
                                    remove_obstacles_from_world_config: bool = False) -> bool:
        
        """Attach an object or objects from world to a robot's link.

        This method assumes that the objects exist in the world configuration. If attaching
        objects that are not in world, use :meth:`MotionGen.attach_external_objects_to_robot`.

        Args:
            joint_state: Joint state of the robot.
            object_name: Name of object in the world to attach to the robot.
            surface_sphere_radius: Radius (in meters) to use for points sampled on surface of the
                object. A smaller radius will allow for generating motions very close to obstacles.
            link_name: Name of the link (frame) to attach the objects to. The assumption is that
                this link does not have any geometry and all spheres of this link represent
                attached objects.
            sphere_fit_type: Sphere fit algorithm to use. See :ref:`attach_object_note` for more
                details. The default method :attr:`SphereFitType.VOXEL_VOLUME_SAMPLE_SURFACE`
                voxelizes the volume of the objects and adds spheres representing the voxels, then
                samples points on the surface of the object, adds :attr:`surface_sphere_radius` to
                these points. This should be used for most cases.
            voxelize_method: Method to use for voxelization, passed to
                :py:func:`trimesh.voxel.creation.voxelize`.
            world_objects_pose_offset: Offset to apply to the object poses before attaching to the
                robot. This is useful when attaching an object that's in contact with the world.
                The offset is applied in the world frame before attaching to the robot.
            remove_obstacles_from_world_config: Remove the obstacles from the world cache after
                attaching to the robot to reduce memory usage. Note that when an object is attached
                to the robot, it's disabled in the world collision checker. This flag when enabled,
                also removes the object from world cache. For most cases, this should be set to
                False.
        """   

        if(self._motion_gen.kinematics.ee_link != attaching_link_name):
            print("Attaching is not possible | You should Rewarmup MotionGen with the Working TCP")
            return False

        log_info("MG: Attach objects to robot")
        kin_state = self._motion_gen.compute_kinematics(joint_state)
        ee_pose = kin_state.ee_pose
        if world_object_pose_offset is not None:
            ee_pose = world_object_pose_offset.inverse().multiply(ee_pose)
        ee_pose = ee_pose.inverse()
        max_spheres = self._motion_gen.robot_cfg.kinematics.kinematics_config.get_number_of_spheres(attaching_link_name)

        n_spheres = int(max_spheres / len(object_names))
        sphere_tensor = torch.zeros((max_spheres, 4))
        sphere_tensor[:, 3] = -10.0
        sph_list = []

        Testing_Spheres: List[Sphere] = None

        for i, x in enumerate(object_names):
            obs = self._motion_gen.world_model.get_obstacle(x)
            if obs is None:
                log_error(
                    "Object not found in world. Object name: "
                    + x
                    + " Name of objects in world: "
                    + " ".join([i.name for i in self._motion_gen.world_model.objects])
                )
            sph = obs.get_bounding_spheres(
                n_spheres,
                surface_sphere_radius,
                pre_transform_pose=ee_pose,
                tensor_args=self._tensor_args,
                fit_type=sphere_fit_type,
                voxelize_method=voxelize_method,
            )
            sph_list += [s.position + [s.radius] for s in sph]
            Testing_Spheres = sph

            self._motion_gen.world_coll_checker.enable_obstacle(enable=False, name=x)
            if remove_obstacles_from_world_config:
                self._motion_gen.world_model.remove_obstacle(x)
        log_info("MG: Computed spheres for attach objects to robot")

# Visualizing Spheres
        # # Check if the Sphere Representation is Generated Previously
        # Vis_Seph: List[sphere.VisualSphere] = []
        # # create spheres:
        # s: Sphere

        # # Appending the Created Spheres
        # for si, s in enumerate(Testing_Spheres):
        #     # We should Update Sphere Position with the Robot's Position (It's not 0, 0, 0 always)
        #     updated_s_pose = [s.position[0],
        #                         s.position[1],
        #                         s.position[2]]
        #     sp = sphere.VisualSphere(
        #         prim_path="/curobo/AttachedObj" + "/sph_obj" + str(si),
        #         position=np.ravel(updated_s_pose),
        #         radius=float(s.radius),
        #         color=np.array([0, 0.8, 0.2]),
        #     )
        #     Vis_Seph.append(sp)

        spheres = self._tensor_args.to_device(torch.as_tensor(sph_list)) 
        if spheres.shape[0] >= max_spheres:
            spheres = spheres[: spheres.shape[0]]
        sphere_tensor[: spheres.shape[0], :] = spheres.contiguous()

        self._motion_gen.attach_spheres_to_robot(sphere_tensor=sphere_tensor, link_name=attaching_link_name)
        return True
    
    # Used for Simulation Attach
    def isaac_tcp_attach(self,
                   robot_name: str = None,
                   tcp_name: str = "T1",
                   obj_name: str = "SP"):
        
        # Adding Object Attributes to make the Surface Gripper Attach the Object to it!
        Obj_Prim = self._temp_world_manager._stage.GetPrimAtPath("/world/obstacles/" + obj_name)
        
        # Enabling Colliders !!
        self._temp_world_manager._my_world.step(render= True)
        Dis_RigidBody = Obj_Prim.GetAttribute("physics:rigidBodyEnabled").Set(True)
        self._temp_world_manager._my_world.step(render= True)
        Dis_Collider = Obj_Prim.GetAttribute("physics:collisionEnabled").Set(True)
        self._temp_world_manager._my_world.step(render= True)
        Mass_Succ = Obj_Prim.GetAttribute("physics:mass").Set(1e-20)

        # Close
        og.Controller.set(og.Controller.attribute("/action_graph_"+self._ROS_JS_robot_indicator+"_"+tcp_name+"/close_tick.state:enableImpulse"), True)

        self._temp_world_manager._my_world.step(render= True)
        Gravity_Disabler = Obj_Prim.GetAttribute("physxRigidBody:disableGravity").Set(True)
    
    # Used for Simulation Detach
    def isaac_tcp_detach(self,
                   robot_name: str = None,
                   tcp_name: str = "T1",
                   obj_name: str = None):
        if obj_name == None:
            print("An Object Name Should Be Mentioned | Otherwise detach won't work (Isaac Sim Purposes)")
            return False

        # Disabling Gravity Right After Detaching
        Obj_Prim = self._temp_world_manager._stage.GetPrimAtPath("/world/obstacles/" + obj_name)

        self._temp_world_manager._my_world.step(render= True)
        Dis_Grav = Obj_Prim.GetAttribute("physxRigidBody:disableGravity").Set(False)

        # Open
        og.Controller.set(og.Controller.attribute("/action_graph_"+self._ROS_JS_robot_indicator+"_"+tcp_name+"/open_tick.state:enableImpulse"), True)

        self._temp_world_manager._my_world.step(render= True)       
        # Re-Disabling Gravity For the Object    
        Dis_Grav = Obj_Prim.GetAttribute("physxRigidBody:disableGravity").Set(True)

        self._temp_world_manager._my_world.step(render= True)
        Dis_Collider = Obj_Prim.GetAttribute("physics:collisionEnabled").Set(False)

        self._temp_world_manager._my_world.step(render= True)
        # Disabling Colliders !! (To Avoid Collision Problems After Detaching)
        Dis_RigidBody = Obj_Prim.GetAttribute("physics:rigidBodyEnabled").Set(False)
        self._temp_world_manager._my_world.step(render= True)

    def eef_attach(self,
                   r_name: str = "IRB6620_R1",
                   tool_name: str = "tool1",
                   attaching_object_name: str = None,
                   gen_sphere_radius: float = 0.001,
                   voxelization_method: SphereFitType = SphereFitType.VOXEL_VOLUME_SAMPLE_SURFACE):
        
        if(attaching_object_name == None):
            print("No Object Name Mentioned As Robot " + self._ROS_JS_robot_indicator + " Attachment | Command Aborted")
            print("Program will continue in 5 seconds ...")
            T_Now = time.time()
            while time.time() - T_Now <= 5:
                self._temp_world_manager._my_world.step(render=True)
            return False

        # Check for the active toolname
        active_tool_name = self._motion_gen.kinematics.kinematics_config.ee_link
        if(tool_name != active_tool_name):
            print("MotionGen is warmed up for " + active_tool_name + " on " + self._ROS_JS_robot_indicator + ", but " + 
                  tool_name + " is requested for attachement")
            print("Program will rewarmup " + self._ROS_JS_robot_indicator + " for " + tool_name + " in 5 seconds ...")
            T_Now = time.time()
            while time.time() - T_Now <= 5:
                self._temp_world_manager._my_world.step(render=True)

            self.motion_gen_warmup(TCP_Name=tool_name)          

        # Attaching the object withing the simulation to the virtual link (SurfaceGripper Attach)
        CV_Tool_name = f"T{tool_name[-1]}"
        self.isaac_tcp_attach(tcp_name= CV_Tool_name,
                              obj_name=attaching_object_name)
        
        # Attaching the object as spheres to the robot in MotionGen
        # 2. Getting Current JS
        sim_js = self._robot.get_joints_state()
        sim_js_names = self._robot.dof_names

        cu_js = JointState(
            position=self._tensor_args.to_device(sim_js.positions),
            velocity=self._tensor_args.to_device(sim_js.velocities) * 0.0,
            acceleration=self._tensor_args.to_device(sim_js.velocities) * 0.0,
            jerk=self._tensor_args.to_device(sim_js.velocities) * 0.0,
            joint_names=sim_js_names,
        )

        Updated_Obj_Name = "/world/obstacles/" + attaching_object_name
        # self._motion_gen.attach_objects_to_robot(
        #     cu_js,
        #     [Updated_Obj_Name],
        #     link_name= 'tool1',
        #     sphere_fit_type=SphereFitType.VOXEL_VOLUME_SAMPLE_SURFACE,
        #     surface_sphere_radius=0.05,
        #     remove_obstacles_from_world_config= True,
        #     world_object_pose_offset=Pose.from_list([attaching_object_pose[0],
        #                                               attaching_object_pose[1],
        #                                               attaching_object_pose[2],
        #                                               attaching_object_pose[3],
        #                                               attaching_object_pose[4],
        #                                               attaching_object_pose[5],
        #                                               attaching_object_pose[6]], self._tensor_args),
        # )

        # Using Corrent Motion Gen
        Is_Attach_Succ: bool = False
        Is_Attach_Succ = self.IDC_Attach_Object_To_Robot(
                            cu_js,
                            [Updated_Obj_Name],
                            sphere_fit_type=voxelization_method,
                            surface_sphere_radius=gen_sphere_radius,
                            attaching_link_name= tool_name,
                            # If you put 0 as Z axis, you have to avoid capturing the attaching object's collision representation
                            # to prevent the INVALID.START.STATE.WORLD.COLLISION error
                            
                            # To do so, _is_obj_attached variable is being set to True after attaching
                            # and this would avoid the collision world updated to get that object's mesh representation from Isaac Sim
                            world_object_pose_offset=Pose.from_list([0,0,0,1,0,0,0], self._tensor_args),
                            # remove_obstacles_from_world_config= True,
                        )
        
        # If attaching was succesful, the Attached Object Information should be updated
        if Is_Attach_Succ is True:
            print("Object " + attaching_object_name + " Sphere Representation Generated : " +
                    str(self._motion_gen.kinematics.kinematics_config.get_number_of_spheres(tool_name)) + " Spheres Used")
            print("Object " + attaching_object_name + " Attached to " + self._ROS_JS_robot_indicator)
            self._is_obj_attached = True
            self._attached_obj_prim = Updated_Obj_Name
        else:
            print("Failed to Attach Object" + attaching_object_name + " To Robot " + self._ROS_JS_robot_indicator)
            print("Proceeding to the Next Step")
    
    def eef_detach(self,
                   r_name: str = "IRB6620_R1",
                   tool_name: str = "tool0",
                   detaching_object_name: str = None):
        
        # Detaching the item from the robot in simulation (SurfaceGripper Action Graph)
        # Checking for tool prefix for calling the corresponding action graph e.g. => "tool0" => "T0"
        CV_Tool_name = f"T{tool_name[-1]}"
        self.isaac_tcp_detach(tcp_name= CV_Tool_name,
                              obj_name=detaching_object_name)
        

        # Detaching the item from the actual MotionGen object (in CuRobo)
        # It basically removes the generated sepheres attached to the virtual link (tool0 for example)
        self._motion_gen.detach_object_from_robot(tool_name)

        # Robot's Attaching Information Should Get Empty Again
        print("Object " + detaching_object_name + " Detached from " + self._ROS_JS_robot_indicator)
        self._is_obj_attached = False
        self._attached_obj_prim = "/world/obstacles/DummyObstacle"

    # Debuging Purposes (To Deactivate Robot's EndEffector Collision => To Attach Studs to the Grippers)
    def disable_eef_collider(self):
        Obj_Prim = self._temp_world_manager._stage.GetPrimAtPath("/" + self._ROS_JS_robot_indicator + "/Link_7/collisions")
        self._temp_world_manager._my_world.step(render=True)
        Dis_Co = Obj_Prim.GetAttribute("physics:collisionEnabled").Set(False)

    def ros_js_publisher(self):
        try:
            # Check if the simulation is running and playing
            if self._temp_world_manager._my_world.is_playing():
                # self._temp_world_manager._my_world.step(render=False)
                sim_js = self._robot.get_joints_state()
                sim_js_names = self._robot.dof_names
                # Check for NaN values
                if np.any(np.isnan(sim_js.positions)):
                    log_error("Can't Publish JointState for Robot: " + self._js_working_name)
                    return

                # Create and populate the JointState message
                joint_state_msg = sensor_msgs.msg.JointState()
                joint_state_msg.header.stamp = rospy.Time.now()
                joint_state_msg.name = sim_js_names
                joint_state_msg.position = self._tensor_args.to_device(sim_js.positions).cpu().numpy().tolist()
                joint_state_msg.velocity = self._tensor_args.to_device(sim_js.velocities).cpu().numpy().tolist()
                joint_state_msg.effort = [0.0] * len(sim_js_names)

                # Publish the joint state
                self._js_publisher.publish(joint_state_msg)

        except Exception as e:
            rospy.logwarn(f"Error publishing joint state: {e}")

#AI Agent
class AI_Agent:
    """OpenAI o3 helper that forces a get_robot_actions function call."""

    #OpenAI:
    # "https://api.openai.com/v1"
    # OPENAI_API_KEY

    #UFL:
    # "https://api.ai.it.ufl.edu"
    # UFL_API_KEY

    def __init__(
        self,
        api_key: str | None = None,
        model: str = "gpt-4o",
        base_url: str = "https://api.ai.it.ufl.edu",
    ):
        self.api_key = api_key or os.getenv("UFL_API_KEY")
        if not self.api_key:
            print("UFL_API_KEY not found; set it or pass api_key='…'")
            sys.exit(1)

        self.client = OpenAI(api_key=self.api_key, base_url=base_url)
        self.model = model

    # ------------------------------------------------------------------ #
    #  PUBLIC                                                            #
    # ------------------------------------------------------------------ #
    def get_robot_actions(self, prompt: str, image_uri: str) -> List[List]:
        """
        Pull one frame from /camera_rgb, send prompt + image to the model,
        force a get_robot_actions() function-call, and return decoded actions.
        """

        response = self.client.chat.completions.create(
            model= self.model,
            messages=[{
                        "role": "user",
                        "content": [
                            # --- your prompt & image ---
                            {"type": "text",  "text": prompt},
                            {"type": "image_url", "image_url": {"url": image_uri}}
                        ],
                    }],
            functions= FUNC_ACTION,
            function_call={"name": "get_robot_actions"}
        )

        # parse the function_call arguments
        func_call = response.choices[0].message.function_call
        reporting_list = json.loads(func_call.arguments).get("actions")

        return reporting_list


class RGBDCameraROS:
    """
    Utility wrapper that:
      1. Creates a Camera prim (RGB-D intrinsics) at the requested pose.
      2. Wires up ROS 1 publishers for /tf, RGB, depth, CameraInfo, and a
         depth-derived point-cloud.
    """

    def __init__(
        self,
        prim_path: str,
        position,              # list/np.ndarray [x,y,z] (world, Z-up, metres)
        orientation,           # list/np.ndarray Euler-deg [roll,pitch,yaw]   OR quat [w,x,y,z]
        resolution=(640, 480),
        frequency=30,          # sensor rate [Hz]  (render time is 60 Hz)
        pointcloud_rate=1,      # point-cloud down-sample [Hz]
        focal_length_mm: float | None = None,
        fov_deg: float | None = None
    ):
        self.prim_path = prim_path
        self.position  = np.asarray(position, dtype=float)
        self.focal_length_mm = focal_length_mm
        self.fov_deg         = fov_deg
        # accept either Euler-deg (length 3) or quaternion (length 4)
        if len(orientation) == 3:
            quat = rot_utils.euler_angles_to_quats(np.asarray(orientation, dtype=float),
                                                   degrees=True)
        else:
            quat = np.asarray(orientation, dtype=float)
        self.orientation     = quat
        self.resolution      = resolution
        self.frequency       = int(frequency)
        self.pointcloud_rate = int(pointcloud_rate)

        self._camera: Camera | None = None      # populated in spawn()

    # --------------------------------------------------------------------- #
    #  Internal helpers (straight from NVIDIA tutorial, wrapped in methods) #
    # --------------------------------------------------------------------- #
    def _set_gate_step(self, render_product: str, step_size: int, render_var: str):
        gate_path = omni.syntheticdata.SyntheticData._get_node_path(
            render_var + "IsaacSimulationGate", render_product
        )
        og.Controller.attribute(gate_path + ".inputs:step").set(step_size)

    def _attach_writer(self, sensor_type: sd.SensorType, writer_suffix: str,
                       topic: str, freq: int):
        render_product = self._camera._render_product_path
        step_size      = int(60 // freq)
        frame_id       = self._camera.prim_path.split("/")[-1]

        rv     = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
                     sensor_type.name)
        writer = rep.writers.get(rv + writer_suffix)
        writer.initialize(frameId=frame_id, nodeNamespace="", queueSize=1,
                          topicName=topic)
        writer.attach([render_product])
        self._set_gate_step(render_product, step_size, rv)

    # --------------------------------------------------------------------- #
    #  Public API                                                           #
    # --------------------------------------------------------------------- #
    def spawn(self):
        """Create camera prim + wire all publishers."""
        if self._camera is not None:
            return self._camera                         # already spawned

        # 1) Camera prim ----------------------------------------------------
        self._camera = Camera(
            prim_path=self.prim_path,
            resolution=self.resolution,
            frequency=self.frequency,
        )
        self._camera.initialize()

        # Setting Camera Pose
        self._camera.set_world_pose(position=self.position,
                                    orientation=self.orientation,
                                    camera_axes="usd")

        usd_cam = UsdGeom.Camera(
            omni.usd.get_context().get_stage().GetPrimAtPath(self.prim_path)
        )

        if self.focal_length_mm is not None:
            usd_cam.GetFocalLengthAttr().Set(float(self.focal_length_mm))

        elif self.fov_deg is not None:                       # compute focal length from desired FOV
            h_ap = usd_cam.GetHorizontalApertureAttr().Get() # mm
            f_len = 0.5 * h_ap / tan(radians(self.fov_deg) / 2)
            usd_cam.GetFocalLengthAttr().Set(float(f_len))

        # 2) ROS publishers -------------------------------------------------
        self._publish_camera_tf()
        self._publish_camera_info(self.frequency)
        self._publish_rgb(self.frequency)
        self._publish_depth(self.frequency)
        self._publish_pointcloud(self.pointcloud_rate)
        return self._camera

    # ------------------------------------------------------------------ #
    #  Individual publisher helpers                                      #
    # ------------------------------------------------------------------ #
    def _publish_camera_tf(self):
        """Dynamic world→camera and static camera→camera_optical /tf frames."""
        cam   = self._camera
        frame = cam.prim_path.split("/")[-1]
        graph_path = "/CameraTFActionGraph"
        if not is_prim_path_valid(graph_path):
            # skeleton (OnTick + clock) only once
            og.Controller.edit(
                {"graph_path": graph_path,
                 "evaluator_name": "execution",
                 "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION},
                {og.Controller.Keys.CREATE_NODES: [
                     ("OnTick",    "omni.graph.action.OnTick"),
                     ("IsaacTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                     ("ClockPub",  "omni.isaac.ros_bridge.ROS1PublishClock"),
                 ],
                 og.Controller.Keys.CONNECT: [
                     ("OnTick.outputs:tick",    "ClockPub.inputs:execIn"),
                     ("IsaacTime.outputs:simulationTime", "ClockPub.inputs:timeStamp"),
                 ]}
            )

        # camera TF nodes (can create multiple cameras safely)
        og.Controller.edit(
            graph_path,
            {og.Controller.Keys.CREATE_NODES: [
                 (f"TF_{frame}",       "omni.isaac.ros_bridge.ROS1PublishTransformTree"),
                 (f"TF_{frame}_world", "omni.isaac.ros_bridge.ROS1PublishRawTransformTree"),
             ],
             og.Controller.Keys.SET_VALUES: [
                 (f"TF_{frame}.inputs:topicName", "/tf"),
                 (f"TF_{frame}_world.inputs:topicName", "/tf"),
                 (f"TF_{frame}_world.inputs:parentFrameId", frame),
                 (f"TF_{frame}_world.inputs:childFrameId",  f"{frame}_world"),
                 (f"TF_{frame}_world.inputs:rotation",      [0.5, -0.5, 0.5, 0.5]),
             ],
             og.Controller.Keys.CONNECT: [
                 (f"{graph_path}/OnTick.outputs:tick",        f"TF_{frame}.inputs:execIn"),
                 (f"{graph_path}/OnTick.outputs:tick",        f"TF_{frame}_world.inputs:execIn"),
                 (f"{graph_path}/IsaacTime.outputs:simulationTime",
                      f"TF_{frame}.inputs:timeStamp"),
                 (f"{graph_path}/IsaacTime.outputs:simulationTime",
                      f"TF_{frame}_world.inputs:timeStamp"),
             ]}
        )
        set_target_prims(f"{graph_path}/TF_{frame}",
                         "inputs:targetPrims", [cam.prim_path])

    def _publish_camera_info(self, freq):
        render_product = self._camera._render_product_path
        step_size      = int(60 // freq)
        frame_id       = self._camera.prim_path.split("/")[-1]

        writer = rep.writers.get("ROS1PublishCameraInfo")
        writer.initialize(frameId=frame_id, nodeNamespace="", queueSize=1,
                          topicName=f"{self._camera.name}_camera_info",
                          stereoOffset=[0.0, 0.0])
        writer.attach([render_product])
        self._set_gate_step(render_product, step_size, "PostProcessDispatch")

    def _publish_rgb(self, freq):
        self._attach_writer(sd.SensorType.Rgb, "ROS1PublishImage",
                            topic=f"{self._camera.name}_rgb", freq=freq)

    def _publish_depth(self, freq):
        self._attach_writer(sd.SensorType.DistanceToImagePlane, "ROS1PublishImage",
                            topic=f"{self._camera.name}_depth", freq=freq)

    def _publish_pointcloud(self, freq):
        self._attach_writer(sd.SensorType.DistanceToImagePlane, "ROS1PublishPointCloud",
                            topic=f"{self._camera.name}_pointcloud", freq=freq)

############################
############################
##### Global Objects #######
############################
############################

###############################
###############################
#### Object Identification ####
###############################
###############################

test = WorldManager()
Robot_1 = None
Robot_1 = CuRoboRobot(working_world=test, 
                R_Name="IRB6620_R1",
                pose=[0,0,0.025],
                input_tool="tool0", 
                w_dir=INSTALLATION_DIRECTORY+"/Isaac_sim_ws/robot", 
                r_conf_name="IRB6620_Config.yaml",
                Gripper_List=[RobotGripper(RobName= "IRB6620_R1",
                                           ParentLink= "Link_6",
                                           TCP_Name= "T0",
                                           C_Pose= [0.09 , -0.3, -0.29]),
                                RobotGripper(RobName= "IRB6620_R1",
                                             ParentLink= "Link_6",
                                             TCP_Name= "T1",
                                             C_Pose= [0.55, 0.2, 0])
                                           ],
                Cuda_Device= 0)

Robot_2 = None
Robot_2 = CuRoboRobot(working_world=test,
                R_Name="IRB6620_R2",
                pose=[4.6, 0, 0.025],
                input_tool="tool0",
                w_dir=INSTALLATION_DIRECTORY+"/Isaac_sim_ws/robot_2",
                r_conf_name="IRB6620_Config.yaml",
                Gripper_List=[RobotGripper(RobName= "IRB6620_R2",
                                           ParentLink= "Link_6",
                                           TCP_Name="T0",
                                           C_Pose=[0.62, -0.13, -0.11]),
                                           ],
                Cuda_Device= 0)

Smart_Conv = CuRoboConv(working_world=test,
               Conv_Name="Smart_Conveyor",
               pose=[2.3, -3.25, (SMART_CONV_REST_ELEVATION-SMART_CONV_MODEL_ON_GROUND_HEIGHT)],
               w_dir=INSTALLATION_DIRECTORY+"/Isaac_sim_ws/smart_conveyor",
               c_conf_name="Smart_Conveyor.yaml")

IDC_Agent = AI_Agent()

# Camera
cam = RGBDCameraROS(
    prim_path="/World/Observer",
    position=[3.3, 3, 4.5],
    orientation=[0.3886, 0.05626, -0.13582, -0.9096],
    resolution=(1024, 768),
    pointcloud_rate=60,
    fov_deg=110                     # ultra-wide lens
).spawn()


###############################
###############################
#### Object Identification ####
###############################
###############################

############################
############################
#### Global Functions ######
############################
############################

def euler_to_quat(roll: float, pitch: float, yaw: float):
    """
    Convert Euler angles to a quaternion.

    Parameters
    ----------
    roll : float
        Rotation around the x-axis (in radians).
    pitch : float
        Rotation around the y-axis (in radians).
    yaw : float
        Rotation around the z-axis (in radians).

    Returns
    -------
    numpy.ndarray
        Quaternion [x, y, z, w] corresponding to the input Euler angles.
    """
    quat = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
    return quat

def Add_Rigid_Object_To_Scene(
    World_Manager: WorldManager,
    ObjectType: str = "Cuboid",
    obj: Any = Cuboid,
    rigid_body_disabler: bool = False,
    make_invisible: bool = False,
    base_frame: str = None
):
    """
    Add a rigid object to the simulation scene.

    Parameters
    ----------
    World_Manager : WorldManager
        The world scene manager you want to add the object to.
    ObjectType : str, default "Cuboid"
        The type of object to add. One of:
        "Cuboid", "Mesh", "Cylinder", or "Sphere".
    obj : Any, default Cuboid
        The pre-created object instance to add to the scene.
    rigid_body_disabler : bool, default False
        If False, enable rigid-body physics and collision for the object.
        If True, skip adding physics components (used for inverse kinematics exclusions).
    make_invisible : bool, default False
        If True, set the object’s visibility to invisible in the scene.

    Returns
    -------
    None
    """
    Added_Obj_Prim_Root: str = None
    placing_frame = "/world/obstacles"
    if base_frame != None:
        placing_frame = base_frame

    # It's better not to use CuRobo's enable_physics Attribute!
    if ObjectType == "Cuboid":
        Added_Obj_Prim_Root = World_Manager._usd_help.add_cuboid_to_stage(
            obstacle=obj, enable_physics=False, base_frame=placing_frame
        )
    elif ObjectType == "Mesh":
        Added_Obj_Prim_Root = World_Manager._usd_help.add_mesh_to_stage(
            obstacle=obj, enable_physics=False, base_frame=placing_frame
        )
    elif ObjectType == "Cylinder":
        Added_Obj_Prim_Root = World_Manager._usd_help.add_cylinder_to_stage(
            obstacle=obj, enable_physics=False, base_frame=placing_frame
        )
    elif ObjectType == "Sphere":
        Added_Obj_Prim_Root = World_Manager._usd_help.add_sphere_to_stage(
            obstacle=obj, enable_physics=False, base_frame=placing_frame
        )

    stage = World_Manager._my_world.stage
    Obj_Prim = stage.GetPrimAtPath(Added_Obj_Prim_Root)

    if not rigid_body_disabler:
        # Adding RigidBody
        execute("AddPhysicsComponentCommand",
                usd_prim=Obj_Prim,
                component="PhysicsRigidBodyAPI")
        # Adding Colliders
        execute("AddPhysicsComponentCommand",
                usd_prim=Obj_Prim,
                component="PhysicsCollisionAPI")

        # Here is an Alternative Method to add Collision Attribute (Using UsdPhysics)
        # UsdPhysics.CollisionAPI.Apply(Obj_Prim)

        # Adding ConvexHull for meshes (Not Working as of 12/06/2024)
        # Obj_Prim.CreateAttribute("physics:approximation").Set(UsdPhysics.Tokens.boundingCube)

        # Adding Colliders
        execute("AddPhysicsComponentCommand",
                usd_prim=Obj_Prim,
                component="PhysicsMassAPI")

        # Adding a Small Positive Mass to Avoid Robot's Effort Calculation using CuRobo
        Obj_Prim.GetAttribute("physics:mass").Set(0.0001)
        # Disabling Gravity
        Obj_Prim.GetAttribute("physxRigidBody:disableGravity").Set(True)

    # Making them Invisible
    if make_invisible:
        Obj_Prim.GetAttribute("visibility").Set("invisible")

    print(f"Object {obj.name} Added to the Simulation | PRIM: {Added_Obj_Prim_Root}")

    # Updating the Collision World for Each Robot After Adding an Object to the Scene
    if Robot_1 is not None:
        Robot_1.motion_gen_update_world()
    if Robot_2 is not None:
        Robot_2.motion_gen_update_world()

def Create_Wooden_Element_For_Smart_Mat_Table(
    el_name: str = None,
    L: float = None,
    W: float = None,
    H: float = None,
    Debug_Offset: bool = False
):
    """
    Create the wooden elements for the Smart Material Table.

    Parameters
    ----------
    el_name : str
        Name of the element.
    L : float
        Length of the wooden element (meters).
    W : float
        Width (thickness) of the wooden element (meters).
    H : float
        Height of the wooden element (meters).
    Debug_Offset : bool, default False
        If True, applies a debug offset when positioning the element
        (used for debugging purposes).

    Returns
    -------
    None
    """
    # compute an optional debugging offset along the table’s Y-axis
    Debugger: float = STUD_TO_SAW_OFFSET + L
    if not Debug_Offset:
        Debugger = 0

    Element = Cuboid(
        name=el_name,
        # 12 ft = 3.6576 m is the max length of the Smart Material Table
        pose=[
            SMART_MAT_TABLE[0] + (H/2),
            SMART_MAT_TABLE[1] - SMART_MAT_TABLE_MAX_LENGTH + (L/2) - Debugger,
            SMART_MAT_TABLE[2] + (W/2),
            1, 0, 0, 0
        ],
        dims=[H, L, W],
        color=[0.4, 0.2, 0, 1]
    )
    Add_Rigid_Object_To_Scene(test, "Cuboid", Element)

def Create_Wooden_Element_For_Sloped_Table(
    el_name: str = None,
    # L is constant since this table is only for 8 ft studs
    L: float = 2.4384,
    W: float = None,
    H: float = None
):
    """
    Create wooden elements for the sloped material table (fixed 8 ft length).

    Parameters
    ----------
    el_name : str
        Name of the element.
    L : float, default 2.4384
        Length of the wooden element (meters). Fixed at 8 ft (2.4384 m).
    W : float, optional
        Width (thickness) of the wooden element (meters).
    H : float, optional
        Height of the wooden element (meters).

    Returns
    -------
    None
    """
    # Creating the 8 ft wooden elements in Sloped Supply Table
    diagonal_stud_l = np.sqrt(W**2 + H**2)
    angle_rad = np.radians(SLOPED_MAT_TABLE_ANGLE)
    offset_angle = angle_rad + np.arcsin((W / 2) / (diagonal_stud_l / 2))
    z_increase = (diagonal_stud_l / 2) * np.sin(offset_angle)
    y_increase = (diagonal_stud_l / 2) * np.cos(offset_angle)

    # Convert Euler (-theta, 0, 0) to quaternion (xyzw)
    quat = R.from_euler('xyz', [-angle_rad, 0, 0]).as_quat()

    Element = Cuboid(
        name=el_name,
        # 8 ft element placement on the sloped table
        pose=[
            SLOPED_MAT_TABLE[0] + (L / 2),
            SLOPED_MAT_TABLE[1] - y_increase,
            SLOPED_MAT_TABLE[2] + z_increase,
            quat[3], quat[0], quat[1], quat[2]  # quaternion (w, x, y, z)
        ],
        dims=[L, H, W],
        color=[0.4, 0.2, 0, 1]
    )

    Add_Rigid_Object_To_Scene(test, "Cuboid", Element)

def Create_Wooden_Element_For_Sheathing_Table(
    el_name: str = None,
    L: float = None,
    W: float = None,
    H: float = None
):
    """
    Create wooden elements for the sheathing plate table.

    Parameters
    ----------
    el_name : str
        Name of the element.
    L : float
        Length of the sheathing element (meters).
    W : float
        Thickness of the sheathing element (meters).
    H : float
        Width of the sheathing element (meters).

    Returns
    -------
    None
    """
    Element = Cuboid(
        name=el_name,
        pose=[
            SHEATHING_PLATE_TABLE_BOTTOM_CENTER[0],
            SHEATHING_PLATE_TABLE_BOTTOM_CENTER[1] - (L / 2),
            SHEATHING_PLATE_TABLE_BOTTOM_CENTER[2] + (H / 2),
            1, 0, 0, 0
        ],
        dims=[W, L, H],
        color=[0.4, 0.2, 0, 1]
    )
    Add_Rigid_Object_To_Scene(test, "Cuboid", Element)

def Create_BearLoading_Element(
    el_name: str = None,
    X: float = None,
    Y: float = None,
    Z: float = None
):
    """
    Create a default dimensioned (RAW_HEADER_DIMENSIONS) bear loading header element at the specified position.

    Parameters
    ----------
    el_name : str
        Name of the bear loading element.
    X : float
        X-coordinate of the element’s position.
    Y : float
        Y-coordinate of the element’s position.
    Z : float
        Z-coordinate of the element’s position.

    Returns
    -------
    None
    """
    Element = Cuboid(
        name=el_name,
        pose=[X, Y, Z, 1, 0, 0, 0],
        dims=[
            RAW_HEADER_DIMENSIONS[2],
            RAW_HEADER_DIMENSIONS[0],
            RAW_HEADER_DIMENSIONS[1]
        ],
        color=[0.4, 0.2, 0, 1]
    )
    Add_Rigid_Object_To_Scene(test, "Cuboid", Element)

def Drag_Stud(
    el_name: str = None,
    el_dims: List[float] = None
):
    """
    Drag a 12ft raw stud from the Smart Material Table to cut in length (if lower than 8ft/2.4384m length is needed) for framing.

    Parameters
    ----------
    el_name : str
        Base name of the element to drag.
    el_dims : List[float]
        Dimensions of the element [length, width (thickness), height] in meters.

    Returns
    -------
    None
    """
    # Correcting Movement Before Reaching the Smart Material Table
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [5.0, 1.08, 0.87],
                    target_orientation= [0, 1, 0, 0],
                    update_world_needed= True)
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)

    # Helping Pick (X -= -0.3)
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [SMART_MAT_TABLE[0]+PICK_OFFSET_FROM_W_CORNER-0.3,
                                  SMART_MAT_TABLE[1]-SMART_MAT_TABLE_MAX_LENGTH+PICK_OFFSET_FROM_L_CORNER+(ROBOT_2_GRIPPER_LENGTH/2),
                                  SMART_MAT_TABLE[2]+(el_dims[1]/2)],
                    target_orientation= [0, ev, 0, ev],
                    update_world_needed= True)
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)
    
    # Drag Position
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [SMART_MAT_TABLE[0]+PICK_OFFSET_FROM_W_CORNER,
                                  SMART_MAT_TABLE[1]-SMART_MAT_TABLE_MAX_LENGTH+PICK_OFFSET_FROM_L_CORNER+(ROBOT_2_GRIPPER_LENGTH/2),
                                  SMART_MAT_TABLE[2]+(el_dims[1]/2)],
                    target_orientation= [0, ev, 0, ev],
                    update_world_needed= True,
                    removing_primitives=["world/obstacles"],
                    orientational_restriction=torch.tensor([1,1,1], dtype=torch.float32))
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)
    
    # Letting the Robot to Drag up to 30cm for each step !
    ALLOWED_DRAG_STEP: float = 0.5
    # Robot_2 needs to drag the stud by the Jack's length to let the saw cut it !
    Drag_Counts: int = (el_dims[0]+STUD_TO_SAW_OFFSET) // ALLOWED_DRAG_STEP
    Drag_Remained: float = np.round((el_dims[0]+STUD_TO_SAW_OFFSET) % ALLOWED_DRAG_STEP, 5)

    Drag_Counter: int = 0

    dc=_dynamic_control.acquire_dynamic_control_interface()
    # Creating Element (With Respect to the Smart Material Supply's Length)
    Create_Wooden_Element_For_Smart_Mat_Table(el_name= el_name+"Temp", L= SMART_MAT_TABLE_MAX_LENGTH, W= el_dims[1], H= el_dims[2])

    while Drag_Counter < Drag_Counts:
        # Attach
        Robot_2.eef_attach(tool_name= "tool0",
                           attaching_object_name= el_name+"Temp")
        
        # Drag with Linear Restriction !
        # Drag Forward
        object=dc.get_rigid_body("/"+Robot_2._ROS_JS_robot_indicator+"/tool0")
        object_pose=dc.get_rigid_body_pose(object)
        Robot_2.plan(tcp_name= "tool0",
                        target_pose= [object_pose.p[0], object_pose.p[1]-ALLOWED_DRAG_STEP, object_pose.p[2]],
                        target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                        update_world_needed= True,
                        removing_primitives=["world/obstacles"],
                        direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=1))
        Robot_2.render_exec(renderInstance= True,
                                Show_Sphere= False)

        Robot_2.eef_detach(tool_name= "tool0",
                           detaching_object_name= el_name+"Temp")

        object=dc.get_rigid_body("/"+Robot_2._ROS_JS_robot_indicator+"/tool0")
        object_pose=dc.get_rigid_body_pose(object)
        Robot_2.plan(tcp_name= "tool0",
                        target_pose= [object_pose.p[0], object_pose.p[1]+ALLOWED_DRAG_STEP, object_pose.p[2]],
                        target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                        update_world_needed= True,
                        removing_primitives=["world/obstacles/R2_Smart_Mat_Table_Box2", "world/obstacles/R2_Smart_Mat_Table_Box1"],
                        direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=1))
        Robot_2.render_exec(renderInstance= True,
                                Show_Sphere= False)

        Drag_Counter += 1
    
    # Drag Once More For The Remaining Length !

    # Attach
    Robot_2.eef_attach(tool_name= "tool0",
                        attaching_object_name= el_name+"Temp")
    
    # Drag with Linear Restriction !
    # Drag Forward
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [SMART_MAT_TABLE[0]+PICK_OFFSET_FROM_W_CORNER,
                                SMART_MAT_TABLE[1]-SMART_MAT_TABLE_MAX_LENGTH+PICK_OFFSET_FROM_L_CORNER+(ROBOT_2_GRIPPER_LENGTH/2)-Drag_Remained,
                                SMART_MAT_TABLE[2]+(el_dims[1]/2)],
                    target_orientation= [0, ev, 0, ev],
                    update_world_needed= True,
                    removing_primitives=["world/obstacles"],
                    direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=1))
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)

    Robot_2.eef_detach(tool_name= "tool0",
                        detaching_object_name= el_name+"Temp")
    
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [SMART_MAT_TABLE[0]+PICK_OFFSET_FROM_W_CORNER,
                                SMART_MAT_TABLE[1]-SMART_MAT_TABLE_MAX_LENGTH+PICK_OFFSET_FROM_L_CORNER+(ROBOT_2_GRIPPER_LENGTH/2),
                                SMART_MAT_TABLE[2]+(el_dims[1]/2)],
                    target_orientation= [0, ev, 0, ev],
                    update_world_needed= True,
                    removing_primitives=["world/obstacles/R2_Smart_Mat_Table_Box2", "world/obstacles/R2_Smart_Mat_Table_Box1"],
                    direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=1))
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)
    
    # Post Drag Position
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [SMART_MAT_TABLE[0]+PICK_OFFSET_FROM_W_CORNER-0.3,
                                  SMART_MAT_TABLE[1]-SMART_MAT_TABLE_MAX_LENGTH+PICK_OFFSET_FROM_L_CORNER+(ROBOT_2_GRIPPER_LENGTH/2),
                                  SMART_MAT_TABLE[2]+(el_dims[1]/2)],
                    target_orientation= [0, ev, 0, ev],
                    update_world_needed= True,
                    removing_primitives=["world/obstacles/R2_Smart_Mat_Table_Box2", "world/obstacles/R2_Smart_Mat_Table_Box1"],
                    orientational_restriction=torch.tensor([1,1,1], dtype=torch.float32))
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)

    print("DRAGGING DONE ! Lenth Dragged Out To Cut: "+str(el_dims[0])+"m")

############################
############################
#### Global Functions ######
############################
############################

###############################
###############################
##### Station Functions #######
###############################
###############################

# Automatically Extract the 8 Key Corners of a Wooden Element
def stud_corners_quat(center, quat, length, width, height):
    w, x, y, z = quat
    n = np.sqrt(w*w + x*x + y*y + z*z)
    w, x, y, z = w/n, x/n, y/n, z/n
    
    R = np.array([
        [1 - 2*(y*y + z*z),  2*(x*y - z*w),      2*(x*z + y*w)],
        [2*(x*y + z*w),      1 - 2*(x*x + z*z),  2*(y*z - x*w)],
        [2*(x*z - y*w),      2*(y*z + x*w),      1 - 2*(x*x + y*y)]
    ])
    
    L_local = np.array([0, 1, 0])
    W_local = np.array([0, 0, 1])
    H_local = np.array([1, 0, 0])
    
    L_hat = R @ L_local
    W_hat = R @ W_local
    H_hat = R @ H_local
    
    vL = 0.5 * length * L_hat
    vW = 0.5 * width  * W_hat
    vH = 0.5 * height * H_hat
    
    c = np.asarray(center)
    corners = [c + sL*vL + sW*vW + sH*vH
               for sL in (-1, 1)
               for sW in (-1, 1)
               for sH in (-1, 1)]
    return np.vstack(corners)

def rgb_to_uri(rgb: np.ndarray) -> str:
    """
    Convert an (H×W×3) or (N×3) RGB array into a PNG data-URI.

    Args:
        rgb: RGB data returned by cam.get_rgb(). Values may be uint8 (0-255)
             or float (0-1).

    Returns:
        A string of the form "data:image/png;base64,<...>".
    """
    # ---- normalise to uint8 -------------------------------------------------
    if rgb.dtype != np.uint8:
        rgb = np.clip(rgb * (255 if rgb.max() <= 1 else 1), 0, 255).astype(np.uint8)

    # ---- reshape if input was flat (N×3) ------------------------------------
    if rgb.ndim == 2:                       # (N, 3)  →  (H, W, 3)
        N = rgb.shape[0]
        H = int(np.sqrt(N))
        W = N // H
        rgb = rgb.reshape(H, W, 3)

    # ---- encode as PNG in-memory -------------------------------------------
    img     = Image.fromarray(rgb, mode="RGB")
    buffer  = BytesIO()
    img.save(buffer, format="PNG")
    b64_png = base64.b64encode(buffer.getvalue()).decode()

    return f"data:image/png;base64,{b64_png}"

# ---------------------------------------------------------------
#  Helper: pick robot instance by id (1 → Robot_1, 2 → Robot_2)
# ---------------------------------------------------------------
def _pick_robot(rid: int):
    if rid == 1:
        return Robot_1
    elif rid == 2:
        return Robot_2
    else:
        raise ValueError(f"[AI] invalid robot id {rid} (only 1 or 2 allowed)")

def Automated_Pick_Place_No_Cut(
    el_name: str = None,
    X: float = None,
    Y: float = None,
    Z: float = None,
    L: float = None,
    W: float = None,
    H: float = None):

    '''
    Parameters:
        el_name (str): Name of the element to place.
        X (float): X value of element's position from the top left corner of the building panel.
        Y (float): Y value of element's position from the top left corner of the building panel.
        Z (float): Z value of element's position from the top left corner of the building panel.
        L (float): Length of the element.
        W (float): Width (Thickness) of the element.
        H (float): Height of the element.
    '''
    dc=_dynamic_control.acquire_dynamic_control_interface()

    # Creating the Stud to Pick within the simulation
    Create_Wooden_Element_For_Smart_Mat_Table(el_name=el_name, L=L, W=W, H=H)
    # Calculate Corner Coordinations
    object=dc.get_rigid_body("/world/obstacles/"+el_name)
    object_pose_pick=dc.get_rigid_body_pose(object)
    Picking_Stud_Corners= stud_corners_quat(object_pose_pick.p, [object_pose_pick.r[3], object_pose_pick.r[0], object_pose_pick.r[1], object_pose_pick.r[2]], L, W, H)
    object_pose_pick.r = [object_pose_pick.r[3], object_pose_pick.r[0], object_pose_pick.r[1], object_pose_pick.r[2]]

    # Assuming object_pose_pick.r is [w, x, y, z]
    quat = np.array([object_pose_pick.r[0],
                     object_pose_pick.r[1],
                     object_pose_pick.r[2],
                     object_pose_pick.r[3]], dtype=float)           # (4,)

    # If rot_utils.quats_to_euler_angles expects shape (N,4), wrap an extra axis
    quat = quat.reshape(1, 4)                                       # (1,4)

    rpy = rot_utils.quats_to_euler_angles(
        quaternions=quat,
        degrees=True,      # return degrees
        extrinsic=True     # XYZ extrinsic convention
    )

    # round to whole degrees (still float so that -0. appears as 0.)
    rpy_rounded = np.round(rpy, 0)
    print("RPY rounded:", rpy_rounded)

    # ─── fixed transform that carries [0 0 0] → [180 90 0] ───
    from scipy.spatial.transform import Rotation as R
    R_T = np.array([[ 0.,  0., -1.],        # Rz(0°) · Ry(90°) · Rx(180°)
                    [ 0., -1.,  0.],
                    [-1.,  0.,  0.]])

    # compose the transform with the current rotation
    R_current = R.from_euler('xyz', rpy_rounded[0], degrees=True).as_matrix()
    R_new     = R_T @ R_current

    # back to Euler, round, then normalise to 0-360 for nicer printing
    rpy_transformed = np.rint(
        R.from_matrix(R_new).as_euler('xyz', degrees=True)
    ).astype(int)

    rpy_transformed = (rpy_transformed + 360) % 360   # put −180 → 180 etc.
    rpy_transformed[rpy_transformed == 360] = 0        # avoid 360 showing up

    print("RPY after transform:", rpy_transformed)     # → [180  90   0]

    Robot_2.free_TCP_movement("tool0")

    # Move Conveyor Belt (Temporary)
    Smart_Conv.render_exec('Joint_1', Y - (OVERALL_PANEL_LENGTH/2) + ((L/2)-PICK_OFFSET_FROM_L_CORNER-(ROBOT_2_GRIPPER_LENGTH/2)+(SMART_CONV_RANGE_OF_MOTION_J1/2)))

    # Creating the Stud that represents the Placed element within the simulation
    Element = Cuboid(
        name=el_name+"_Final_Pose",
        # 12 ft = 3.6576 m is the max length of the Smart Material Table
        pose=[3.63075, 1.48443, 0.94819, ev, 0, ev, 0],
        dims=[H, L, W],
        color=[1, 0, 0, 1]
    )
    Add_Rigid_Object_To_Scene(test, "Cuboid", Element)
    # Calcuate Corner Coordinations
    object=dc.get_rigid_body("/world/obstacles/"+el_name+"_Final_Pose")
    object_pose_place=dc.get_rigid_body_pose(object)
    Placing_Stud_Corners= stud_corners_quat(object_pose_place.p, [object_pose_place.r[3], object_pose_place.r[0], object_pose_place.r[1], object_pose_place.r[2]], L, W, H)
    object_pose_place.r = [object_pose_place.r[3], object_pose_place.r[0], object_pose_place.r[1], object_pose_place.r[2]]

    # Assuming object_pose_pick.r is [w, x, y, z]
    quat = np.array([object_pose_place.r[0],
                     object_pose_place.r[1],
                     object_pose_place.r[2],
                     object_pose_place.r[3]], dtype=float)           # (4,)

    # If rot_utils.quats_to_euler_angles expects shape (N,4), wrap an extra axis
    quat = quat.reshape(1, 4)                                       # (1,4)

    rpy = rot_utils.quats_to_euler_angles(
        quaternions=quat,
        degrees=True,      # return degrees
        extrinsic=True     # XYZ extrinsic convention
    )

    # round to whole degrees (still float so that -0. appears as 0.)
    rpy_rounded = np.round(rpy, 0)
    print("RPY rounded:", rpy_rounded)

    # ─── fixed transform that carries [0 0 0] → [180 90 0] ───
    from scipy.spatial.transform import Rotation as R
    R_T = np.array([[ 0.,  0., -1.],        # Rz(0°) · Ry(90°) · Rx(180°)
                    [ 0., -1.,  0.],
                    [-1.,  0.,  0.]])

    # compose the transform with the current rotation
    R_current = R.from_euler('xyz', rpy_rounded[0], degrees=True).as_matrix()
    R_new     = R_T @ R_current

    # back to Euler, round, then normalise to 0-360 for nicer printing
    rpy_transformed = np.rint(
        R.from_matrix(R_new).as_euler('xyz', degrees=True)
    ).astype(int)

    print("RPY after transform:", rpy_transformed)     # → [180  90   0]

    Robot_2.free_TCP_movement("tool0")

    # Robots Gripper TCPs:
    Rob_1_GTCP= dc.get_rigid_body_pose(dc.get_rigid_body("/IRB6620_R1/tool0"))
    Rob_1_GTCP.r = [Rob_1_GTCP.r[3], Rob_1_GTCP.r[0], Rob_1_GTCP.r[1], Rob_1_GTCP.r[2]]

    Rob_2_GTCP= dc.get_rigid_body_pose(dc.get_rigid_body("/IRB6620_R2/tool0"))
    Rob_2_GTCP.r = [Rob_2_GTCP.r[3], Rob_2_GTCP.r[0], Rob_2_GTCP.r[1], Rob_2_GTCP.r[2]]

    # ── failure-code → plain-text legend ─────────────────────────────
    STATUS_LEGEND = {
        "IK_FAIL":                          "Inverse kinematics failed to find a solution. The robot cannot reach that location or obstacles avoiding such reachability.",
        "INVALID_START_STATE_WORLD_COLLISION":  "Start state is colliding with the world. The robot is in collision with the world environment.",
        "INVALID_START_STATE_SELF_COLLISION":   "Start state is in self-collision. The robot is in the collision with itself.",
        "INVALID_START_STATE_JOINT_LIMITS":     "Start state violates joint limits.",
        "INVALID_PARTIAL_POSE_COST_METRIC":     "Invalid partial-pose cost metric."
    }
    # ---- build legend text once ------------------------------------
    legend_txt = "\n".join(
        f"  • {code} → {desc}" for code, desc in STATUS_LEGEND.items()
    )

    #Creating an LLM Feedback Generator
    Initial_Prompt = (
        "Assume you are a robotic-station executive command generator.\n"
        "Your task is to devise a sequence for picking and placing a wooden stud.\n"
        "You may call two fundamental functions:\n"
        "  1.  Robot_X.move(Tool_Name, [X,Y,Z], [R, P ,Y]) — performs inverse-kinematic "
        "      path planning to move the specified tool to the target pose.\n"
        "  2.  Robot_X.trigger(Tool_Name, Element_Name) — activates the specified tool "
        "      (nail-gun, gripper, or suction cup). If the tool carries an element—"
        "      e.g., the gripper lifts a stud—pass the element’s name.\n\n"
        "You are given below data to design the task:\n"
        # "  •  The stud’s current centre pose and its destination centre pose "
        # "     (both in [X,Y,Z], [W,X,Y,Z] format).\n"
        "  •  Eight corner coordinates (X,Y,Z) for the stud in both the pick and "
        "     place locations.\n"
        # "  •  The current poses of all robotic grippers in the station (Robot 1 and 2 are only presented in the station as of now) "
        # "     ([X,Y,Z], [W,X,Y,Z]).\n"
        # "  •  An image visualizing the stage. Within this image including RED SPHERES in it beside the stud and the scene. "
        # "     These RED SPHERES are representing the failure points that you previously tested upon generating 1 sub action to accomplish the task. "
        # "     in other word, the failed locations within the section below IMPORTANT NOTES are depicted to help you decide a better choice for the next"
        # "     step."
        # "  •  Position and Orientation of the Frame that took the given picture([X,Y,Z], [W,X,Y,Z])."

        "Using this information, produce an ordered array of function calls that "
        "achieves the pick-and-place operation. If execution fails, details from the "
        "simulation appear at the end of this prompt under **IMPORTANT NOTES**; you "
        "must incorporate any statements found there."
        "The IMPORTANT NOTES section lists coded reasons for any movement failures/success that you must consider on "
        "this attempt. For the sake of resources, Make sure to do binary search in the direction of stud's length to find the best executable location.\n"
        "Lastly, legend translating each code into its plain-language explanation is also included for failure reasons.\n"
        "if a success statement is mentioned within the IMPORTANT NOTES section, you have to only consider that action for the "
        "corresponding substep of compliting the task.\n"
        # "Output specification:\n"
        # "  •  Return a *list of arrays*.\n"
        # "  •  Index 0:  0 for a movement call, 1 for a trigger call.\n"
        # "  •  Index 1:  the robot number used for the action.\n"
        # "  •  Indices 2-8:  [X, Y, Z, W, X, Y, Z] — the target pose and orientation "
        # "     (use zeros if this is a trigger call).\n"
        # "  •  Indices 9-10:  strings → Tool_Name (always “tool0” is acceptable) and "
        # "     Element_Name (“None” when the entry represents a movement).\n\n"

        "Information Needed:\n\n"

        # " • Stud's current centre pose:\n"
        # f"{object_pose_pick}\n\n"
        # " • Stud's final centre pose:\n"
        # f"{object_pose_place}\n\n"
        " • Stud's current corner coordinates:\n"
        f"{Picking_Stud_Corners}\n\n"
        " • Stud's final corner coordinates:\n"
        f"{Placing_Stud_Corners}\n\n"
        # " • Robot 1's gripper center point pose:\n"
        # f"{Rob_1_GTCP}\n\n"
        # " • Robot 2's gripper center point pose:\n"
        # f"{Rob_2_GTCP}\n\n"
       " • Coded Reasons of Failure:\n"
        f"{legend_txt}\n\n"
    #    " • Camera frame pose\n"
    #     f"{cam.get_world_pose()}\n\n"

        "**IMPORTANT NOTES**\n"
    )

    Success_Flag = False
    counter=1

    Movement_Counter=0

    while Success_Flag == False:
        # Extracting RGB-Array and Sending Over:
        Image_URI = rgb_to_uri(cam.get_rgb())

        Action_Sequence = IDC_Agent.get_robot_actions(Initial_Prompt, Image_URI)
        print(Action_Sequence)

        Solver = True
        for step_idx, arr in enumerate(Action_Sequence, start=1):

            if len(arr) != 11:
                print(f"[WARN] Step {step_idx}: expected 11 items, got {len(arr)} → skipping")
                continue

            call_type, rid        = arr[0], arr[1]
            pose                  = arr[2:5]
            orient                = arr[5:9]
            tool_name, elem_name  = arr[9], arr[10]

            robot = _pick_robot(rid)

            # ─────────────── movement call ───────────────
            if call_type == 0:
                ok = robot.plan(
                    tcp_name=tool_name,
                    target_pose=pose,
                    target_orientation=orient,
                    removing_primitives=["world/obstacles", "Smart_Conveyor", "World/Vis_Dots"],
                )

                if not ok:
                    # ── 1. Acquire status text ───────────────────────────────────
                    status_obj = getattr(robot, "_computed_path_result", None)
                    status_txt = status_obj.status if status_obj else "<status unavailable>"

                    # ── 2. Build failure statement ───────────────────────────────
                    failure_note = (
                        f"It's impossible to move Robot {rid}'s {tool_name} "
                        f"to the location Position={pose} | Orientation={orient} "
                        f"due to the reason: {status_txt}."
                    )

                    # ── 3. Append to next prompt under IMPORTANT NOTES ───────────
                    Initial_Prompt += f"\n{failure_note}"

                    # Creating a Red Sphere that is representing the failed Goal Pose
                    new_dot = Sphere(
                        name="Dot_"+str(counter),
                        pose= arr[2:9],
                        radius=0.07,
                        color=[1, 0, 0, 1]
                    )
                    counter+=1
                    Add_Rigid_Object_To_Scene(test, "Sphere", new_dot, True, False, "/World/Vis_Dots")

                    # 2 Sec pause to render
                    TT = time.time()
                    while time.time() - TT <= 2:
                        test._my_world.step(render=True)
                    
                    # ── 4. Console diagnostics ───────────────────────────────────
                    print(f"[FAIL] Step {step_idx}: {failure_note}")
                    Solver = False
                    break

                if ok:
                    if(Movement_Counter < step_idx):
                        print("A new Substep Solved")
                        Movement_Counter = step_idx

                        # Resetting all the Dots !
                        counter = 0
                        prims_utils.delete_prim("/World/Vis_Dots")

                        success_note = (
                            f"Robot {rid}'s {tool_name} can reach the location: Position={pose} | Orientation={orient} "
                            f"which should be used as the subtask number {Movement_Counter} from now on."
                        )

                        Initial_Prompt += f"\n{success_note}"

                        #Removing Useless prompts (Associated with the previous dots)
                        head, *rest = Initial_Prompt.split("**IMPORTANT NOTES**", 1)
                        tail = rest[0]
                        # Keep every line that does *not* match the unwanted phrase
                        kept_lines = [
                            line for line in tail.splitlines()
                            if not re.match(r"\s*It's impossible to move Robot", line)
                        ]
                        # Re-assemble the cleaned prompt
                        cleaned_prompt = head + "**IMPORTANT NOTES**\n" + "\n".join(kept_lines)
                        Initial_Prompt = cleaned_prompt

            # ─────────────── bad opcode ──────────────────
            else:
                print(f"[WARN] Step {step_idx}: unknown call_type {call_type}; skipping")

        # At this point Success_Flag is True only if every move succeeded.
        if Solver == True:
            Success_Flag = True

        print(Initial_Prompt)

        # Robot_1.free_TCP_movement("tool0")

    print("Succeeded Result:")
    print(Action_Sequence)

###############################
###############################
##### Station Functions #######
###############################
###############################

##############################
##############################
#### Execution Platform ######
##############################
##############################

def main():

    rospy.init_node("tutorial_subscriber", anonymous=True)

    i=0

    while simulation_app.is_running():
        # Rendering The World
        test._my_world.step(render=True)
        if not test._my_world.is_playing():
                if i % 100 == 0:
                    print("**** Click Play to start simulation *****")
                i += 1
                continue

        step_index = test._my_world.current_time_step_index
        # Re initializing robots defined in the list
        if(Robot_1 != None):
            Robot_1.articulation_controller_init(step_index)
        if(Robot_2 != None):
            Robot_2.articulation_controller_init(step_index)
        
        # Re initializing conveyors defined in the list
        Smart_Conv.articulation_controller_init(step_index)

        if step_index < 20:
            continue

        # Creating The Bear Loading Pile 
        it: int = 0
        while it < NUMBER_OF_HEADERS:
            r=1
            Create_BearLoading_Element(el_name= "Bear_Loading_Element_"+str(it+1),
                                        X= HEADER_CENTER_COORINATION[0]-(RAW_HEADER_DIMENSIONS[2]/2),
                                        Y= HEADER_CENTER_COORINATION[1],
                                        Z= HEADER_CENTER_COORINATION[2]+((it+0.5)*RAW_HEADER_DIMENSIONS[1]))
            # Diabling Colliders
            test._stage.GetPrimAtPath("/world/obstacles/Bear_Loading_Element_"+str(it+1)).GetAttribute("physics:collisionEnabled").Set(False)
            it+=1

        # Making the Ground Invisible
        GR_Primitive = test._stage.GetPrimAtPath("/World/obstacles/Ground")
        GR_Primitive.GetAttribute("visibility").Set("invisible")

        # Smart Material Table's Collision
        R2_Smart_Mat_Table_Box1 = Cuboid(
            name= "R2_Smart_Mat_Table_Box1",
            pose= [6.8, 2.7, 0.45, 1, 0, 0, 0],
            dims= [1, 4, 0.8],
            color= [1, 1, 1, 0]
        )
        Add_Rigid_Object_To_Scene(test, "Cuboid", R2_Smart_Mat_Table_Box1, True, True)

        R2_Smart_Mat_Table_Box2 = Cuboid(
            name= "R2_Smart_Mat_Table_Box2",
            pose= [6.75, 2.3, 1.57, 1, 0, 0, 0],
            dims= [0.4, 4.6, 1.5],
            color= [1, 1, 1, 0]
        )
        Add_Rigid_Object_To_Scene(test, "Cuboid", R2_Smart_Mat_Table_Box2, True, True)

        # Human_Representation_Box = Cuboid(
        #     name= "Human_Box",
        #     pose= [4.2, 1.5, 1, 1, 0, 0, 0],
        #     dims= [0.5, 1, 2],
        #     color= [1, 1, 1, 0]
        # )
        # Add_Rigid_Object_To_Scene(test, "Cuboid", Human_Representation_Box, True, True)

        STUD_HEIGHT = 0.1016

        ################################
        ################################
        ### Code Writing STARTS Here ###
        ################################
        ################################

        # Pick_Long_Element_From_Mat_Supply("Bottom_Plate", 3.6576, 0.04, 0.1016)
        # Place_Long_Element_On_Smart_Conveyor_by_Rob2_Gripper("Bottom_Plate", 2.4984, 1.8288, 3.6576, 0.1016)

        # Creating the Stud to Pick within the simulation
        Create_Wooden_Element_For_Smart_Mat_Table(el_name="Test_Wood", L=3.6576, W=0.04, H=0.1016)

        # # Move Conveyor Belt (Temporary)
        # Smart_Conv.render_exec('Joint_1', 1.8288 - (OVERALL_PANEL_LENGTH/2) + ((3.6576/2)-PICK_OFFSET_FROM_L_CORNER-(ROBOT_2_GRIPPER_LENGTH/2)+(SMART_CONV_RANGE_OF_MOTION_J1/2)))

        # # Creating the Stud that represents the Placed element within the simulation
        # Element = Cuboid(
        #     name="Test_Wood"+"_Final_Pose",
        #     # 12 ft = 3.6576 m is the max length of the Smart Material Table
        #     pose=[3.63075, 1.48443, 0.94819, ev, 0, ev, 0],
        #     dims=[0.1016, 3.6576, 0.04],
        #     color=[1, 0, 0, 1]
        # )
        # Add_Rigid_Object_To_Scene(test, "Cuboid", Element)

        # # Robot 2 Pre Place Movement
        # Robot_2.plan(tcp_name= "tool0",
        #                 target_pose= [2.3+(2.4984-(OVERALL_PANEL_HEIGHT/2))+SMART_CONV_X_SHIFT+0.2,
        #                             0,
        #                             SMART_CONV_REST_ELEVATION+0.1016-PICK_OFFSET_FROM_W_CORNER+0.2],
        #                 target_orientation= [0, 1, 0, 0],
        #                 update_world_needed= True)
        # Robot_2.render_exec(renderInstance= True,
        #                         Show_Sphere= False)

        # Stage 2: Approach offset for picking
        Robot_2.plan(
            tcp_name="tool0",
            target_pose=[
                SMART_MAT_TABLE[0] + PICK_OFFSET_FROM_W_CORNER - 0.3,
                SMART_MAT_TABLE[1]
                - SMART_MAT_TABLE_MAX_LENGTH
                + PICK_OFFSET_FROM_L_CORNER
                + (ROBOT_2_GRIPPER_LENGTH / 2),
                SMART_MAT_TABLE[2] + (0.04 / 2),
            ],
            target_orientation=[0, ev, 0, ev],
            update_world_needed=True,
        )
        Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

        Automated_Pick_Place_No_Cut("BPL", 2.4984, 1.8288, 0, 3.6576, 0.04, 0.1016)

        Robot_2.free_TCP_movement("tool0")

        ################################
        ################################
        #### Code Writing ENDS Here ####
        ################################
        ################################


if __name__ == "__main__":
    main()

##############################
##############################
#### Execution Platform ######
##############################
##############################