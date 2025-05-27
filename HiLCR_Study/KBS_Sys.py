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

# Xform Creation and Transform
import omni.kit.commands as cmd
# Adding UsdPhysics to Add Mass To the Object
from pxr import Gf, Usd, Sdf, UsdGeom, UsdPhysics

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
MOTION_ACCELERAION_VALUE: float = 0.9
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

        IDC_Lab = Mesh(
            name="idc_lab_model",
            pose=[0, 0, 0, 1, 0, 0, 0],
            file_path= cur_dir + "lab_model/idc_lab_visualization.stl",
            color= [0.1, 0.05, 0, 1],
            scale=[0.001, 0.001, 0.001]
        )


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

        Human_Worker = Mesh(
            name="Human_Worker",
            pose=[4.2, 1.5, 0, ev, 0, 0, ev],
            file_path= cur_dir + "Human_Worker/Worker.stl",
            color= [0.1, 0.05, 0, 1],
            scale=[0.001, 0.001, 0.001]
        )


        # Small_Cutting_Table
        world_model = WorldConfig(
            mesh=[IDC_Lab, Smart_Mat_Table, Sloped_Table, SheathingTable, Small_Cutting_Table, Human_Worker],
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
            self._robot_cfg["kinematics"]["extra_collision_spheres"] = {"tool0": 100, "tool1": 1,}
        if self._ROS_JS_robot_indicator == "IRB6620_R2":
            self._robot_cfg["kinematics"]["extra_collision_spheres"] = {"tool0": 100,}

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
            collision_cache={"obb":30, "mesh": 100}
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

            
            input("Press Enter to continue...")

        # Cleaning out !
        self._computed_path_result = None
        self._computed_cmd_plan = None
        self._computed_idx_list = []

    def move_to_home(self,
                     if_show_spheres: bool = False,
                     Customized_JS: List[float] = [0, -0.5, 0.5, 0, 0, 0]):

        # If Robot is Already at Home Position
        if self._is_at_home == True:
            print("Robot " + self._ROS_JS_robot_indicator + " is Already at Home Position")
            return True

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
    make_invisible: bool = False
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

    # It's better not to use CuRobo's enable_physics Attribute!
    if ObjectType == "Cuboid":
        Added_Obj_Prim_Root = World_Manager._usd_help.add_cuboid_to_stage(
            obstacle=obj, enable_physics=False
        )
    elif ObjectType == "Mesh":
        Added_Obj_Prim_Root = World_Manager._usd_help.add_mesh_to_stage(
            obstacle=obj, enable_physics=False
        )
    elif ObjectType == "Cylinder":
        Added_Obj_Prim_Root = World_Manager._usd_help.add_cylinder_to_stage(
            obstacle=obj, enable_physics=False
        )
    elif ObjectType == "Sphere":
        Added_Obj_Prim_Root = World_Manager._usd_help.add_sphere_to_stage(
            obstacle=obj, enable_physics=False
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
    Drag a stud from the Smart Material Table to right to cut in length with the smart material supply's saw.

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
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[5.0, 1.08, 0.87],
        target_orientation=[0, 1, 0, 0],
        update_world_needed=True
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

    # Helping Pick (X -= -0.3)
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[
            SMART_MAT_TABLE[0] + PICK_OFFSET_FROM_W_CORNER - 0.3,
            SMART_MAT_TABLE[1] - SMART_MAT_TABLE_MAX_LENGTH + PICK_OFFSET_FROM_L_CORNER + (ROBOT_2_GRIPPER_LENGTH / 2),
            SMART_MAT_TABLE[2] + (el_dims[1] / 2)
        ],
        target_orientation=[0, ev, 0, ev],
        update_world_needed=True
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)
    
    # Drag Position
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[
            SMART_MAT_TABLE[0] + PICK_OFFSET_FROM_W_CORNER,
            SMART_MAT_TABLE[1] - SMART_MAT_TABLE_MAX_LENGTH + PICK_OFFSET_FROM_L_CORNER + (ROBOT_2_GRIPPER_LENGTH / 2),
            SMART_MAT_TABLE[2] + (el_dims[1] / 2)
        ],
        target_orientation=[0, ev, 0, ev],
        update_world_needed=True,
        removing_primitives=["world/obstacles"],
        orientational_restriction=torch.tensor([1, 1, 1], dtype=torch.float32)
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)
    
    # Letting the Robot Drag up to 50 cm for each step
    ALLOWED_DRAG_STEP: float = 0.5
    # Robot_2 needs to drag the stud by its length to let the saw cut it
    Drag_Counts: int = (el_dims[0] + STUD_TO_SAW_OFFSET) // ALLOWED_DRAG_STEP
    Drag_Remained: float = np.round((el_dims[0] + STUD_TO_SAW_OFFSET) % ALLOWED_DRAG_STEP, 5)

    Drag_Counter: int = 0
    dc = _dynamic_control.acquire_dynamic_control_interface()

    # Create a temporary full-length stud for dragging
    Create_Wooden_Element_For_Smart_Mat_Table(
        el_name=el_name + "Temp",
        L=SMART_MAT_TABLE_MAX_LENGTH,
        W=el_dims[1],
        H=el_dims[2]
    )

    while Drag_Counter < Drag_Counts:
        # Attach and drag forward
        Robot_2.eef_attach(tool_name="tool0", attaching_object_name=el_name + "Temp")
        obj = dc.get_rigid_body(f"/{Robot_2._ROS_JS_robot_indicator}/tool0")
        pose = dc.get_rigid_body_pose(obj)
        Robot_2.plan(
            tcp_name="tool0",
            target_pose=[pose.p[0], pose.p[1] - ALLOWED_DRAG_STEP, pose.p[2]],
            target_orientation=[pose.r[3], pose.r[0], pose.r[1], pose.r[2]],
            update_world_needed=True,
            removing_primitives=["world/obstacles"],
            direct_pose_cost=PoseCostMetric.create_grasp_approach_metric(
                offset_position=0.0, tstep_fraction=0.001, linear_axis=1
            )
        )
        Robot_2.render_exec(renderInstance=True, Show_Sphere=False)
        Robot_2.eef_detach(tool_name="tool0", detaching_object_name=el_name + "Temp")

        # Drag back to release
        obj = dc.get_rigid_body(f"/{Robot_2._ROS_JS_robot_indicator}/tool0")
        pose = dc.get_rigid_body_pose(obj)
        Robot_2.plan(
            tcp_name="tool0",
            target_pose=[pose.p[0], pose.p[1] + ALLOWED_DRAG_STEP, pose.p[2]],
            target_orientation=[pose.r[3], pose.r[0], pose.r[1], pose.r[2]],
            update_world_needed=True,
            removing_primitives=[
                "world/obstacles/R2_Smart_Mat_Table_Box2",
                "world/obstacles/R2_Smart_Mat_Table_Box1"
            ],
            direct_pose_cost=PoseCostMetric.create_grasp_approach_metric(
                offset_position=0.0, tstep_fraction=0.001, linear_axis=1
            )
        )
        Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

        Drag_Counter += 1
    
    # Drag remaining length
    Robot_2.eef_attach(tool_name="tool0", attaching_object_name=el_name + "Temp")
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[
            SMART_MAT_TABLE[0] + PICK_OFFSET_FROM_W_CORNER,
            SMART_MAT_TABLE[1] - SMART_MAT_TABLE_MAX_LENGTH + PICK_OFFSET_FROM_L_CORNER + (ROBOT_2_GRIPPER_LENGTH / 2) - Drag_Remained,
            SMART_MAT_TABLE[2] + (el_dims[1] / 2)
        ],
        target_orientation=[0, ev, 0, ev],
        update_world_needed=True,
        removing_primitives=["world/obstacles"],
        direct_pose_cost=PoseCostMetric.create_grasp_approach_metric(
            offset_position=0.0, tstep_fraction=0.001, linear_axis=1
        )
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)
    Robot_2.eef_detach(tool_name="tool0", detaching_object_name=el_name + "Temp")

    # Return to post-drag position
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[
            SMART_MAT_TABLE[0] + PICK_OFFSET_FROM_W_CORNER,
            SMART_MAT_TABLE[1] - SMART_MAT_TABLE_MAX_LENGTH + PICK_OFFSET_FROM_L_CORNER + (ROBOT_2_GRIPPER_LENGTH / 2),
            SMART_MAT_TABLE[2] + (el_dims[1] / 2)
        ],
        target_orientation=[0, ev, 0, ev],
        update_world_needed=True,
        removing_primitives=[
            "world/obstacles/R2_Smart_Mat_Table_Box2",
            "world/obstacles/R2_Smart_Mat_Table_Box1"
        ],
        direct_pose_cost=PoseCostMetric.create_grasp_approach_metric(
            offset_position=0.0, tstep_fraction=0.001, linear_axis=1
        )
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

    # Post Drag Position
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[
            SMART_MAT_TABLE[0] + PICK_OFFSET_FROM_W_CORNER - 0.3,
            SMART_MAT_TABLE[1] - SMART_MAT_TABLE_MAX_LENGTH + PICK_OFFSET_FROM_L_CORNER + (ROBOT_2_GRIPPER_LENGTH / 2),
            SMART_MAT_TABLE[2] + (el_dims[1] / 2)
        ],
        target_orientation=[0, ev, 0, ev],
        update_world_needed=True,
        removing_primitives=[
            "world/obstacles/R2_Smart_Mat_Table_Box2",
            "world/obstacles/R2_Smart_Mat_Table_Box1"
        ],
        orientational_restriction=torch.tensor([1, 1, 1], dtype=torch.float32)
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

    print(f"DRAGGING DONE! Length Dragged Out To Cut: {el_dims[0]} m")

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

def Pick_Long_Element_From_Mat_Supply(
    el_name: str = None,
    L: float = None,
    W: float = None,
    H: float = None,
):
    """
    Pick a long wooden element (stud of length ≥ 8 ft / 2.4384 m) from the smart material table
    using Robot_2, attach it to the gripper, and reposition slightly after pick-up.

    Parameters:
        el_name (str): Name to assign to the new wooden element.
        L (float): Length of the element.
        W (float): Width/thickness of the element.
        H (float): Height of the element.

    Procedure:
    1. Move Robot_2 to a pre-pick pose just above the material table.
    2. Approach the long element offset from the table corner to align gripper.
    3. Execute the pick (with world-obstacle primitives removed and orientation locked).
    4. Create the cuboid object representing the element in the scene.
    5. Attach the element to Robot_2’s tool0.
    6. Retract the gripper in two small post-pick motions.
    7. Perform a final “post pick” retraction back toward the approach offset.
    """
    # Stage 1: Pre-pick positioning above the material table
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[5.0, 1.08, 0.87],
        target_orientation=[0, 1, 0, 0],
        update_world_needed=True,
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 2: Approach offset for picking
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[
            SMART_MAT_TABLE[0] + PICK_OFFSET_FROM_W_CORNER - 0.3,
            SMART_MAT_TABLE[1]
            - SMART_MAT_TABLE_MAX_LENGTH
            + PICK_OFFSET_FROM_L_CORNER
            + (ROBOT_2_GRIPPER_LENGTH / 2),
            SMART_MAT_TABLE[2] + (W / 2),
        ],
        target_orientation=[0, ev, 0, ev],
        update_world_needed=True,
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 3: Pick the element
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[
            SMART_MAT_TABLE[0] + PICK_OFFSET_FROM_W_CORNER,
            SMART_MAT_TABLE[1]
            - SMART_MAT_TABLE_MAX_LENGTH
            + PICK_OFFSET_FROM_L_CORNER
            + (ROBOT_2_GRIPPER_LENGTH / 2),
            SMART_MAT_TABLE[2] + (W / 2),
        ],
        target_orientation=[0, ev, 0, ev],
        update_world_needed=True,
        removing_primitives=["world/obstacles"],
        orientational_restriction=torch.tensor([1, 1, 1], dtype=torch.float32),
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 4: Create and attach the picked element
    Create_Wooden_Element_For_Smart_Mat_Table(el_name=el_name, L=L, W=W, H=H)
    Robot_2.eef_attach(tool_name="tool0", attaching_object_name=el_name)
    print("Wooden Element Attached to Robot_2")

    # Stage 5: Retract slightly in X
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[
            SMART_MAT_TABLE[0] + PICK_OFFSET_FROM_W_CORNER + 0.1,
            SMART_MAT_TABLE[1]
            - SMART_MAT_TABLE_MAX_LENGTH
            + PICK_OFFSET_FROM_L_CORNER
            + (ROBOT_2_GRIPPER_LENGTH / 2),
            SMART_MAT_TABLE[2] + (W / 2),
        ],
        target_orientation=[0, ev, 0, ev],
        update_world_needed=True,
        removing_primitives=["world/obstacles"],
        orientational_restriction=torch.tensor([1, 1, 1], dtype=torch.float32),
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 6: Retract slightly upward in Z
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[
            SMART_MAT_TABLE[0] + PICK_OFFSET_FROM_W_CORNER + 0.1,
            SMART_MAT_TABLE[1]
            - SMART_MAT_TABLE_MAX_LENGTH
            + PICK_OFFSET_FROM_L_CORNER
            + (ROBOT_2_GRIPPER_LENGTH / 2),
            SMART_MAT_TABLE[2] + (W / 2) + 0.2,
        ],
        target_orientation=[0, ev, 0, ev],
        update_world_needed=True,
        removing_primitives=["world/obstacles"],
        orientational_restriction=torch.tensor([1, 1, 1], dtype=torch.float32),
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 7: Post Pick Last
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[
            SMART_MAT_TABLE[0] + PICK_OFFSET_FROM_W_CORNER - 0.3,
            SMART_MAT_TABLE[1]
            - SMART_MAT_TABLE_MAX_LENGTH
            + PICK_OFFSET_FROM_L_CORNER
            + (ROBOT_2_GRIPPER_LENGTH / 2),
            SMART_MAT_TABLE[2] + (W / 2) + 0.2,
        ],
        target_orientation=[0, ev, 0, ev],
        update_world_needed=True,
        removing_primitives=["world/obstacles"],
        orientational_restriction=torch.tensor([1, 1, 1], dtype=torch.float32),
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

def Pass_Long_Element_G2G(
    el_name: str = None,
    L: float = None,
    H: float = None,
):
    """
    Transfer a long wooden element (length ≥ 8 ft / 2.4384 m) gripper-to-gripper 
    from Robot_2 to Robot_1.

    Parameters:
        el_name (str): Name of the element to transfer.
        L (float): Length of the element.
        H (float): Height of the element (used to set passing elevation).

    Procedure:
    1. Return Robot_2 to its home position.
    2. Move the smart conveyor out of the way to clear the path.
    3. Robot_2 positions the element over the passing station.
    4. Robot_1 moves in to receive the element at the same station.
    5. Robot_2 releases (detaches) the element.
    6. Robot_1 picks up (attaches) the element.
    7. Robot_2 retracts slightly from the passing station.
    8. Both robots return to their home positions.
    """
    # Stage 1: Home Robot_2
    Robot_2.move_to_home()

    # Stage 2: Clear conveyor path
    Smart_Conv.render_exec('Joint_1', SMART_CONV_RANGE_OF_MOTION_J1)

    # Stage 3: Robot_2 positions element for handoff
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[
            2.3 + (L / 2) - PICK_OFFSET_FROM_L_CORNER - (ROBOT_2_GRIPPER_LENGTH / 2),
            -1,
            PASSING_ELEVATION + H,
        ],
        target_orientation=[0, ev, ev, 0],
        update_world_needed=True,
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 4: Robot_1 moves in to receive element
    Robot_1.plan(
        tcp_name="tool0",
        target_pose=[
            2.3 - (L / 2) + PICK_OFFSET_FROM_L_CORNER + (ROBOT_1_GRIPPER_LENGTH / 2),
            -1,
            PASSING_ELEVATION + H + 0.1,
        ],
        target_orientation=[0, ev, -ev, 0],
        update_world_needed=True,
    )
    Robot_1.render_exec(renderInstance=True, Show_Sphere=False)

    Robot_1.plan(
        tcp_name="tool0",
        target_pose=[
            2.3 - (L / 2) + PICK_OFFSET_FROM_L_CORNER + (ROBOT_1_GRIPPER_LENGTH / 2),
            -1,
            PASSING_ELEVATION + H,
        ],
        target_orientation=[0, ev, -ev, 0],
        update_world_needed=True,
    )
    Robot_1.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 5: Transfer attachment
    Robot_2.eef_detach(tool_name="tool0", detaching_object_name=el_name)
    Robot_1.eef_attach(tool_name="tool0", attaching_object_name=el_name)

    # Stage 6: Robot_2 post-detach retract
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[
            2.3 + (L / 2) - PICK_OFFSET_FROM_L_CORNER - (ROBOT_2_GRIPPER_LENGTH / 2),
            -1,
            PASSING_ELEVATION + H + 0.1,
        ],
        target_orientation=[0, ev, ev, 0],
        update_world_needed=True,
        orientational_restriction=torch.tensor([1, 1, 1], dtype=torch.float32),
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 7: Return both robots home
    Robot_2.move_to_home()
    Robot_1.move_to_home()

def Place_Long_Element_On_Smart_Conveyor_by_Rob1_Gripper(
    el_name: str = None,
    X: float = None,
    Y: float = None,
    L: float = None,
    H: float = None,
):
    """
    Place a greater than 8ft (2.4384 m) wooden element on the smart conveyor using Robot_1, then detach and return to home.

    Parameters:
        el_name (str): Name of the element to place.
        X (float): X value of element's position from the top left corner of the building panel.
        Y (float): Y value of element's position from the top left corner of the building panel.
        L (float): Length of the element.
        H (float): Height of the element.

    Procedure:
    1. Move the smart conveyor to align under Robot_1’s TCP.
    2. Move Robot_1 into a pre-place pose above the conveyor.
    3. Execute the placement motion, removing obstacles and using a grasp approach cost metric.
    4. Detach the element from Robot_1 and attach it to the conveyor.
    5. Retract Robot_1 back through the pre-place pose.
    6. Return Robot_1 to its home position.
    """
    # Stage 1: Align smart conveyor
    Smart_Conv.render_exec(
        'Joint_1',
        Y - (OVERALL_PANEL_LENGTH / 2)
        + ((L / 2) - PICK_OFFSET_FROM_L_CORNER - (ROBOT_1_GRIPPER_LENGTH / 2))
        + (SMART_CONV_RANGE_OF_MOTION_J1 / 2),
    )

    # Stage 2: Robot_1 pre-place pose
    Robot_1.plan(
        tcp_name="tool0",
        target_pose=[
            2.3 + (X - (OVERALL_PANEL_HEIGHT / 2)) + SMART_CONV_X_SHIFT,
            0,
            SMART_CONV_REST_ELEVATION + H - PICK_OFFSET_FROM_W_CORNER + 0.2,
        ],
        target_orientation=[0, 1, 0, 0],
        update_world_needed=True,
    )
    Robot_1.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 3: Robot_1 place motion
    Robot_1.plan(
        tcp_name="tool0",
        target_pose=[
            2.3 + (X - (OVERALL_PANEL_HEIGHT / 2)) + SMART_CONV_X_SHIFT,
            0,
            SMART_CONV_REST_ELEVATION + H - PICK_OFFSET_FROM_W_CORNER,
        ],
        target_orientation=[0, 1, 0, 0],
        update_world_needed=True,
        removing_primitives=["Smart_Conveyor", "world/obstacles"],
        direct_pose_cost=PoseCostMetric.create_grasp_approach_metric(
            offset_position=0.0, tstep_fraction=0.001, linear_axis=2
        ),
    )
    Robot_1.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 4: Detach from Robot_1 and attach to conveyor
    Robot_1.eef_detach(tool_name="tool0", detaching_object_name=el_name)
    Smart_Conv.attach_object_to_conv(obj_name=el_name)

    # Stage 5: Robot_1 retract through pre-place pose
    Robot_1.plan(
        tcp_name="tool0",
        target_pose=[
            2.3 + (X - (OVERALL_PANEL_HEIGHT / 2)) + SMART_CONV_X_SHIFT,
            0,
            SMART_CONV_REST_ELEVATION + H - PICK_OFFSET_FROM_W_CORNER + 0.2,
        ],
        target_orientation=[0, 1, 0, 0],
        update_world_needed=True,
        removing_primitives=["Smart_Conveyor", "world/obstacles"],
        direct_pose_cost=PoseCostMetric.create_grasp_approach_metric(
            offset_position=0.0, tstep_fraction=0.001, linear_axis=2
        ),
    )
    Robot_1.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 6: Return Robot_1 home
    Robot_1.move_to_home()

def Place_Long_Element_On_Smart_Conveyor_by_Rob2_Gripper(el_name: str = None,
        X: float = None,
        Y: float = None,
        L: float = None,
        H: float = None):
    '''
    Brief Description:
        Places a greater than 8ft (2.4384 m) wooden element onto the smart conveyor using Robot 2's gripper,
        performing approach, placement, detachment, and post-placement motions.

    Parameters:
        el_name (str): Name of the element to place.
        X (float): X value of element's position from the top left corner of the building panel.
        Y (float): Y value of element's position from the top left corner of the building panel.
        L (float): Length of the element.
        H (float): Height of the element.

    Steps of Working:
        1. Move the smart conveyor to align the element with Robot 2 based on Y and element length.
        2. Execute pre-place movement: plan an approach pose and render the motion.
        3. Execute main place movement: plan exact placement pose (removing obstacles) and render motion.
        4. Detach the element from Robot 2's gripper and attach it to the conveyor.
        5. Execute post-place movement: plan a retract pose (removing obstacles) and render the motion.
    '''

    # Move Conveyor To TCP's 0 Position For Placement
    Smart_Conv.render_exec('Joint_1', Y - (OVERALL_PANEL_LENGTH/2) + ((L/2)-PICK_OFFSET_FROM_L_CORNER-(ROBOT_2_GRIPPER_LENGTH/2)+(SMART_CONV_RANGE_OF_MOTION_J1/2)))

    # Robot 2 Pre Place Movement
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [2.3+(X-(OVERALL_PANEL_HEIGHT/2))+SMART_CONV_X_SHIFT+0.2,
                                  0,
                                  SMART_CONV_REST_ELEVATION+H-PICK_OFFSET_FROM_W_CORNER+0.2],
                    target_orientation= [0, 1, 0, 0],
                    update_world_needed= True)
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)
    
    # Robot 1 Place Movement
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [2.3+(X-(OVERALL_PANEL_HEIGHT/2))+SMART_CONV_X_SHIFT,
                                  0,
                                  SMART_CONV_REST_ELEVATION+H-PICK_OFFSET_FROM_W_CORNER],
                    target_orientation= [0, 1, 0, 0],
                    update_world_needed= True,
                    removing_primitives=["Smart_Conveyor" ,"world/obstacles"],
                    orientational_restriction=torch.tensor([1,1,1], dtype=torch.float32))
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)

    # Detach
    Robot_2.eef_detach(tool_name="tool0",
                        detaching_object_name= el_name)
    Smart_Conv.attach_object_to_conv(obj_name= el_name)

    # Robot 1 Post Place Movement
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [2.3+(X-(OVERALL_PANEL_HEIGHT/2))+SMART_CONV_X_SHIFT,
                                  0,
                                  SMART_CONV_REST_ELEVATION+H-PICK_OFFSET_FROM_W_CORNER+0.2],
                    target_orientation= [0, 1, 0, 0],
                    update_world_needed= True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles"],
                    orientational_restriction=torch.tensor([1,1,1], dtype=torch.float32))
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)

def Pick_8ft_Element_From_Sloped_Table(
    el_name: str = None,
    L: float = 2.4384,
    W: float = None,
    H: float = None,
):
    """
    Pick an 8 ft (2.4384 m) wooden stud from a sloped material table using Robot_2, without cutting.

    Parameters:
        el_name (str): Name to assign to the new stud.
        L (float): Stud length (fixed at 2.4384 m for 8 ft).
        W (float): Stud width/thickness.
        H (float): Stud height.

    Procedure:
    1. Compute the 3D pose of the stud’s center on the sloped table.
    2. Compute the pick orientation quaternion adjusted for the table angle.
    3. Move Robot_2 into a pre-pick pose just above the stud.
    4. Execute the pick motion, removing sloped-table obstacles and locking orientation.
    5. Create the stud object in the simulation and attach it to Robot_2’s gripper.
    6. Retract Robot_2 slightly upward along the slope.
    7. Return Robot_2 to its home position.
    """
    # Compute stud center pose on the sloped table
    diagonal_stud_l = np.sqrt(W**2 + H**2)
    angle = np.radians(SLOPED_MAT_TABLE_ANGLE)
    half_diagonal = diagonal_stud_l / 2
    z_increase = half_diagonal * np.sin(angle + np.arcsin((W / 2) / half_diagonal))
    y_increase = half_diagonal * np.cos(angle + np.arcsin((W / 2) / half_diagonal))
    Stud_Pose = [
        SLOPED_MAT_TABLE[0] + (L / 2),
        SLOPED_MAT_TABLE[1] - y_increase,
        SLOPED_MAT_TABLE[2] + z_increase,
    ]

    # Compute pick orientation quaternion
    quat = R.from_euler(
        'xyz',
        [(np.pi / 2) - angle, 0, np.pi / 2]
    ).as_quat()
    quat[1] *= -1  # flip Y component to avoid Euler singularity

    # Stage 1: Pre-pick approach
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[
            SLOPED_MAT_TABLE[0] + SLOPED_TABLE_PICK_OFFSET_FROM_L_CORNER + (ROBOT_2_GRIPPER_LENGTH / 2),
            Stud_Pose[1] + PRE_PICK_OFFSET,
            Stud_Pose[2] - (PRE_PICK_OFFSET * np.tan(angle)),
        ],
        target_orientation=[quat[3], quat[0], quat[1], quat[2]],
        update_world_needed=True,
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 2: Pick the stud
    pick_val = (H / 2) - PICK_OFFSET_FROM_W_CORNER
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[
            SLOPED_MAT_TABLE[0] + SLOPED_TABLE_PICK_OFFSET_FROM_L_CORNER + (ROBOT_2_GRIPPER_LENGTH / 2),
            Stud_Pose[1] + (pick_val * np.cos(angle)),
            Stud_Pose[2] - (pick_val * np.sin(angle)),
        ],
        target_orientation=[quat[3], quat[0], quat[1], quat[2]],
        update_world_needed=True,
        removing_primitives=["world/obstacles", "World/obstacles/Sloped_Table"],
        orientational_restriction=torch.tensor([1, 1, 1], dtype=torch.float32),
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 3: Create and attach the stud
    Create_Wooden_Element_For_Sloped_Table(el_name=el_name, L=L, W=W, H=H)
    Robot_2.eef_attach(tool_name="tool0", attaching_object_name=el_name)

    # Stage 4: Post-pick retract along slope
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[
            SLOPED_MAT_TABLE[0] + SLOPED_TABLE_PICK_OFFSET_FROM_L_CORNER + (ROBOT_2_GRIPPER_LENGTH / 2),
            Stud_Pose[1] + (pick_val * np.cos(angle)),
            Stud_Pose[2] - (pick_val * np.sin(angle)) + (PRE_PICK_OFFSET * 2),
        ],
        target_orientation=[quat[3], quat[0], quat[1], quat[2]],
        update_world_needed=True,
        removing_primitives=["world/obstacles", "World/obstacles/Sloped_Table"],
        orientational_restriction=torch.tensor([1, 1, 1], dtype=torch.float32),
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 5: Return to home
    Robot_2.move_to_home()

def Place_and_Hold_8ft_Element_On_Smart_Conveyor(
    X: float = None,
    Y: float = None,
    L: float = 2.4384,
    H: float = None,
):
    """
    Place and hold an 8 ft (2.4384 m) element on the smart conveyor using Robot_2,
    then record conveyor poses for subsequent nailing operations.

    Parameters:
        X (float): X value of element's position from the top left corner of the building panel.
        Y (float): Y value of element's position from the top left corner of the building panel.
        L (float): Length of the element (fixed at 2.4384 m).
        H (float): Height of the element.

    Returns:
        Side_Selector (int): +1 or -1, indicating which side the conveyor moved to for nailing.

    Procedure:
    1. Determine Side_Selector based on Y and conveyor range.
    2. Move conveyor to the placement/nailing position and record nail poses.
    3. Robot_2 moves into a pre-place pose above the conveyor.
    4. Robot_2 executes the placement motion and holds the element.
    5. Return the Side_Selector for downstream use.
    """
    # Stage 1: Determine conveyor side for placement/nailing
    half_length = OVERALL_PANEL_LENGTH / 2
    mid_range = SMART_CONV_RANGE_OF_MOTION_J1 / 2
    target_offset = half_length - Y + mid_range + NAILING_CONV_TARGET
    if target_offset > SMART_CONV_RANGE_OF_MOTION_J1:
        Side_Selector = -1
    else:
        Side_Selector = 1

    # Stage 2: Move conveyor and record nail poses
    conv_pose = half_length - Y + mid_range + NAILING_CONV_TARGET * Side_Selector
    Smart_Conv.render_exec('Joint_1', conv_pose)
    Smart_Conv._nail_poses.append((conv_pose, Side_Selector))
    Smart_Conv._vertical_nail_poses.append(half_length - Y + mid_range)

    # Stage 3: Robot_2 pre-place approach
    pre_place_x = (
        2.3
        + (X - (OVERALL_PANEL_HEIGHT / 2))
        + SMART_CONV_X_SHIFT
        + ((L / 2) - (ROBOT_2_GRIPPER_LENGTH / 2) - SLOPED_TABLE_PICK_OFFSET_FROM_L_CORNER)
        + 0.2
    )
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[
            pre_place_x,
            NAILING_CONV_TARGET * Side_Selector,
            SMART_CONV_REST_ELEVATION + H - PICK_OFFSET_FROM_W_CORNER + 0.1,
        ],
        target_orientation=[0, ev, ev, 0],
        update_world_needed=True,
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 4: Robot_2 place and hold
    place_x = (
        2.3
        + (X - (OVERALL_PANEL_HEIGHT / 2))
        + SMART_CONV_X_SHIFT
        + ((L / 2) - (ROBOT_2_GRIPPER_LENGTH / 2) - SLOPED_TABLE_PICK_OFFSET_FROM_L_CORNER)
    )
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[
            place_x,
            NAILING_CONV_TARGET * Side_Selector,
            SMART_CONV_REST_ELEVATION + H - PICK_OFFSET_FROM_W_CORNER,
        ],
        target_orientation=[0, ev, ev, 0],
        update_world_needed=True,
        removing_primitives=["Smart_Conveyor", "world/obstacles"],
        orientational_restriction=torch.tensor([1, 1, 1], dtype=torch.float32),
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

    return Side_Selector

def Nail_and_Release_Vertical_Element(
    el_name: str = None,
    X: float = None,
    push_to_nail: float = PUSH_TO_NAIL_OFFSET,
    L: float = 2.4384,
    H: float = None,
    Side_Selector: int = 0,
    Is_Held: bool = False,
):
    """
    Nail and release an 8 ft (2.4384 m) element on the smart conveyor.

    Parameters:
        el_name (str): Name of the element to nail and release.
        X (float): X value of element's position from the top left corner of the building panel.
        push_to_nail (float): Offset distance to push into the nail.
        H (float): Height of the element.
        Side_Selector (int): +1 or -1 indicating conveyor side used for placement of the element.
        Is_Held (bool): If True, there were no hold by Robot 1 (e.g., Placing Top Cripple), so there will be no Release.
    Procedure:
    1. Compute the Y coordinate for nailing based on Side_Selector and TCP flag.
    2. Robot_1 performs a sequence of nail motions:
       a. Initial approach to nail position.
       b. Push in for first nail.
       c. Retract after first nail.
       d. Move up for second nail.
       e. Push in for top nail.
       f. Retract after top nail.
       g. Return Robot_1 home.
    3. Robot_2 detaches the element and places it on the conveyor.
    4. Robot_2 retracts from the conveyor and returns home.
    """
    # Stage 1: Compute nailing Y coordinate
    if Side_Selector == 1 or Side_Selector == -1:
        proceeding_y = NAILING_CONV_TARGET * Side_Selector
        if Is_Held and Side_Selector == -1:
            proceeding_y -= NAILING_CONV_TARGET

        # Acquire dynamic control interface for precise pose queries
        dc = _dynamic_control.acquire_dynamic_control_interface()
        body = dc.get_rigid_body(f"/{Robot_1._ROS_JS_robot_indicator}/tool2")

        # Stage 2a: Robot_1 approach to nail position
        Robot_1.plan(
            tcp_name="tool2",
            target_pose=[
                1.1,
                proceeding_y,
                SMART_CONV_REST_ELEVATION + H - (0.7 * H),
            ],
            target_orientation=[ev, 0, ev, 0],
            update_world_needed=True,
        )
        Robot_1.render_exec(renderInstance=True, Show_Sphere=False)

        # Stage 2b: First nail push
        pose = dc.get_rigid_body_pose(body)
        Robot_1.release_path_plan_restriction()
        Robot_1.plan(
            tcp_name="tool2",
            target_pose=[
                pose.p[0] + push_to_nail,
                pose.p[1],
                pose.p[2],
            ],
            target_orientation=[pose.r[3], pose.r[0], pose.r[1], pose.r[2]],
            update_world_needed=True,
            removing_primitives=["Smart_Conveyor", "world/obstacles"],
            direct_pose_cost=PoseCostMetric.create_grasp_approach_metric(
                offset_position=0.0, tstep_fraction=0.001, linear_axis=2
            ),
        )
        Robot_1.render_exec(renderInstance=True, Show_Sphere=False)

        # Stage 2c: Retract after first nail
        pose = dc.get_rigid_body_pose(body)
        Robot_1.release_path_plan_restriction()
        Robot_1.plan(
            tcp_name="tool2",
            target_pose=[
                pose.p[0] - push_to_nail,
                pose.p[1],
                pose.p[2],
            ],
            target_orientation=[pose.r[3], pose.r[0], pose.r[1], pose.r[2]],
            update_world_needed=True,
            removing_primitives=["Smart_Conveyor", "world/obstacles"],
            direct_pose_cost=PoseCostMetric.create_grasp_approach_metric(
                offset_position=0.0, tstep_fraction=0.001, linear_axis=2
            ),
        )
        Robot_1.render_exec(renderInstance=True, Show_Sphere=False)

        # Stage 2d: Move up for second nail
        pose = dc.get_rigid_body_pose(body)
        Robot_1.release_path_plan_restriction()
        Robot_1.plan(
            tcp_name="tool2",
            target_pose=[
                pose.p[0],
                pose.p[1],
                pose.p[2] + (0.4 * H),
            ],
            target_orientation=[pose.r[3], pose.r[0], pose.r[1], pose.r[2]],
            update_world_needed=True,
            removing_primitives=["Smart_Conveyor", "world/obstacles"],
            direct_pose_cost=PoseCostMetric.create_grasp_approach_metric(
                offset_position=0.0, tstep_fraction=0.001, linear_axis=0
            ),
        )
        Robot_1.render_exec(renderInstance=True, Show_Sphere=False)

        # Stage 2e: Top nail push
        pose = dc.get_rigid_body_pose(body)
        Robot_1.release_path_plan_restriction()
        Robot_1.plan(
            tcp_name="tool2",
            target_pose=[
                pose.p[0] + push_to_nail,
                pose.p[1],
                pose.p[2],
            ],
            target_orientation=[pose.r[3], pose.r[0], pose.r[1], pose.r[2]],
            update_world_needed=True,
            removing_primitives=["Smart_Conveyor", "world/obstacles"],
            direct_pose_cost=PoseCostMetric.create_grasp_approach_metric(
                offset_position=0.0, tstep_fraction=0.001, linear_axis=2
            ),
        )
        Robot_1.render_exec(renderInstance=True, Show_Sphere=False)

        # Stage 2f: Retract after top nail
        pose = dc.get_rigid_body_pose(body)
        Robot_1.release_path_plan_restriction()
        Robot_1.plan(
            tcp_name="tool2",
            target_pose=[
                pose.p[0] - push_to_nail,
                pose.p[1],
                pose.p[2],
            ],
            target_orientation=[pose.r[3], pose.r[0], pose.r[1], pose.r[2]],
            update_world_needed=True,
            removing_primitives=["Smart_Conveyor", "world/obstacles"],
            direct_pose_cost=PoseCostMetric.create_grasp_approach_metric(
                offset_position=0.0, tstep_fraction=0.001, linear_axis=2
            ),
        )
        Robot_1.render_exec(renderInstance=True, Show_Sphere=False)

        # Stage 2g: Return Robot_1 home
        Robot_1.move_to_home()

    if Is_Held == False:
        # Stage 2: Release and place element on conveyor
        Robot_2.eef_detach(tool_name="tool0", detaching_object_name=el_name)
        Smart_Conv.attach_object_to_conv(obj_name=el_name)

        # Stage 3: Robot_2 post-place retract
        post_x = (
            2.3
            + (X - (OVERALL_PANEL_HEIGHT / 2))
            + SMART_CONV_X_SHIFT
            + ((L / 2) - (ROBOT_2_GRIPPER_LENGTH / 2) - SLOPED_TABLE_PICK_OFFSET_FROM_L_CORNER)
            + 0.2
        )
        Robot_2.plan(
            tcp_name="tool0",
            target_pose=[
                post_x,
                NAILING_CONV_TARGET * Side_Selector,
                SMART_CONV_REST_ELEVATION + H - PICK_OFFSET_FROM_W_CORNER + 0.1,
            ],
            target_orientation=[0, ev, ev, 0],
            update_world_needed=True,
            removing_primitives=["Smart_Conveyor", "world/obstacles"],
            orientational_restriction=torch.tensor([1, 1, 1], dtype=torch.float32),
        )
        Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

        # Stage 4: Return Robot_2 home
        Robot_2.move_to_home()

def Pick_Short_Element_From_Mat_Supply(
    el_name: str = None,
    L: float = None,
    W: float = None,
    H: float = None,
):
    """
    Pick a less than 8ft (2.4384 m) and cut wooden element from the smart material table using Robot_2.

    Parameters:
        el_name (str): Name to assign to the new element.
        L (float): Final length of the element after cutting.
        W (float): Width/thickness of the element.
        H (float): Height of the element.

    Procedure:
    1. Move Robot_2 into a pre-pick pose offset from the table corner.
    2. Execute the pick motion.
    3. Delete the temporary 12 ft primitive and create the cut element.
    4. Attach the new element to Robot_2’s tool.
    5. Retract Robot_2 back to the pre-pick pose.
    """
    # Stage 1: Pre-pick positioning
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[
            SMART_MAT_TABLE[0] + PICK_OFFSET_FROM_W_CORNER - 0.3,
            SMART_MAT_TABLE[1]
            - SMART_MAT_TABLE_MAX_LENGTH
            - STUD_TO_SAW_OFFSET
            - PICK_OFFSET_FROM_L_CORNER
            - (ROBOT_2_GRIPPER_LENGTH / 2),
            SMART_MAT_TABLE[2] + (W / 2),
        ],
        target_orientation=[ev, 0, ev, 0],
        update_world_needed=True,
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 2: Pick the element
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[
            SMART_MAT_TABLE[0] + PICK_OFFSET_FROM_W_CORNER,
            SMART_MAT_TABLE[1]
            - SMART_MAT_TABLE_MAX_LENGTH
            - STUD_TO_SAW_OFFSET
            - PICK_OFFSET_FROM_L_CORNER
            - (ROBOT_2_GRIPPER_LENGTH / 2),
            SMART_MAT_TABLE[2] + (W / 2),
        ],
        target_orientation=[ev, 0, ev, 0],
        update_world_needed=True,
        removing_primitives=["Smart_Conveyor", "world/obstacles"],
        orientational_restriction=torch.tensor([1, 1, 1], dtype=torch.float32),
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 3: Perform saw action in place
    prims_utils.delete_prim(f"/world/obstacles/{el_name}Temp")
    Create_Wooden_Element_For_Smart_Mat_Table(
        el_name=el_name, L=L, W=W, H=H, Debug_Offset=True
    )
    Robot_2.eef_attach(tool_name="tool0", attaching_object_name=el_name)

    # Stage 4: Post-pick retract
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[
            SMART_MAT_TABLE[0] + PICK_OFFSET_FROM_W_CORNER - 0.3,
            SMART_MAT_TABLE[1]
            - SMART_MAT_TABLE_MAX_LENGTH
            - STUD_TO_SAW_OFFSET
            - PICK_OFFSET_FROM_L_CORNER
            - (ROBOT_2_GRIPPER_LENGTH / 2),
            SMART_MAT_TABLE[2] + (W / 2),
        ],
        target_orientation=[ev, 0, ev, 0],
        update_world_needed=True,
        removing_primitives=["Smart_Conveyor", "world/obstacles"],
        orientational_restriction=torch.tensor([1, 1, 1], dtype=torch.float32),
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

    # Move To Home ! 
    Robot_2.move_to_home()

def Drop_Short_Vertical_Element_With_Tangent_to_an_Element(
    el_name: str = None,
    X: float = None,
    Y: float = None,
    L: float = None,
    H: float = None,
    If_Tangent_From_Left: bool = False,
) -> int:
    """
    Position and drop a less than 8ft (2.4384 m) vertical element onto the smart conveyor, allowing it to fall
    if it’s tangent to an adjacent vertical element.

    Parameters:
        el_name (str): Name of the element to drop.
        X (float): X value of element's position from the top left corner of the building panel.
        Y (float): Y value of element's position from the top left corner of the building panel.
        L (float): Length of the element.
        H (float): Height of the element.
        If_Tangent_From_Left (bool): True if the element is tangent from the left side.

    Returns:
        Side_Selector (int): +1 or -1 indicating which side the conveyor moved to.
    """
    # Determine conveyor side and jack offset based on tangency
    half_length = OVERALL_PANEL_LENGTH / 2
    mid_range = SMART_CONV_RANGE_OF_MOTION_J1 / 2
    if If_Tangent_From_Left:
        Side_Selector = -1
        Jack_Y_Placement_Offset = JACK_PLACEMENT_SIDE_DRAG
        if half_length - Y + mid_range + NAILING_CONV_TARGET < 0:
            Side_Selector = 1
    else:
        Side_Selector = 1
        Jack_Y_Placement_Offset = -JACK_PLACEMENT_SIDE_DRAG
        if half_length - Y + mid_range + NAILING_CONV_TARGET > SMART_CONV_RANGE_OF_MOTION_J1:
            Side_Selector = -1

    # Move conveyor into position and record nail pose
    conv_pose = half_length - Y + mid_range + NAILING_CONV_TARGET * Side_Selector
    Smart_Conv.render_exec('Joint_1', conv_pose)
    Smart_Conv._nail_poses.append((conv_pose, Side_Selector))

    # Stage 1: Pre-place approach
    pre_place_x = (
        2.3
        + (X - (OVERALL_PANEL_HEIGHT / 2))
        + SMART_CONV_X_SHIFT
        + ((L / 2) - (ROBOT_2_GRIPPER_LENGTH / 2) - PICK_OFFSET_FROM_L_CORNER)
        + 0.1
    )
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[
            pre_place_x,
            NAILING_CONV_TARGET * Side_Selector + Jack_Y_Placement_Offset,
            SMART_CONV_REST_ELEVATION + H - PICK_OFFSET_FROM_W_CORNER + 0.05,
        ],
        target_orientation=[0, ev, ev, 0],
        update_world_needed=True,
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 2: Place and drop
    place_x = pre_place_x - 0.1
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[
            place_x,
            NAILING_CONV_TARGET * Side_Selector,
            SMART_CONV_REST_ELEVATION + H - PICK_OFFSET_FROM_W_CORNER + 0.05,
        ],
        target_orientation=[0, ev, ev, 0],
        update_world_needed=True,
        removing_primitives=["Smart_Conveyor", "world/obstacles"],
        orientational_restriction=torch.tensor([1, 1, 1], dtype=torch.float32),
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 3: Detach and enable gravity for drop
    Robot_2.eef_detach(tool_name="tool0", detaching_object_name=el_name)
    test._stage.GetPrimAtPath(f"/world/obstacles/{el_name}") \
         .GetAttribute("physxRigidBody:disableGravity") \
         .Set(False)
    Smart_Conv.attach_object_to_conv(obj_name=el_name)

    return Side_Selector

def Nail_Vertical_Element_With_Tangent_to_an_Element(
    push_to_nail: float = PUSH_TO_NAIL_OFFSET_TANGENT,
    el_pose: List[float] = None,
    el_dims: List[float] = None,
    If_Tangent_From_Left: bool = False,
    Side_Selector: int = 0,
) -> None:
    """
    Nail a less than 8ft (2.4384 m) vertical element to an adjacent element at an angle,
    handling tangency direction and conveyor side selection.

    Parameters:
        push_to_nail (float): Distance to push into the nail.
        el_pose (List[float]): [x, y, z] pose of the element on the by assuming the origin is at the top left corner of the building panel.
        el_dims (List[float]): [length, width/thickness, height] dimensions of the element.
        If_Tangent_From_Left (bool): True when the vertical element is tangent from the left side.
        Side_Selector (int): +1 or -1 indicating conveyor side used for placement of the element.

    Procedure:
    1. Determine jack side multiplier based on tangency direction.
    2. Compute the nailing orientation quaternion adjusted for side.
    3. Move Robot_1 into a pre-nail pose and verify reachability.
    4. For each of two nails:
       a. Push in along the angled axis.
       b. Retract to original pose.
    5. Return Robot_1 to its home position.
    """
    # Stage 1: Determine jack side multiplier
    side_mult = -1 if If_Tangent_From_Left else 1

    # Stage 2: Proceed only if conveyor side is valid
    if Side_Selector in (-1, 1):
        # Compute orientation quaternion for angled nailing
        base_angle = np.pi / 2
        nail_angle = np.radians(JACK_SIDE_NAILING_ANGLE)
        quat = R.from_euler(
            'xyz',
            [
                (base_angle + nail_angle) * (-side_mult),
                0,
                base_angle * side_mult,
            ]
        ).as_quat()
        quat[1] *= -1  # adjust for Euler singularity

        # Stage 3: Pre-nail approach
        pre_x = (
            2.3
            + (el_pose[0] - (OVERALL_PANEL_HEIGHT / 2))
            + SMART_CONV_X_SHIFT
            - (el_dims[0] / 2)
            + 0.06096
        )
        pre_y = (NAILING_CONV_TARGET * Side_Selector
                 + (JACK_NAILING_OFFSET * np.cos(nail_angle) * -side_mult))
        pre_z = (SMART_CONV_REST_ELEVATION
                 + (el_dims[2] * 0.3)
                 + (JACK_NAILING_OFFSET * np.sin(nail_angle)))
        doable = Robot_1.plan(
            tcp_name="tool2",
            target_pose=[pre_x, pre_y, pre_z],
            target_orientation=[quat[3], quat[0], quat[1], quat[2]],
            update_world_needed=True,
        )
        if not doable:
            print("Robot_1 cannot reach pre-nail pose (joint limits).")
            return
        Robot_1.render_exec(renderInstance=True, Show_Sphere=False)

        dc = _dynamic_control.acquire_dynamic_control_interface()
        body = dc.get_rigid_body(f"/{Robot_1._ROS_JS_robot_indicator}/tool2")

        # Stage 4: Perform two angled nails
        for _ in range(2):
            pose = dc.get_rigid_body_pose(body)
            # Push in along angled axis
            offset = JACK_NAILING_OFFSET - ((el_dims[1] / 2) / np.cos(nail_angle)) + push_to_nail
            dx = side_mult * offset * np.cos(nail_angle)
            dz = -offset * np.sin(nail_angle)
            Robot_1.release_path_plan_restriction()
            Robot_1.plan(
                tcp_name="tool2",
                target_pose=[pose.p[0], pose.p[1] + dx, pose.p[2] + dz],
                target_orientation=[pose.r[3], pose.r[0], pose.r[1], pose.r[2]],
                update_world_needed=True,
                removing_primitives=["Smart_Conveyor", "world/obstacles"],
                direct_pose_cost=PoseCostMetric.create_grasp_approach_metric(
                    offset_position=0.0, tstep_fraction=0.001, linear_axis=2
                ),
            )
            Robot_1.render_exec(renderInstance=True, Show_Sphere=False)

            # Retract back
            pose = dc.get_rigid_body_pose(body)
            Robot_1.release_path_plan_restriction()
            Robot_1.plan(
                tcp_name="tool2",
                target_pose=[pose.p[0], pose.p[1] - dx, pose.p[2] - dz],
                target_orientation=[pose.r[3], pose.r[0], pose.r[1], pose.r[2]],
                update_world_needed=True,
                removing_primitives=["Smart_Conveyor", "world/obstacles"],
                direct_pose_cost=PoseCostMetric.create_grasp_approach_metric(
                    offset_position=0.0, tstep_fraction=0.001, linear_axis=2
                ),
            )
            Robot_1.render_exec(renderInstance=True, Show_Sphere=False)

            # Prepare for the second nail by lifting
            pose = dc.get_rigid_body_pose(body)
            Robot_1.release_path_plan_restriction()
            Robot_1.plan(
                tcp_name="tool2",
                target_pose=[pose.p[0], pose.p[1], pose.p[2] + (0.4 * el_dims[2])],
                target_orientation=[pose.r[3], pose.r[0], pose.r[1], pose.r[2]],
                update_world_needed=True,
                removing_primitives=["Smart_Conveyor", "world/obstacles"],
                direct_pose_cost=PoseCostMetric.create_grasp_approach_metric(
                    offset_position=0.0, tstep_fraction=0.001, linear_axis=0
                ),
            )
            Robot_1.render_exec(renderInstance=True, Show_Sphere=False)

        # Stage 5: Return Robot_1 home
        Robot_1.move_to_home()
    else:
        print("Error: Side_Selector must be +1 or -1.")

def Pass_Short_Element_G2G(
    el_name: str = None,
    H: float = None,
):
    """
    Transfer a less than 8ft (2.4384 m) wooden element gripper-to-gripper between Robot_2 and Robot_1 at a fixed passing location.

    Parameters:
        el_name (str): Name of the element to transfer.
        H (float): Height of the element (used for XY offsets).

    Procedure:
    1. Compute the fixed passing location.
    2. Robot_1 moves to a pre-take pose offset for the short element.
    3. Robot_2 moves to the passing location.
    4. Robot_1 reaches in to grasp the element (with obstacles removed and approach cost).
    5. Robot_2 detaches and Robot_1 attaches the element.
    6. Robot_2 retracts slightly and both robots return to home.
    """
    PASSING_LOC = [2.05, 0, 1.55]
    half_grip = ROBOT_2_GRIPPER_LENGTH / 2
    delta = PICK_OFFSET_FROM_W_CORNER - (H / 2)
    lateral = PICK_OFFSET_FROM_L_CORNER - (PICK_OFFSET_FROM_L_CORNER_AFTER_PASS + (ROBOT_1_GRIPPER_LENGTH / 2))

    # Stage 1: Robot_1 pre-take positioning
    Robot_1.plan(
        tcp_name="tool0",
        target_pose=[
            PASSING_LOC[0] + 2 * delta - 0.3,
            half_grip + lateral,
            PASSING_LOC[2],
        ],
        target_orientation=[ev, 0, ev, 0],
        update_world_needed=True,
    )
    Robot_1.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 2: Robot_2 move to passing location
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=PASSING_LOC,
        target_orientation=[0, ev, 0, -ev],
        update_world_needed=True,
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 3: Robot_1 reach in to grasp
    Robot_1.plan(
        tcp_name="tool0",
        target_pose=[
            PASSING_LOC[0] + 2 * delta,
            half_grip + lateral,
            PASSING_LOC[2],
        ],
        target_orientation=[ev, 0, ev, 0],
        update_world_needed=True,
        removing_primitives=["Smart_Conveyor", "world/obstacles", "IRB6620_R2"],
        direct_pose_cost=PoseCostMetric.create_grasp_approach_metric(
            offset_position=0.0, tstep_fraction=0.001, linear_axis=2
        ),
    )
    Robot_1.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 4: Transfer attachment
    Robot_2.eef_detach(tool_name="tool0", detaching_object_name=el_name)
    Robot_1.motion_gen_update_world()
    Robot_1.eef_attach(tool_name="tool0", attaching_object_name=el_name)

    # Stage 5: Robot_2 retract slightly
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[PASSING_LOC[0] + 0.2, PASSING_LOC[1], PASSING_LOC[2]],
        target_orientation=[0, ev, 0, -ev],
        update_world_needed=True,
        removing_primitives=["Smart_Conveyor", "world/obstacles", "IRB6620_R1"],
        orientational_restriction=torch.tensor([1, 1, 1], dtype=torch.float32),
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 6: Return both robots home
    Robot_2.move_to_home()
    Robot_1.move_to_home()

def Place_Short_Horizontal_Element_On_Smart_Conveyor_by_Rob1_Gripper(
    el_name: str = None,
    X: float = None,
    Y: float = None,
    L: float = None,
    H: float = None,
) -> float:
    """
    Place a less than 8ft (2.4384 m) upper horizontal element on the smart conveyor using Robot_1.

    Parameters:
        el_name (str): Name of the element to place.
        X (float): X value of element's position from the top left corner of the building panel.
        Y (float): Y value of element's position from the top left corner of the building panel.
        L (float): Length of the element.
        H (float): Height of the element.

    Returns:
        Conv_Curr_Loc (float): Conveyor joint value when element is at TCP zero position.

    Procedure:
    1. Move the smart conveyor into place under Robot_1’s TCP.
    2. Compute and return the conveyor joint value at zero position.
    3. Move Robot_1 into a pre-place pose above the conveyor.
    4. Execute the placement motion, removing obstacles and using a grasp metric.
    5. Detach the element, enable gravity, and attach it to the conveyor.
    6. Return Robot_1 to its home position.
    """
    # Stage 1: Align smart conveyor
    Smart_Conv.render_exec(
        'Joint_1',
        -(Y - (OVERALL_PANEL_LENGTH / 2))
        + ((L / 2) - PICK_OFFSET_FROM_L_CORNER_AFTER_PASS - (ROBOT_1_GRIPPER_LENGTH / 2))
        + (SMART_CONV_RANGE_OF_MOTION_J1 / 2),
    )

    # Stage 2: Compute conveyor zero pose
    Conv_Curr_Loc = (
        -(Y - (OVERALL_PANEL_LENGTH / 2))
        + (SMART_CONV_RANGE_OF_MOTION_J1 / 2)
    )

    # Stage 3: Robot_1 pre-place pose (above conveyor)
    Robot_1.plan(
        tcp_name="tool0",
        target_pose=[
            2.3 + (X - (OVERALL_PANEL_HEIGHT / 2)) + SMART_CONV_X_SHIFT,
            0,
            SMART_CONV_REST_ELEVATION + H - PICK_OFFSET_FROM_W_CORNER + 0.3,
        ],
        target_orientation=[0, 1, 0, 0],
        update_world_needed=True,
    )
    Robot_1.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 4: Robot_1 place motion
    Robot_1.plan(
        tcp_name="tool0",
        target_pose=[
            2.3 + (X - (OVERALL_PANEL_HEIGHT / 2)) + SMART_CONV_X_SHIFT,
            0,
            SMART_CONV_REST_ELEVATION + H - PICK_OFFSET_FROM_W_CORNER + 0.1,
        ],
        target_orientation=[0, 1, 0, 0],
        update_world_needed=True,
        removing_primitives=["Smart_Conveyor", "world/obstacles"],
        direct_pose_cost=PoseCostMetric.create_grasp_approach_metric(
            offset_position=0.0, tstep_fraction=0.001, linear_axis=2
        ),
    )
    Robot_1.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 5: Release and enable gravity
    Robot_1.eef_detach(tool_name="tool0", detaching_object_name=el_name)
    test._stage.GetPrimAtPath(f"/world/obstacles/{el_name}") \
         .GetAttribute("physxRigidBody:disableGravity") \
         .Set(False)
    Smart_Conv.attach_object_to_conv(obj_name=el_name)

    # Stage 6: Return Robot_1 home
    Robot_1.move_to_home()

    return Conv_Curr_Loc

def Nail_Short_Horizontal_Element_by_Rob1_NailGun(
    push_to_nail: float = PUSH_TO_NAIL_OFFSET_TANGENT,
    el_pose: List[float] = None,
    el_dims: List[float] = None,
    conv_current_location: float = None,
) -> None:
    """
    Nail a less than 8ft (2.4384 m) upper horizontal element at two positions (left and right)
    onto the smart conveyor using Robot_1.

    Parameters:
        push_to_nail (float): Distance to push into the nail.
        el_pose (List[float]): [x, y, z] pose of the element on the by assuming the origin is at the top left corner of the building panel.
        el_dims (List[float]): [length, width, height] of the element.
        conv_current_location (float): Conveyor joint position when element is placed at TCP zero position.

    Procedure:
    1. Compute nailing quaternion for left side.
    2. If left nail is reachable:
       a. Move conveyor to left nailing pose.
       b. Move Robot_1 to pre-nail pose and verify reachability.
       c. Perform first nail: push in and retract.
       d. Lift for second nail and repeat push/retract.
       e. Return Robot_1 home.
    3. Compute nailing quaternion for right side.
    4. If right nail is reachable:
       a. Move conveyor to right nailing pose.
       b. Move Robot_1 to pre-nail pose and verify reachability.
       c. Perform two nails as above.
       d. Return Robot_1 home.
    """
    # Compute left-side nailing orientation
    left_quat = R.from_euler(
        'xyz',
        [-(np.pi/2 + np.radians(SILL_NAILING_ANGLE)), 0, np.pi/2]
    ).as_quat()
    left_quat[1] *= -1

    # Attempt left nail
    left_edge = conv_current_location + (el_dims[0] / 2) + NAILING_CONV_TARGET
    if left_edge > SMART_CONV_RANGE_OF_MOTION_J1:
        print("Left Nail is not possible due to conveyor reachability.")
    else:
        Smart_Conv.render_exec('Joint_1', left_edge)

        # Pre-nail approach
        pre_pose = [
            2.3 + (el_pose[0] - (OVERALL_PANEL_HEIGHT / 2)) + SMART_CONV_X_SHIFT,
            NAILING_CONV_TARGET + ((SILL_NAILING_OFFSET * np.cos(np.radians(SILL_NAILING_ANGLE)) + (el_dims[1] / 2)) * -1),
            SMART_CONV_REST_ELEVATION + (el_dims[2] * 0.3) + (SILL_NAILING_OFFSET * np.sin(np.radians(SILL_NAILING_ANGLE))),
        ]
        doable = Robot_1.plan(
            tcp_name="tool2",
            target_pose=pre_pose,
            target_orientation=[left_quat[3], left_quat[0], left_quat[1], left_quat[2]],
            update_world_needed=True,
        )
        if doable:
            Robot_1.render_exec(renderInstance=True, Show_Sphere=False)
            dc = _dynamic_control.acquire_dynamic_control_interface()
            body = dc.get_rigid_body(f"/{Robot_1._ROS_JS_robot_indicator}/tool2")

            # Perform two nails
            for _ in range(2):
                pose = dc.get_rigid_body_pose(body)
                offset = SILL_NAILING_OFFSET - ((el_dims[1]/2)/np.cos(np.radians(SILL_NAILING_ANGLE))) + push_to_nail
                dx = offset * np.cos(np.radians(SILL_NAILING_ANGLE)) * -1
                dz = -offset * np.sin(np.radians(SILL_NAILING_ANGLE))

                Robot_1.release_path_plan_restriction()
                Robot_1.plan(
                    tcp_name="tool2",
                    target_pose=[pose.p[0], pose.p[1] + dx, pose.p[2] + dz],
                    target_orientation=[pose.r[3], pose.r[0], pose.r[1], pose.r[2]],
                    update_world_needed=True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles"],
                    direct_pose_cost=PoseCostMetric.create_grasp_approach_metric(
                        offset_position=0.0, tstep_fraction=0.001, linear_axis=2
                    ),
                )
                Robot_1.render_exec(renderInstance=True, Show_Sphere=False)

                # Retract
                pose = dc.get_rigid_body_pose(body)
                Robot_1.release_path_plan_restriction()
                Robot_1.plan(
                    tcp_name="tool2",
                    target_pose=[pose.p[0], pose.p[1] - dx, pose.p[2] - dz],
                    target_orientation=[pose.r[3], pose.r[0], pose.r[1], pose.r[2]],
                    update_world_needed=True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles"],
                    direct_pose_cost=PoseCostMetric.create_grasp_approach_metric(
                        offset_position=0.0, tstep_fraction=0.001, linear_axis=2
                    ),
                )
                Robot_1.render_exec(renderInstance=True, Show_Sphere=False)

                # Lift for next nail
                pose = dc.get_rigid_body_pose(body)
                Robot_1.release_path_plan_restriction()
                Robot_1.plan(
                    tcp_name="tool2",
                    target_pose=[pose.p[0], pose.p[1], pose.p[2] + (0.4 * el_dims[2])],
                    target_orientation=[pose.r[3], pose.r[0], pose.r[1], pose.r[2]],
                    update_world_needed=True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles"],
                    direct_pose_cost=PoseCostMetric.create_grasp_approach_metric(
                        offset_position=0.0, tstep_fraction=0.001, linear_axis=0
                    ),
                )
                Robot_1.render_exec(renderInstance=True, Show_Sphere=False)

            Robot_1.move_to_home()
        else:
            print("Left nail not possible due to Robot_1 reachability.")

    # Compute right-side nailing orientation
    right_quat = R.from_euler(
        'xyz',
        [(np.pi/2 + np.radians(SILL_NAILING_ANGLE)), 0, -np.pi/2]
    ).as_quat()
    right_quat[1] *= -1

    # Attempt right nail
    right_edge = conv_current_location - (el_dims[0] / 2) - NAILING_CONV_TARGET
    if right_edge < 0:
        print("Right Nail is not possible due to conveyor reachability.")
    else:
        Smart_Conv.render_exec('Joint_1', right_edge)

        # Pre-nail approach
        pre_pose = [
            2.3 + (el_pose[0] - (OVERALL_PANEL_HEIGHT / 2)) + SMART_CONV_X_SHIFT,
            -NAILING_CONV_TARGET + (SILL_NAILING_OFFSET * np.cos(np.radians(SILL_NAILING_ANGLE)) + (el_dims[1] / 2)),
            SMART_CONV_REST_ELEVATION + (el_dims[2] * 0.3) + (SILL_NAILING_OFFSET * np.sin(np.radians(SILL_NAILING_ANGLE))),
        ]
        doable = Robot_1.plan(
            tcp_name="tool2",
            target_pose=pre_pose,
            target_orientation=[right_quat[3], right_quat[0], right_quat[1], right_quat[2]],
            update_world_needed=True,
        )
        if doable:
            Robot_1.render_exec(renderInstance=True, Show_Sphere=False)
            dc = _dynamic_control.acquire_dynamic_control_interface()
            body = dc.get_rigid_body(f"/{Robot_1._ROS_JS_robot_indicator}/tool2")

            # Perform two nails
            for _ in range(2):
                pose = dc.get_rigid_body_pose(body)
                offset = SILL_NAILING_OFFSET - ((el_dims[1]/2)/np.cos(np.radians(SILL_NAILING_ANGLE))) + push_to_nail
                dx = offset * np.cos(np.radians(SILL_NAILING_ANGLE)) * 1
                dz = -offset * np.sin(np.radians(SILL_NAILING_ANGLE))

                Robot_1.release_path_plan_restriction()
                Robot_1.plan(
                    tcp_name="tool2",
                    target_pose=[pose.p[0], pose.p[1] + dx, pose.p[2] + dz],
                    target_orientation=[pose.r[3], pose.r[0], pose.r[1], pose.r[2]],
                    update_world_needed=True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles"],
                    direct_pose_cost=PoseCostMetric.create_grasp_approach_metric(
                        offset_position=0.0, tstep_fraction=0.001, linear_axis=2
                    ),
                )
                Robot_1.render_exec(renderInstance=True, Show_Sphere=False)

                # Retract
                pose = dc.get_rigid_body_pose(body)
                Robot_1.release_path_plan_restriction()
                Robot_1.plan(
                    tcp_name="tool2",
                    target_pose=[pose.p[0], pose.p[1] - dx, pose.p[2] - dz],
                    target_orientation=[pose.r[3], pose.r[0], pose.r[1], pose.r[2]],
                    update_world_needed=True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles"],
                    direct_pose_cost=PoseCostMetric.create_grasp_approach_metric(
                        offset_position=0.0, tstep_fraction=0.001, linear_axis=2
                    ),
                )
                Robot_1.render_exec(renderInstance=True, Show_Sphere=False)

                # Lift for next nail
                pose = dc.get_rigid_body_pose(body)
                Robot_1.release_path_plan_restriction()
                Robot_1.plan(
                    tcp_name="tool2",
                    target_pose=[pose.p[0], pose.p[1], pose.p[2] + (0.4 * el_dims[2])],
                    target_orientation=[pose.r[3], pose.r[0], pose.r[1], pose.r[2]],
                    update_world_needed=True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles"],
                    direct_pose_cost=PoseCostMetric.create_grasp_approach_metric(
                        offset_position=0.0, tstep_fraction=0.001, linear_axis=0
                    ),
                )
                Robot_1.render_exec(renderInstance=True, Show_Sphere=False)

            Robot_1.move_to_home()
        else:
            print("Right nail not possible due to Robot_1 reachability.")

def Place_Short_Horizontal_Element_On_Smart_Conveyor_by_Rob2_Gripper(
    el_name: str = None,
    X: float = None,
    Y: float = None,
    L: float = None,
    H: float = None,
) -> float:
    """
    Place a less than 8ft (2.4384 m) vertical element onto the smart conveyor using Robot_2’s gripper.

    Parameters:
        el_name (str): Name of the element to place.
        X (float): X value of element's position from the top left corner of the building panel.
        Y (float): Y value of element's position from the top left corner of the building panel.
        L (float): Length of the element.
        H (float): Height of the element.

    Returns:
        Conv_Curr_Loc (float): Conveyor joint value when element at TCP zero.
    """
    # Stage 1: Align smart conveyor for placement
    half_length = OVERALL_PANEL_LENGTH / 2
    mid_range = SMART_CONV_RANGE_OF_MOTION_J1 / 2
    conv_target = -(Y - half_length) - ((L / 2) - PICK_OFFSET_FROM_L_CORNER - (ROBOT_1_GRIPPER_LENGTH / 2)) + mid_range
    Smart_Conv.render_exec('Joint_1', conv_target)

    # Stage 2: Compute and store zero-position joint value
    Conv_Curr_Loc = -(Y - half_length) + mid_range

    # Stage 3: Attempt Robot_2 pre-place approach
    pre_pose = [
        2.3 + (X - (OVERALL_PANEL_HEIGHT / 2)) + SMART_CONV_X_SHIFT,
        0,
        SMART_CONV_REST_ELEVATION + H - PICK_OFFSET_FROM_W_CORNER + 0.3,
    ]
    reachable = Robot_2.plan(
        tcp_name="tool0",
        target_pose=pre_pose,
        target_orientation=[0, -1, 0, 0],
        update_world_needed=True,
    )

    if reachable:
        # Stage 4a: Execute place motion
        Robot_2.render_exec(renderInstance=True, Show_Sphere=False)
        place_pose = [
            pre_pose[0],
            pre_pose[1],
            SMART_CONV_REST_ELEVATION + H - PICK_OFFSET_FROM_W_CORNER + 0.1,
        ]
        Robot_2.plan(
            tcp_name="tool0",
            target_pose=place_pose,
            target_orientation=[0, -1, 0, 0],
            update_world_needed=True,
            removing_primitives=["Smart_Conveyor", "world/obstacles"],
            direct_pose_cost=PoseCostMetric.create_grasp_approach_metric(
                offset_position=0.0, tstep_fraction=0.001, linear_axis=2
            ),
        )
        Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

        # Stage 4b: Detach and enable gravity
        Robot_2.eef_detach(tool_name="tool0", detaching_object_name=el_name)
        test._stage.GetPrimAtPath(f"/world/obstacles/{el_name}") \
             .GetAttribute("physxRigidBody:disableGravity") \
             .Set(False)
        Smart_Conv.attach_object_to_conv(obj_name=el_name)
    else:
        # Stage 5: Manual placement fallback
        print("Bottom sill plate placement is out of Robot_2’s reach. Manual action required.")
        fallback_pose = [
            pre_pose[0] + (H / 2 - PICK_OFFSET_FROM_W_CORNER),
            0,
            SMART_CONV_REST_ELEVATION + H - PICK_OFFSET_FROM_W_CORNER + 0.3,
        ]
        Robot_2.plan(
            tcp_name="tool0",
            target_pose=fallback_pose,
            target_orientation=[0, -ev, 0, ev],
            update_world_needed=True,
        )
        Robot_2.render_exec(renderInstance=True, Show_Sphere=False)
        Robot_2.eef_detach(tool_name="tool0", detaching_object_name=el_name)

        # Stage 6: Snap element to valid pose under gravity
        test._stage.GetPrimAtPath(f"/world/obstacles/{el_name}") \
             .GetAttribute("physics:rigidBodyEnabled") \
             .Set(True)
        dc = _dynamic_control.acquire_dynamic_control_interface()
        body = dc.get_rigid_body(f"/world/obstacles/{el_name}")
        pose = dc.get_rigid_body_pose(body)

        new_loc = Gf.Vec3d(pose.p[0], pose.p[1], SMART_CONV_REST_ELEVATION + (H / 2))
        quat = Gf.Quatd(ev, 0, 1, 0)
        rot = Gf.Rotation(quat)
        q = rot.GetQuat()
        transform = _dynamic_control.Transform()
        transform.p.x, transform.p.y, transform.p.z = new_loc
        transform.r.w, transform.r.x, transform.r.y, transform.r.z = (
            q.GetReal(), *q.GetImaginary()
        )
        dc.set_rigid_body_pose(body, transform)

        test._stage.GetPrimAtPath(f"/world/obstacles/{el_name}") \
             .GetAttribute("physics:rigidBodyEnabled") \
             .Set(False)
        test._stage.GetPrimAtPath(f"/world/obstacles/{el_name}") \
             .GetAttribute("physxRigidBody:disableGravity") \
             .Set(False)
        Smart_Conv.attach_object_to_conv(obj_name=el_name)

    # Stage 7: Return Robot_2 home
    Robot_2.move_to_home()

    return Conv_Curr_Loc

def Nail_Short_Horizontal_Element_by_Rob2_NailGun(
    push_to_nail: float = PUSH_TO_NAIL_OFFSET_TANGENT,
    el_pose: List[float] = None,
    el_dims: List[float] = None,
    conv_current_location: float = None,
) -> None:
    """
    Nail a less than 8ft (2.4384 m) upper horizontal element at two positions (left and right)
    onto the smart conveyor using Robot_2.

    Parameters:
        push_to_nail (float): Distance to push into the nail.
        el_pose (List[float]): [x, y, z] pose of the element on the by assuming the origin is at the top left corner of the building panel.
        el_dims (List[float]): [length, width, height] of the element.
        conv_current_location (float): Conveyor joint position when element is placed at TCP zero position.

    Procedure:
    1. Compute nailing quaternion for left side.
    2. If left side within conveyor reach:
       a. Move conveyor to left nail pose.
       b. Robot_2 approaches pre-nail pose and verifies reachability.
       c. Perform two angled nails (push/retract, lift, push/retract).
       d. Return Robot_2 home.
    3. Compute nailing quaternion for right side.
    4. Repeat the above steps for the right nail.
    """
    # Precompute constants
    sill_angle = np.radians(SILL_NAILING_ANGLE)
    half_length = el_dims[0] / 2

    # Compute base quaternion and transform to [w,x,y,z]
    base_quat = R.from_euler('xyz', [-(np.pi/2 + sill_angle), 0, 0]).as_quat()
    quat = [base_quat[3], base_quat[1], base_quat[2], -base_quat[0]]

    # --- Left Nail ---
    left_target = conv_current_location + half_length + NAILING_CONV_TARGET
    if left_target <= SMART_CONV_RANGE_OF_MOTION_J1:
        Smart_Conv.render_exec('Joint_1', left_target)

        # Pre-nail approach pose
        pre_pose = [
            2.3 + (el_pose[0] - (OVERALL_PANEL_HEIGHT / 2)) + SMART_CONV_X_SHIFT,
            NAILING_CONV_TARGET + ((SILL_NAILING_OFFSET * np.cos(sill_angle) + (el_dims[1] * 1.5)) * -1),
            SMART_CONV_REST_ELEVATION + (el_dims[2] * 0.3) + (SILL_NAILING_OFFSET * np.sin(sill_angle)),
        ]
        doable = Robot_2.plan(
            tcp_name="tool1",
            target_pose=pre_pose,
            target_orientation=quat,
            update_world_needed=True,
        )
        if doable:
            Robot_2.render_exec(renderInstance=True, Show_Sphere=False)
            dc = _dynamic_control.acquire_dynamic_control_interface()
            body = dc.get_rigid_body(f"/{Robot_2._ROS_JS_robot_indicator}/tool1")

            # Perform two nail operations
            for _ in range(2):
                pose = dc.get_rigid_body_pose(body)
                offset = SILL_NAILING_OFFSET - ((el_dims[1]/2)/np.cos(sill_angle)) + push_to_nail
                dx = offset * np.cos(sill_angle)
                dz = -offset * np.sin(sill_angle)

                Robot_2.release_path_plan_restriction()
                Robot_2.plan(
                    tcp_name="tool1",
                    target_pose=[pose.p[0], pose.p[1] + dx, pose.p[2] + dz],
                    target_orientation=pose.r.tolist(),
                    update_world_needed=True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles"],
                    direct_pose_cost=PoseCostMetric.create_grasp_approach_metric(
                        offset_position=0.0, tstep_fraction=0.001, linear_axis=2
                    ),
                )
                Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

                # Retract
                pose = dc.get_rigid_body_pose(body)
                Robot_2.release_path_plan_restriction()
                Robot_2.plan(
                    tcp_name="tool1",
                    target_pose=[pose.p[0], pose.p[1] - dx, pose.p[2] - dz],
                    target_orientation=pose.r.tolist(),
                    update_world_needed=True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles"],
                    direct_pose_cost=PoseCostMetric.create_grasp_approach_metric(
                        offset_position=0.0, tstep_fraction=0.001, linear_axis=2
                    ),
                )
                Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

                # Lift for next nail
                pose = dc.get_rigid_body_pose(body)
                Robot_2.release_path_plan_restriction()
                Robot_2.plan(
                    tcp_name="tool1",
                    target_pose=[pose.p[0], pose.p[1], pose.p[2] + (0.4 * el_dims[2])],
                    target_orientation=pose.r.tolist(),
                    update_world_needed=True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles"],
                    direct_pose_cost=PoseCostMetric.create_grasp_approach_metric(
                        offset_position=0.0, tstep_fraction=0.001, linear_axis=1
                    ),
                )
                Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

            Robot_2.move_to_home()
        else:
            print("Left nail not possible due to Robot_2 reachability.")
    else:
        print("Left nail impossible due to conveyor reachability.")

    # --- Right Nail ---
    right_base = R.from_euler('xyz', [-(np.pi/2) + sill_angle, 0, np.pi]).as_quat()
    right_quat = [right_base[3], right_base[1], right_base[2], -right_base[0]]

    right_target = conv_current_location - half_length - NAILING_CONV_TARGET
    if right_target >= 0:
        Smart_Conv.render_exec('Joint_1', right_target)

        pre_pose = [
            2.3 + (el_pose[0] - (OVERALL_PANEL_HEIGHT / 2)) + SMART_CONV_X_SHIFT,
            -NAILING_CONV_TARGET + ((SILL_NAILING_OFFSET * np.cos(sill_angle) + (el_dims[1] * 1.5))),
            SMART_CONV_REST_ELEVATION + (el_dims[2] * 0.3) + (SILL_NAILING_OFFSET * np.sin(sill_angle)),
        ]
        doable = Robot_2.plan(
            tcp_name="tool1",
            target_pose=pre_pose,
            target_orientation=right_quat,
            update_world_needed=True,
        )
        if doable:
            Robot_2.render_exec(renderInstance=True, Show_Sphere=False)
            dc = _dynamic_control.acquire_dynamic_control_interface()
            body = dc.get_rigid_body(f"/{Robot_2._ROS_JS_robot_indicator}/tool1")

            for _ in range(2):
                pose = dc.get_rigid_body_pose(body)
                offset = SILL_NAILING_OFFSET - ((el_dims[1]/2)/np.cos(sill_angle)) + push_to_nail
                dx = -offset * np.cos(sill_angle)
                dz = -offset * np.sin(sill_angle)

                Robot_2.release_path_plan_restriction()
                Robot_2.plan(
                    tcp_name="tool1",
                    target_pose=[pose.p[0], pose.p[1] + dx, pose.p[2] + dz],
                    target_orientation=pose.r.tolist(),
                    update_world_needed=True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles"],
                    direct_pose_cost=PoseCostMetric.create_grasp_approach_metric(
                        offset_position=0.0, tstep_fraction=0.001, linear_axis=2
                    ),
                )
                Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

                pose = dc.get_rigid_body_pose(body)
                Robot_2.release_path_plan_restriction()
                Robot_2.plan(
                    tcp_name="tool1",
                    target_pose=[pose.p[0], pose.p[1] - dx, pose.p[2] - dz],
                    target_orientation=pose.r.tolist(),
                    update_world_needed=True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles"],
                    direct_pose_cost=PoseCostMetric.create_grasp_approach_metric(
                        offset_position=0.0, tstep_fraction=0.001, linear_axis=2
                    ),
                )
                Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

                pose = dc.get_rigid_body_pose(body)
                Robot_2.release_path_plan_restriction()
                Robot_2.plan(
                    tcp_name="tool1",
                    target_pose=[pose.p[0], pose.p[1], pose.p[2] + (0.4 * el_dims[2])],
                    target_orientation=pose.r.tolist(),
                    update_world_needed=True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles"],
                    direct_pose_cost=PoseCostMetric.create_grasp_approach_metric(
                        offset_position=0.0, tstep_fraction=0.001, linear_axis=1
                    ),
                )
                Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

            Robot_2.move_to_home()
        else:
            print("Right nail not possible due to Robot_2 reachability.")
    else:
        print("Right nail impossible due to conveyor reachability.")

def Place_Short_Vertical_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name: str = None,
        X: float = None,
        Y: float = None,
        L: float = None,
        H: float = None):

    '''
    Place a less than 8ft (2.4384 m) vertical element onto the Smart Conveyor
    using Robot 1’s gripper.

    Parameters
    ----------
    el_name : str
        Name (USD prim path) of the element currently held by Robot 1’s tool0.
    X : float
        X value of element's position from the top left corner of the building panel.
    Y : float
        Y value of element's position from the top left corner of the building panel.
    L : float
        Length of the element along its longitudinal axis (metres).
    H : float
        Height (thickness in Z) of the element (metres).

    Returns
    -------
    int
        `Side_Selector` +1 or -1 indicating which side the conveyor moved to.

    Workflow
    --------
    1. **Align conveyor** – Jog Smart Conveyor joint 1 so the element’s Y origin is
    centred in the robot workspace.
    2. **Reachability check** – Evaluate whether a positive-Y (+ NAILING_CONV_TARGET)
    or negative-Y (-2 × NAILING_CONV_TARGET) approach is kinematically reachable.
    3. **Positive-Y sequence (preferred)**  
    a. Fine-position conveyor.  
    b. Plan and execute Robot 1 pre-place pose.  
    c. Lower, detach element, enable gravity, and attach it to the conveyor.  
    d. Retract and return Robot 1 to home; set `Side_Selector = 1`.
    4. **Negative-Y fallback** – Mirror the above manoeuvre on the negative side; set
    `Side_Selector = -1`.
    5. **Last-resort manual assist** – If neither pose is reachable, drop the element
    at a neutral pose for human assistance and leave `Side_Selector` unchanged.
    6. **Return** the computed `Side_Selector` so downstream nailing routines know
    which side of the conveyor to target.
    '''

    # Moving Conveyor
    Smart_Conv.render_exec('Joint_1', -(Y - (OVERALL_PANEL_LENGTH/2)) + (SMART_CONV_RANGE_OF_MOTION_J1/2))

    # # It Calculates the Conveyor Joint Value where the Stud is in 0 Position
    Conv_Curr_Loc: float = -(Y - (OVERALL_PANEL_LENGTH/2)) + (SMART_CONV_RANGE_OF_MOTION_J1/2)
    Solver_Flag: bool = False
    Side_Selector: float = 1

    # Check if Positive Value is Suitable for Robot !
    if((-(Y - (OVERALL_PANEL_LENGTH/2)) + (SMART_CONV_RANGE_OF_MOTION_J1/2)+NAILING_CONV_TARGET <= SMART_CONV_RANGE_OF_MOTION_J1)):
        Smart_Conv.render_exec('Joint_1', -(Y - (OVERALL_PANEL_LENGTH/2)) + (SMART_CONV_RANGE_OF_MOTION_J1/2) + NAILING_CONV_TARGET)
        # Robot 1 Pre Place
        Robot_1.plan(tcp_name= "tool0",
                        target_pose= [2.3+(X-(OVERALL_PANEL_HEIGHT/2))+SMART_CONV_X_SHIFT-((ROBOT_1_GRIPPER_LENGTH/2)+PICK_OFFSET_FROM_L_CORNER_AFTER_PASS-(L/2)),
                                        NAILING_CONV_TARGET,
                                        SMART_CONV_REST_ELEVATION+H-PICK_OFFSET_FROM_W_CORNER+0.2],
                        target_orientation= [0, -ev, -ev, 0],
                        update_world_needed= True)
        Robot_1.render_exec(renderInstance= True,
                                Show_Sphere= False)

        # Robot 1 Drop
        Robot_1.plan(tcp_name= "tool0",
                        target_pose= [2.3+(X-(OVERALL_PANEL_HEIGHT/2))+SMART_CONV_X_SHIFT-((ROBOT_1_GRIPPER_LENGTH/2)+PICK_OFFSET_FROM_L_CORNER_AFTER_PASS-(L/2)),
                                        NAILING_CONV_TARGET,
                                        SMART_CONV_REST_ELEVATION+H-PICK_OFFSET_FROM_W_CORNER+0.1],
                        target_orientation= [0, -ev, -ev, 0],
                        update_world_needed= True,
                        removing_primitives=["Smart_Conveyor", "world/obstacles", "IRB6620_R2"],
                        direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
        Robot_1.render_exec(renderInstance= True,
                                Show_Sphere= False)

        Robot_1.eef_detach(tool_name="tool0",
                            detaching_object_name= el_name)
        test._stage.GetPrimAtPath("/world/obstacles/" + el_name).GetAttribute("physxRigidBody:disableGravity").Set(False)
        Smart_Conv.attach_object_to_conv(obj_name= el_name)

        # Robot 1 Drop
        Robot_1.plan(tcp_name= "tool0",
                        target_pose= [2.3+(X-(OVERALL_PANEL_HEIGHT/2))+SMART_CONV_X_SHIFT-((ROBOT_1_GRIPPER_LENGTH/2)+PICK_OFFSET_FROM_L_CORNER_AFTER_PASS-(L/2)),
                                        NAILING_CONV_TARGET,
                                        SMART_CONV_REST_ELEVATION+H-PICK_OFFSET_FROM_W_CORNER+0.2],
                        target_orientation= [0, -ev, -ev, 0],
                        update_world_needed= True,
                        removing_primitives=["Smart_Conveyor", "world/obstacles", "IRB6620_R2"],
                        direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
        Robot_1.render_exec(renderInstance= True,
                                Show_Sphere= False)

        Robot_1.move_to_home()

        # Robot_1_Do_Side_Nail(push_to_nail= PUSH_TO_NAIL_OFFSET, H= H, Side_Selector= 1)

        Solver_Flag = True

    # Check if Negative Value is Suitable for Robot !
    if((-(Y - (OVERALL_PANEL_LENGTH/2)) + (SMART_CONV_RANGE_OF_MOTION_J1/2)-NAILING_CONV_TARGET*2 >= 0) and Solver_Flag == False):
        Smart_Conv.render_exec('Joint_1', -(Y - (OVERALL_PANEL_LENGTH/2)) + (SMART_CONV_RANGE_OF_MOTION_J1/2) - 2*NAILING_CONV_TARGET)
        # Robot 1 Pre Place
        Robot_1.plan(tcp_name= "tool0",
                        target_pose= [2.3+(X-(OVERALL_PANEL_HEIGHT/2))+SMART_CONV_X_SHIFT+((ROBOT_1_GRIPPER_LENGTH/2)+PICK_OFFSET_FROM_L_CORNER_AFTER_PASS-(L/2)),
                                        -2*NAILING_CONV_TARGET,
                                        SMART_CONV_REST_ELEVATION+H-PICK_OFFSET_FROM_W_CORNER+0.2],
                        target_orientation= [0, -ev, ev, 0],
                        update_world_needed= True)
        Robot_1.render_exec(renderInstance= True,
                                Show_Sphere= False)

        # Robot 1 Drop
        Robot_1.plan(tcp_name= "tool0",
                        target_pose= [2.3+(X-(OVERALL_PANEL_HEIGHT/2))+SMART_CONV_X_SHIFT+((ROBOT_1_GRIPPER_LENGTH/2)+PICK_OFFSET_FROM_L_CORNER_AFTER_PASS-(L/2)),
                                        -2*NAILING_CONV_TARGET,
                                        SMART_CONV_REST_ELEVATION+H-PICK_OFFSET_FROM_W_CORNER+0.1],
                        target_orientation= [0, -ev, ev, 0],
                        update_world_needed= True,
                        removing_primitives=["Smart_Conveyor", "world/obstacles", "IRB6620_R2"],
                        direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
        Robot_1.render_exec(renderInstance= True,
                                Show_Sphere= False)

        Robot_1.eef_detach(tool_name="tool0",
                            detaching_object_name= el_name)
        test._stage.GetPrimAtPath("/world/obstacles/" + el_name).GetAttribute("physxRigidBody:disableGravity").Set(False)
        Smart_Conv.attach_object_to_conv(obj_name= el_name)

        # Robot 1 Drop
        Robot_1.plan(tcp_name= "tool0",
                        target_pose= [2.3+(X-(OVERALL_PANEL_HEIGHT/2))+SMART_CONV_X_SHIFT+((ROBOT_1_GRIPPER_LENGTH/2)+PICK_OFFSET_FROM_L_CORNER_AFTER_PASS-(L/2)),
                                        -2*NAILING_CONV_TARGET,
                                        SMART_CONV_REST_ELEVATION+H-PICK_OFFSET_FROM_W_CORNER+0.2],
                        target_orientation= [0, -ev, ev, 0],
                        update_world_needed= True,
                        removing_primitives=["Smart_Conveyor", "world/obstacles", "IRB6620_R2"],
                        direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
        Robot_1.render_exec(renderInstance= True,
                                Show_Sphere= False)

        Robot_1.move_to_home()

        # Changing Is_TCP to true for the condition that cripple is being dropped in Y = -nail_offset (to set the nailgun's target to -2*nail_offset rather than nail_offset)
        # why? because on y = -nail_offset the robot's nailgun will hit the links, so we have to push it further !
        # Robot_1_Do_Side_Nail(push_to_nail= PUSH_TO_NAIL_OFFSET, H= H, Side_Selector= -1, Is_TCP= True)

        Solver_Flag = True

    # Check if it's reachable for Robot 1 to Place the Cripple
    if((-(Y - (OVERALL_PANEL_LENGTH/2)) + (SMART_CONV_RANGE_OF_MOTION_J1/2)+NAILING_CONV_TARGET > SMART_CONV_RANGE_OF_MOTION_J1) and 
        (-(Y - (OVERALL_PANEL_LENGTH/2)) + (SMART_CONV_RANGE_OF_MOTION_J1/2)-NAILING_CONV_TARGET*2 < 0) and
        Solver_Flag == False):
        # Robot 1 Can't Reach The Place Location with the Placing Orientation
        print("Robot 1 Needs Human's Help to Place the Top Cripple")
        # [0, -1, 0, 0]

        # Robot 1 Reach
        Robot_1.plan(tcp_name= "tool0",
                        target_pose= [2.3+(X-(OVERALL_PANEL_HEIGHT/2))+SMART_CONV_X_SHIFT,
                                        (ROBOT_1_GRIPPER_LENGTH/2)+PICK_OFFSET_FROM_L_CORNER_AFTER_PASS-(L/2),
                                        SMART_CONV_REST_ELEVATION+H-PICK_OFFSET_FROM_W_CORNER+0.2],
                        target_orientation= [0, 1, 0, 0],
                        update_world_needed= True)
        Robot_1.render_exec(renderInstance= True,
                                Show_Sphere= False)

        # Relocating the Stud into its Location
        # [90, 0, 90]
        # [0.5, 0.5, -0.5, 0.5]
        Robot_1.eef_detach(tool_name="tool0",
                            detaching_object_name= el_name) 

        dc = _dynamic_control.acquire_dynamic_control_interface()
        test._stage.GetPrimAtPath("/world/obstacles/" + el_name).GetAttribute("physics:rigidBodyEnabled").Set(True)
        object = dc.get_rigid_body("/world/obstacles/" + el_name)
        object_pose = dc.get_rigid_body_pose(object)

        # Create new position
        New_Loc = Gf.Vec3d(object_pose.p[0], object_pose.p[1], SMART_CONV_REST_ELEVATION + (H / 2))

        # Define the new orientation quaternion directly
        quat_as_tuple = (0.5, 0.5, -0.5, 0.5)  # (real, x, y, z)

        # Create a Transform object
        new_transform = _dynamic_control.Transform()
        new_transform.p.x = New_Loc[0]
        new_transform.p.y = New_Loc[1]
        new_transform.p.z = New_Loc[2]
        new_transform.r.w = quat_as_tuple[0]  # real part
        new_transform.r.x = quat_as_tuple[1]
        new_transform.r.y = quat_as_tuple[2]
        new_transform.r.z = quat_as_tuple[3]

        # Apply transformation
        dc.set_rigid_body_pose(object, new_transform)

        test._stage.GetPrimAtPath("/world/obstacles/" + el_name).GetAttribute("physics:rigidBodyEnabled").Set(False)
        test._stage.GetPrimAtPath("/world/obstacles/" + el_name).GetAttribute("physxRigidBody:disableGravity").Set(False)
        Smart_Conv.attach_object_to_conv(obj_name=el_name)

        Robot_1.move_to_home()

    if (OVERALL_PANEL_LENGTH/2)- Y +(SMART_CONV_RANGE_OF_MOTION_J1/2)+NAILING_CONV_TARGET > SMART_CONV_RANGE_OF_MOTION_J1:
        Side_Selector = -1
        return Side_Selector
    elif (OVERALL_PANEL_LENGTH/2)- Y +(SMART_CONV_RANGE_OF_MOTION_J1/2)-(NAILING_CONV_TARGET*2) < 0:
        Side_Selector = 1
        return Side_Selector

def Place_Short_Vertical_Element_On_Smart_Conveyor_by_Rob2_Gripper(el_name: str = None,
        X: float = None,
        Y: float = None,
        L: float = None,
        H: float = None):
    '''
    Brief Description:
        Places a less than 8ft (2.4384 m) vertical element onto the smart conveyor using Robot 2's gripper.
    Parameters:
        el_name (str): Name of the element to place.
        X (float): X value of element's position from the top left corner of the building panel.
        Y (float): Y value of element's position from the top left corner of the building panel.
        L (float): Length of the element.
        H (float): Height of the element.

    Steps of Working:
        1. Move the smart conveyor so the element aligns with Robot 2 based on Y.
        2. Send Robot 2 to its home position.
        3. Perform "Pre Place1" motion: plan approach pose and render.
        4. Perform "Pre Place2" motion: plan exact placement pose (removing obstacles) and render.
        5. Detach the element from the gripper, enable its gravity, and attach it to the conveyor.
        6. Execute "Post Place" motion: retract to a safe pose and render.
        7. Return Robot 2 to its home position.
        8. Calculate the correct side selector for nailing based on Y, and record the conveyor joint
           and side selector in Smart_Conv._nail_poses for nailing operations.
    '''

    # Moving Conveyor
    Smart_Conv.render_exec('Joint_1', -(Y - (OVERALL_PANEL_LENGTH/2)) + (SMART_CONV_RANGE_OF_MOTION_J1/2))

    # Home
    Robot_2.move_to_home()

    # Pre Place1
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [2.3+(X-(OVERALL_PANEL_HEIGHT/2))+SMART_CONV_X_SHIFT-((L/2)-(ROBOT_2_GRIPPER_LENGTH/2)-PICK_OFFSET_FROM_L_CORNER)+0.1,
                                  0,
                                  SMART_CONV_REST_ELEVATION+H-PICK_OFFSET_FROM_W_CORNER+0.15],
                    target_orientation= [0, -ev, -ev, 0],
                    update_world_needed= True)
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)

    # Pre Place2
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [2.3+(X-(OVERALL_PANEL_HEIGHT/2))+SMART_CONV_X_SHIFT-((L/2)-(ROBOT_2_GRIPPER_LENGTH/2)-PICK_OFFSET_FROM_L_CORNER),
                                  0,
                                  SMART_CONV_REST_ELEVATION+H-PICK_OFFSET_FROM_W_CORNER],
                    target_orientation= [0, -ev, -ev, 0],
                    update_world_needed= True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles"],
                    orientational_restriction=torch.tensor([1,1,1], dtype=torch.float32))
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)
    
    Robot_2.eef_detach(tool_name="tool0",
                        detaching_object_name= el_name)
    # Enabling Gravity For The Stud
    test._stage.GetPrimAtPath("/world/obstacles/" + el_name).GetAttribute("physxRigidBody:disableGravity").Set(False)
    Smart_Conv.attach_object_to_conv(obj_name= el_name)

    # Post Place
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [2.3+(X-(OVERALL_PANEL_HEIGHT/2))+SMART_CONV_X_SHIFT-((L/2)-(ROBOT_2_GRIPPER_LENGTH/2)-PICK_OFFSET_FROM_L_CORNER)+0.1,
                                  0,
                                  SMART_CONV_REST_ELEVATION+H-PICK_OFFSET_FROM_W_CORNER+0.15],
                    target_orientation= [0, -ev, -ev, 0],
                    update_world_needed= True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles"],
                    orientational_restriction=torch.tensor([1,1,1], dtype=torch.float32))
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)

    Robot_2.move_to_home()

    # Adding Nail Targets For BPL !

    # Conveyor Move For Placement
    Side_Selector: float = 0

    Side_Selector = 1
    if ((OVERALL_PANEL_LENGTH/2)- Y +(SMART_CONV_RANGE_OF_MOTION_J1/2)+NAILING_CONV_TARGET > SMART_CONV_RANGE_OF_MOTION_J1):
        Side_Selector = -1

    Side_Selector = -1
    if ((OVERALL_PANEL_LENGTH/2)- Y +(SMART_CONV_RANGE_OF_MOTION_J1/2)+NAILING_CONV_TARGET < 0):
        Side_Selector = 1

    # # Saving Joint Location For Nailing
    Smart_Conv._nail_poses.append(((OVERALL_PANEL_LENGTH/2)- Y +(SMART_CONV_RANGE_OF_MOTION_J1/2)+NAILING_CONV_TARGET*Side_Selector, Side_Selector))

def Pass_8ft_Element_G2S(el_name: str = None,
        L: float = None,
        H: float = None):
    '''
    Brief Description:
        Transfers an 8-foot element from Robot 2's gripper to Robot 1's suction tool.

    Parameters:
        el_name (str): Name of the element to transfer.
        L (float): Length of the element.
        H (float): Height of the element.

    Steps of Working:
        1. Move the smart conveyor to its start position (Joint_1 = 0).
        2. Define a constant pass location (R2_LU_Pass_Loc) for Robot 2.
        3. Compute the length-based offset (Length_Diff_Term) for Robot 1 approach.
        4. Have Robot 2 plan and execute to the pass location, removing world obstacles.
        5. Robot 1 performs a pre-passing motion to the computed approach pose and renders.
        6. Robot 1 executes the passing motion closer to the element, removes the IRB6620_R2 primitive,
           and applies a grasp-approach cost metric, then renders.
        7. Detach the element from Robot 2’s gripper; attach it to Robot 1’s tool.
        8. Return both Robot 2 and Robot 1 to their home positions.
    '''

    # Moving Conveyor To Start Point
    Smart_Conv.render_exec('Joint_1', 0)

    # We Consider this As a Constant Point Regardless of the Dimensions of the Passing L_U
    R2_LU_Pass_Loc: List[float] = [2.81634, 1.25053, 1.35738]

    Length_Diff_Term: float = (-ROBOT_2_GRIPPER_LENGTH/2)-SLOPED_TABLE_PICK_OFFSET_FROM_L_CORNER+L-(ROBOT_1_SUCTION_CUP_R)

    Robot_2.plan(tcp_name= "tool0",
                    target_pose= R2_LU_Pass_Loc,
                    target_orientation= [-0.6123661 ,  0.6123697 ,  0.35354862,  0.35357386],
                    update_world_needed= True,
                    removing_primitives=["world/obstacles"],)
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)

    # Pre Passing
    Robot_1.plan(tcp_name= "tool1",
                    target_pose= [R2_LU_Pass_Loc[0]-(Length_Diff_Term*np.cos(np.radians(L_U_PASS_ANGLE))),
                                  R2_LU_Pass_Loc[1]-PICK_OFFSET_FROM_W_CORNER+H/2+(ROBOT_1_SUCTION_WIDTH/2),
                                  R2_LU_Pass_Loc[2]+(Length_Diff_Term*np.sin(np.radians(L_U_PASS_ANGLE)))-0.1],
                    target_orientation= [0.96593, 0, 0.25882, 0],
                    update_world_needed= True)
    Robot_1.render_exec(renderInstance= True,
                            Show_Sphere= False)

    # Passing
    Robot_1.plan(tcp_name= "tool1",
                    target_pose= [R2_LU_Pass_Loc[0]-(Length_Diff_Term*np.cos(np.radians(L_U_PASS_ANGLE))),
                                  R2_LU_Pass_Loc[1]-PICK_OFFSET_FROM_W_CORNER+H/2+(ROBOT_1_SUCTION_WIDTH/2),
                                  R2_LU_Pass_Loc[2]+(Length_Diff_Term*np.sin(np.radians(L_U_PASS_ANGLE)))-0.06],
                    target_orientation= [0.96593, 0, 0.25882, 0],
                    update_world_needed= True,
                    removing_primitives=["IRB6620_R2"],
                    direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
    Robot_1.render_exec(renderInstance= True,
                            Show_Sphere= False)

    # Attach
    Robot_2.eef_detach(tool_name="tool0",
                        detaching_object_name= el_name)
    # Isaac Attach is Not Working Cause the OmniGraph Attacher Coordination is Outside of the Stud for Suction !!
    Robot_1.eef_attach(tool_name="tool1",
                       attaching_object_name=el_name)
    
    Robot_2.move_to_home()
    Robot_1.move_to_home()

def Place_8ft_Vertical_Element_On_Smart_Conveyor_by_Rob1_Suction(el_name: str = None,
        X: float = None,
        Y: float = None,
        Z: float = None,
        L: float = None,
        W: float = None,
        H: float = None):
    '''
    Brief Description:
        Places an 8-foot vertical element onto the smart conveyor using Robot 1’s suction tool.
    Parameters:
        el_name (str): Name of the element to place.
        X (float): X value of element's position from the top left corner of the building panel.
        Y (float): Y value of element's position from the top left corner of the building panel.
        Z (float): Z value of element's position from the top left corner of the building panel.
        L (float): Length of the element.
        W (float): Width of the element.
        H (float): Height of the element.

    Steps of Working:
        1. Move the smart conveyor to align the element based on Y.
        2. Compute a small placement offset constant.
        3. Plan and render Robot 1’s pre-place approach pose.
        4. Plan and render the main place pose (removing obstacles, using grasp-approach cost).
        5. Detach the element from the suction tool, enable its gravity, attach it to the conveyor,
           and enable its colliders.
        6. Plan and render Robot 1’s post-place retract pose (removing obstacles, using grasp-approach cost).
        7. Return Robot 1 to its home position.
    '''

    # Moving Conveyor
    Smart_Conv.render_exec('Joint_1', -(Y - (OVERALL_PANEL_LENGTH/2)) + (SMART_CONV_RANGE_OF_MOTION_J1/2))

    Placement_Offset: float = 0.00565

    #Robot 1 Pre Place
    Robot_1.plan(tcp_name= "tool1",
                    target_pose= [2.3-(X-(OVERALL_PANEL_HEIGHT/2))+SMART_CONV_X_SHIFT-((L/2)-2*ROBOT_1_SUCTION_CUP_R)-Placement_Offset,
                                  (-ROBOT_1_SUCTION_WIDTH)/2,
                                  SMART_CONV_REST_ELEVATION+Z+0.1+ROBOT_1_SUCTION_CUP_TO_TOOL_OFFSET+(W/2)],
                    target_orientation= [0, 1, 0, 0],
                    update_world_needed= True)
    Robot_1.render_exec(renderInstance= True,
                            Show_Sphere= False)
    
    #Robot 1 Place
    Robot_1.plan(tcp_name= "tool1",
                    target_pose= [2.3-(X-(OVERALL_PANEL_HEIGHT/2))+SMART_CONV_X_SHIFT-((L/2)-2*ROBOT_1_SUCTION_CUP_R)-Placement_Offset,
                                  (-ROBOT_1_SUCTION_WIDTH)/2,
                                  SMART_CONV_REST_ELEVATION+Z+ROBOT_1_SUCTION_CUP_TO_TOOL_OFFSET+(W/2)],
                    target_orientation= [0, 1, 0, 0],
                    update_world_needed= True,
                    removing_primitives=["world/obstacles", "Smart_Conveyor"],
                    direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
    Robot_1.render_exec(renderInstance= True,
                            Show_Sphere= False)

    Robot_1.eef_detach(tool_name="tool1", detaching_object_name= el_name)
    test._stage.GetPrimAtPath("/world/obstacles/"+el_name).GetAttribute("physxRigidBody:disableGravity").Set(False)
    Smart_Conv.attach_object_to_conv(obj_name= el_name, Enable_Gravity= False)
    # Enabling Colliders
    test._stage.GetPrimAtPath("/world/obstacles/"+el_name).GetAttribute("physics:collisionEnabled").Set(True)

    #Robot 1 Post Place
    Robot_1.plan(tcp_name= "tool1",
                    target_pose= [2.3-(X-(OVERALL_PANEL_HEIGHT/2))+SMART_CONV_X_SHIFT-((L/2)-2*ROBOT_1_SUCTION_CUP_R)-Placement_Offset,
                                  (-ROBOT_1_SUCTION_WIDTH)/2,
                                  SMART_CONV_REST_ELEVATION+Z+0.1+ROBOT_1_SUCTION_CUP_TO_TOOL_OFFSET+(W/2)],
                    target_orientation= [0, 1, 0, 0],
                    update_world_needed= True,
                    removing_primitives=["world/obstacles", "Smart_Conveyor"],
                    direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
    Robot_1.render_exec(renderInstance= True,
                            Show_Sphere= False)

    Robot_1.move_to_home()

def Pick_2x10_Header_by_Rob1_Suction(X: float = None,
        L: float = None,):
    '''
    Brief Description:
        Picks a 2x10 header element from the Bear Loading pile using Robot 1’s suction tool
    Parameters:
        X (float): X value of element's position from the top left corner of the building panel.
        L (float): Requested header length.

    Steps of Working:
        1. Verify that the requested length L does not exceed the maximum available header length.
        2. Plan and execute the pre-pick motion to position the suction cup above the target header.
        3. Plan and execute the pick motion, removing obstacles and applying grasp-approach metrics.
        4. Update the world state and attach the selected header to the suction tool.
        5. Plan and execute the post-pick retraction motion to lift the header from the pile.
    '''

    global NUMBER_OF_HEADERS

    # Check if the Requested Length is less or equal to the Maximum Length of the Bear Loading elements pilled up
    if RAW_HEADER_DIMENSIONS[0] < L:
        print("The Requested Length is Less Than the Maximum Length of the Bear Loading Elements")
        return

    # Pick Orientation [0, ev, -ev, 0] => From The Pile
    # Pre Pick
    Robot_1.plan(tcp_name= "tool1",
                    target_pose= [HEADER_CENTER_COORINATION[0]-ROBOT_1_SUCTION_WIDTH+(ROBOT_1_SUCTION_WIDTH-RAW_HEADER_DIMENSIONS[2])/2,
                                  HEADER_CENTER_COORINATION[1]-(RAW_HEADER_DIMENSIONS[0]/2)+ROBOT_1_SUCTION_LENGTH+ROBOT_1_SUCTION_CUP_R+HEADER_PICK_OFFSET_L,
                                  HEADER_CENTER_COORINATION[2]+NUMBER_OF_HEADERS*RAW_HEADER_DIMENSIONS[1]+ROBOT_1_SUCTION_CUP_TO_TOOL_OFFSET+0.1],
                    target_orientation= [0, ev, -ev, 0],
                    update_world_needed= True)
    Robot_1.render_exec(renderInstance= True,
                            Show_Sphere= False)

    # Pick
    Robot_1.plan(tcp_name= "tool1",
                    target_pose= [HEADER_CENTER_COORINATION[0]-ROBOT_1_SUCTION_WIDTH+(ROBOT_1_SUCTION_WIDTH-RAW_HEADER_DIMENSIONS[2])/2,
                                  HEADER_CENTER_COORINATION[1]-(RAW_HEADER_DIMENSIONS[0]/2)+ROBOT_1_SUCTION_LENGTH+ROBOT_1_SUCTION_CUP_R+HEADER_PICK_OFFSET_L,
                                  HEADER_CENTER_COORINATION[2]+NUMBER_OF_HEADERS*RAW_HEADER_DIMENSIONS[1]+ROBOT_1_SUCTION_CUP_TO_TOOL_OFFSET],
                    target_orientation= [0, ev, -ev, 0],
                    update_world_needed= True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles"],
                    direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
    Robot_1.render_exec(renderInstance= True,
                            Show_Sphere= False)

    Robot_1.motion_gen_update_world()
    # "/world/obstacles/Bear_Loading_Element_"+str(it+1)
    Robot_1.eef_attach(tool_name="tool1", attaching_object_name= "Bear_Loading_Element_"+str(NUMBER_OF_HEADERS))

    # Post Pick
    Robot_1.plan(tcp_name= "tool1",
                    target_pose= [HEADER_CENTER_COORINATION[0]-ROBOT_1_SUCTION_WIDTH+(ROBOT_1_SUCTION_WIDTH-RAW_HEADER_DIMENSIONS[2])/2,
                                  HEADER_CENTER_COORINATION[1]-(RAW_HEADER_DIMENSIONS[0]/2)+ROBOT_1_SUCTION_LENGTH+ROBOT_1_SUCTION_CUP_R+HEADER_PICK_OFFSET_L,
                                  HEADER_CENTER_COORINATION[2]+NUMBER_OF_HEADERS*RAW_HEADER_DIMENSIONS[1]+ROBOT_1_SUCTION_CUP_TO_TOOL_OFFSET+0.1],
                    target_orientation= [0, ev, -ev, 0],
                    update_world_needed= True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles"],
                    direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
    Robot_1.render_exec(renderInstance= True,
                            Show_Sphere= False)

def Cut_2x10_Header(L: float = None,
    W: float = None,
    H: float = None
):
    '''
    Brief Description:
        Cuts a 2x10 header element using Robot 1’s suction tool and the small cutting table saw.

    Parameters:
        L (float): Desired new length of the header after cutting.
        W (float): Width of the header element.
        H (float): Height (thickness) of the header element.

    Steps of Working:
        1. Plan and execute the pre-cut approach pose above the table saw.
        2. Plan and execute the cutting motion, removing obstacles and applying a grasp-approach cost.
        3. Simulate the cut:
           a. Detach the original element from the suction tool.
           b. Enable physics on the original element and capture its pose.
           c. Delete the original prim.
           d. Create a new Cuboid with updated dimensions (H, L, W) at the captured pose.
        4. Attach the newly created cut element to Robot 1’s suction tool.
        5. Plan and execute the post-cut retraction pose and render.
    '''

    # Pre Cut
    Robot_1.plan(tcp_name= "tool1",
                    target_pose= [SMALL_CUT_TABLE_SAW_POSE[0]+(ROBOT_1_SUCTION_LENGTH+ROBOT_1_SUCTION_CUP_R+HEADER_PICK_OFFSET_L-L),
                                  SMALL_CUT_TABLE_SAW_POSE[1]+(ROBOT_1_SUCTION_WIDTH/2),
                                  SMALL_CUT_TABLE_SAW_POSE[2]+W+ROBOT_1_SUCTION_CUP_TO_TOOL_OFFSET+0.5],
                    target_orientation= [0, 0, -1, 0],
                    update_world_needed= True)
    Robot_1.render_exec(renderInstance= True,
                            Show_Sphere= False)

    # Cut
    Robot_1.plan(tcp_name= "tool1",
                    target_pose= [SMALL_CUT_TABLE_SAW_POSE[0]+(ROBOT_1_SUCTION_LENGTH+ROBOT_1_SUCTION_CUP_R+HEADER_PICK_OFFSET_L-L),
                                  SMALL_CUT_TABLE_SAW_POSE[1]+(ROBOT_1_SUCTION_WIDTH/2),
                                  SMALL_CUT_TABLE_SAW_POSE[2]+W+ROBOT_1_SUCTION_CUP_TO_TOOL_OFFSET],
                    target_orientation= [0, 0, -1, 0],
                    update_world_needed= True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles", "World/obstacles/Small_Cutting_Table"],
                    direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
    Robot_1.render_exec(renderInstance= True,
                            Show_Sphere= False)

    # Simulated CUT
    Robot_1.eef_detach(tool_name="tool1", detaching_object_name= "Bear_Loading_Element_"+str(NUMBER_OF_HEADERS))

    dc=_dynamic_control.acquire_dynamic_control_interface()
    test._stage.GetPrimAtPath("/world/obstacles/Bear_Loading_Element_"+str(NUMBER_OF_HEADERS)).GetAttribute("physics:rigidBodyEnabled").Set(True)
    object = dc.get_rigid_body("/world/obstacles/Bear_Loading_Element_"+str(NUMBER_OF_HEADERS))
    object_pose=dc.get_rigid_body_pose(object)

    prims_utils.delete_prim("/world/obstacles/Bear_Loading_Element_"+str(NUMBER_OF_HEADERS))

    new_el = Cuboid(
        name= "Bear_Loading_Element_"+str(NUMBER_OF_HEADERS),
        pose= [object_pose.p[0]-((RAW_HEADER_DIMENSIONS[0]-L)/2),
               object_pose.p[1],
               object_pose.p[2],
               object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
        dims= [H, L, W],
        color= [0.4, 0.2, 0, 1]
    )
    Add_Rigid_Object_To_Scene(test, "Cuboid", new_el)

    # Attaching the Cut Element
    Robot_1.eef_attach(tool_name="tool1", attaching_object_name= "Bear_Loading_Element_"+str(NUMBER_OF_HEADERS))

    # Post Cut
    Robot_1.plan(tcp_name= "tool1",
                    target_pose= [SMALL_CUT_TABLE_SAW_POSE[0]+(ROBOT_1_SUCTION_LENGTH+ROBOT_1_SUCTION_CUP_R+HEADER_PICK_OFFSET_L-L),
                                  SMALL_CUT_TABLE_SAW_POSE[1]+(ROBOT_1_SUCTION_WIDTH/2),
                                  SMALL_CUT_TABLE_SAW_POSE[2]+W+ROBOT_1_SUCTION_CUP_TO_TOOL_OFFSET+0.5],
                    target_orientation= [0, 0, -1, 0],
                    update_world_needed= True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles", "World/obstacles/Small_Cutting_Table"],
                    direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
    Robot_1.render_exec(renderInstance= True,
                            Show_Sphere= False)

def Place_2x10_Header(X: float = None,
    Y: float = None,
    Z: float = None,
    L: float = None,
    W: float = None,
    H: float = None
):
    '''
    Brief Description:
        Places a cut 2x10 header element onto the smart conveyor using Robot 1’s suction tool.

    Parameters:
        X (float): X value of element's position from the top left corner of the building panel.
        Y (float): Y value of element's position from the top left corner of the building panel.
        Z (float): Z value of element's position from the top left corner of the building panel.
        L (float): Length of the header element.
        W (float): Width of the header element.
        H (float): Height (thickness) of the header element.

    Steps of Working:
        1. Move the smart conveyor to align the header based on Y and header length offsets.
        2. Compute the current conveyor location (Conv_Curr_Loc) for return.
        3. Plan the pre-place motion and check if placement is doable.
        4. If placement is doable:
            a. Execute pre-drop render.
            b. Execute drop motion and render.
            c. Detach the header from the suction tool.
            d. Enable gravity and attach the header to the conveyor.
            e. Enable the header’s colliders.
        5. If placement is not doable:
            a. Plan and render fallback placement at human-accessible location.
            b. Detach the header from the suction tool.
            c. Re-enable physics on the header and get its pose.
            d. Compute a new transform at height Z and apply it to the header.
            e. Disable gravity and attach the header to the conveyor.
        6. Return Robot 1 to its home position.
        7. Decrement the global header count and return Conv_Curr_Loc.
    '''

    # Moving Conveyor
    Smart_Conv.render_exec('Joint_1', -(Y - (OVERALL_PANEL_LENGTH/2)) - ((L/2)-ROBOT_1_SUCTION_LENGTH-ROBOT_1_SUCTION_CUP_R-HEADER_PICK_OFFSET_L) + (SMART_CONV_RANGE_OF_MOTION_J1/2))

    Conv_Curr_Loc: float = -(Y - (OVERALL_PANEL_LENGTH/2)) + (SMART_CONV_RANGE_OF_MOTION_J1/2)

    # Pre Place
    Place_Doable: bool = Robot_1.plan(tcp_name= "tool1",
                    target_pose= [2.3+(X-(OVERALL_PANEL_HEIGHT/2))+SMART_CONV_X_SHIFT+(ROBOT_1_SUCTION_WIDTH/2),
                                  0,
                                  SMART_CONV_REST_ELEVATION+W+ROBOT_1_SUCTION_CUP_TO_TOOL_OFFSET+0.3],
                    target_orientation= [0, ev, ev, 0],
                    update_world_needed= True)

    if Place_Doable == True:
        # Execute Pre Drop
        Robot_1.render_exec(renderInstance= True,
                                Show_Sphere= False)
        # Drop
        Robot_1.plan(tcp_name= "tool1",
                        target_pose= [2.3+(X-(OVERALL_PANEL_HEIGHT/2))+SMART_CONV_X_SHIFT+(ROBOT_1_SUCTION_WIDTH/2),
                                    0,
                                    SMART_CONV_REST_ELEVATION+W+ROBOT_1_SUCTION_CUP_TO_TOOL_OFFSET+0.15],
                        target_orientation= [0, ev, ev, 0],
                        update_world_needed= True)
        Robot_1.render_exec(renderInstance= True,
                                Show_Sphere= False)

        Robot_1.eef_detach(tool_name="tool1", detaching_object_name= "Bear_Loading_Element_"+str(NUMBER_OF_HEADERS))
        test._stage.GetPrimAtPath("/world/obstacles/Bear_Loading_Element_"+str(NUMBER_OF_HEADERS)).GetAttribute("physxRigidBody:disableGravity").Set(False)
        Smart_Conv.attach_object_to_conv(obj_name= "Bear_Loading_Element_"+str(NUMBER_OF_HEADERS))
        # Enabling Colliders
        test._stage.GetPrimAtPath("/world/obstacles/Bear_Loading_Element_"+str(NUMBER_OF_HEADERS)).GetAttribute("physics:collisionEnabled").Set(True)
    else:
        # Automatic Placement of the Bear Loading when Robot 1 Can't Place it
        # Pass To Human Location
        Robot_1.plan(tcp_name= "tool1",
                        target_pose= [2.3+(X-(OVERALL_PANEL_HEIGHT/2))+SMART_CONV_X_SHIFT-(ROBOT_1_SUCTION_CUP_TO_TOOL_OFFSET+(W/2)),
                                      0,
                                      SMART_CONV_REST_ELEVATION+H+ROBOT_1_SUCTION_WIDTH],
                        target_orientation= [0.5, 0.5, 0.5, 0.5],
                        update_world_needed= True)
        Robot_1.render_exec(renderInstance= True,
                                Show_Sphere= False)

        Robot_1.eef_detach(tool_name="tool1", detaching_object_name= "Bear_Loading_Element_"+str(NUMBER_OF_HEADERS))

        # [ev, 0, ev, 0]
        dc=_dynamic_control.acquire_dynamic_control_interface()
        test._stage.GetPrimAtPath("/world/obstacles/Bear_Loading_Element_"+str(NUMBER_OF_HEADERS)).GetAttribute("physics:rigidBodyEnabled").Set(True)
        object = dc.get_rigid_body("/world/obstacles/Bear_Loading_Element_"+str(NUMBER_OF_HEADERS))
        object_pose=dc.get_rigid_body_pose(object)

        # Create new position
        New_Loc = Gf.Vec3d(object_pose.p[0], object_pose.p[1], SMART_CONV_REST_ELEVATION + Z)
        # Create a valid quaternion and rotation
        quat = Gf.Quatd(1, 0, 0, 0)  # Ensure this follows (real, x, y, z)
        New_Or = Gf.Rotation(quat)
        # Convert quaternion to required format
        quat_as_tuple = (New_Or.GetQuat().GetReal(), *New_Or.GetQuat().GetImaginary())
        # Create a Transform object
        new_transform = _dynamic_control.Transform()
        new_transform.p.x = New_Loc[0]
        new_transform.p.y = New_Loc[1]
        new_transform.p.z = New_Loc[2]
        new_transform.r.w = quat_as_tuple[0]  # real part
        new_transform.r.x = quat_as_tuple[1]
        new_transform.r.y = quat_as_tuple[2]
        new_transform.r.z = quat_as_tuple[3]
        # Apply transformation
        dc.set_rigid_body_pose(object, new_transform)

        test._stage.GetPrimAtPath("/world/obstacles/Bear_Loading_Element_"+str(NUMBER_OF_HEADERS)).GetAttribute("physxRigidBody:disableGravity").Set(True)
        Smart_Conv.attach_object_to_conv(obj_name= "Bear_Loading_Element_"+str(NUMBER_OF_HEADERS), Enable_Gravity= False)

    Robot_1.move_to_home()

    # We Used The Top Bear Loading Element
    NUMBER_OF_HEADERS-=1

    return Conv_Curr_Loc

def Nail_2x10_Header(push_to_nail: float = PUSH_TO_NAIL_OFFSET_TANGENT*2,
                        el_pose: List[float] = [],
                        el_dims: List[float] = [],
                        conv_current_location: float = None):
    '''
    Brief Description:
        Performs nailing operations on a placed 2x10 header element at both left and right sides,
        coordinating conveyor positioning and Robot 1’s nailing motions with dynamic control.

    Parameters:
        push_to_nail (float): Distance to push into the nail.
        el_pose (List[float]): [x, y, z] pose of the element on the by assuming the origin is at the top left corner of the building panel.
        el_dims (List[float]): [length, width, height] of the element.
        conv_current_location (float): Conveyor joint position when header is placed at TCP zero position.

    Steps of Working:
        1. Compute the quaternion orientation for left-side nailing.
        2. For Left Nail:
           a. Check conveyor reachability; if not possible, log and skip.
           b. Move conveyor to left-nail position.
           c. Plan and render pre-nail approach.
           d. If doable, acquire dynamic control, perform forward nail push, retract backward,
              perform second nail prep and nail, then retract, and finally return Robot 1 home.
           e. If not doable, log reachability error.
        3. Compute quaternion orientation for right-side nailing.
        4. For Right Nail:
           a. Check conveyor reachability; if not possible, log and skip.
           b. Move conveyor to right-nail position.
           c. Plan and render pre-nail approach.
           d. If doable, acquire dynamic control, perform forward nail push, retract backward,
              perform second nail prep and nail, then retract, and finally return Robot 1 home.
           e. If not doable, log reachability error.
    '''

    quat = R.from_euler('xyz', [((np.pi/2)+np.radians(BR_NAILING_ANGLE))*(-1), 0, (np.pi/2)]).as_quat()
    quat[1] *= -1

    # Left Nail
    if(conv_current_location+(el_dims[0]/2)+NAILING_CONV_TARGET > SMART_CONV_RANGE_OF_MOTION_J1):
        print("Left Nail is Not Possible ! Due To Conveyor Reachability")
    else:
        Smart_Conv.render_exec('Joint_1', conv_current_location+(el_dims[0]/2)+NAILING_CONV_TARGET)
        # Robot 1 Move For Nail
        # Pre Nail
        Nail_Doable: bool = Robot_1.plan(tcp_name= "tool2",
                        target_pose= [2.3+(el_pose[0]-(OVERALL_PANEL_HEIGHT/2))+SMART_CONV_X_SHIFT+0.2*el_dims[2],
                                    NAILING_CONV_TARGET+((BR_NAILING_OFFSET*np.cos(np.radians(BR_NAILING_ANGLE))+(el_dims[1]/2))*(-1)),
                                    SMART_CONV_REST_ELEVATION+(el_pose[2])+((BR_NAILING_OFFSET)*np.sin(np.radians(BR_NAILING_ANGLE)))+STUD_HEIGHT*0.2],
                        target_orientation= [quat[3], quat[0], quat[1], quat[2]],
                        update_world_needed= True)
        if (Nail_Doable == True):
            Robot_1.render_exec(renderInstance= True,
                                    Show_Sphere= False)

            dc=_dynamic_control.acquire_dynamic_control_interface()

            # Nail 1
            object=dc.get_rigid_body("/"+Robot_1._ROS_JS_robot_indicator+"/tool2")
            object_pose=dc.get_rigid_body_pose(object)
            Robot_1.plan(tcp_name= "tool2",
                            target_pose= [object_pose.p[0],
                                        object_pose.p[1]+((BR_NAILING_OFFSET-((el_dims[1]/2)/np.cos(np.radians(BR_NAILING_ANGLE)))+push_to_nail))*np.cos(np.radians(BR_NAILING_ANGLE)),
                                        object_pose.p[2]-((BR_NAILING_OFFSET-((el_dims[1]/2)/np.cos(np.radians(BR_NAILING_ANGLE)))+push_to_nail))*np.sin(np.radians(BR_NAILING_ANGLE))],
                            target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                            update_world_needed= True,
                            removing_primitives=["Smart_Conveyor", "world/obstacles"],
                            direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
            Robot_1.render_exec(renderInstance= True,
                                    Show_Sphere= False)


            # Nail 1 Backward
            object=dc.get_rigid_body("/"+Robot_1._ROS_JS_robot_indicator+"/tool2")
            object_pose=dc.get_rigid_body_pose(object)
            Robot_1.plan(tcp_name= "tool2",
                            target_pose= [object_pose.p[0],
                                        object_pose.p[1]-((BR_NAILING_OFFSET-((el_dims[1]/2)/np.cos(np.radians(BR_NAILING_ANGLE)))+push_to_nail))*np.cos(np.radians(BR_NAILING_ANGLE)),
                                        object_pose.p[2]+((BR_NAILING_OFFSET-((el_dims[1]/2)/np.cos(np.radians(BR_NAILING_ANGLE)))+push_to_nail))*np.sin(np.radians(BR_NAILING_ANGLE))],
                            target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                            update_world_needed= True,
                            removing_primitives=["Smart_Conveyor", "world/obstacles"],
                            direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
            Robot_1.render_exec(renderInstance= True,
                                    Show_Sphere= False)
            
            # Nail 2 Prep
            object=dc.get_rigid_body("/"+Robot_1._ROS_JS_robot_indicator+"/tool2")
            object_pose=dc.get_rigid_body_pose(object)
            Robot_1.plan(tcp_name= "tool2",
                            target_pose= [object_pose.p[0]-0.4*el_dims[2],
                                        object_pose.p[1],
                                        object_pose.p[2]],
                            target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                            update_world_needed= True,
                            removing_primitives=["Smart_Conveyor", "world/obstacles"],
                            direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=1))
            Robot_1.render_exec(renderInstance= True,
                                    Show_Sphere= False)
            
            # Nail 2
            object=dc.get_rigid_body("/"+Robot_1._ROS_JS_robot_indicator+"/tool2")
            object_pose=dc.get_rigid_body_pose(object)
            Robot_1.plan(tcp_name= "tool2",
                            target_pose= [object_pose.p[0],
                                        object_pose.p[1]+((BR_NAILING_OFFSET-((el_dims[1]/2)/np.cos(np.radians(BR_NAILING_ANGLE)))+push_to_nail))*np.cos(np.radians(BR_NAILING_ANGLE)),
                                        object_pose.p[2]-((BR_NAILING_OFFSET-((el_dims[1]/2)/np.cos(np.radians(BR_NAILING_ANGLE)))+push_to_nail))*np.sin(np.radians(BR_NAILING_ANGLE))],
                            target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                            update_world_needed= True,
                            removing_primitives=["Smart_Conveyor", "world/obstacles"],
                            direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
            Robot_1.render_exec(renderInstance= True,
                                    Show_Sphere= False)

            # Nail 2 Backward
            object=dc.get_rigid_body("/"+Robot_1._ROS_JS_robot_indicator+"/tool2")
            object_pose=dc.get_rigid_body_pose(object)
            Robot_1.plan(tcp_name= "tool2",
                            target_pose= [object_pose.p[0],
                                        object_pose.p[1]-((SILL_NAILING_OFFSET-((el_dims[1]/2)/np.cos(np.radians(SILL_NAILING_ANGLE)))+push_to_nail))*np.cos(np.radians(SILL_NAILING_ANGLE)),
                                        object_pose.p[2]+((SILL_NAILING_OFFSET-((el_dims[1]/2)/np.cos(np.radians(SILL_NAILING_ANGLE)))+push_to_nail))*np.sin(np.radians(SILL_NAILING_ANGLE))],
                            target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                            update_world_needed= True,
                            removing_primitives=["Smart_Conveyor", "world/obstacles"],
                            direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
            Robot_1.render_exec(renderInstance= True,
                                    Show_Sphere= False)

            Robot_1.move_to_home()
        else:
            print("Left Nail is Not Possible ! Due To Robot NailGun Reachability")

    #NAILING OTHER SIDE
    quat = R.from_euler('xyz', [((np.pi/2)+np.radians(BR_NAILING_ANGLE)), 0, (np.pi/2)*(-1)]).as_quat()
    quat[1] *= -1

    # Right Nail
    if(conv_current_location-(el_dims[0]/2)-NAILING_CONV_TARGET < 0):
        print("Right Nail is Not Possible ! Due To Conveyor Reachability")
    else:
        Smart_Conv.render_exec('Joint_1', conv_current_location-(el_dims[0]/2)-NAILING_CONV_TARGET)

        # Pre Nail
        Nail_Doable: bool = Robot_1.plan(tcp_name= "tool2",
                        target_pose= [2.3+(el_pose[0]-(OVERALL_PANEL_HEIGHT/2))+SMART_CONV_X_SHIFT+0.2*el_dims[2],
                                    -NAILING_CONV_TARGET+((BR_NAILING_OFFSET*np.cos(np.radians(BR_NAILING_ANGLE))+(el_dims[1]/2))),
                                    SMART_CONV_REST_ELEVATION+(el_pose[2])+((BR_NAILING_OFFSET)*np.sin(np.radians(BR_NAILING_ANGLE)))+STUD_HEIGHT*0.2],
                        target_orientation= [quat[3], quat[0], quat[1], quat[2]],
                        update_world_needed= True)

        if(Nail_Doable == True):

            Robot_1.render_exec(renderInstance= True,
                                    Show_Sphere= False)

            dc=_dynamic_control.acquire_dynamic_control_interface()

            # Nail 1
            object=dc.get_rigid_body("/"+Robot_1._ROS_JS_robot_indicator+"/tool2")
            object_pose=dc.get_rigid_body_pose(object)
            Robot_1.plan(tcp_name= "tool2",
                            target_pose= [object_pose.p[0],
                                        object_pose.p[1]+((-1)*(BR_NAILING_OFFSET-((el_dims[1]/2)/np.cos(np.radians(BR_NAILING_ANGLE)))+push_to_nail))*np.cos(np.radians(BR_NAILING_ANGLE)),
                                        object_pose.p[2]-((BR_NAILING_OFFSET-((el_dims[1]/2)/np.cos(np.radians(BR_NAILING_ANGLE)))+push_to_nail))*np.sin(np.radians(BR_NAILING_ANGLE))],
                            target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                            update_world_needed= True,
                            removing_primitives=["Smart_Conveyor", "world/obstacles"],
                            direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
            Robot_1.render_exec(renderInstance= True,
                                    Show_Sphere= False)

            # Nail 1 Backward
            object=dc.get_rigid_body("/"+Robot_1._ROS_JS_robot_indicator+"/tool2")
            object_pose=dc.get_rigid_body_pose(object)
            Robot_1.plan(tcp_name= "tool2",
                            target_pose= [object_pose.p[0],
                                        object_pose.p[1]-((-1)*(BR_NAILING_OFFSET-((el_dims[1]/2)/np.cos(np.radians(BR_NAILING_ANGLE)))+push_to_nail))*np.cos(np.radians(BR_NAILING_ANGLE)),
                                        object_pose.p[2]+((BR_NAILING_OFFSET-((el_dims[1]/2)/np.cos(np.radians(BR_NAILING_ANGLE)))+push_to_nail))*np.sin(np.radians(BR_NAILING_ANGLE))],
                            target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                            update_world_needed= True,
                            removing_primitives=["Smart_Conveyor", "world/obstacles"],
                            direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
            Robot_1.render_exec(renderInstance= True,
                                    Show_Sphere= False)

            # Nail 2 Prep
            object=dc.get_rigid_body("/"+Robot_1._ROS_JS_robot_indicator+"/tool2")
            object_pose=dc.get_rigid_body_pose(object)
            Robot_1.plan(tcp_name= "tool2",
                            target_pose= [object_pose.p[0]-0.4*el_dims[2],
                                        object_pose.p[1],
                                        object_pose.p[2]],
                            target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                            update_world_needed= True,
                            removing_primitives=["Smart_Conveyor", "world/obstacles"],
                            direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=1))
            Robot_1.render_exec(renderInstance= True,
                                    Show_Sphere= False)
    
            # Nail 2
            object=dc.get_rigid_body("/"+Robot_1._ROS_JS_robot_indicator+"/tool2")
            object_pose=dc.get_rigid_body_pose(object)
            Robot_1.plan(tcp_name= "tool2",
                            target_pose= [object_pose.p[0],
                                        object_pose.p[1]+((-1)*(BR_NAILING_OFFSET-((el_dims[1]/2)/np.cos(np.radians(BR_NAILING_ANGLE)))+push_to_nail))*np.cos(np.radians(BR_NAILING_ANGLE)),
                                        object_pose.p[2]-((BR_NAILING_OFFSET-((el_dims[1]/2)/np.cos(np.radians(BR_NAILING_ANGLE)))+push_to_nail))*np.sin(np.radians(BR_NAILING_ANGLE))],
                            target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                            update_world_needed= True,
                            removing_primitives=["Smart_Conveyor", "world/obstacles"],
                            direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
            Robot_1.render_exec(renderInstance= True,
                                    Show_Sphere= False)

            # Nail 2 Backward
            object=dc.get_rigid_body("/"+Robot_1._ROS_JS_robot_indicator+"/tool2")
            object_pose=dc.get_rigid_body_pose(object)
            Robot_1.plan(tcp_name= "tool2",
                            target_pose= [object_pose.p[0],
                                        object_pose.p[1]-((-1)*(BR_NAILING_OFFSET-((el_dims[1]/2)/np.cos(np.radians(BR_NAILING_ANGLE)))+push_to_nail))*np.cos(np.radians(BR_NAILING_ANGLE)),
                                        object_pose.p[2]+((BR_NAILING_OFFSET-((el_dims[1]/2)/np.cos(np.radians(BR_NAILING_ANGLE)))+push_to_nail))*np.sin(np.radians(BR_NAILING_ANGLE))],
                            target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                            update_world_needed= True,
                            removing_primitives=["Smart_Conveyor", "world/obstacles"],
                            direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
            Robot_1.render_exec(renderInstance= True,
                                    Show_Sphere= False)

            Robot_1.move_to_home()
        else:
            print("Right Nail is Not Possible ! Due To Robot NailGun Reachability")

def Complementary_Nail_Operation(push_to_nail: float = None,
                         H: float = None):
    '''
    Brief Description:
        Performs complementary nailing operations for each recorded nail pose on the smart conveyor,
        coordinating conveyor movements and Robot 2’s nailing motions with dynamic control.

    Parameters:
        push_to_nail (float): Distance to push forward for the nailing action.
        H (float): Height of the element being nailed, used to compute vertical offsets.

    Steps of Working:
        1. Iterate through each target pose stored in Smart_Conv._nail_poses.
        2. Check if the conveyor position is reachable; if not, adjust the target to a replacing position.
        3. Move the conveyor to the computed nailing joint position.
        4. Plan and execute Robot 2’s initial nailing approach at a fixed pose relative to the conveyor.
        5. Acquire dynamic control interface and get the current pose of Robot 2’s tool.
        6. Release path planning restrictions and perform the restricted bot nail forward push.
        7. Render the nail action and then retract by pushing back to the original pose.
        8. Execute an “other nail” motion upward by 0.4 * H.
        9. Perform two additional restricted top nail pushes forward and back at the same offset.
    '''

    for target in Smart_Conv._nail_poses:
        # Check if the target is reachable or not (for the conveyor !)
        if(target[0]+(NAILING_CONV_TARGET*target[1]) > SMART_CONV_RANGE_OF_MOTION_J1):
            Replacing_Target: Tuple[float, float] = (target[0]-2*NAILING_CONV_TARGET, -target[1])
            target = Replacing_Target

        Smart_Conv.render_exec('Joint_1', target[0]+(NAILING_CONV_TARGET*target[1]))
        # Robot 1 Nail
        Robot_2.plan(tcp_name= "tool1",
                        target_pose= [3.69, NAILING_CONV_TARGET*target[1]*2, SMART_CONV_REST_ELEVATION+H-(0.7*H)],
                        target_orientation= [0.5, 0.5, 0.5, 0.5],
                        update_world_needed= True)
        Robot_2.render_exec(renderInstance= True,
                                Show_Sphere= False)

        dc=_dynamic_control.acquire_dynamic_control_interface()
        object=dc.get_rigid_body("/"+Robot_2._ROS_JS_robot_indicator+"/tool1")

        # Restricted Bot Nail
        object_pose=dc.get_rigid_body_pose(object)
        Robot_2.release_path_plan_restriction()
        Robot_2.plan(tcp_name= "tool1",
                        target_pose= [object_pose.p[0]-push_to_nail, object_pose.p[1], object_pose.p[2]],
                        target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                        update_world_needed= True,
                        removing_primitives=["Smart_Conveyor", "world/obstacles"],
                        direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
        Robot_2.render_exec(renderInstance= True,
                                Show_Sphere= False)
        # Nail Done
        object_pose=dc.get_rigid_body_pose(object)
        Robot_2.release_path_plan_restriction()
        Robot_2.plan(tcp_name= "tool1",
                        target_pose= [object_pose.p[0]+push_to_nail, object_pose.p[1], object_pose.p[2]],
                        target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                        update_world_needed= True,
                        removing_primitives=["Smart_Conveyor", "world/obstacles"],
                        direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
        Robot_2.render_exec(renderInstance= True,
                                Show_Sphere= False)
        
        # Other Nail
        object_pose=dc.get_rigid_body_pose(object)
        Robot_2.release_path_plan_restriction()
        Robot_2.plan(tcp_name= "tool1",
                        target_pose= [object_pose.p[0], object_pose.p[1], object_pose.p[2]+(0.4*H)],
                        target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                        update_world_needed= True,
                        removing_primitives=["Smart_Conveyor", "world/obstacles"],
                        direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=1))
        Robot_2.render_exec(renderInstance= True,
                                Show_Sphere= False)

        # Restricted Top Nail
        object_pose=dc.get_rigid_body_pose(object)
        Robot_2.release_path_plan_restriction()
        Robot_2.plan(tcp_name= "tool1",
                        target_pose= [object_pose.p[0]-push_to_nail, object_pose.p[1], object_pose.p[2]],
                        target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                        update_world_needed= True,
                        removing_primitives=["Smart_Conveyor", "world/obstacles"],
                        direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
        Robot_2.render_exec(renderInstance= True,
                                Show_Sphere= False)
        # Restricted Top Nail
        object_pose=dc.get_rigid_body_pose(object)
        Robot_2.release_path_plan_restriction()
        Robot_2.plan(tcp_name= "tool1",
                        target_pose= [object_pose.p[0]+push_to_nail, object_pose.p[1], object_pose.p[2]],
                        target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                        update_world_needed= True,
                        removing_primitives=["Smart_Conveyor", "world/obstacles"],
                        direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
        Robot_2.render_exec(renderInstance= True,
                                Show_Sphere= False)

def Pick_OSB_Plate(el_name: str = None,
        L: float = None,
        W: float = None,
        H: float = None):
    '''
    Brief Description:
        Picks an OSB sheathing plate from the sheathing table using Robot 1’s suction tool

    Parameters:
        el_name (str): Name to assign to the picked OSB plate element.
        L (float): Length of the OSB plate.
        W (float): Width of the OSB plate.
        H (float): Height (thickness) of the OSB plate.

    Steps of Working:
        1. Plan and execute the pre-pick approach pose above the sheathing table.
        2. Plan and execute the pick motion, removing obstacles and applying grasp-approach cost.
        3. Call Create_Wooden_Element_For_Sheathing_Table to instantiate the plate in the scene.
        4. Attach the newly created plate to Robot 1’s suction tool.
        5. Plan and execute the post-pick retraction pose, removing table obstacles.
    '''

    # Pre Pick
    Robot_1.plan(tcp_name= "tool1",
                    target_pose= [SHEATHING_PLATE_TABLE_BOTTOM_CENTER[0]+(ROBOT_1_SUCTION_WIDTH/2),
                                  SHEATHING_PLATE_TABLE_BOTTOM_CENTER[1]-(ROBOT_1_SUCTION_LENGTH)-(ROBOT_1_SUCTION_CUP_R+OFFSET_FROM_SHEATHING_PLATE_TABLE_BOT),
                                  SHEATHING_PLATE_TABLE_BOTTOM_CENTER[2]+H+0.1],
                    target_orientation= [0, ev, ev, 0],
                    update_world_needed= True)
    Robot_1.render_exec(renderInstance= True,
                            Show_Sphere= False)

    # Pick
    Robot_1.plan(tcp_name= "tool1",
                    target_pose= [SHEATHING_PLATE_TABLE_BOTTOM_CENTER[0]+(ROBOT_1_SUCTION_WIDTH/2),
                                  SHEATHING_PLATE_TABLE_BOTTOM_CENTER[1]-(ROBOT_1_SUCTION_LENGTH)-(ROBOT_1_SUCTION_CUP_R+OFFSET_FROM_SHEATHING_PLATE_TABLE_BOT),
                                  SHEATHING_PLATE_TABLE_BOTTOM_CENTER[2]+H+ROBOT_1_SUCTION_CUP_TO_TOOL_OFFSET],
                    target_orientation= [0, ev, ev, 0],
                    update_world_needed= True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles"],
                    direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
    Robot_1.render_exec(renderInstance= True,
                            Show_Sphere= False)
    
    Create_Wooden_Element_For_Sheathing_Table(el_name= el_name, L= L, W= W, H= H)

    # Attach
    Robot_1.eef_attach(tool_name= "tool1", attaching_object_name= el_name)

    # Post Pick
    Robot_1.plan(tcp_name= "tool1",
                    target_pose= [SHEATHING_PLATE_TABLE_BOTTOM_CENTER[0]+(ROBOT_1_SUCTION_WIDTH/2),
                                  SHEATHING_PLATE_TABLE_BOTTOM_CENTER[1]-(ROBOT_1_SUCTION_LENGTH)-(ROBOT_1_SUCTION_CUP_R+OFFSET_FROM_SHEATHING_PLATE_TABLE_BOT),
                                  SHEATHING_PLATE_TABLE_BOTTOM_CENTER[2]+H+0.1],
                    target_orientation= [0, ev, ev, 0],
                    update_world_needed= True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles", "World/obstacles/Sheathing_Table"],
                    direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
    Robot_1.render_exec(renderInstance= True,
                            Show_Sphere= False)

def Place_OSB_Plate(el_name: str = None,
        X: float = None,
        Y: float = None,
        Z: float = None) -> float:
    '''
    Brief Description:
        Places an OSB sheathing plate onto the smart conveyor using Robot 1’s suction tool,
        while Robot 2 makes space for the placement, and returns the final conveyor joint location.

    Parameters:
        el_name (str): Name of the sheathing plate element.
        X (float): X-coordinate offset for placement relative to the panel.
        Y (float): Y-coordinate offset for conveyor alignment.
        Z (float): Z-coordinate (height) offset for the plate placement.

    Returns:
        Sht_plate_on_conv_loc (float): The Smart_Conv Joint_1 position after placement.

    Steps of Working:
        1. Move Robot 2 to a home position that provides clearance for Robot 1.
        2. Move the smart conveyor to align the plate based on Y and suction width.
        3. Plan and execute Robot 1’s pre-place approach pose and render.
        4. Plan and execute Robot 1’s main placement pose (removing obstacles, applying orientation restriction) and render.
        5. Detach the plate from Robot 1’s suction tool and attach it to the conveyor (gravity disabled).
        6. Plan and execute Robot 1’s post-place retract pose (removing obstacles, applying orientation restriction) and render.
        7. Return both Robot 1 and Robot 2 to their home positions.
        8. Compute and return the final conveyor joint location for the sheathing plate.
    '''

    # Letting Space For the First Robot To Place The Sheathing Plate
    Robot_2.move_to_home(Customized_JS=[np.radians(90), -0.5, 0.5, 0, 0, 0])

    # Moving the Conveyor
    Smart_Conv.render_exec(
        'Joint_1',
        -(Y - (OVERALL_PANEL_LENGTH/2))
        + (SMART_CONV_RANGE_OF_MOTION_J1/2)
        - (ROBOT_1_SUCTION_WIDTH/2)
    )

    # Pre Place
    Robot_1.plan(
        tcp_name="tool1",
        target_pose=[
            2.3 - (X)
            + OFFSET_FROM_SHEATHING_PLATE_TABLE_BOT
            + ROBOT_1_SUCTION_LENGTH
            + ROBOT_1_SUCTION_CUP_R
            + SMART_CONV_X_SHIFT
            - 0.1,
            0,
            SMART_CONV_REST_ELEVATION + Z + 0.1,
        ],
        target_orientation=[0, 0, -1, 0],
        update_world_needed=True,
    )
    Robot_1.render_exec(renderInstance=True, Show_Sphere=False)

    # Place
    Robot_1.plan(
        tcp_name="tool1",
        target_pose=[
            2.3 - (X)
            + OFFSET_FROM_SHEATHING_PLATE_TABLE_BOT
            + ROBOT_1_SUCTION_LENGTH
            + ROBOT_1_SUCTION_CUP_R
            + SMART_CONV_X_SHIFT,
            0,
            SMART_CONV_REST_ELEVATION
            + Z
            + ROBOT_1_SUCTION_CUP_TO_TOOL_OFFSET,
        ],
        target_orientation=[0, 0, -1, 0],
        update_world_needed=True,
        removing_primitives=["Smart_Conveyor", "world/obstacles"],
        orientational_restriction=torch.tensor([1, 1, 1], dtype=torch.float32),
    )
    Robot_1.render_exec(renderInstance=True, Show_Sphere=False)

    # Detach
    Robot_1.eef_detach(tool_name="tool1", detaching_object_name=el_name)
    Smart_Conv.attach_object_to_conv(obj_name=el_name, Enable_Gravity=False)

    # Post Place
    Robot_1.plan(
        tcp_name="tool1",
        target_pose=[
            2.3 - (X)
            + OFFSET_FROM_SHEATHING_PLATE_TABLE_BOT
            + ROBOT_1_SUCTION_LENGTH
            + ROBOT_1_SUCTION_CUP_R
            + SMART_CONV_X_SHIFT
            - 0.1,
            0,
            SMART_CONV_REST_ELEVATION + Z + 0.1,
        ],
        target_orientation=[0, 0, -1, 0],
        update_world_needed=True,
        removing_primitives=["Smart_Conveyor", "world/obstacles"],
        orientational_restriction=torch.tensor([1, 1, 1], dtype=torch.float32),
    )
    Robot_1.render_exec(renderInstance=True, Show_Sphere=False)

    # Move To Home
    Robot_1.move_to_home()
    Robot_2.move_to_home()

    # Compute and return conveyor location
    Sht_plate_on_conv_loc: float = (
        -(Y - (OVERALL_PANEL_LENGTH / 2)) + (SMART_CONV_RANGE_OF_MOTION_J1 / 2)
    )
    return Sht_plate_on_conv_loc

def Nail_OSB_Plate(push_to_nail: float = PUSH_TO_NAIL_OFFSET_TANGENT,
                     nail_num: int = 6,
                     el_dims: list[float] = [],
                     W: float = None,
                     Sht_plate_on_conv_loc: float = None):
    '''
    Brief Description:
        Performs vertical nailing operations on an OSB sheathing plate at specified positions
        along its width using both Robot 1 and Robot 2 in coordination with the smart conveyor.

    Parameters:
        push_to_nail (float): Offset distance to push forward during each nailing action.
        nail_num (int): Total number of nails to place vertically (half by Robot 1, half by Robot 2).
        el_dims (list[float]): Dimensions of the plate [length, width, thickness].
        W (float): Width of the OSB plate, used to filter valid nail poses.
        Sht_plate_on_conv_loc (float): Current conveyor Joint_1 position upon placing the OSB plate.

    Steps of Working:
        1. Loop through each stored vertical nail pose in Smart_Conv._vertical_nail_poses.
        2. For poses within the plate's width around Sht_plate_on_conv_loc:
           a. Move the conveyor to the nail position.
           b. Compute starting and ending X positions and the spacing L between nails.
           c. Acquire the dynamic control interface for pose queries.
           d. Robot 1 performs vertical nailing for half the nails:
              i. Plan and render pre-nail approach for each nail at increasing X positions.
              ii. Execute nail push, render, then retract to original pose, render.
           e. Return Robot 1 to its home.
           f. Robot 2 performs vertical nailing for the remaining nails in mirror fashion:
              i. Plan and render pre-nail approach at decreasing X positions.
              ii. Execute nail push, render, then retract to original pose, render.
    '''

    for nail in Smart_Conv._vertical_nail_poses:
        # It means that we should vertically nail the sheathing plate to the king at these locations
        if(nail <= Sht_plate_on_conv_loc + (W/2) and nail >= Sht_plate_on_conv_loc - (W/2)):
            # Move Nail Target To Y=0
            Smart_Conv.render_exec('Joint_1', nail)

            Starting_X: float = 2.3 - (OVERALL_PANEL_HEIGHT/2)+(STUD_THICKNESS/2) + SMART_CONV_X_SHIFT
            Ending_X: float = 2.3 + (OVERALL_PANEL_HEIGHT/2)-(STUD_THICKNESS/2) + SMART_CONV_X_SHIFT
            # Might Be Less Than 1, so 2 decimals
            L: float = np.round((Ending_X - Starting_X) / (nail_num-1), 2)
            it: int = 0
            dc=_dynamic_control.acquire_dynamic_control_interface()

            # Robot 1 vertical nailing
            while it < (nail_num/2):
                # Pre Nail Rob1
                Robot_1.plan(tcp_name= "tool3",
                                target_pose= [2.3 - (OVERALL_PANEL_HEIGHT/2)+(STUD_THICKNESS/2) + SMART_CONV_X_SHIFT + (it*L),
                                            0,
                                            SMART_CONV_REST_ELEVATION+STUD_HEIGHT+el_dims[2]+0.05],
                                target_orientation= [1, 0, 0, 0],
                                update_world_needed= True)
                Robot_1.render_exec(renderInstance= True,
                                        Show_Sphere= False)

                # Nail Rob1
                object=dc.get_rigid_body("/"+Robot_1._ROS_JS_robot_indicator+"/tool3")
                object_pose=dc.get_rigid_body_pose(object)
                Robot_1.plan(tcp_name= "tool3",
                                target_pose= [object_pose.p[0],
                                            object_pose.p[1],
                                            object_pose.p[2]-push_to_nail-0.05],
                                target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                                update_world_needed= True,
                                removing_primitives=["Smart_Conveyor", "world/obstacles"],
                                direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
                Robot_1.render_exec(renderInstance= True,
                                        Show_Sphere= False)

                # Post Nail Rob1
                object=dc.get_rigid_body("/"+Robot_1._ROS_JS_robot_indicator+"/tool3")
                object_pose=dc.get_rigid_body_pose(object)
                Robot_1.plan(tcp_name= "tool3",
                                target_pose= [object_pose.p[0],
                                            object_pose.p[1],
                                            object_pose.p[2]+push_to_nail+0.05],
                                target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                                update_world_needed= True,
                                removing_primitives=["Smart_Conveyor", "world/obstacles"],
                                direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
                Robot_1.render_exec(renderInstance= True,
                                        Show_Sphere= False)
                it+=1

            # Reset iterator and send Robot 1 home
            it: int = 0
            Robot_1.move_to_home()

            # Robot 2 vertical nailing
            while it < (nail_num/2):
                # Pre Nail Rob2
                Robot_2.plan(tcp_name= "tool2",
                                target_pose= [2.3 + (OVERALL_PANEL_HEIGHT/2)-(STUD_THICKNESS/2) + SMART_CONV_X_SHIFT - (it*L),
                                            0,
                                            SMART_CONV_REST_ELEVATION+STUD_HEIGHT+el_dims[2]+0.05],
                                target_orientation= [1, 0, 0, 0],
                                update_world_needed= True)
                Robot_2.render_exec(renderInstance= True,
                                        Show_Sphere= False)

                # Nail Rob2
                object=dc.get_rigid_body("/"+Robot_2._ROS_JS_robot_indicator+"/tool2")
                object_pose=dc.get_rigid_body_pose(object)
                Robot_2.plan(tcp_name= "tool2",
                                target_pose= [object_pose.p[0],
                                            object_pose.p[1],
                                            object_pose.p[2]-push_to_nail-0.05],
                                target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                                update_world_needed= True,
                                removing_primitives=["Smart_Conveyor", "world/obstacles"],
                                direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
                Robot_2.render_exec(renderInstance= True,
                                        Show_Sphere= False)

                # Post Nail Rob2
                object=dc.get_rigid_body("/"+Robot_2._ROS_JS_robot_indicator+"/tool2")
                object_pose=dc.get_rigid_body_pose(object)
                Robot_2.plan(tcp_name= "tool2",
                                target_pose= [object_pose.p[0],
                                            object_pose.p[1],
                                            object_pose.p[2]+push_to_nail+0.05],
                                target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                                update_world_needed= True,
                                removing_primitives=["Smart_Conveyor", "world/obstacles"],
                                direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
                Robot_2.render_exec(renderInstance= True,
                                        Show_Sphere= False)
                it+=1

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

        Human_Representation_Box = Cuboid(
            name= "Human_Box",
            pose= [4.2, 1.5, 1, 1, 0, 0, 0],
            dims= [0.5, 1, 2],
            color= [1, 1, 1, 0]
        )
        Add_Rigid_Object_To_Scene(test, "Cuboid", Human_Representation_Box, True, True)

        ################################
        ################################
        ### Code Writing STARTS Here ###
        ################################
        ################################

        Robot_1.free_TCP_movement("tool0")

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