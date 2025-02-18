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
# prim = utils.createJoint()

#Dynamic Control API
#Tutorial: https://forums.developer.nvidia.com/t/get-position-of-primitive/146702
# Use Case:
# dc=_dynamic_control.acquire_dynamic_control_interface()

# object=dc.get_rigid_body("/Stage/bin/SmallKLT")
# object_pose=dc.get_rigid_body_pose(object)

# print("position:", object_pose.p)
# print("rotation:", object_pose.r)
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


####################
#### PARAMETERS ####
####################

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

ROBOT_2_GRIPPER_LENGTH: float = 0.590042
# Robotic Movement Accelaration
MOTION_ACCELERAION_VALUE: float = 0.6

SMART_CONV_RANGE_OF_MOTION_J1: float = 4.55
SMART_CONV_RANGE_OF_MOTION_J2: float = 0.5
SMART_CONV_REST_ELEVATION: float = 0.89546
# This shift is required to satisfy the symmetry of the smart conveyor platform
SMART_CONV_X_SHIFT: float = 0.09179

# Smart Material Table ZEORO
SMART_MAT_TABLE: list[float] = [6.444, 4.611, 0.803]
STUD_TO_SAW_OFFSET: float = 0.12093

# Offset Required to Pick Woods From the Table
PICK_OFFSET_FROM_L_CORNER: float = 0.05
PICK_OFFSET_FROM_L_CORNER_AFTER_PASS: float = 0.2
PICK_OFFSET_FROM_W_CORNER: float = 0.0061

JACK_PLACEMENT_SIDE_DRAG: float = 0.1
# This represents the angle in which Robot 1 will nail the Jacks to the Kings ! (20 To 45 is a good range)
JACK_SIDE_NAILING_ANGLE: float = 30
JACK_NAILING_OFFSET: float = 0.1

# Smart Material Table's Maximum Length Capability
SMART_MAT_TABLE_MAX_LENGTH: float = 3.6576


#### SLOPED MATERIAL TABLE
SLOPED_MAT_TABLE: list[float] = [4.009, -2.417, 1.045]
SLOPED_MAT_TABLE_ANGLE: float = 30

SLOPED_TABLE_PICK_OFFSET_FROM_L_CORNER: float = 0.17
SLOPED_TABLE_PICK_OFFSET_FROM_W_CORNER: float = 0.0061

### Small Cut Table
# Upon Reloating the Table, The Offset Should be Adjusted Accordingly
SMALL_CUT_TABLE_SAW_POSE: list[float] = [1, 1, 0.5]

NUMBER_OF_HEADERS: int = 2
# 2in x 10in 
# 1 Meters Length Timers Stacked !
RAW_HEADER_DIMENSIONS: list[float] = [1, 0.0508, 0.254]

# Position: ['3.999', '-2.417', '1.045'], Orientation: ['0.966', '-0.259', '0.000', '0.000']

# 0.5m Is a Suitable Value. Changing it might ruine the Whole Assembly Process
NAILING_CONV_TARGET: float = 0.5



####################
#### END PARAMS ####
####################

# Variables That Should be Changed For Each Design (For Now)
OVERALL_PANEL_LENGTH: float = SMART_MAT_TABLE_MAX_LENGTH
OVERALL_PANEL_HEIGHT: float = 2.5184

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
            pose=[-0.5 , -1.2, 0, 0, 0, -ev, -ev],
            file_path= cur_dir + "cutting_table/Small_Cut_Table.stl",
            color= [0.2, 0.2, 0.2, 1],
            scale=[0.001, 0.001, 0.001]
        )
        SMALL_CUT_TABLE_SAW_POSE[0] = Small_Cutting_Table.pose[0]+0.15
        SMALL_CUT_TABLE_SAW_POSE[1] = Small_Cutting_Table.pose[1]
        SMALL_CUT_TABLE_SAW_POSE[2] = Small_Cutting_Table.pose[2] 

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
                                                                                                                                                                                                                                                                                            
        world_model = WorldConfig(
            mesh=[IDC_Lab, Smart_Mat_Table, Sloped_Table, Small_Cutting_Table],
            cuboid=[Cube],
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

        # Nailing Positions For Sheathing (Vertical Nailing)
        self._nail_poses: List[Tuple[float, float]] = []

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
            if np.round(cube_position[2],0) == 500:
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
                              obj_name: str = None):
        
        # Disabling Gravity Right After Detaching
        Obj_Prim = self._temp_world_manager._stage.GetPrimAtPath("/world/obstacles/" + obj_name)
        Conv_Prim = self._temp_world_manager._stage.GetPrimAtPath("/Smart_Conveyor/Link_2")

        Conv_Collision_Prim = self._temp_world_manager._stage.GetPrimAtPath("/Smart_Conveyor/Link_2/collisions")   

        Conv_Collision_Prim.GetAttribute("physics:collisionEnabled").Set(True)
        self._temp_world_manager._my_world.step(render= True)

        Obj_Prim.GetAttribute("physics:rigidBodyEnabled").Set(True)
        Obj_Prim.GetAttribute("physics:collisionEnabled").Set(True)
        Obj_Prim.GetAttribute("physxRigidBody:disableGravity").Set(False)
        self._temp_world_manager._my_world.step(render= True)    

        Time = time.time()
        while time.time() - Time <= 0.1:
            self._temp_world_manager._my_world.step(render= True)
        
        # Fix Jointing Stud To Conveyor
        prim = utils.createJoint(self._temp_world_manager._stage, "Fixed", Obj_Prim, Conv_Prim)
        self._temp_world_manager._my_world.step(render= True) 

        #Damper ! (20 is Good !!!)
        Obj_Prim.GetAttribute("physxRigidBody:linearDamping").Set(20)
        Obj_Prim.GetAttribute("physxRigidBody:angularDamping").Set(20)


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
                        direct_pose_cost: PoseCostMetric = None):

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

        # dc=_dynamic_control.acquire_dynamic_control_interface()
        # object=dc.get_rigid_body("/"+self._ROS_JS_robot_indicator+"/tool0")
        # object_pose=dc.get_rigid_body_pose(object)
        # print(object_pose.p)
        # print(object_pose.r)
        # print("__________________________________________________")

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
                    Show_Sphere: Optional[bool] = True):

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

                # set desired joint angles obtained from IK:
                self._articulation_controller.apply_action(art_action)

                # Publishing To ROS
                self.ros_js_publisher()

                cmd_idx += 1
                if renderInstance:
                    for _ in range(2):
                        self._temp_world_manager._my_world.step(render=False)
        
        # Cleaning out !
        self._computed_path_result = None
        self._computed_cmd_plan = None
        self._computed_idx_list = []

    def move_to_home(self,
                     if_show_spheres: bool = False):

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
            Home_Loc = torch.zeros(1,6).cuda()

            # Sleep Point For Robots To Reduce Collision !
            Home_Loc[0, 1] = -0.5
            Home_Loc[0, 2] = 0.5
            Home_Loc[0, 4] = 0

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
            result = self._motion_gen.plan_single_js(cu_js.unsqueeze(0), home_state, self._plan_config)
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
                   gen_sphere_radius: float = 0.01,
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
                                             C_Pose= [0.55, 0.435, -0.175])
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
               pose=[2.3, -3.25, 0],
               w_dir=INSTALLATION_DIRECTORY+"/Isaac_sim_ws/smart_conveyor",
               c_conf_name="Smart_Conveyor.yaml")

def euler_to_quat(roll, pitch, yaw):
    quat = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
    return quat

# Default Assumption (Cuboid), but it can be changed to Capsule and Mesh
def Add_Rigid_Object_To_Scene(World_Manager: WorldManager,
                              ObjectType: str = "Cuboid",
                              obj: Any = Cuboid,
                              rigid_body_disabler: bool = False,
                              make_invisible: bool = False
                              ):
    Added_Obj_Prim_Root: str = None
    # It's better not to use CuRobo's enable_physics Attribute !
    if ObjectType == "Cuboid":
        Added_Obj_Prim_Root = World_Manager._usd_help.add_cuboid_to_stage(obstacle=obj,
                                                                          enable_physics= False)
    if ObjectType == "Mesh":
        Added_Obj_Prim_Root = World_Manager._usd_help.add_mesh_to_stage(obstacle=obj,
                                                                        enable_physics= False)
    if ObjectType == "Cylinder":
        Added_Obj_Prim_Root = World_Manager._usd_help.add_cylinder_to_stage(obstacle=obj,
                                                                            enable_physics= False)
    if ObjectType == "Sphere":
        Added_Obj_Prim_Root = World_Manager._usd_help.add_sphere_to_stage(obstacle=obj,
                                                                          enable_physics= False)

    stage = World_Manager._my_world.stage

    # All the added object are in the prim path of : "/world/obstacles/<OBJ Name>"
    Obj_Prim = stage.GetPrimAtPath(Added_Obj_Prim_Root)

    if rigid_body_disabler == False:

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
        # print(dir(UsdPhysics.Tokens))

        # Adding Colliders
        execute("AddPhysicsComponentCommand",
                            usd_prim=Obj_Prim,
                            component="PhysicsMassAPI")

        # Adding a Small Positive Mass to Avoid Robot's Effort Calculation using CuRobo
        Mass_Succ = Obj_Prim.GetAttribute("physics:mass").Set(0.0001)
        # Disabling Gravity
        Dis_Grav = Obj_Prim.GetAttribute("physxRigidBody:disableGravity").Set(True)

    # Making them Invisible
    if make_invisible == True:
        visibility_attribute = Obj_Prim.GetAttribute("visibility").Set("invisible")

    print("Object " + obj.name + " Added to the Simulation | PRIM: " + Added_Obj_Prim_Root)

    # Updating the Collision World for Each Robot After Adding an Object to the Scene
    if(Robot_1 != None):
        Robot_1.motion_gen_update_world()
    if(Robot_2 != None):
        Robot_2.motion_gen_update_world()

# Making the Ground Invisible
GR_Primitive = test._stage.GetPrimAtPath("/World/obstacles/Ground")
GR_Primitive.GetAttribute("visibility").Set("invisible")

# Smart Material Table's Collision        # obstacles.save_world_as_mesh("Testing.obj")
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



#############
####Start#### Manufacturing Strategies !!!
#############

# Robot_2_To Pick Position
def Do_Pick(Stud_Name: str = None,
            Stud_Dims: List[float] = None,
            Stud_Pose: List[float] = None):

        Robot_2.move_to_home()
        # Helping Pick
        Robot_2.plan(tcp_name= "tool0",
                       target_pose= [5.5, 1.44, 0.82],
                       target_orientation= [ev, 0, ev, 0],
                       update_world_needed= True)
        Robot_2.render_exec(renderInstance= True,
                              Show_Sphere= False)

        # Pick
        Robot_2.plan(tcp_name= "tool0",
                       target_pose= [6.17, 1.44, 0.82],
                       target_orientation= [ev, 0, ev, 0],
                       update_world_needed= True,
                       removing_primitives=["world/obstacles"])
        Robot_2.render_exec(renderInstance= True,
                              Show_Sphere= False)

        # Attach

        # Create Element
        Attaching_Element = Cuboid(
            name= Stud_Name,
            pose= [Stud_Pose[0], Stud_Pose[1], Stud_Pose[2], 1, 0, 0, 0],
            dims= [Stud_Dims[0], Stud_Dims[1], Stud_Dims[2]],
            color= [0.87, 0.72, 0.53, 1]
        )
        Add_Rigid_Object_To_Scene(test, "Cuboid", Attaching_Element)

        Robot_2.eef_attach(r_name= "IRB6620_R2",
                           tool_name="tool0",
                           attaching_object_name=Attaching_Element.name)

        # Helping Pick      
        Robot_2.plan(tcp_name= "tool0",
                       target_pose= [5.5, 1.44, 0.82],
                       target_orientation= [ev, 0, ev, 0],
                       update_world_needed= True,
                       removing_primitives=["world/obstacles"])
        Robot_2.render_exec(renderInstance= True,
                              Show_Sphere= False)

        Robot_2.plan(tcp_name= "tool0",
                       target_pose= [3.3, 0, 1.7],
                       target_orientation= [0, -ev, 0, ev],
                       update_world_needed= True)
        Robot_2.render_exec(renderInstance= True,
                              Show_Sphere= False)
        
        Robot_1.move_to_home()      
        
def Vertical(el_name: str = None,
             el_dims: List[float] = None,
             el_pose: List[float] = None,
             conveyor_pose: float = 0):

    if(el_name == None):
        return False
    
    # Moving Conveyor To Target Location
    Smart_Conv.render_exec('Joint_1', conveyor_pose)

    Do_Pick(Stud_Name= el_name,
            Stud_Dims= el_dims,
            Stud_Pose= el_pose)

    # Place Location (Helping)
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [3.08, 0.00, 1.05],
                    target_orientation= [0, ev, ev, 0],
                    update_world_needed= True)
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)

    # Place Location
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [3.1, 0.00, 0.98],
                    target_orientation= [0, ev, ev, 0],
                    update_world_needed= True,
                    removing_primitives=["Smart_Conveyor/Link_2", "world/obstacles"])
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)
    
    Robot_2.eef_detach(tool_name="tool0",
                        detaching_object_name= el_name)

    # Place Location (Helping)
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [3.08, 0.00, 1.05],
                    target_orientation= [0, ev, ev, 0],
                    update_world_needed= True,
                    removing_primitives=["Smart_Conveyor/Link_2", "world/obstacles"],
                    orientational_restriction=torch.tensor([1,1,1], dtype=torch.float32))
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)
    
    Robot_2.release_path_plan_restriction()

    # Home Position
    # Robot_2.plan(tcp_name= "tool0",
    #                 target_pose= [3.3, 0, 1.7],
    #                 target_orientation= [0, -ev, 0, ev],
    #                 update_world_needed= True,
    #                 removing_primitives=["Smart_Conveyor", "world/obstacles"])
    # Robot_2.render_exec(renderInstance= True,
    #                         Show_Sphere= False)
    Robot_2.move_to_home()
    
    ## Attaching The Placed Stud To Conveyor
    print(Smart_Conv.attach_object_to_conv(obj_name= el_name))
    test._my_world.step(render= True)

def Horizontal(el_name: str = None,
               el_dims: List[float] = None,
               el_pose: List[float] = None,
               conveyor_pose: float = 0):

    # Moving Conveyor To Target Location
    Smart_Conv.render_exec('Joint_1', conveyor_pose)

    Do_Pick(Stud_Name= el_name,
            Stud_Dims= el_dims,
            Stud_Pose= el_pose)

    # Place Location (Helping)
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [3.542, -0.21, 1.12],
                    target_orientation= [0, 0, 1, 0],
                    update_world_needed= True)
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)
    


    # Place
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [3.542, -0.21, 0.98],
                    target_orientation= [0, 0, 1, 0],
                    update_world_needed= True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles"],
                    orientational_restriction=torch.tensor([1,1,1], dtype=torch.float32))
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)

    Robot_2.eef_detach(tool_name="tool0",
                        detaching_object_name= el_name)

    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [3.542, -0.21, 1.12],
                    target_orientation= [0, 0, 1, 0],
                    update_world_needed= True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles"],
                    orientational_restriction=torch.tensor([1,1,1], dtype=torch.float32))
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)

    Robot_2.release_path_plan_restriction()

    # Home Position
    # Robot_2.plan(tcp_name= "tool0",
    #                 target_pose= [3.3, 0, 1.7],
    #                 target_orientation= [0, -ev, 0, ev],
    #                 update_world_needed= True)
    # Robot_2.render_exec(renderInstance= True,
    #                         Show_Sphere= False)
    Robot_2.move_to_home()

    ## Attaching The Placed Stud To Conveyor
    print(Smart_Conv.attach_object_to_conv(obj_name= el_name))
    test._my_world.step(render= True)

def Create_Wooden_Element_For_Smart_Mat_Table(el_name: str = None,
                                L: float = None,
                                W: float = None,
                                H: float = None,
                                Debug_Offset: bool = False):

    # Creating the 12ft Wooden Elements in Material Supply Table
    Debugger: float = STUD_TO_SAW_OFFSET+L
    if Debug_Offset == False:
        Debugger = 0
    Element = Cuboid(
        name= el_name,
        # 12ft is equal to 3.6576 meters (which is the maximum length of the Smart Material Table !)
        pose= [SMART_MAT_TABLE[0]+(H/2), SMART_MAT_TABLE[1]-SMART_MAT_TABLE_MAX_LENGTH+(L/2)-Debugger, SMART_MAT_TABLE[2]+(W/2), 1, 0, 0, 0],
        dims= [H, L, W],
        color= [0.4, 0.2, 0, 1]
    )
    Add_Rigid_Object_To_Scene(test, "Cuboid", Element)

def Robot_2_Do_Side_Nail(push_to_nail: float = None,
                         H: float = None):
    # ORientation [0.5, 0.5, 0.5, 0.5]
    # X 3.69
    # Y 1
    # Z TBD
    for target in Smart_Conv._nail_poses:
        # Check if the target is reachable or not (for the conveyor !)
        if(target[0]+(NAILING_CONV_TARGET*target[1]) > SMART_CONV_RANGE_OF_MOTION_J1):
            Replacing_Target: Tuple[float, float] = (target[0]-2*NAILING_CONV_TARGET, -target[1])
            target = Replacing_Target
        Smart_Conv.render_exec('Joint_1', target[0]+(NAILING_CONV_TARGET*target[1]))
        ###
        ### Nailing For That KING/1ST
        ###

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

# Pick and Placing the Bottom Plate
def BPL(el_name: str = None,
        X: float = None,
        Y: float = None,
        Z: float = None,
        L: float = None,
        W: float = None,
        H: float = None):
    
    # Move Robot Close To Pick Position

    # Correcting Movement Before Reaching the Smart Material Table
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [4.70, 1.08, 0.87],
                    target_orientation= [0, 1, 0, 0],
                    update_world_needed= True)
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)

    # Helping Pick (X -= -0.3)
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [SMART_MAT_TABLE[0]+PICK_OFFSET_FROM_W_CORNER-0.3,
                                  SMART_MAT_TABLE[1]-SMART_MAT_TABLE_MAX_LENGTH+PICK_OFFSET_FROM_L_CORNER+(ROBOT_2_GRIPPER_LENGTH/2),
                                  SMART_MAT_TABLE[2]+(W/2)],
                    target_orientation= [0, ev, 0, ev],
                    update_world_needed= True)
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)
    
    # Pick
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [SMART_MAT_TABLE[0]+PICK_OFFSET_FROM_W_CORNER,
                                  SMART_MAT_TABLE[1]-SMART_MAT_TABLE_MAX_LENGTH+PICK_OFFSET_FROM_L_CORNER+(ROBOT_2_GRIPPER_LENGTH/2),
                                  SMART_MAT_TABLE[2]+(W/2)],
                    target_orientation= [0, ev, 0, ev],
                    update_world_needed= True,
                    removing_primitives=["world/obstacles"],
                    orientational_restriction=torch.tensor([1,1,1], dtype=torch.float32))
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)
    
    # Creating the Wooden Element Within the Smart Material Supply
    Create_Wooden_Element_For_Smart_Mat_Table(el_name= el_name, L= L, W= W, H= H)

    # Attach
    Robot_2.eef_attach(tool_name="tool0",
                       attaching_object_name=el_name)
    print("Wooden Element Attached to Robot_2")
    # Post Pick 1 (X += 0.2)
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [SMART_MAT_TABLE[0]+PICK_OFFSET_FROM_W_CORNER+0.1,
                                  SMART_MAT_TABLE[1]-SMART_MAT_TABLE_MAX_LENGTH+PICK_OFFSET_FROM_L_CORNER+(ROBOT_2_GRIPPER_LENGTH/2),
                                  SMART_MAT_TABLE[2]+(W/2)],
                    target_orientation= [0, ev, 0, ev],
                    update_world_needed= True,
                    removing_primitives=["world/obstacles"],
                    orientational_restriction=torch.tensor([1,1,1], dtype=torch.float32))
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)
    
    # Post Pick 2(X += 0.2, Z+= 0.04)
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [SMART_MAT_TABLE[0]+PICK_OFFSET_FROM_W_CORNER+0.1,
                                  SMART_MAT_TABLE[1]-SMART_MAT_TABLE_MAX_LENGTH+PICK_OFFSET_FROM_L_CORNER+(ROBOT_2_GRIPPER_LENGTH/2),
                                  SMART_MAT_TABLE[2]+(W/2)+0.2],
                    target_orientation= [0, ev, 0, ev],
                    update_world_needed= True,
                    removing_primitives=["world/obstacles"],
                    orientational_restriction=torch.tensor([1,1,1], dtype=torch.float32))
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)
    
    # Post Pick Last
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [SMART_MAT_TABLE[0]+PICK_OFFSET_FROM_W_CORNER-0.3,
                                  SMART_MAT_TABLE[1]-SMART_MAT_TABLE_MAX_LENGTH+PICK_OFFSET_FROM_L_CORNER+(ROBOT_2_GRIPPER_LENGTH/2),
                                  SMART_MAT_TABLE[2]+(W/2)+0.2],
                    target_orientation= [0, ev, 0, ev],
                    update_world_needed= True,
                    removing_primitives=["world/obstacles"],
                    orientational_restriction=torch.tensor([1,1,1], dtype=torch.float32))
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)

    # Home Position
    Robot_2.move_to_home()

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

    PUSH_TO_NAIL_OFFSET: float = 0.05
    Robot_2_Do_Side_Nail(push_to_nail= PUSH_TO_NAIL_OFFSET, H= H)

    # Back To Home
    Robot_2.move_to_home()

# Pick and Placing the Top Plate
def TPL(el_name: str = None,
        X: float = None,
        Y: float = None,
        Z: float = None,
        L: float = None,
        W: float = None,
        H: float = None):

    # Move Robot Close To Pick Position

    # Correcting Movement Before Reaching the Smart Material Table
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [4.70, 1.08, 0.87],
                    target_orientation= [0, 1, 0, 0],
                    update_world_needed= True)
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)

    # Helping Pick (X -= -0.3)
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [SMART_MAT_TABLE[0]+PICK_OFFSET_FROM_W_CORNER-0.3,
                                  SMART_MAT_TABLE[1]-SMART_MAT_TABLE_MAX_LENGTH+PICK_OFFSET_FROM_L_CORNER+(ROBOT_2_GRIPPER_LENGTH/2),
                                  SMART_MAT_TABLE[2]+(W/2)],
                    target_orientation= [0, ev, 0, ev],
                    update_world_needed= True)
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)
    
    # Pick
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [SMART_MAT_TABLE[0]+PICK_OFFSET_FROM_W_CORNER,
                                  SMART_MAT_TABLE[1]-SMART_MAT_TABLE_MAX_LENGTH+PICK_OFFSET_FROM_L_CORNER+(ROBOT_2_GRIPPER_LENGTH/2),
                                  SMART_MAT_TABLE[2]+(W/2)],
                    target_orientation= [0, ev, 0, ev],
                    update_world_needed= True,
                    removing_primitives=["world/obstacles"],
                    orientational_restriction=torch.tensor([1,1,1], dtype=torch.float32))
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)
    
    # Creating the Wooden Element Within the Smart Material Supply
    Create_Wooden_Element_For_Smart_Mat_Table(el_name= el_name, L= L, W= W, H= H)

    # Attach
    Robot_2.eef_attach(tool_name="tool0",
                       attaching_object_name=el_name)
    print("Wooden Element Attached to Robot_2")

    # Post Pick 1 (X += 0.2)
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [SMART_MAT_TABLE[0]+PICK_OFFSET_FROM_W_CORNER+0.1,
                                  SMART_MAT_TABLE[1]-SMART_MAT_TABLE_MAX_LENGTH+PICK_OFFSET_FROM_L_CORNER+(ROBOT_2_GRIPPER_LENGTH/2),
                                  SMART_MAT_TABLE[2]+(W/2)],
                    target_orientation= [0, ev, 0, ev],
                    update_world_needed= True,
                    removing_primitives=["world/obstacles"],
                    orientational_restriction=torch.tensor([1,1,1], dtype=torch.float32))
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)
    
    # Post Pick 2(X += 0.2, Z+= 0.04)
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [SMART_MAT_TABLE[0]+PICK_OFFSET_FROM_W_CORNER+0.1,
                                  SMART_MAT_TABLE[1]-SMART_MAT_TABLE_MAX_LENGTH+PICK_OFFSET_FROM_L_CORNER+(ROBOT_2_GRIPPER_LENGTH/2),
                                  SMART_MAT_TABLE[2]+(W/2)+0.2],
                    target_orientation= [0, ev, 0, ev],
                    update_world_needed= True,
                    removing_primitives=["world/obstacles"],
                    orientational_restriction=torch.tensor([1,1,1], dtype=torch.float32))
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)
    
    # Post Pick Last
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [SMART_MAT_TABLE[0]+PICK_OFFSET_FROM_W_CORNER-0.3,
                                  SMART_MAT_TABLE[1]-SMART_MAT_TABLE_MAX_LENGTH+PICK_OFFSET_FROM_L_CORNER+(ROBOT_2_GRIPPER_LENGTH/2),
                                  SMART_MAT_TABLE[2]+(W/2)+0.2],
                    target_orientation= [0, ev, 0, ev],
                    update_world_needed= True,
                    removing_primitives=["world/obstacles"],
                    orientational_restriction=torch.tensor([1,1,1], dtype=torch.float32))
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)

    # Home Position
    Robot_2.move_to_home()

    ####
    # IF Pass To Robot 1

    # Move Conveyor Away !!!
    Smart_Conv.render_exec('Joint_1', SMART_CONV_RANGE_OF_MOTION_J1)

    PASSING_ELEVATION: float = 0.85
    # Passing To Rob 1
    
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [2.3+(L/2)-PICK_OFFSET_FROM_L_CORNER-(ROBOT_2_GRIPPER_LENGTH/2),
                                  -1,
                                  PASSING_ELEVATION],
                    target_orientation= [0, ev, ev, 0],
                    update_world_needed= True)
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)
    
    # ROBOT 1 MOVEMENT
    Robot_1.plan(tcp_name= "tool0",
                    target_pose= [2.3-(L/2)+PICK_OFFSET_FROM_L_CORNER+(ROBOT_1_GRIPPER_LENGTH/2),
                                  -1,
                                  PASSING_ELEVATION+0.1],
                    target_orientation= [0, ev, -ev, 0],
                    update_world_needed= True)
    Robot_1.render_exec(renderInstance= True,
                            Show_Sphere= False)
    
    Robot_1.plan(tcp_name= "tool0",
                    target_pose= [2.3-(L/2)+PICK_OFFSET_FROM_L_CORNER+(ROBOT_1_GRIPPER_LENGTH/2),
                                  -1,
                                  PASSING_ELEVATION],
                    target_orientation= [0, ev, -ev, 0],
                    update_world_needed= True)
    Robot_1.render_exec(renderInstance= True,
                            Show_Sphere= False)
    
    # Attach
    Robot_2.eef_detach(tool_name="tool0",
                        detaching_object_name= el_name)
    Robot_1.eef_attach(tool_name="tool0",
                       attaching_object_name=el_name)

    # Rob2 Post Detach before Reaching Home Pose
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [2.3+(L/2)-PICK_OFFSET_FROM_L_CORNER-(ROBOT_2_GRIPPER_LENGTH/2),
                                  -1,
                                  PASSING_ELEVATION+0.1],
                    target_orientation= [0, ev, ev, 0],
                    update_world_needed= True,
                    orientational_restriction=torch.tensor([1,1,1], dtype=torch.float32))
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)

    Robot_2.move_to_home()
    Robot_1.move_to_home()

    # Move Conveyor To TCP's 0 Position For Placement
    Smart_Conv.render_exec('Joint_1', Y - (OVERALL_PANEL_LENGTH/2) + ((L/2)-PICK_OFFSET_FROM_L_CORNER-(ROBOT_1_GRIPPER_LENGTH/2)+(SMART_CONV_RANGE_OF_MOTION_J1/2)))

# 2.3+(X-(OVERALL_PANEL_HEIGHT/2))

    # Robot 1 Pre Place Movement
    Robot_1.plan(tcp_name= "tool0",
                    target_pose= [2.3+(X-(OVERALL_PANEL_HEIGHT/2))+SMART_CONV_X_SHIFT,
                                  0,
                                  SMART_CONV_REST_ELEVATION+H-PICK_OFFSET_FROM_W_CORNER+0.2],
                    target_orientation= [0, 1, 0, 0],
                    update_world_needed= True)
    Robot_1.render_exec(renderInstance= True,
                            Show_Sphere= False)
    
    # Robot 1 Place Movement
    Robot_1.plan(tcp_name= "tool0",
                    target_pose= [2.3+(X-(OVERALL_PANEL_HEIGHT/2))+SMART_CONV_X_SHIFT,
                                  0,
                                  SMART_CONV_REST_ELEVATION+H-PICK_OFFSET_FROM_W_CORNER],
                    target_orientation= [0, 1, 0, 0],
                    update_world_needed= True,
                    removing_primitives=["Smart_Conveyor" ,"world/obstacles"],
                    orientational_restriction=torch.tensor([1,1,1], dtype=torch.float32))
    Robot_1.render_exec(renderInstance= True,
                            Show_Sphere= False)

    # Detach
    Robot_1.eef_detach(tool_name="tool0",
                        detaching_object_name= el_name)
    Smart_Conv.attach_object_to_conv(obj_name= el_name)

    # Robot 1 Post Place Movement
    # Robot 1 Pre Place Movement
    Robot_1.plan(tcp_name= "tool0",
                    target_pose= [2.3+(X-(OVERALL_PANEL_HEIGHT/2))+SMART_CONV_X_SHIFT,
                                  0,
                                  SMART_CONV_REST_ELEVATION+H-PICK_OFFSET_FROM_W_CORNER+0.2],
                    target_orientation= [0, 1, 0, 0],
                    update_world_needed= True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles"],
                    orientational_restriction=torch.tensor([1,1,1], dtype=torch.float32))
    Robot_1.render_exec(renderInstance= True,
                            Show_Sphere= False)

    # Back To Home
    Robot_1.move_to_home()

def Create_Wooden_Element_For_Sloped_Table(el_name: str = None,
                                # L is constant since this table is only for 8ft studs
                                L: float = 2.4384,
                                W: float = None,
                                H: float = None):
    # Creating the 8ft Wooden Elements in Sloped Supply Table
    diagonal_stud_l = np.sqrt(W**2+H**2)
    z_increase = (diagonal_stud_l/2)*np.sin(np.radians(SLOPED_MAT_TABLE_ANGLE)+np.arcsin((W/2)/(diagonal_stud_l/2)))
    y_increase = (diagonal_stud_l/2)*np.cos(np.radians(SLOPED_MAT_TABLE_ANGLE)+np.arcsin((W/2)/(diagonal_stud_l/2)))


    # Convert Euler (-theta, 0, 0) to quaternion (xyzw)
    quat = R.from_euler('xyz', [-np.radians(SLOPED_MAT_TABLE_ANGLE), 0, 0]).as_quat()

    Element = Cuboid(
        name=el_name,
        # 8ft element placement on the Sloped Table
        pose=[SLOPED_MAT_TABLE[0] + (L / 2), 
            SLOPED_MAT_TABLE[1] - y_increase, 
            SLOPED_MAT_TABLE[2] + z_increase, 
            quat[3], quat[0], quat[1], quat[2]],  # Quaternion (wxyz)
        dims=[L, H, W],
        color=[0.4, 0.2, 0, 1]
    )

    Add_Rigid_Object_To_Scene(test, "Cuboid", Element)

def Robot_1_Do_Side_Nail(push_to_nail: float = None,
                         H: float = None,
                         Side_Selector: float = 0):

        # Robot 1 Nail
        Robot_1.plan(tcp_name= "tool2",
                        target_pose= [1.1, NAILING_CONV_TARGET*Side_Selector, SMART_CONV_REST_ELEVATION+H-(0.7*H)],
                        target_orientation= [ev, 0, ev, 0],
                        update_world_needed= True)
        Robot_1.render_exec(renderInstance= True,
                                Show_Sphere= False)

        dc=_dynamic_control.acquire_dynamic_control_interface()
        object=dc.get_rigid_body("/"+Robot_1._ROS_JS_robot_indicator+"/tool2")

        # Restricted Bot Nail
        object_pose=dc.get_rigid_body_pose(object)
        Robot_1.release_path_plan_restriction()
        Robot_1.plan(tcp_name= "tool2",
                        target_pose= [object_pose.p[0]+push_to_nail, object_pose.p[1], object_pose.p[2]],
                        target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                        update_world_needed= True,
                        removing_primitives=["Smart_Conveyor", "world/obstacles"],
                        direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
        Robot_1.render_exec(renderInstance= True,
                                Show_Sphere= False)
        # Nail Done
        object_pose=dc.get_rigid_body_pose(object)
        Robot_1.release_path_plan_restriction()
        Robot_1.plan(tcp_name= "tool2",
                        target_pose= [object_pose.p[0]-push_to_nail, object_pose.p[1], object_pose.p[2]],
                        target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                        update_world_needed= True,
                        removing_primitives=["Smart_Conveyor", "world/obstacles"],
                        direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
        Robot_1.render_exec(renderInstance= True,
                                Show_Sphere= False)
        
        # Other Nail
        object_pose=dc.get_rigid_body_pose(object)
        Robot_1.release_path_plan_restriction()
        Robot_1.plan(tcp_name= "tool2",
                        target_pose= [object_pose.p[0], object_pose.p[1], object_pose.p[2]+(0.4*H)],
                        target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                        update_world_needed= True,
                        removing_primitives=["Smart_Conveyor", "world/obstacles"],
                        direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=0))
        Robot_1.render_exec(renderInstance= True,
                                Show_Sphere= False)


        # Restricted Top Nail
        object_pose=dc.get_rigid_body_pose(object)
        Robot_1.release_path_plan_restriction()
        Robot_1.plan(tcp_name= "tool2",
                        target_pose= [object_pose.p[0]+push_to_nail, object_pose.p[1], object_pose.p[2]],
                        target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                        update_world_needed= True,
                        removing_primitives=["Smart_Conveyor", "world/obstacles"],
                        direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
        Robot_1.render_exec(renderInstance= True,
                                Show_Sphere= False)
        # Restricted Top Nail
        object_pose=dc.get_rigid_body_pose(object)
        Robot_1.release_path_plan_restriction()
        Robot_1.plan(tcp_name= "tool2",
                        target_pose= [object_pose.p[0]-push_to_nail, object_pose.p[1], object_pose.p[2]],
                        target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                        update_world_needed= True,
                        removing_primitives=["Smart_Conveyor", "world/obstacles"],
                        direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
        Robot_1.render_exec(renderInstance= True,
                                Show_Sphere= False)

        # Back To Home
        Robot_1.move_to_home()

# Pick and Placing the 8ft Studs (1st and King)
def KING(el_name: str = None,
        X: float = None,
        Y: float = None,
        Z: float = None,
        L: float = 2.4384,
        W: float = None,
        H: float = None):

    # 8ft Stud Pose:
    diagonal_stud_l = np.sqrt(W**2+H**2)
    z_increase = (diagonal_stud_l/2)*np.sin(np.radians(SLOPED_MAT_TABLE_ANGLE)+np.arcsin((W/2)/(diagonal_stud_l/2)))
    y_increase = (diagonal_stud_l/2)*np.cos(np.radians(SLOPED_MAT_TABLE_ANGLE)+np.arcsin((W/2)/(diagonal_stud_l/2)))
    Stud_Pose = [SLOPED_MAT_TABLE[0] + (L / 2), 
                 SLOPED_MAT_TABLE[1] - y_increase, 
                 SLOPED_MAT_TABLE[2] + z_increase]
    
    PRE_PICK_OFFSET: float = 0.2
    quat = R.from_euler('xyz', [(np.pi/2)-np.radians(SLOPED_MAT_TABLE_ANGLE), 0, np.pi/2]).as_quat()
    # Because of Euler Lack of Singularity !!!!
    quat[1] *= -1  # Negate the Y component

    # Helping Pick
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [SLOPED_MAT_TABLE[0]+SLOPED_TABLE_PICK_OFFSET_FROM_L_CORNER+(ROBOT_2_GRIPPER_LENGTH/2),
                                  Stud_Pose[1]+PRE_PICK_OFFSET,
                                  Stud_Pose[2]-(PRE_PICK_OFFSET*np.tan(np.radians(SLOPED_MAT_TABLE_ANGLE)))],
                    target_orientation= [quat[3], quat[0], quat[1], quat[2]],
                    update_world_needed= True)
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)

    # Pick
    Val = (H/2) - PICK_OFFSET_FROM_W_CORNER
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [SLOPED_MAT_TABLE[0]+SLOPED_TABLE_PICK_OFFSET_FROM_L_CORNER+(ROBOT_2_GRIPPER_LENGTH/2),
                                  Stud_Pose[1]+(Val*np.cos(np.radians(SLOPED_MAT_TABLE_ANGLE))),
                                  Stud_Pose[2]-(Val*np.sin(np.radians(SLOPED_MAT_TABLE_ANGLE)))],
                    target_orientation= [quat[3], quat[0], quat[1], quat[2]],
                    update_world_needed= True,
                    removing_primitives=["world/obstacles", "World/obstacles/Sloped_Table"],
                    orientational_restriction=torch.tensor([1,1,1], dtype=torch.float32))
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)

    # Creating the Stud
    Create_Wooden_Element_For_Sloped_Table(el_name= el_name, L=L, W=W, H=H)

    # Attach
    Robot_2.eef_attach(tool_name="tool0",
                       attaching_object_name=el_name)

    # Post Pick
    Val = (H/2) - PICK_OFFSET_FROM_W_CORNER
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [SLOPED_MAT_TABLE[0]+SLOPED_TABLE_PICK_OFFSET_FROM_L_CORNER+(ROBOT_2_GRIPPER_LENGTH/2),
                                  Stud_Pose[1]+(Val*np.cos(np.radians(SLOPED_MAT_TABLE_ANGLE))),
                                  Stud_Pose[2]-(Val*np.sin(np.radians(SLOPED_MAT_TABLE_ANGLE)))+PRE_PICK_OFFSET*2],
                    target_orientation= [quat[3], quat[0], quat[1], quat[2]],
                    update_world_needed= True,
                    removing_primitives=["world/obstacles", "World/obstacles/Sloped_Table"],
                    orientational_restriction=torch.tensor([1,1,1], dtype=torch.float32))
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)
    
    # Back To Home
    Robot_2.move_to_home()

    # Conveyor Move For Placement

    Side_Selector: float = 0
    if (OVERALL_PANEL_LENGTH/2)- Y +(SMART_CONV_RANGE_OF_MOTION_J1/2)+NAILING_CONV_TARGET > SMART_CONV_RANGE_OF_MOTION_J1:
        Side_Selector = -1
    else:
        Side_Selector = 1

    Smart_Conv.render_exec('Joint_1', (OVERALL_PANEL_LENGTH/2)- Y +(SMART_CONV_RANGE_OF_MOTION_J1/2)+NAILING_CONV_TARGET*Side_Selector)
    # Saving Joint Location For Nailing
    Smart_Conv._nail_poses.append(((OVERALL_PANEL_LENGTH/2)- Y +(SMART_CONV_RANGE_OF_MOTION_J1/2)+NAILING_CONV_TARGET*Side_Selector, Side_Selector))

    # Pre Place
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [2.3+(X-(OVERALL_PANEL_HEIGHT/2))+SMART_CONV_X_SHIFT+((L/2)-(ROBOT_2_GRIPPER_LENGTH/2)-SLOPED_TABLE_PICK_OFFSET_FROM_L_CORNER)+0.2,
                                  NAILING_CONV_TARGET*Side_Selector,
                                  SMART_CONV_REST_ELEVATION+H-PICK_OFFSET_FROM_W_CORNER+0.1],
                    target_orientation= [0, ev, ev, 0],
                    update_world_needed= True)
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)
    # Place
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [2.3+(X-(OVERALL_PANEL_HEIGHT/2))+SMART_CONV_X_SHIFT+((L/2)-(ROBOT_2_GRIPPER_LENGTH/2)-SLOPED_TABLE_PICK_OFFSET_FROM_L_CORNER),
                                  NAILING_CONV_TARGET*Side_Selector,
                                  SMART_CONV_REST_ELEVATION+H-PICK_OFFSET_FROM_W_CORNER],
                    target_orientation= [0, ev, ev, 0],
                    update_world_needed= True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles"],
                    orientational_restriction=torch.tensor([1,1,1], dtype=torch.float32))
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)

    ###
    ###NAILING
    ###

    PUSH_TO_NAIL_OFFSET: float = 0.05
    Robot_1_Do_Side_Nail(push_to_nail= PUSH_TO_NAIL_OFFSET, H= H, Side_Selector= Side_Selector)

    ###
    ###NAILING DONE
    ###

    # Detech
    Robot_2.eef_detach(tool_name="tool0",
                        detaching_object_name= el_name)
    Smart_Conv.attach_object_to_conv(obj_name= el_name)

    # Post Place
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [2.3+(X-(OVERALL_PANEL_HEIGHT/2))+SMART_CONV_X_SHIFT+((L/2)-(ROBOT_2_GRIPPER_LENGTH/2)-SLOPED_TABLE_PICK_OFFSET_FROM_L_CORNER)+0.2,
                                  NAILING_CONV_TARGET*Side_Selector,
                                  SMART_CONV_REST_ELEVATION+H-PICK_OFFSET_FROM_W_CORNER+0.1],
                    target_orientation= [0, ev, ev, 0],
                    update_world_needed= True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles"],
                    orientational_restriction=torch.tensor([1,1,1], dtype=torch.float32))
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)
    # Home
    Robot_2.move_to_home()

    # Sheathing Nail Location Saving !

    # To Pick Position    
# Angular (90-SLOPE, 0, 90)

def Drag_Stud(el_name: str = None,
              el_dims: List[float] = None):


    # Correcting Movement Before Reaching the Smart Material Table
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [4.70, 1.08, 0.87],
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

def Robot_1_Do_Jack_Nail(push_to_nail: float = None,
                         el_pose: List[float] = [],
                         el_dims: List[float] = [],
                         Is_LJCK: bool = False,
                         Side_Selector: float = 1):
    
    Left_Jack_Checker: float = 0
    if (Is_LJCK == False):
        Left_Jack_Checker = 1
    else:
        Left_Jack_Checker = -1

    # [-90, 0, 90]
    # [0.5, -0.5, 0.5, 0.5]
    quat = R.from_euler('xyz', [((np.pi/2)+np.radians(JACK_SIDE_NAILING_ANGLE))*(-Left_Jack_Checker), 0, (np.pi/2)*(Left_Jack_Checker)]).as_quat()
    quat[1] *= -1

    # Pre Nail
    Nail_Doable: bool = Robot_1.plan(tcp_name= "tool2",
                    target_pose= [2.3+(el_pose[0]-(OVERALL_PANEL_HEIGHT/2))+SMART_CONV_X_SHIFT-(el_dims[0]/2)+0.06096,
                                  NAILING_CONV_TARGET*Side_Selector+((JACK_NAILING_OFFSET*np.cos(np.radians(JACK_SIDE_NAILING_ANGLE)))*(-Left_Jack_Checker)),
                                  SMART_CONV_REST_ELEVATION+(el_dims[2]*0.3)+((JACK_NAILING_OFFSET)*np.sin(np.radians(JACK_SIDE_NAILING_ANGLE)))],
                    target_orientation= [quat[3], quat[0], quat[1], quat[2]],
                    update_world_needed= True)
    
    if(Nail_Doable == False):
        print("Robot 1 Cannot Nail The Jack to the King (Joint Limitation !!!)")
        return

    Robot_1.render_exec(renderInstance= True,
                            Show_Sphere= False)

    dc=_dynamic_control.acquire_dynamic_control_interface()

    # Nail 1
    object=dc.get_rigid_body("/"+Robot_1._ROS_JS_robot_indicator+"/tool2")
    object_pose=dc.get_rigid_body_pose(object)
    Robot_1.plan(tcp_name= "tool2",
                    target_pose= [object_pose.p[0],
                                  object_pose.p[1]+(Left_Jack_Checker*(JACK_NAILING_OFFSET-((el_dims[1]/2)/np.cos(np.radians(JACK_SIDE_NAILING_ANGLE)))+push_to_nail))*np.cos(np.radians(JACK_SIDE_NAILING_ANGLE)),
                                  object_pose.p[2]-((JACK_NAILING_OFFSET-((el_dims[1]/2)/np.cos(np.radians(JACK_SIDE_NAILING_ANGLE)))+push_to_nail))*np.sin(np.radians(JACK_SIDE_NAILING_ANGLE))],
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
                                  object_pose.p[1]-(Left_Jack_Checker*(JACK_NAILING_OFFSET-((el_dims[1]/2)/np.cos(np.radians(JACK_SIDE_NAILING_ANGLE)))+push_to_nail))*np.cos(np.radians(JACK_SIDE_NAILING_ANGLE)),
                                  object_pose.p[2]+((JACK_NAILING_OFFSET-((el_dims[1]/2)/np.cos(np.radians(JACK_SIDE_NAILING_ANGLE)))+push_to_nail))*np.sin(np.radians(JACK_SIDE_NAILING_ANGLE))],
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
                    target_pose= [object_pose.p[0],
                                  object_pose.p[1],
                                  object_pose.p[2]+(0.4*el_dims[2])],
                    target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                    update_world_needed= True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles"],
                    direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=0))
    Robot_1.render_exec(renderInstance= True,
                            Show_Sphere= False)
    
    # Nail 2
    object=dc.get_rigid_body("/"+Robot_1._ROS_JS_robot_indicator+"/tool2")
    object_pose=dc.get_rigid_body_pose(object)
    Robot_1.plan(tcp_name= "tool2",
                    target_pose= [object_pose.p[0],
                                  object_pose.p[1]+(Left_Jack_Checker*(JACK_NAILING_OFFSET-((el_dims[1]/2)/np.cos(np.radians(JACK_SIDE_NAILING_ANGLE)))+push_to_nail))*np.cos(np.radians(JACK_SIDE_NAILING_ANGLE)),
                                  object_pose.p[2]-((JACK_NAILING_OFFSET-((el_dims[1]/2)/np.cos(np.radians(JACK_SIDE_NAILING_ANGLE)))+push_to_nail))*np.sin(np.radians(JACK_SIDE_NAILING_ANGLE))],
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
                                  object_pose.p[1]-(Left_Jack_Checker*(JACK_NAILING_OFFSET-((el_dims[1]/2)/np.cos(np.radians(JACK_SIDE_NAILING_ANGLE)))+push_to_nail))*np.cos(np.radians(JACK_SIDE_NAILING_ANGLE)),
                                  object_pose.p[2]+((JACK_NAILING_OFFSET-((el_dims[1]/2)/np.cos(np.radians(JACK_SIDE_NAILING_ANGLE)))+push_to_nail))*np.sin(np.radians(JACK_SIDE_NAILING_ANGLE))],
                    target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                    update_world_needed= True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles"],
                    direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
    Robot_1.render_exec(renderInstance= True,
                            Show_Sphere= False)

    Robot_1.move_to_home()

def RJCK(el_name: str = None,
        X: float = None,
        Y: float = None,
        Z: float = None,
        L: float = None,
        W: float = 0.04,
        H: float = None,
        Is_LJCK: bool = False):

    Drag_Stud(el_name= el_name, el_dims= [L, W, H])

    #[ev, 0, ev, 0]
    # Pre Pick
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [SMART_MAT_TABLE[0]+PICK_OFFSET_FROM_W_CORNER-0.3,
                                  SMART_MAT_TABLE[1]-SMART_MAT_TABLE_MAX_LENGTH-STUD_TO_SAW_OFFSET-PICK_OFFSET_FROM_L_CORNER-(ROBOT_2_GRIPPER_LENGTH/2),
                                  SMART_MAT_TABLE[2]+(W/2)],
                    target_orientation= [ev, 0, ev, 0],
                    update_world_needed= True)
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)
    # Pick
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [SMART_MAT_TABLE[0]+PICK_OFFSET_FROM_W_CORNER,
                                  SMART_MAT_TABLE[1]-SMART_MAT_TABLE_MAX_LENGTH-STUD_TO_SAW_OFFSET-PICK_OFFSET_FROM_L_CORNER-(ROBOT_2_GRIPPER_LENGTH/2),
                                  SMART_MAT_TABLE[2]+(W/2)],
                    target_orientation= [ev, 0, ev, 0],
                    update_world_needed= True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles"],
                    orientational_restriction=torch.tensor([1,1,1], dtype=torch.float32))

    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)
    # Saw Action
    # Removing 12ft Primitive and Replacing it with L !
    prims_utils.delete_prim("/world/obstacles/"+el_name+"Temp")
    Create_Wooden_Element_For_Smart_Mat_Table(el_name= el_name, L= L, W= W, H= H, Debug_Offset= True)

    Robot_2.eef_attach(tool_name= "tool0", attaching_object_name= el_name)
    #.....

    # Post Pick
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [SMART_MAT_TABLE[0]+PICK_OFFSET_FROM_W_CORNER-0.3,
                                  SMART_MAT_TABLE[1]-SMART_MAT_TABLE_MAX_LENGTH-STUD_TO_SAW_OFFSET-PICK_OFFSET_FROM_L_CORNER-(ROBOT_2_GRIPPER_LENGTH/2),
                                  SMART_MAT_TABLE[2]+(W/2)],
                    target_orientation= [ev, 0, ev, 0],
                    update_world_needed= True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles"],
                    orientational_restriction=torch.tensor([1,1,1], dtype=torch.float32))
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)

    # Conveyor Move For Placement
    Side_Selector: float = 0
    Jack_Y_Placement_Offset: float = None

    if (Is_LJCK == False):
        Side_Selector = 1
        Jack_Y_Placement_Offset =-JACK_PLACEMENT_SIDE_DRAG
        if ((OVERALL_PANEL_LENGTH/2)- Y +(SMART_CONV_RANGE_OF_MOTION_J1/2)+NAILING_CONV_TARGET > SMART_CONV_RANGE_OF_MOTION_J1):
            Side_Selector = -1

    if (Is_LJCK == True):
        Side_Selector = -1
        Jack_Y_Placement_Offset = JACK_PLACEMENT_SIDE_DRAG
        if ((OVERALL_PANEL_LENGTH/2)- Y +(SMART_CONV_RANGE_OF_MOTION_J1/2)+NAILING_CONV_TARGET < 0):
            Side_Selector = 1

    Smart_Conv.render_exec('Joint_1', (OVERALL_PANEL_LENGTH/2)- Y +(SMART_CONV_RANGE_OF_MOTION_J1/2)+NAILING_CONV_TARGET*Side_Selector)
    # Saving Joint Location For Nailing
    Smart_Conv._nail_poses.append(((OVERALL_PANEL_LENGTH/2)- Y +(SMART_CONV_RANGE_OF_MOTION_J1/2)+NAILING_CONV_TARGET*Side_Selector, Side_Selector))
        
    Robot_2.move_to_home()

    # Pre Place
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [2.3+(X-(OVERALL_PANEL_HEIGHT/2))+SMART_CONV_X_SHIFT+((L/2)-(ROBOT_2_GRIPPER_LENGTH/2)-PICK_OFFSET_FROM_L_CORNER),
                                  NAILING_CONV_TARGET*Side_Selector,
                                  SMART_CONV_REST_ELEVATION+H-PICK_OFFSET_FROM_W_CORNER+0.1],
                    target_orientation= [0, ev, ev, 0],
                    update_world_needed= True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles"])
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)

    # Place
    # Robot_2.plan(tcp_name= "tool0",
    #                 target_pose= [2.3+(X-(OVERALL_PANEL_HEIGHT/2))+SMART_CONV_X_SHIFT+((L/2)-(ROBOT_2_GRIPPER_LENGTH/2)-PICK_OFFSET_FROM_L_CORNER),
    #                               NAILING_CONV_TARGET*Side_Selector,
    #                               SMART_CONV_REST_ELEVATION+H-PICK_OFFSET_FROM_W_CORNER],
    #                 target_orientation= [0, ev, ev, 0],
    #                 update_world_needed= True,
    #                 removing_primitives=["Smart_Conveyor", "world/obstacles"],
    #                 orientational_restriction=torch.tensor([1,1,1], dtype=torch.float32))
    # Robot_2.render_exec(renderInstance= True,
    #                         Show_Sphere= False)

    # Detech
    Robot_2.eef_detach(tool_name="tool0",
                        detaching_object_name= el_name)
    # Enabling Gravity For The Stud
    test._stage.GetPrimAtPath("/world/obstacles/" + el_name).GetAttribute("physxRigidBody:disableGravity").Set(False)
    Smart_Conv.attach_object_to_conv(obj_name= el_name)

    # Robot 1 Nail !!
    PUSH_TO_NAIL_OFFSET: float = 0.01
    Robot_1_Do_Jack_Nail(push_to_nail=PUSH_TO_NAIL_OFFSET, el_pose=[X,Y,Z], el_dims=[L,W,H], Is_LJCK= Is_LJCK, Side_Selector= Side_Selector)

    # Post Place
    # Robot_2.plan(tcp_name= "tool0",
    #                 target_pose= [2.3+(X-(OVERALL_PANEL_HEIGHT/2))+SMART_CONV_X_SHIFT+((L/2)-(ROBOT_2_GRIPPER_LENGTH/2)-PICK_OFFSET_FROM_L_CORNER)+0.2,
    #                               NAILING_CONV_TARGET*Side_Selector,
    #                               SMART_CONV_REST_ELEVATION+H-PICK_OFFSET_FROM_W_CORNER+0.1],
    #                 target_orientation= [0, ev, ev, 0],
    #                 update_world_needed= True,
    #                 removing_primitives=["Smart_Conveyor", "world/obstacles"],
    #                 orientational_restriction=torch.tensor([1,1,1], dtype=torch.float32))
    # Robot_2.render_exec(renderInstance= True,
    #                         Show_Sphere= False)

    Robot_2.move_to_home()

    # # robot 1 move to nailing target
    # self.move_group = self.planning_groups["EF1_NG"]
    # self.pose_goal.orientation.x = -0.6123724
    # self.pose_goal.orientation.y = 0.6123724
    # self.pose_goal.orientation.z = 0.3535534
    # self.pose_goal.orientation.w = 0.3535534
    # self.pose_goal.position.x = final_object_pose.position.x - box_size[1]*0.5 + 0.06096
    # self.pose_goal.position.y = final_object_pose.position.y - box_size[0]*0.5
    # self.pose_goal.position.z = final_object_pose.position.z + box_size[2]/6
    # self.plan_and_execute(self.pose_goal)
    r=1

def LJCK(el_name: str = None,
        X: float = None,
        Y: float = None,
        Z: float = None,
        L: float = None,
        W: float = None,
        H: float = None):
    # Same As RJCK !
    RJCK(el_name= el_name, X= X, Y= Y, Z= Z, L= L, W= W, H= H, Is_LJCK= True)

def HDR(el_name: str = None,
        X: float = None,
        Y: float = None,
        Z: float = None,
        L: float = None,
        W: float = None,
        H: float = None):
    
    # Dragging the Required Length
    Drag_Stud(el_name= el_name, el_dims= [L, W, H])

    #[0, -ev, 0, -ev]
    # Pre Pick
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [SMART_MAT_TABLE[0]+PICK_OFFSET_FROM_W_CORNER-0.3,
                                  SMART_MAT_TABLE[1]-SMART_MAT_TABLE_MAX_LENGTH-STUD_TO_SAW_OFFSET-PICK_OFFSET_FROM_L_CORNER-(ROBOT_2_GRIPPER_LENGTH/2),
                                  SMART_MAT_TABLE[2]+(W/2)],
                    target_orientation= [0, -ev, 0, -ev],
                    update_world_needed= True)
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)
    
    # Pick
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [SMART_MAT_TABLE[0]+PICK_OFFSET_FROM_W_CORNER,
                                  SMART_MAT_TABLE[1]-SMART_MAT_TABLE_MAX_LENGTH-STUD_TO_SAW_OFFSET-PICK_OFFSET_FROM_L_CORNER-(ROBOT_2_GRIPPER_LENGTH/2),
                                  SMART_MAT_TABLE[2]+(W/2)],
                    target_orientation= [0, -ev, 0, -ev],
                    update_world_needed= True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles"],
                    orientational_restriction=torch.tensor([1,1,1], dtype=torch.float32))
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)
    
    # Saw Action + Attach
    # Removing 12ft Primitive and Replacing it with L !
    prims_utils.delete_prim("/world/obstacles/"+el_name+"Temp")
    Create_Wooden_Element_For_Smart_Mat_Table(el_name= el_name, L= L, W= W, H= H, Debug_Offset= True)

    Robot_2.eef_attach(tool_name= "tool0", attaching_object_name= el_name)
    #.....

    # Post Pick
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [SMART_MAT_TABLE[0]+PICK_OFFSET_FROM_W_CORNER-0.3,
                                  SMART_MAT_TABLE[1]-SMART_MAT_TABLE_MAX_LENGTH-STUD_TO_SAW_OFFSET-PICK_OFFSET_FROM_L_CORNER-(ROBOT_2_GRIPPER_LENGTH/2),
                                  SMART_MAT_TABLE[2]+(W/2)],
                    target_orientation= [0, -ev, 0, -ev],
                    update_world_needed= True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles"],
                    orientational_restriction=torch.tensor([1,1,1], dtype=torch.float32))
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)
    
    # Home
    Robot_2.move_to_home()

    # Height Passing Location

    # Small Stud Passing
    # X: 2.05, Y: 0, Z: 1.55
    # Rob_1: [ev, 0, ev, 0] (GR)
    # Rob_2: [0, ev, 0, -ev] (GR)

    PASSING_LOC: list[float] = [2.05, 0, 1.55]

    # Robot 1 Pre Take Location
    Robot_1.plan(tcp_name= "tool0",
                    target_pose= [PASSING_LOC[0]+2*(PICK_OFFSET_FROM_W_CORNER-(H/2))-0.3,
                                  (ROBOT_2_GRIPPER_LENGTH/2)+PICK_OFFSET_FROM_L_CORNER-(PICK_OFFSET_FROM_L_CORNER_AFTER_PASS+(ROBOT_1_GRIPPER_LENGTH/2)), 1.55],
                    target_orientation= [ev, 0, ev, 0],
                    update_world_needed= True)
    Robot_1.render_exec(renderInstance= True,
                            Show_Sphere= False)

    Robot_2.plan(tcp_name= "tool0",
                    target_pose= PASSING_LOC,
                    target_orientation= [0, ev, 0, -ev],
                    update_world_needed= True)
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)

    # Robot 1 Reach
    Robot_1.plan(tcp_name= "tool0",
                    target_pose= [PASSING_LOC[0]+2*(PICK_OFFSET_FROM_W_CORNER-(H/2)),
                                  (ROBOT_2_GRIPPER_LENGTH/2)+PICK_OFFSET_FROM_L_CORNER-(PICK_OFFSET_FROM_L_CORNER_AFTER_PASS+(ROBOT_1_GRIPPER_LENGTH/2)), 1.55],
                    target_orientation= [ev, 0, ev, 0],
                    update_world_needed= True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles", "IRB6620_R2"],
                    orientational_restriction=torch.tensor([1,1,1], dtype=torch.float32))
    Robot_1.render_exec(renderInstance= True,
                            Show_Sphere= False)

    # Transfer Attachment
    Robot_2.eef_detach(tool_name="tool0", detaching_object_name= el_name)
    # To See the Detached Object within the Scene !!!
    Robot_1.motion_gen_update_world()
    Robot_1.eef_attach(tool_name="tool0", attaching_object_name= el_name)

    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [PASSING_LOC[0]+0.2, PASSING_LOC[1], PASSING_LOC[2]],
                    target_orientation= [0, ev, 0, -ev],
                    update_world_needed= True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles", "IRB6620_R1"],
                    orientational_restriction=torch.tensor([1,1,1], dtype=torch.float32))
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)
    
    Robot_2.move_to_home()
    Robot_1.move_to_home()

    Robot_1.free_TCP_movement(moving_tcp= "tool0")
###########
####END####
###########


def main():

    rospy.init_node("tutorial_subscriber", anonymous=True)

    i=0

    # for robot in robots:
        # robot._js_pub_interval = rospy.Timer(rospy.Duration(10.0 / publish_rate), robot.ros_js_publisher)

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

# Publishing ROS JointState on Movement
        # for robot in robots:
        #         robot.ros_js_publisher()

        # T = time.time()
        # while time.time() - T <= 5:
        #     test._my_world.step(render= True)

        # Robot_1.plan(tcp_name= "tool1",
        #                 target_pose= [-0.95, -1.2, 0.6],
        #                 target_orientation= [0, ev, ev, 0],
        #                 update_world_needed= True)
        # Robot_1.render_exec(renderInstance= True,
        #                         Show_Sphere= False)
        #
        # Robot_1.plan(tcp_name= "tool1",
        #                 target_pose= [2, -0.28, 1.05],
        #                 target_orientation= [0, -ev, -ev, 0],
        #                 update_world_needed= True)
        # Robot_1.render_exec(renderInstance= True,
        #                         Show_Sphere= False)        


        HDR("Small_Stud_1", 0.4584, 2, 0, 1, 0.04, 0.12)
        # TPL DONE !
        # TPL("Wooden_Element_1", 0.02, SMART_MAT_TABLE_MAX_LENGTH/2, 0.06, SMART_MAT_TABLE_MAX_LENGTH, 0.04, 0.12)
        # KING DONE !
        # KING("Wooden_Element_2", 1.2592, 0.02, 0, 2.4384, 0.04, 0.12)
        # KING("Wooden_Element_3", 1.2592, 0.4, 0, 2.4384, 0.04, 0.12)
        # KING("Wooden_Element_4", 1.2592, 0.9, 0, 2.4384, 0.04, 0.12)
        # KING("Wooden_Element_5", 1.2592, 1.4, 0, 2.4384, 0.04, 0.12)
        # KING("Wooden_Element_6", 1.2592, 1.52, 0, 2.4384, 0.04, 0.12)
        # KING("Wooden_Element_7", 1.2592, 2.52, 0, 2.4384, 0.04, 0.12)
        # KING("Wooden_Element_8", 1.2592, 2.64, 0, 2.4384, 0.04, 0.12)
        # KING("Wooden_Element_9", 1.2592, 3.14, 0, 2.4384, 0.04, 0.12)
        # KING("Wooden_Element_10", 1.2592, SMART_MAT_TABLE_MAX_LENGTH-0.02, 0, 2.4384, 0.04, 0.12)

        # JACK DONE !
        # LJCK("Wooden_Element_12", 1.4784, 1.56, 0, 2, 0.04, 0.12)
        # RJCK("Wooden_Element_11", 1.4784, 2.48, 0, 2, 0.04, 0.12)

        # # BPL DONE !
        # BPL("Wooden_Element_13", OVERALL_PANEL_HEIGHT-0.02, SMART_MAT_TABLE_MAX_LENGTH/2, 0.06, SMART_MAT_TABLE_MAX_LENGTH, 0.04, 0.12)

        Robot_2.free_TCP_movement(moving_tcp= "tool0")

if __name__ == "__main__":
    main()

