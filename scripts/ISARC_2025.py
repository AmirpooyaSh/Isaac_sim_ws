from curobo.wrap.reacher import motion_gen
from isaacsim import SimulationApp
# import sensor_msgs.msg

simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

from omni.isaac.core import World
from omni.isaac.core.objects import cuboid, sphere
import numpy as np
import math

# Adding mesh to the world (Standalone Format)
from omni.physx.scripts import utils
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
from omni.isaac.core.robots import Robot
from omni.isaac.core.scenes.scene import Scene


# This is used to make VScode understand ArticulationController's type (which is fed from the robot privately)
from typing import cast, List, Optional, Any

from omni.isaac.core.utils.extensions import enable_extension
# enable ROS bridge extension
# enable_extension("omni.isaac.ros_bridge")

# enable SurfaceGripper extention
enable_extension("omni.isaac.surface_gripper")
simulation_app.update()

# check if rosmaster node is running
# this is to prevent this sample from waiting indefinetly if roscore is not running
# can be removed in regular usage
# import rosgraph

# if not rosgraph.is_master_online():
#     carb.log_error("Please run roscore before executing this script")
#     simulation_app.close()
#     exit()

# import rospy
# import sensor_msgs
# import omni.graph.core as og

# Xform Creation and Transform
import omni.kit.commands as cmd
# Adding UsdPhysics to Add Mass To the Object
from pxr import Gf, Usd, Sdf, UsdGeom, UsdPhysics

import omni.usd
# Creating SurfaceGripper OmniGraph
import omni.graph.core as og

from omni.isaac.surface_gripper import SurfaceGripper

from omni.kit.commands import execute

#Dynamic Control API
from omni.isaac.dynamic_control import _dynamic_control




import re
import time
import torch
import threading



# Rate for Publishing ROS Topics from this project !
publish_rate = 10.0  # Frequency in Hz

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
        # Creating "Prim !"
        self._stage = self._my_world.stage
        self._xform = self._stage.DefinePrim("/World", "Xform")
        self._stage.SetDefaultPrim(self._xform)
        # stage.DefinePrim("/curobo", "Xform")
        self._stage = self._my_world.stage

        # Adding the default Ground to the World
        cast(Scene, self._my_world.scene).add_default_ground_plane()

        # Creating USD Helper (CuRobo Object) to handle objects
        self._usd_help = UsdHelper()
        
        # ?????
        self._usd_help.load_stage(self._my_world.stage)

        # Adding the required Isaac Sim Extensions to the Simulation (Using CuRobo Helper Library)
        add_extensions(simulation_app, "False")

        # Adding CuRobo World Config to the Stage !!!
        self._curobo_world_cfg = self.init_world_model()
        self._usd_help.add_world_to_stage(self._curobo_world_cfg, base_frame="/World")
    
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
        cur_dir = "/home/apshirazi/Isaac_sim_ws/"

        MatTable_3Level = Mesh(
            name="MatTable_3Level",
            pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            file_path= cur_dir + "meshes/MatTable_3Level.stl",
            scale=[0.001, 0.001, 0.001],
        )
        MatTable_6Level = Mesh(
            name="MatTable_6Level",
            pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            file_path= cur_dir + "meshes/MatTable_6Level.stl",
            scale=[0.001, 0.001, 0.001],
        )
        MatTable_tilted = Mesh(
            name="MatTable_tilted",
            pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            file_path= cur_dir + "meshes/MatTable_tilted.stl",
            scale=[0.001, 0.001, 0.001],
        )
        NewConvn1_V2 = Mesh(
            name="NewConvn1_V2",
            pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            file_path= cur_dir + "meshes/NewConvn1_V2.stl",
            scale=[0.001, 0.001, 0.001],
            color=[0.36,0.25,0.20, 1.0],
        )
        NewConvn2_V2 = Mesh(
            name="NewConvn2_V2",
            pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            file_path= cur_dir + "meshes/NewConvn2_V2.stl",
            scale=[0.001, 0.001, 0.001],
            color=[0.36,0.25,0.20, 1.0],    
        )
        NewSheathingRack = Mesh(
            name="NewSheathingRack",
            pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            file_path= cur_dir + "meshes/NewSheathingRack.stl",
            scale=[0.001, 0.001, 0.001],
            color=[0.36,0.25,0.20, 1.0],  
        )

        # SheathingPlate = Cuboid(
        #     name="SheathingPlate",
        #     pose=[-1.8, -2.1, 0.37, 0, 0, 0, 1],
        #     dims=[3, 1.5, 0.015],
        #     color=[0.87, 0.72, 0.53, 1]
        # )

        # Smart Material Table
        # Smart_Mat_Table_Quat = self.quat_transfer_world_generator(90, 0, 180)

        # Smart_Mat_Table = Mesh(
        #     name="Smart_Mat_Table",
        #     pose=[-3.0, -0.1, -0.039, Smart_Mat_Table_Quat[0], 
        #                               Smart_Mat_Table_Quat[1],
        #                               Smart_Mat_Table_Quat[2],
        #                               Smart_Mat_Table_Quat[3]],
        #     file_path= cur_dir + "smart_table/Smart_Mat_Supply.stl",
        #     scale=[0.001, 0.001, 0.001],
        # )


        
        # Excluded models : Conveyors : NewConvn1_V2, NewConvn2_V2, MatTable_3Level, MatTable_6Level, MatTable_tilted, NewSheathingRack
        world_model = WorldConfig(
            mesh=[NewConvn1_V2, NewConvn2_V2, MatTable_3Level, MatTable_6Level, MatTable_tilted, NewSheathingRack],
            cuboid=[],
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
                 w_dir: str = "/home/apshirazi/Isaac_sim_ws/smart_conveyor",
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
            time.sleep(0.01)
        
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


class CuRoboRobot(object):
    def __init__(self,
                 working_world: WorldManager,
                 R_Name: str = "IRB6620_R1",
                 pose: np.array = np.array([0, 0, 0]),
                 #This Feature has not been added to CuRobo
                 orientation: np.array = np.array([1, 0, 0, 0]),
                 input_tool: str = "tool0",
                 w_dir: str = "/home/apshirazi/Isaac_sim_ws/robot", 
                 r_conf_name: str = "IRB6620_Config.yaml",
                 Gripper_List: Optional[List[RobotGripper]] = None):
        
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
            self._robot_cfg["kinematics"]["extra_collision_spheres"] = {"tool0": 400, "tool1": 2,}
        if self._ROS_JS_robot_indicator == "IRB6620_R2":
            self._robot_cfg["kinematics"]["extra_collision_spheres"] = {"tool0": 100,}

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

        self._articulation_controller: ArticulationController = None

        # Creating and Warming Up the MotionGen
        self._r_conf_name = r_conf_name

        # Warming up Robot 1 (Just for the Case Study)
        if(self._ROS_JS_robot_indicator == "IRB6620_R1"):
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
        # self._js_publisher = rospy.Publisher(self._ros_js_publsiher, sensor_msgs.msg.JointState, queue_size=10)
        self._js_pub_interval: None

        self._computed_path_result: MotionGenResult = None
        self._computed_cmd_plan: JointState = None
        self._computed_idx_list = []

        self._world_updater_counter = 0

        # Deactivating EndEffector's Collision Representation in the Simulation
        # Attaching Surface Grippers to Link 6 rather than Link 7
        # Everything Will Workd ^-^ !!!
        self._EEF_Prim = self._temp_world_manager._stage.GetPrimAtPath("/" + self._ROS_JS_robot_indicator + "/Link_7/collisions")
        self._Collider_Off = self._EEF_Prim.GetAttribute("physics:collisionEnabled").Set(False)


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

    def motion_gen_warmup(self,
                          TCP_Name: str = None):
        # Default Parameters
        trajopt_dt = None
        optimize_dt = False
        trajopt_tsteps = 32
        trim_steps = [1, None]
        max_attempts = 2
        interpolation_dt = 0.05
        enable_finetune_trajopt = False
        # MotionGen StartUp

        # This means that there is no need to change the current TCP Config
        if TCP_Name != None:
            self._robot_cfg["kinematics"]["ee_link"] = TCP_Name
        
        # Creating MotionGenConfig after updating the TCP !
        self._motion_gen_config = MotionGenConfig.load_from_robot_config(
            self._robot_cfg,
            self._temp_world_manager.init_world_model(),
            self._tensor_args,
            collision_checker_type=CollisionCheckerType.MESH,
            num_trajopt_seeds=12,
            num_graph_seeds=12,
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
        self._motion_gen.warmup(enable_graph=False, warmup_js_trajopt=False)
        print("Curobo for robot ( " + self._r_conf_name + " ) is ready ... | TCP = " + self._current_tool)

        self._plan_config = MotionGenPlanConfig(enable_graph=False,
                                                enable_graph_attempt=2,
                                                max_attempts=max_attempts,
                                                enable_finetune_trajopt=enable_finetune_trajopt,
                                                time_dilation_factor=0.5)

    def motion_gen_update_world(self):

        # 1. Get an Update of the Collision World for the Robot:
        # Ignoring Other Robot's Visual Representation
        # This should be updated for the list of robots !!!!
        Rob_name: str= ""
        if self._ROS_JS_robot_indicator == "IRB6620_R1":
            Rob_name = "IRB6620_R2"
        if self._ros_js_publsiher == "IRB6620_R2":
            Rob_name = "IRB6620_R1"
        
        obstacles = self._temp_world_manager._usd_help.get_obstacles_from_stage(
            reference_prim_path=self._robot_prim_path,
            ignore_substring=[
                self._robot_prim_path,
                "/World/defaultGroundPlane",
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
                # Other Robot's Prim Path Should also be Ignored !
                # This feature is to be developed (MPC)
            ],
        ).get_collision_check_world()

        self._motion_gen.update_world(obstacles)
        self._temp_world_manager._my_world.step(render=True)

        print(" Collision World Updated for " + self._ROS_JS_robot_indicator)

    def plan(self,
                        tcp_name: str = "tool1",
                        target_pose: np.array = [0, 0, 0],
                        target_orientation: np.array = [1, 0, 0, 0],
                        update_world_needed: bool = True):
        
        # Updating MotionGenConfig if there is any new TCP Being Used
        if tcp_name != self._robot_cfg["kinematics"]["ee_link"]:
            self.motion_gen_warmup(TCP_Name=tcp_name)

        # A New Approach to Avoid CUDA Memory Occupation:
        if(update_world_needed):
            self.motion_gen_update_world()

        result: MotionGenResult = None
        succ = None
        # Start the timer
        TimeOut_Timer = time.time()

        # Giving a 5-second timer to solve IK
        while (time.time() - TimeOut_Timer <= 100):
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
            ik_goal = Pose(
                position=self._tensor_args.to_device(target_pose),
                quaternion=self._tensor_args.to_device(target_orientation),
            )
            result = self._motion_gen.plan_single(cu_js.unsqueeze(0), ik_goal, self._plan_config)
            succ = result.success.item()

            # Adding the solution to Robot Object
            self._computed_path_result = result

            if succ and np.max(np.abs(sim_js.velocities)) < 0.02:
                print("solution found")
                
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
                return True
            
        carb.log_warn("Plan did not converge to a solution: " + str(result.status))
        # No IK could solve this movement within 5 sec
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

                Active_Tool_Name = self._motion_gen.kinematics.kinematics_config.ee_link
                Tool_Prim_Path = "/"+self._ROS_JS_robot_indicator+"/"+Active_Tool_Name
                #Tool_Prim = self._temp_world_manager._my_world.stage.GetPrimAtPath(Tool_Prim_Path)

                # DYNAMIC INFORMATION TO GET THE POSITION/ORIENTATION

                dc=_dynamic_control.acquire_dynamic_control_interface()
                object=dc.get_rigid_body(Tool_Prim_Path)
                object_pose=dc.get_rigid_body_pose(object)

                print("position:", object_pose.p)
                print("rotation:", object_pose.r)


                # set desired joint angles obtained from IK:
                self._articulation_controller.apply_action(art_action)
                cmd_idx += 1
                if renderInstance:
                    for _ in range(2):
                        self._temp_world_manager._my_world.step(render=False)
        
        # Cleaning out !
        self._computed_path_result = None
        self._computed_cmd_plan = None
        self._computed_idx_list = []

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
                   robot_name: str = "IRB6620_R1",
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
        Mass_Succ = Obj_Prim.GetAttribute("physics:mass").Set(0.0001)

        # Close
        og.Controller.set(og.Controller.attribute("/action_graph_"+robot_name+"_"+tcp_name+"/close_tick.state:enableImpulse"), True)
    
    # Used for Simulation Detach
    def isaac_tcp_detach(self,
                   robot_name: str = "IRB6620_R1",
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
        og.Controller.set(og.Controller.attribute("/action_graph_"+robot_name+"_"+tcp_name+"/open_tick.state:enableImpulse"), True)

        self._temp_world_manager._my_world.step(render= True)       
        # Re-Disabling Gravity For the Object    
        Dis_Grav = Obj_Prim.GetAttribute("physxRigidBody:disableGravity").Set(True)

        self._temp_world_manager._my_world.step(render= True)
        Dis_Collider = Obj_Prim.GetAttribute("physics:collisionEnabled").Set(False)

        self._temp_world_manager._my_world.step(render= True)
        # Disabling Colliders !! (To Avoid Collision Problems After Detaching)
        Dis_RigidBody = Obj_Prim.GetAttribute("physics:rigidBodyEnabled").Set(False)

    def eef_attach(self,
                   r_name: str = "IRB6620_R1",
                   tool_name: str = "tool1",
                   attaching_object_name: str = None,
                   gen_sphere_radius: float = 0.005,
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
        self.isaac_tcp_attach(robot_name= r_name,
                              tcp_name= CV_Tool_name,
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
    
    def eef_detach(self,
                   r_name: str = "IRB6620_R1",
                   tool_name: str = "tool0",
                   detaching_object_name: str = None):
        
        # Detaching the item from the robot in simulation (SurfaceGripper Action Graph)
        # Checking for tool prefix for calling the corresponding action graph e.g. => "tool0" => "T0"
        CV_Tool_name = f"T{tool_name[-1]}"
        self.isaac_tcp_detach(robot_name= r_name,
                              tcp_name= CV_Tool_name,
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
    
    # def ros_js_publisher(self, event):
    #     r=1
    #     try:
    #         # Check if the simulation is running and playing
    #         if self._temp_world_manager._my_world.is_playing() and (self._temp_world_manager._my_world.current_time_step_index > 20):
    #             # self._temp_world_manager._my_world.step(render=False)
    #             sim_js = self._robot.get_joints_state()
    #             sim_js_names = self._robot.dof_names
    #             # Check for NaN values
    #             if np.any(np.isnan(sim_js.positions)):
    #                 log_error("Can't Publish JointState for Robot: " + self._js_working_name)
    #                 return

                # Create and populate the JointState message
                # joint_state_msg = sensor_msgs.msg.JointState()
                # joint_state_msg.header.stamp = rospy.Time.now()
                # joint_state_msg.name = sim_js_names
                # joint_state_msg.position = self._tensor_args.to_device(sim_js.positions).cpu().numpy().tolist()
                # joint_state_msg.velocity = self._tensor_args.to_device(sim_js.velocities).cpu().numpy().tolist()
                # joint_state_msg.effort = [0.0] * len(sim_js_names)

                # Publish the joint state
                # self._js_publisher.publish(joint_state_msg)

        # except Exception as e:
            # rospy.logwarn(f"Error publishing joint state: {e}")



test = WorldManager()

robots = [
    # IRB6620_R1
    CuRoboRobot(working_world=test, 
                R_Name="IRB6620_R1",
                pose=[0,0,0.025],
                input_tool="tool0", 
                w_dir="home/apshirazi/Isaac_sim_ws/robot", 
                r_conf_name="IRB6620_Config.yaml",
                Gripper_List=[RobotGripper(RobName= "IRB6620_R1",
                                           ParentLink= "Link_6",
                                           TCP_Name= "T0",
                                           C_Pose= [0.09 , 0, -0.29]),
                                RobotGripper(RobName= "IRB6620_R1",
                                             ParentLink= "Link_6",
                                             TCP_Name= "T1",
                                             C_Pose= [0.55, 0.435, -0.175])
                                           ]),
    # IRB6620_R2 (Commented)
    CuRoboRobot(working_world=test,
                R_Name="IRB6620_R2",
                pose=[4.6, 0, 0.025],
                input_tool="tool0",
                w_dir="home/apshirazi/Isaac_sim_ws/robot_2",
                r_conf_name="IRB6620_Config.yaml",
                Gripper_List=[RobotGripper(RobName= "IRB6620_R2",
                                           ParentLink= "Link_6",
                                           TCP_Name="T0",
                                           C_Pose=[0.62, 0.15, -0.11]),
                                           ])
    # Add more robots as needed
]

# conveyors = [
#     # Smart Conveyor
#     CuRoboConv(working_world=test,
#                Conv_Name="Smart_Conveyor",
#                pose=[2.3, -3.25, 0],
#                w_dir="/home/apshirazi/Isaac_sim_ws/smart_conveyor",
#                c_conf_name="Smart_Conveyor.yaml")
# ]

def euler_to_quat(roll, pitch, yaw):
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



# Default Assumption (Cuboid), but it can be changed to Capsule and Mesh
def Add_Rigid_Object_To_Scene(World_Manager: WorldManager,
                              ObjectType: str = "Cuboid",
                              obj: Any = Cuboid
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

    print("Object " + obj.name + " Added to the Simulation | PRIM: " + Added_Obj_Prim_Root)

    # Updating the Collision World for Each Robot After Adding an Object to the Scene (Just for Rob 1)
    # for robot in robots:
    #     robot.motion_gen_update_world()
    robots[0].motion_gen_update_world()

def parallel_movement(
    Rob_idx_list: List[int] = [0, 1],
    Goal_List_Pose: List[np.ndarray] = [np.array([-0.49, -1.54, 0.44], dtype=float), np.array([-0.022, -1.85, 0.57], dtype=float)],
    Goal_List_Orientation: List[np.ndarray] = [np.array([1, 0, 0, 0], dtype=float), np.array([1, 0, 0, 0], dtype=float)]
):
    for idx in Rob_idx_list:
        # Ensure each pose and orientation is in the correct format
        target_pose = np.array(Goal_List_Pose[idx][:3], dtype=float).reshape(3)
        target_orientation = np.array(Goal_List_Orientation[idx][:4], dtype=float).reshape(4)

        # Call plan with the correctly shaped pose and orientation
        robots[idx].plan(tcp_name="tool1",
                         target_pose=target_pose,
                         target_orientation= target_orientation)

    cmd_idx = 0
    while cmd_idx < len(robots[Rob_idx_list[0]]._computed_cmd_plan.position):
        test._my_world.step(render=True)
        
        for idx in Rob_idx_list:
            cmd_state = robots[idx]._computed_cmd_plan[cmd_idx]

            # Apply articulation action
            art_action = ArticulationAction(
                cmd_state.position.cpu().numpy(),
                cmd_state.velocity.cpu().numpy(),
                joint_indices=robots[idx]._computed_idx_list,
            )
            robots[idx]._articulation_controller.apply_action(art_action)
            for _ in range(2):
                test._my_world.step(render=False)
        
        cmd_idx += 1

# Timer to check for updating the 
cc_world_step_updater = 200

def mpc_movement(
    Rob_idx_list: List[int] = [0, 1],
    Goal_List_Pose: List[np.ndarray] = [np.array([-0.49, -1.54, 0.44], dtype=float), np.array([-0.022, -1.85, 0.57], dtype=float)],
    Goal_List_Orientation: List[np.ndarray] = [np.array([1, 0, 0, 0], dtype=float), np.array([1, 0, 0, 0], dtype=float)]
):
    while 1>0:
        for idx in Rob_idx_list:
            # Ensure each pose and orientation is in the correct format
            target_pose = np.array(Goal_List_Pose[idx][:3], dtype=float).reshape(3)
            target_orientation = np.array(Goal_List_Orientation[idx][:4], dtype=float).reshape(4)

            # Call plan with the correctly shaped pose and orientation
            robots[idx].plan(tcp_name="tool1",
                            target_pose=target_pose,
                            target_orientation= target_orientation)

        Traj_Ending_Flag = False
        cmd_idx = 0
        Starting_Step = test._my_world.current_time_step_index
        while cmd_idx < len(robots[Rob_idx_list[0]]._computed_cmd_plan.position):
            test._my_world.step(render=True)
            if(test._my_world.current_time_step_index - Starting_Step >= cc_world_step_updater):
                break
            
            for idx in Rob_idx_list:
                cmd_state = robots[idx]._computed_cmd_plan[cmd_idx]

                # Apply articulation action
                art_action = ArticulationAction(
                    cmd_state.position.cpu().numpy(),
                    cmd_state.velocity.cpu().numpy(),
                    joint_indices=robots[idx]._computed_idx_list,
                )
                robots[idx]._articulation_controller.apply_action(art_action)
                for _ in range(2):
                    test._my_world.step(render=False)
            
            cmd_idx += 1
            if cmd_idx == len(robots[Rob_idx_list[0]]._computed_cmd_plan.position):
                Traj_Ending_Flag = True
        if Traj_Ending_Flag == True:
            print("Trajectory Done")
            break
        

def main():

    # rospy.init_node("tutorial_subscriber", anonymous=True)

    i=0

    # for robot in robots:
        # robot._js_pub_interval = rospy.Timer(rospy.Duration(10.0 / publish_rate), robot.ros_js_publisher)

    # SheathingPlate = Cuboid(
    #     name="SheathingPlate",
    #     pose=[-1.8, -2.1, 0.37, 1, 0, 0, 0],
    #     dims=[3, 1.5, 0.015],
    #     color=[0.87, 0.72, 0.53, 1]
    # )

    # Creating Surf Gripper Test Object
    # Test_Obj = Cuboid(
    #     name="SG_Tester",
    #     pose=[2.16, -2.5, 1.3, 1.0, 0, 0, 0],
    #     dims=[0.05, 2.0, 0.1]
    # )
    # Add_Rigid_Object_To_Scene(test, "Cuboid", Test_Obj)

    # Add_Rigid_Object_To_Scene(test, "Cuboid", SheathingPlate)

    # Adding Test Sheath
    # Sheathing_Plate = Cuboid(
    #     name="SP",
    #     pose=[-1.4, -2.0, 0.4, 1, 0, 0, 0],
    #     dims=[2.0, 1.0, 0.02]
    # )
    # Add_Rigid_Object_To_Scene(test, "Cuboid", Sheathing_Plate)

    # # Adding Test Stud
    # Stud = Cuboid(
    #     name= "stud_test",
    #     pose= [1.46, 0.14, 1.59, 1, 0, 0, 0],
    #     dims= [0.03, 1.5, 0.1]
    # )
    # Add_Rigid_Object_To_Scene(test, "Cuboid", Stud)

    # #Adding Rob2 Test Stud
    # L_Stud = Cuboid(
    #     name="L_Stud",
    #     pose=[2.6, 0.97, 1.77, 1, 0, 0, 0],
    #     dims= [0.1, 3.0, 0.05]
    # )
    # Add_Rigid_Object_To_Scene(test, "Cuboid", L_Stud)

# Adding the Long Stud !!!
    Long_Stud_Quat = euler_to_quat(0, -23*np.pi/60, 0)
    Long_Stud = Cuboid (
        name="Long_Stud",
        pose=[-1.59, 3.0, 0.775, Long_Stud_Quat[3], Long_Stud_Quat[0], Long_Stud_Quat[1], Long_Stud_Quat[2]],
        dims=[0.0331, 6.091, 0.1347],
        color=[0.87, 0.72, 0.53, 1.0]
    )
    Add_Rigid_Object_To_Scene(test, "Cuboid", Long_Stud)

    # Roof = Cuboid (
    #     name="Roof",
    #     pose=[0,0,5.0,1,0,0,0],
    #     dims=[25, 25, 0.1],
    #     color=[1,1,1,0],
    # )
    # Add_Rigid_Object_To_Scene(test, "Cuboid", Roof)

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
        for robot in robots:
            robot.articulation_controller_init(step_index)
        
        # Re initializing conveyors defined in the list
        # for conv in conveyors:
        #     conv.articulation_controller_init(step_index)

        if step_index < 20:
            continue

# Publishing ROS JointState on Movement
        # for robot in robots:
        #         robot.ros_js_publisher()

# Rob 1 To Pick Position
# tensor([[-1.5250,  1.2839,  0.7433]], device='cuda:0')
# (8.474838425910677e-16, -1.2042772722406216, 3.141592653589792)

        R1_Pick_Pose_Quat = euler_to_quat(0, -23*np.pi/60, np.pi)
        robots[0].plan(tcp_name="tool0",
                                    target_pose=np.array([-1.525, 1.05, 0.725]),
                                    target_orientation=np.array([R1_Pick_Pose_Quat[0], R1_Pick_Pose_Quat[1], R1_Pick_Pose_Quat[2], R1_Pick_Pose_Quat[3]]),
                                    update_world_needed=True)
        robots[0].render_exec(renderInstance=True,
                                Show_Sphere=False)
# Rob 1 Attach to Stud

        robots[0].eef_attach(r_name= "IRB6620_R1",
                             tool_name="tool0",
                             attaching_object_name=Long_Stud.name,
                             gen_sphere_radius=0.005,
                             voxelization_method= SphereFitType.SAMPLE_SURFACE)

# Rob 1 Place Position
# tensor([[1.0250, 0.1864, 1.0553]], device='cuda:0')
# (3.141592653589793, 0.0, 0.0)

        R1_Place_Pose_Quat = euler_to_quat(np.pi, 0, 0)
        robots[0].plan(tcp_name="tool0",
                                    target_pose=np.array([1.0250, 0.1864, 1.1]),
                                    target_orientation=np.array([R1_Place_Pose_Quat[0], R1_Place_Pose_Quat[1], R1_Place_Pose_Quat[2], R1_Place_Pose_Quat[3]]),
                                    update_world_needed=True)
        robots[0].render_exec(renderInstance=True,
                                Show_Sphere=False)

if __name__ == "__main__":
    main()

