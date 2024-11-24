from isaacsim import SimulationApp
# import sensor_msgs.msg

simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

from omni.isaac.core import World
import numpy as np
import math

# Adding mesh to the world (Standalone Format)
from omni.physx.scripts import utils
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot

# This is a CuRobo Library that Converts world_model to USD format for Isaac Sim
from curobo.util.usd_helper import UsdHelper
# This is a CuRobo Library to import Different Object Types to the world model
from curobo.geom.types import WorldConfig, Mesh, Cuboid
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

        # MatTable_3Level = Mesh(
        #     name="MatTable_3Level",
        #     pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        #     file_path= cur_dir + "meshes/MatTable_3Level.stl",
        #     scale=[0.001, 0.001, 0.001],
        # )
        # MatTable_6Level = Mesh(
        #     name="MatTable_6Level",
        #     pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        #     file_path= cur_dir + "meshes/MatTable_6Level.stl",
        #     scale=[0.001, 0.001, 0.001],
        # )
        # MatTable_tilted = Mesh(
        #     name="MatTable_tilted",
        #     pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        #     file_path= cur_dir + "meshes/MatTable_tilted.stl",
        #     scale=[0.001, 0.001, 0.001],
        # )
        # NewConvn1_V2 = Mesh(
        #     name="NewConvn1_V2",
        #     pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        #     file_path= cur_dir + "meshes/NewConvn1_V2.stl",
        #     scale=[0.001, 0.001, 0.001],
        # )
        # NewConvn2_V2 = Mesh(
        #     name="NewConvn2_V2",
        #     pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        #     file_path= cur_dir + "meshes/NewConvn2_V2.stl",
        #     scale=[0.001, 0.001, 0.001],
        # )
        # NewSheathingRack = Mesh(
        #     name="NewSheathingRack",
        #     pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        #     file_path= cur_dir + "meshes/NewSheathingRack.stl",
        #     scale=[0.001, 0.001, 0.001],
        # )

        # SheathingPlate = Cuboid(
        #     name="SheathingPlate",
        #     pose=[-1.8, -2.1, 0.37, 0, 0, 0, 1],
        #     dims=[3, 1.5, 0.015],
        #     color=[0.87, 0.72, 0.53, 1]
        # )

        # Smart Material Table
        Smart_Mat_Table_Quat = self.quat_transfer_world_generator(90, 0, 180)

        Smart_Mat_Table = Mesh(
            name="Smart_Mat_Table",
            pose=[-3.0, -0.1, -0.039, Smart_Mat_Table_Quat[0], 
                                      Smart_Mat_Table_Quat[1],
                                      Smart_Mat_Table_Quat[2],
                                      Smart_Mat_Table_Quat[3]],
            file_path= cur_dir + "smart_table/Smart_Mat_Supply.stl",
            scale=[0.001, 0.001, 0.001],
        )


        # Excluded models : Conveyors : NewConvn1_V2, NewConvn2_V2, MatTable_3Level, MatTable_6Level, MatTable_tilted, NewSheathingRack
        world_model = WorldConfig(
            mesh=[Smart_Mat_Table],
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

        # # Saving the Updated World !
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
                        ("surfX.inputs:DisableGravity", True),
                        ("surfY.inputs:DisableGravity", True),
                        ("surfZ.inputs:DisableGravity", True),
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

        # 11/17/2024 Commented for Test (Shouldn't Be)
        # self.motion_gen_warmup()

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
            # 1. Get an Update of the Collision World for the Robot:
            print("Updating world, reading w.r.t.", self._robot_prim_path)
            obstacles = self._temp_world_manager._usd_help.get_obstacles_from_stage(
                reference_prim_path=self._robot_prim_path,
                ignore_substring=[
                    self._robot_prim_path,
                    "/World/defaultGroundPlane",
                    # Other Robot's Prim Path Should also be Ignored !
                    # This feature is to be developed (MPC)
                ],
            ).get_collision_check_world()

            # Saving the Updated World !
            # file_path = "World_For_" + self._ROS_JS_robot_indicator + "_Step_" + str(self._world_updater_counter) + ".obj"
            # obstacles.save_world_as_mesh(file_path)
            # self._world_updater_counter += 1

            self._motion_gen.update_world(obstacles)
            print("Updated World")

            self._temp_world_manager._my_world.step(render=True)
            
            carb.log_info("Synced CuRobo world from stage for Robot : " + self._r_conf_name)

        result: MotionGenResult = None
        succ = None
        # Start the timer
        TimeOut_Timer = time.time()

        # Giving a 5-second timer to solve IK
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
                break

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
        else:
            carb.log_warn("Plan did not converge to a solution: " + str(result.status))
            # No IK could solve this movement within 5 sec
            return False

    def render_exec(self,
                    renderInstance: bool = True):

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

                cmd_state = cmd_plan[cmd_idx]

                # get full Truedof state
                art_action = ArticulationAction(
                    cmd_state.position.cpu().numpy(),
                    cmd_state.velocity.cpu().numpy(),
                    joint_indices=idx_list,
                )

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

    # Used for Simulation Attach
    def isaac_tcp_attach(self,
                   robot_name: str = "IRB6620_R1",
                   tcp_name: str = "T1"):
        # Close
        og.Controller.set(og.Controller.attribute("/action_graph_"+robot_name+"_"+tcp_name+"/close_tick.state:enableImpulse"), True)
    
    # Used for Simulation Detach
    def isaac_tcp_detach(self,
                   robot_name: str = "IRB6620_R1",
                   tcp_name: str = "T1"):
        # Open
        og.Controller.set(og.Controller.attribute("/action_graph_"+robot_name+"_"+tcp_name+"/open_tick.state:enableImpulse"), True)

    def eef_attach(self,
                   r_name: str = "IRB6620_R1",
                   tool_name: str = "tool0",
                   attaching_object_name: str = None):
        if(attaching_object_name == None):
            print("There should be a valid Object_Name => Attaching Failed")
            return False

        # Attaching the object withing the simulation to the virtual link (SurfaceGripper Attach)
        CV_Tool_name = f"T{tool_name[-1]}"
        self.isaac_tcp_attach(robot_name= r_name,
                              tcp_name= CV_Tool_name)
        
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
        cu_js = cu_js.get_ordered_joint_state(self._motion_gen.kinematics.joint_names)

        # world_objects_pose_offset: Optional[Pose] = None,\
        # World Object Pose Offset is a .tensor variable to suite the Pick/Place
        # It should be read through the documentations before using it !!!

        self._motion_gen.attach_objects_to_robot(
            joint_state=cu_js,
            object_names=[attaching_object_name],
            surface_sphere_radius= 0.001,
            link_name= tool_name,
            sphere_fit_type= SphereFitType.VOXEL_VOLUME,
        )
    
    def eef_detach(self,
                   r_name: str = "IRB6620_R1",
                   tool_name: str = "tool0"):
        
        # Detaching the item from the robot in simulation (SurfaceGripper Action Graph)
        # Checking for tool prefix for calling the corresponding action graph e.g. => "tool0" => "T0"
        CV_Tool_name = f"T{tool_name[-1]}"
        self.isaac_tcp_detach(robot_name= r_name,
                              tcp_name= CV_Tool_name)
        

        # Detaching the item from the actual MotionGen object (in CuRobo)
        # It basically removes the generated sepheres attached to the virtual link (tool0 for example)
        self._motion_gen.detach_object_from_robot(tool_name)
     
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
                input_tool="tool1", 
                w_dir="home/apshirazi/Isaac_sim_ws/robot", 
                r_conf_name="IRB6620_Config.yaml",
                Gripper_List=[RobotGripper(RobName= "IRB6620_R1",
                                           ParentLink= "Link_7",
                                           TCP_Name= "T1",
                                           C_Pose= [0.175 , 0.43, 0.52]),
                                RobotGripper(RobName= "IRB6620_R1",
                                             ParentLink= "Link_7",
                                             TCP_Name= "T0",
                                             C_Pose= [0.28, 0.3, 0.035])
                                           ]),
    # IRB6620_R2
    CuRoboRobot(working_world=test,
                R_Name="IRB6620_R2",
                pose=[4.6, 0, 0.025],
                input_tool="tool1",
                w_dir="home/apshirazi/Isaac_sim_ws/robot_2",
                r_conf_name="IRB6620_Config.yaml",
                Gripper_List=[RobotGripper(RobName= "IRB6620_R2",
                                           ParentLink= "Link_7",
                                           TCP_Name="T0",
                                           C_Pose=[0.11, -0.45, -0.57]),
                                           ])
    # Add more robots as needed
]

conveyors = [
    # Smart Conveyor
    CuRoboConv(working_world=test,
               Conv_Name="Smart_Conveyor",
               pose=[2.3, -3.25, 0],
               w_dir="/home/apshirazi/Isaac_sim_ws/smart_conveyor",
               c_conf_name="Smart_Conveyor.yaml")
]

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

    Obj_Prim = stage.GetPrimAtPath(Added_Obj_Prim_Root)
    # Adding RigidBody
    execute("AddPhysicsComponentCommand",
                          usd_prim=Obj_Prim,
                          component="PhysicsRigidBodyAPI")
    # Adding Colliders
    execute("AddPhysicsComponentCommand",
                          usd_prim=Obj_Prim,
                          component="PhysicsCollisionAPI")
    # Adding Colliders
    execute("AddPhysicsComponentCommand",
                          usd_prim=Obj_Prim,
                          component="PhysicsMassAPI")
    # Adding a Small Positive Mass to Avoid Robot's Effort Calculation using CuRobo
    Mass_Succ = Obj_Prim.GetAttribute("physics:mass").Set(0.0001)
    # Disabling Gravity
    # Dis_Grav = Obj_Prim.GetAttribute("physxRigidBody:disableGravity").Set(True)

    print("For Obj (" + obj.name + ") Mass Creation : " + str(Mass_Succ))
    # print("For Obj (" + obj.name + ") DisableGravity Creation : " + str(Dis_Grav))

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
        for conv in conveyors:
            conv.articulation_controller_init(step_index)

        if step_index < 20:
            continue

        # Movement Publish !
        # for robot in robots:
        #         robot.ros_js_publisher()

        quat = euler_to_quat(0, 0, -np.pi)
        quat_test= euler_to_quat(np.pi, 0, 0)
        quat_2= euler_to_quat(np.pi/2, 0, np.pi)


        # robots[0].plan(target_pose=np.array([-0.49, -1.54, 0.44]),
        #                         target_orientation=np.array([quat[0], quat[1], quat[2], quat[3]]),
        #                         update_world_needed=True)
        
        # robots[1].plan(target_pose=np.array([-0.022, -1.85, 0.57]),
        #                 target_orientation=np.array([quat_test[0], quat_test[1], quat_test[2], quat_test[3]]))
        
        # robots[0].render_exec(renderInstance=True)
        # robots[1].render_exec(renderInstance=True)
        

        # robots[0].plan(target_pose=np.array([0, 1.5, 0.08]),
        #                         target_orientation=np.array([quat_2[0], quat_2[1], quat_2[2], quat_2[3]]),
        #                         update_world_needed=False)
        # robots[0].render_exec(renderInstance=True)
        
        # robots[0].render_exec(renderInstance=True)

        # mpc_movement(Goal_List_Orientation=[np.array([quat[0], quat[1], quat[2], quat[3]]),
        #                                          np.array([quat_test[0], quat_test[1], quat_test[2], quat_test[3]])])


        # 5 Sec of sleep to enable reinitialization
        # rospy.sleep(5)
        # rospy.spin()

        # obstacles = test._usd_help.get_obstacles_from_stage().get_collision_check_world()

        # # Saving the Updated World !
        # file_path = "World_For.obj"
        # obstacles.save_world_as_mesh(file_path)

        # conveyors[0].render_exec('Joint_1', 3.0)
        # conveyors[0].render_exec('Joint_2', 0.2)
        # conveyors[0].render_exec('Joint_2', 0.0)
        # conveyors[0].render_exec('Joint_1', 0.0)

if __name__ == "__main__":
    main()

