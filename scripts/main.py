from ap_import_isaac import utils

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

test = utils.WorldManager(simulation_app= simulation_app)

robots = [
    # IRB6620_R1
    utils.CuRoboRobot(working_world=test, 
                R_Name="IRB6620_R1",
                pose=[0,0,0],
                input_tool="tool0", 
                w_dir="home/apshirazi/Isaac_sim_ws/robot", 
                r_conf_name="IRB6620_Config.yaml",
                Gripper_List=[utils.RobotGripper(RobName= "IRB6620_R1",
                                           ParentLink= "Link_6",
                                           TCP_Name= "T0",
                                           C_Pose= [0.09 , 0, -0.29]),
                                utils.RobotGripper(RobName= "IRB6620_R1",
                                             ParentLink= "Link_6",
                                             TCP_Name= "T1",
                                             C_Pose= [0.55, 0.435, -0.175])
                                           ],
                Cuda_Device= 0,
                simulation_app= simulation_app),
    # IRB6620_R2 (Commented)
    utils.CuRoboRobot(working_world=test,
                R_Name="IRB6620_R2",
                pose=[4.6, 0, 0],
                input_tool="tool0",
                w_dir="home/apshirazi/Isaac_sim_ws/robot_2",
                r_conf_name="IRB6620_Config.yaml",
                Gripper_List=[utils.RobotGripper(RobName= "IRB6620_R2",
                                           ParentLink= "Link_6",
                                           TCP_Name="T0",
                                           C_Pose=[0.62, 0.15, -0.11]),
                                           ],
                Cuda_Device= 0, 
                simulation_app= simulation_app)
    # Add more robots as needed
]

conveyors = [
    # Smart Conveyor
    utils.CuRoboConv(working_world=test,
               Conv_Name="Smart_Conveyor",
               pose=[2.3, -3.25, 0],
               w_dir="/home/apshirazi/Isaac_sim_ws/smart_conveyor",
               c_conf_name="Smart_Conveyor.yaml")
]

def main():

    # rospy.init_node("tutorial_subscriber", anonymous=True)

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
        for robot in robots:
            robot.articulation_controller_init(step_index)
        
        # Re initializing conveyors defined in the list
        for conv in conveyors:
            conv.articulation_controller_init(step_index)

        if step_index < 20:
            continue

# Publishing ROS JointState on Movement
        # for robot in robots:
        #         robot.ros_js_publisher()

#         T_Now = time.time()
#         while time.time() - T_Now < 200:
#             test._my_world.step(render=True) 

        robots[0].free_TCP_movement()

if __name__ == "__main__":
    main()
