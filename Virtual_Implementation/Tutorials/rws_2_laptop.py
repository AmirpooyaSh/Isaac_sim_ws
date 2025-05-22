import abb_motion_program_exec as abb
import time


UNIT_NAME: str = "Local_IO_IDC_1"
def Rob_1_Close_GR (client: abb.MotionProgramExecClient = None):

    # Testing Gripper Close Signal For Rob 1
    time.sleep(0.5)
    
    # Sending Close Signal = 1
    client.abb_client.set_digital_io(signal="EF2_GRP_CL", value= True, unit= UNIT_NAME)
    
    # Sending Open Signal = 0
    client.abb_client.set_digital_io(signal="EF2_GRP_OP", value= False, unit= UNIT_NAME)
    time.sleep(0.3)

# Define the home position (all joint angles set to zero)
home_position = abb.jointtarget([30, 0, 0, 0, 0, 0], [0]*6)

# Define the tool data (same as in the original program)
my_tool = abb.tooldata(True, abb.pose([0, 0, 0.1], [1, 0, 0, 0]), abb.loaddata(0.001, [0, 0, 0.001], [1, 0, 0, 0], 0, 0, 0))

# Create a motion program
mp = abb.MotionProgram(tool=my_tool)
mp.MoveAbsJ(home_position, abb.v1000, abb.fine)
mp.WaitTime(1.0)

# Print out RAPID module of motion program for debugging
# print(mp.get_program_rapid())

# Execute the motion program on the robot
# Change base_url to the robot IP address
client = abb.MotionProgramExecClient(base_url="http://192.168.137.1")

# motoroff
# client.abb_client.set_controller_state("motoron")

# Rob_1_Close_GR(client)

log_results = client.execute_motion_program(mp, task= "T_ROB1", wait= True)

# while client.is_motion_program_running():
#     print(client.abb_client.get_jointtarget("ROB_1"))

# client.abb_client.set_controller_state("motoroff")



