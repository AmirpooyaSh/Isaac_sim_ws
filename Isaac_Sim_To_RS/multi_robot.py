import abb_motion_program_exec as abb

# Define the home position (all joint angles set to zero)
home_position = abb.jointtarget([40, 0, 0, 0, 0, 0], [0]*6)

# Define the tool data (same as in the original program)
my_tool = abb.tooldata(True, abb.pose([0, 0, 0.1], [1, 0, 0, 0]), abb.loaddata(0.001, [0, 0, 0.001], [1, 0, 0, 0], 0, 0, 0))

# Create a motion program
mp = abb.MotionProgram(tool=my_tool)
mp.MoveAbsJ(home_position, abb.v1000, abb.z100)
mp.WaitTime(1.0)

mp2 = abb.MotionProgram(tool=my_tool)
mp2.MoveAbsJ(home_position, abb.v1000, abb.fine)
mp2.WaitTime(1.0)

# Print out RAPID module of motion program for debugging
# print(mp.get_program_rapid())

# Execute the motion program on the robot
# Change base_url to the robot IP address
client = abb.MotionProgramExecClient(base_url="http://192.168.137.1")

client.execute_multimove_motion_program([mp, mp2], wait=True)