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

def mpc_movement(
    Rob_idx_list: List[int] = [0, 1],
    Goal_List_Pose: List[np.ndarray] = [np.array([-0.49, -1.54, 0.44], dtype=float), np.array([-0.022, -1.85, 0.57], dtype=float)],
    Goal_List_Orientation: List[np.ndarray] = [np.array([1, 0, 0, 0], dtype=float), np.array([1, 0, 0, 0], dtype=float)]
):
    # Timer to check for updating the 
    cc_world_step_updater = 200
    
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


# Testing MPC Movement (Out Dated as of 12/04/2024)
        # mpc_movement(Goal_List_Orientation=[np.array([quat[0], quat[1], quat[2], quat[3]]),
        #                                          np.array([quat_test[0], quat_test[1], quat_test[2], quat_test[3]])])