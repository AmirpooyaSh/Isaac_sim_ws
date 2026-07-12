# EV NUMBER FOR QUATERNION ORIENTATION
ev = np.sqrt(2) / 2

def Pick_Long_Element_From_Mat_Supply(
    el_name: str = None,
    L: float = None,
    W: float = None,
    H: float = None,
):
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
            6.1501,
            1.298421,
            0.803 + (W / 2),
        ],
        target_orientation=[0, ev, 0, ev],
        update_world_needed=True,
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 3: Pick the element
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[
            6.4501,
            1.298421,
            0.803 + (W / 2),
        ],
        target_orientation=[0, ev, 0, ev],
        update_world_needed=True,
        removing_primitives=["world/obstacles"],
        orientational_restriction=torch.tensor(
            [1, 1, 1],
            dtype=torch.float32,
        ),
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 4: Create and attach the picked element
    Create_Wooden_Element_For_Smart_Mat_Table(
        el_name=el_name,
        L=L,
        W=W,
        H=H,
    )
    Robot_2.eef_attach(
        tool_name="tool0",
        attaching_object_name=el_name,
    )
    print("Wooden Element Attached to Robot_2")

    # Stage 5: Retract slightly in X
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[
            6.5501,
            1.298421,
            0.803 + (W / 2),
        ],
        target_orientation=[0, ev, 0, ev],
        update_world_needed=True,
        removing_primitives=["world/obstacles"],
        orientational_restriction=torch.tensor(
            [1, 1, 1],
            dtype=torch.float32,
        ),
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 6: Retract slightly upward in Z
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[
            6.5501,
            1.298421,
            1.003 + (W / 2),
        ],
        target_orientation=[0, ev, 0, ev],
        update_world_needed=True,
        removing_primitives=["world/obstacles"],
        orientational_restriction=torch.tensor(
            [1, 1, 1],
            dtype=torch.float32,
        ),
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 7: Final post-pick retract
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[
            6.1501,
            1.298421,
            1.003 + (W / 2),
        ],
        target_orientation=[0, ev, 0, ev],
        update_world_needed=True,
        removing_primitives=["world/obstacles"],
        orientational_restriction=torch.tensor(
            [1, 1, 1],
            dtype=torch.float32,
        ),
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
    Smart_Conv.render_exec('Joint_1', 4.55)

    # Stage 3: Robot_2 positions element for handoff
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[
            L / 2 + 1.954979,
            -1,
            H + 0.85,
        ],
        target_orientation=[0, ev, ev, 0],
        update_world_needed=True,
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 4: Robot_1 moves in to receive element
    Robot_1.plan(
        tcp_name="tool0",
        target_pose=[
            -(L / 2) + 2.650101,
            -1,
            H + 0.95,
        ],
        target_orientation=[0, ev, -ev, 0],
        update_world_needed=True,
    )
    Robot_1.render_exec(renderInstance=True, Show_Sphere=False)

    Robot_1.plan(
        tcp_name="tool0",
        target_pose=[
            -(L / 2) + 2.650101,
            -1,
            H + 0.85,
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
            L / 2 + 1.954979,
            -1,
            H + 0.95,
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
    # Stage 1: Align smart conveyor
    Smart_Conv.render_exec(
        'Joint_1',
        Y + L / 2 + 0.096099,
    )

    # Stage 2: Robot_1 pre-place pose
    Robot_1.plan(
        tcp_name="tool0",
        target_pose=[
            X + 1.13259,
            0,
            H + 1.08936,
        ],
        target_orientation=[0, 1, 0, 0],
        update_world_needed=True,
    )
    Robot_1.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 3: Robot_1 place motion
    Robot_1.plan(
        tcp_name="tool0",
        target_pose=[
            X + 1.13259,
            0,
            H + 0.88936,
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
            X + 1.13259,
            0,
            H + 1.08936,
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
    # Move Conveyor To TCP's 0 Position For Placement
    Smart_Conv.render_exec('Joint_1', Y + L / 2 + 0.101179)

    # Robot 2 Pre Place Movement
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [X + 1.33259,
                                  0,
                                  H + 1.08936],
                    target_orientation= [0, 1, 0, 0],
                    update_world_needed= True)
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)
    
    # Robot 1 Place Movement
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [X + 1.13259,
                                  0,
                                  H + 0.88936],
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
                    target_pose= [X + 1.13259,
                                  0,
                                  H + 1.08936],
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
    # Compute stud center pose on the sloped table
    diagonal_stud_l = np.sqrt(W ** 2 + H ** 2)
    angle = 0.523598775598
    half_diagonal = diagonal_stud_l / 2
    z_increase = half_diagonal * np.sin(angle + np.arcsin(W / 2 / half_diagonal))
    y_increase = half_diagonal * np.cos(angle + np.arcsin(W / 2 / half_diagonal))
    Stud_Pose = [L / 2 + 4.009, -y_increase - 2.417, z_increase + 1.045]

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
            4.474021,
            Stud_Pose[1] + 0.2,
            Stud_Pose[2] - 0.2 * np.tan(angle),
        ],
        target_orientation=[quat[3], quat[0], quat[1], quat[2]],
        update_world_needed=True,
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 2: Pick the stud
    pick_val = H / 2 - 0.0061
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[
            4.474021,
            Stud_Pose[1] + pick_val * np.cos(angle),
            Stud_Pose[2] - pick_val * np.sin(angle),
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
            4.474021,
            Stud_Pose[1] + pick_val * np.cos(angle),
            Stud_Pose[2] - pick_val * np.sin(angle) + 0.4,
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
    # Stage 1: Determine conveyor side for placement/nailing
    half_length = 1.8288
    mid_range = 2.275
    target_offset = half_length - Y + mid_range + 0.5
    if target_offset > SMART_CONV_RANGE_OF_MOTION_J1:
        Side_Selector = -1
    else:
        Side_Selector = 1

    # Stage 2: Move conveyor and record nail poses
    conv_pose = half_length - Y + mid_range + 0.5 * Side_Selector
    Smart_Conv.render_exec('Joint_1', conv_pose)
    Smart_Conv._nail_poses.append((conv_pose, Side_Selector))
    Smart_Conv._vertical_nail_poses.append(half_length - Y + mid_range)

    # Stage 3: Robot_2 pre-place approach
    pre_place_x = (
        X + L / 2 + 0.867569
    )
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[
            pre_place_x,
            0.5 * Side_Selector,
            H + 0.98936,
        ],
        target_orientation=[0, ev, ev, 0],
        update_world_needed=True,
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 4: Robot_2 place and hold
    place_x = (
        X + L / 2 + 0.667569
    )
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[
            place_x,
            0.5 * Side_Selector,
            H + 0.88936,
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
    push_to_nail: float = 0.05,
    L: float = 2.4384,
    H: float = None,
    Side_Selector: int = 0,
    Is_Held: bool = False,
):
    # Stage 1: Compute nailing Y coordinate
    if Side_Selector == 1 or Side_Selector == -1:
        proceeding_y = 0.5 * Side_Selector
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
                H - 0.7 * H + 0.89546,
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
                pose.p[2] + 0.4 * H,
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
            X + L / 2 + 0.867569
        )
        Robot_2.plan(
            tcp_name="tool0",
            target_pose=[
                post_x,
                0.5 * Side_Selector,
                H + 0.98936,
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
    # Pre Pick
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [6.1501,
                                  0.487449,
                                  W / 2 + 0.803],
                    target_orientation= [0, -ev, 0, -ev],
                    update_world_needed= True)
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)
    
    # Pick
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [6.4501,
                                  0.487449,
                                  W / 2 + 0.803],
                    target_orientation= [0, -ev, 0, -ev],
                    update_world_needed= True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles"],
                    orientational_restriction=torch.tensor([1,1,1], dtype=torch.float32))
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)
    
    # Saw Action + Attach
    # Removing 12ft Primitive and Replacing it with L !
    # prims_utils.delete_prim("/world/obstacles/"+el_name+"Temp")
    Create_Wooden_Element_For_Smart_Mat_Table(el_name= el_name, L= L, W= W, H= H, Debug_Offset= True)

    Robot_2.eef_attach(tool_name= "tool0", attaching_object_name= el_name)
    #.....

    # Post Pick
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [6.1501,
                                  0.487449,
                                  W / 2 + 0.803],
                    target_orientation= [0, -ev, 0, -ev],
                    update_world_needed= True,
                    removing_primitives=["Smart_Conveyor", "world/obstacles"],
                    direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)
    
    # Home
    Robot_2.move_to_home()

def Drop_Short_Vertical_Element_With_Tangent_to_an_Element(
    el_name: str = None,
    X: float = None,
    Y: float = None,
    L: float = None,
    H: float = None,
    If_Tangent_From_Left: bool = False,
) -> int:
    # Determine conveyor side and jack offset based on tangency
    half_length = 1.8288
    mid_range = 2.275
    if If_Tangent_From_Left:
        Side_Selector = -1
        Jack_Y_Placement_Offset = 0.05
        if half_length - Y + mid_range + NAILING_CONV_TARGET < 0:
            Side_Selector = 1
    else:
        Side_Selector = 1
        Jack_Y_Placement_Offset = -0.05
        if half_length - Y + mid_range + NAILING_CONV_TARGET > SMART_CONV_RANGE_OF_MOTION_J1:
            Side_Selector = -1

    # Move conveyor into position and record nail pose
    conv_pose = half_length - Y + mid_range + 0.5 * Side_Selector
    Smart_Conv.render_exec('Joint_1', conv_pose)
    Smart_Conv._nail_poses.append((conv_pose, Side_Selector))

    # Stage 1: Pre-place approach
    pre_place_x = (
        X + L / 2 + 0.887569
    )
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[
            pre_place_x,
            0.5 * Side_Selector + Jack_Y_Placement_Offset,
            H + 0.93936,
        ],
        target_orientation=[0, -ev, ev, 0],
        update_world_needed=True,
    )
    Robot_2.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 2: Place and drop
    place_x = pre_place_x - 0.1
    Robot_2.plan(
        tcp_name="tool0",
        target_pose=[
            place_x,
            0.5 * Side_Selector,
            H + 0.93936,
        ],
        target_orientation=[0, -ev, ev, 0],
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
    push_to_nail: float = 0.01,
    el_pose: List[float] = None,
    el_dims: List[float] = None,
    If_Tangent_From_Left: bool = False,
    Side_Selector: int = 0,
) -> None:
    # Stage 1: Determine jack side multiplier
    side_mult = -1 if If_Tangent_From_Left else 1

    # Stage 2: Proceed only if conveyor side is valid
    if Side_Selector in (-1, 1):
        # Compute orientation quaternion for angled nailing
        base_angle = 1.570796326795
        nail_angle = 0.523598775598
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
            el_pose[0] - el_dims[0] / 2 + 1.19355
        )
        pre_y = (0.5 * Side_Selector + 0.1 * np.cos(nail_angle) * -side_mult)
        pre_z = (el_dims[2] * 0.3 + 0.1 * np.sin(nail_angle) + 0.89546)
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
            offset = -(el_dims[1] / 2 / np.cos(nail_angle)) + push_to_nail + 0.1
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
                target_pose=[pose.p[0], pose.p[1], pose.p[2] + 0.4 * el_dims[2]],
                target_orientation=[pose.r[3], pose.r[0], pose.r[1], pose.r[2]],
                update_world_needed=True,
                removing_primitives=["Smart_Conveyor", "world/obstacles"],
                direct_pose_cost=PoseCostMetric.create_grasp_approach_metric(
                    offset_position=0.0, tstep_fraction=0.001, linear_axis=0
                ),
            )
            Robot_1.render_exec(renderInstance=True, Show_Sphere=False)

        # Stage 5: Return Robot_1 and Robot_2 home
        Robot_1.move_to_home(removing_primitives= ["Smart_Conveyor", "world/obstacles"])
        Robot_2.move_to_home(removing_primitives= ["Smart_Conveyor", "world/obstacles"])
    else:
        print("Error: Side_Selector must be +1 or -1.")

def Pass_Short_Element_G2G(
    el_name: str = None,
    H: float = None,
):
    PASSING_LOC = [2.05, 0, 1.55]
    half_grip = 0.295021
    delta = -(H / 2) + 0.0061
    lateral = -0.450101

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
    # Stage 1: Align smart conveyor
    Smart_Conv.render_exec(
        'Joint_1',
        -(Y - 1.8288) + L / 2 + 1.774899,
    )

    # Stage 2: Compute conveyor zero pose
    Conv_Curr_Loc = (
        -(Y - 1.8288) + 2.275
    )

    # Stage 3: Robot_1 pre-place pose (above conveyor)
    Robot_1.plan(
        tcp_name="tool0",
        target_pose=[
            X + 1.13259,
            0,
            H + 1.18936,
        ],
        target_orientation=[0, 1, 0, 0],
        update_world_needed=True,
    )
    Robot_1.render_exec(renderInstance=True, Show_Sphere=False)

    # Stage 4: Robot_1 place motion
    Robot_1.plan(
        tcp_name="tool0",
        target_pose=[
            X + 1.13259,
            0,
            H + 0.98936,
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
    push_to_nail: float = 0.01,
    el_pose: List[float] = None,
    el_dims: List[float] = None,
    conv_current_location: float = None,
) -> None:
    quat = R.from_euler('xyz', [((np.pi/2)+np.radians(SILL_NAILING_ANGLE))*(-1), 0, (np.pi/2)]).as_quat()
    quat[1] *= -1

    # Left Nail
    if(conv_current_location+(el_dims[0]/2)+NAILING_CONV_TARGET > SMART_CONV_RANGE_OF_MOTION_J1):
        print("Left Nail is Not Possible ! Due To Conveyor Reachability")
    else:
        Smart_Conv.render_exec('Joint_1', conv_current_location + el_dims[0] / 2 + 0.5)
        # Robot 1 Move For Nail
        # Pre Nail
        Nail_Doable: bool = Robot_1.plan(tcp_name= "tool2",
                        target_pose= [el_pose[0] + 1.13259,
                                    (el_dims[1] / 2 + 0.086602540379) * -1 + 0.5,
                                    el_dims[2] * 0.3 + 0.94546],
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
                                        object_pose.p[1] + (-(el_dims[1] / 2 / 0.866025403785) + push_to_nail + 0.1) * 0.866025403785,
                                        object_pose.p[2] - (-(el_dims[1] / 2 / 0.866025403785) + push_to_nail + 0.1) * 0.5],
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
                                        object_pose.p[1] - (-(el_dims[1] / 2 / 0.866025403785) + push_to_nail + 0.1) * 0.866025403785,
                                        object_pose.p[2] + (-(el_dims[1] / 2 / 0.866025403785) + push_to_nail + 0.1) * 0.5],
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
                                        object_pose.p[2] + 0.4 * el_dims[2]],
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
                                        object_pose.p[1] + (-(el_dims[1] / 2 / 0.866025403785) + push_to_nail + 0.1) * 0.866025403785,
                                        object_pose.p[2] - (-(el_dims[1] / 2 / 0.866025403785) + push_to_nail + 0.1) * 0.5],
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
                                        object_pose.p[1] - (-(el_dims[1] / 2 / 0.866025403785) + push_to_nail + 0.1) * 0.866025403785,
                                        object_pose.p[2] + (-(el_dims[1] / 2 / 0.866025403785) + push_to_nail + 0.1) * 0.5],
                            target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                            update_world_needed= True,
                            removing_primitives=["Smart_Conveyor", "world/obstacles"],
                            direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
            Robot_1.render_exec(renderInstance= True,
                                    Show_Sphere= False)

            Robot_1.move_to_home(removing_primitives= ["Smart_Conveyor", "world/obstacles"])
        else:
            print("Left Nail is Not Possible ! Due To Robot NailGun Reachability")

    quat = R.from_euler('xyz', [((np.pi/2)+np.radians(SILL_NAILING_ANGLE)), 0, (np.pi/2)*(-1)]).as_quat()
    quat[1] *= -1

    # Right Nail
    if(conv_current_location-(el_dims[0]/2)-NAILING_CONV_TARGET < 0):
        print("Right Nail is Not Possible ! Due To Conveyor Reachability")
    else:
        Smart_Conv.render_exec('Joint_1', conv_current_location - el_dims[0] / 2 - 0.5)

        # Pre Nail
        Nail_Doable: bool = Robot_1.plan(tcp_name= "tool2",
                        target_pose= [el_pose[0] + 1.13259,
                                    el_dims[1] / 2 - 0.413397459621,
                                    el_dims[2] * 0.3 + 0.94546],
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
                                        object_pose.p[1] + -1 * (-(el_dims[1] / 2 / 0.866025403785) + push_to_nail + 0.1) * 0.866025403785,
                                        object_pose.p[2] - (-(el_dims[1] / 2 / 0.866025403785) + push_to_nail + 0.1) * 0.5],
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
                                        object_pose.p[1] - -1 * (-(el_dims[1] / 2 / 0.866025403785) + push_to_nail + 0.1) * 0.866025403785,
                                        object_pose.p[2] + (-(el_dims[1] / 2 / 0.866025403785) + push_to_nail + 0.1) * 0.5],
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
                                        object_pose.p[2] + 0.4 * el_dims[2]],
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
                                        object_pose.p[1] + -1 * (-(el_dims[1] / 2 / 0.866025403785) + push_to_nail + 0.1) * 0.866025403785,
                                        object_pose.p[2] - (-(el_dims[1] / 2 / 0.866025403785) + push_to_nail + 0.1) * 0.5],
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
                                        object_pose.p[1] - -1 * (-(el_dims[1] / 2 / 0.866025403785) + push_to_nail + 0.1) * 0.866025403785,
                                        object_pose.p[2] + (-(el_dims[1] / 2 / 0.866025403785) + push_to_nail + 0.1) * 0.5],
                            target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                            update_world_needed= True,
                            removing_primitives=["Smart_Conveyor", "world/obstacles"],
                            direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
            Robot_1.render_exec(renderInstance= True,
                                    Show_Sphere= False)

            Robot_1.move_to_home(removing_primitives= ["Smart_Conveyor", "world/obstacles"])
        else:
            print("Right Nail is Not Possible ! Due To Robot NailGun Reachability")

def Place_Short_Horizontal_Element_On_Smart_Conveyor_by_Rob2_Gripper(
    el_name: str = None,
    X: float = None,
    Y: float = None,
    L: float = None,
    H: float = None,
) -> float:
    # Moving Conveyor
    Smart_Conv.render_exec('Joint_1', -(Y - 1.8288) - L / 2 + 2.625101)

    # It Calculates the Conveyor Joint Value where the Stud is in 0 Position
    Conv_Curr_Loc: float = -(Y - 1.8288) + 2.275

    # Robot 2 Pre Place Movement
    Examiner: bool = Robot_2.plan(tcp_name= "tool0",
                    target_pose= [X + 1.13259,
                                  0,
                                  H + 1.18936],
                    target_orientation= [0, -1, 0, 0],
                    update_world_needed= True)

    if Examiner == True:
        Robot_2.render_exec(renderInstance= True,
                                Show_Sphere= False)

        # Robot 2 Place Movement
        Robot_2.plan(tcp_name= "tool0",
                        target_pose= [X + 1.13259,
                                    0,
                                    H + 0.98936],
                        target_orientation= [0, -1, 0, 0],
                        update_world_needed= True,
                        removing_primitives=["Smart_Conveyor", "world/obstacles"],
                        direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
        Robot_2.render_exec(renderInstance= True,
                                Show_Sphere= False)

        # Detach
        Robot_2.eef_detach(tool_name="tool0",
                            detaching_object_name= el_name)
        test._stage.GetPrimAtPath("/world/obstacles/" + el_name).GetAttribute("physxRigidBody:disableGravity").Set(False)
        Smart_Conv.attach_object_to_conv(obj_name= el_name)

    else:
        print("Bottom Silling Plate Place location is out of Robot#2 Reach !")
        print("Manual Action Needed to Place the Stud")
        Robot_2.plan(tcp_name= "tool0",
                    target_pose= [X + H / 2 + 1.12649,
                                  0,
                                  H + 1.18936],
                    target_orientation= [0, -ev, 0, ev],
                    update_world_needed= True)
        Robot_2.render_exec(renderInstance= True,
                                Show_Sphere= False)

        Robot_2.eef_detach(tool_name="tool0",
                            detaching_object_name= el_name) 

        # Stud
        # [ev, 0, ev, 0]
        dc=_dynamic_control.acquire_dynamic_control_interface()
        test._stage.GetPrimAtPath("/world/obstacles/"+el_name).GetAttribute("physics:rigidBodyEnabled").Set(True)
        object = dc.get_rigid_body("/world/obstacles/"+el_name)
        object_pose=dc.get_rigid_body_pose(object)

        # Create new position
        New_Loc = Gf.Vec3d(object_pose.p[0], object_pose.p[1], H / 2 + 0.89546)
        # Create a valid quaternion and rotation
        quat = Gf.Quatd(ev, 0, 1, 0)  # Ensure this follows (real, x, y, z)
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

        test._stage.GetPrimAtPath("/world/obstacles/"+el_name).GetAttribute("physics:rigidBodyEnabled").Set(False)
        test._stage.GetPrimAtPath("/world/obstacles/" + el_name).GetAttribute("physxRigidBody:disableGravity").Set(False)
        Smart_Conv.attach_object_to_conv(obj_name= el_name)

    # Back To Home
    Robot_2.move_to_home()

    return Conv_Curr_Loc

def Nail_Short_Horizontal_Element_by_Rob2_NailGun(
    push_to_nail: float = 0.01,
    el_pose: List[float] = None,
    el_dims: List[float] = None,
    conv_current_location: float = None,
) -> None:
    quat = R.from_euler('xyz', [(-(np.pi/2)+np.radians(SILL_NAILING_ANGLE)), 0, np.pi]).as_quat()
    quat[1] *= -1

    # [2.6109899999999997, -0.3533974596215561, 0.97594], Orientation (W,X,Y,Z): [-0.8660254037844386, 0.5000000000000001, 0.0, 0.0]
    # Right Nail
    if(conv_current_location-(el_dims[0]/2)-NAILING_CONV_TARGET < 0):
        print("Right Nail is Not Possible ! Due To Conveyor Reachability")
    else:
        Smart_Conv.render_exec('Joint_1', conv_current_location - el_dims[0] / 2 - 0.5)

        # Pre Nail
        Nail_Doable: bool = Robot_2.plan(tcp_name= "tool1",
                        target_pose= [el_pose[0] + 1.13259,
                                    el_dims[1] * 1.5 - 0.413397459621,
                                    el_dims[2] * 0.3 + 0.94546],
                        target_orientation= [quat[3], quat[0], quat[1], quat[2]],
                        update_world_needed= True)
        
        if(Nail_Doable == True):

            Robot_2.render_exec(renderInstance= True,
                                    Show_Sphere= False)

            dc=_dynamic_control.acquire_dynamic_control_interface()

            # Nail 1
            object=dc.get_rigid_body("/"+Robot_2._ROS_JS_robot_indicator+"/tool1")
            object_pose=dc.get_rigid_body_pose(object)
            Robot_2.plan(tcp_name= "tool1",
                            target_pose= [object_pose.p[0],
                                        object_pose.p[1] + -1 * (-(el_dims[1] / 2 / 0.866025403785) + push_to_nail + 0.1) * 0.866025403785,
                                        object_pose.p[2] - (-(el_dims[1] / 2 / 0.866025403785) + push_to_nail + 0.1) * 0.5],
                            target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                            update_world_needed= True,
                            removing_primitives=["Smart_Conveyor", "world/obstacles"],
                            direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
            Robot_2.render_exec(renderInstance= True,
                                    Show_Sphere= False)

            # Nail 1 Backward
            object=dc.get_rigid_body("/"+Robot_2._ROS_JS_robot_indicator+"/tool1")
            object_pose=dc.get_rigid_body_pose(object)
            Robot_2.plan(tcp_name= "tool1",
                            target_pose= [object_pose.p[0],
                                        object_pose.p[1] - -1 * (-(el_dims[1] / 2 / 0.866025403785) + push_to_nail + 0.1) * 0.866025403785,
                                        object_pose.p[2] + (-(el_dims[1] / 2 / 0.866025403785) + push_to_nail + 0.1) * 0.5],
                            target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                            update_world_needed= True,
                            removing_primitives=["Smart_Conveyor", "world/obstacles"],
                            direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
            Robot_2.render_exec(renderInstance= True,
                                    Show_Sphere= False)

            # Nail 2 Prep
            object=dc.get_rigid_body("/"+Robot_2._ROS_JS_robot_indicator+"/tool1")
            object_pose=dc.get_rigid_body_pose(object)
            Robot_2.plan(tcp_name= "tool1",
                            target_pose= [object_pose.p[0],
                                        object_pose.p[1],
                                        object_pose.p[2] + 0.4 * el_dims[2]],
                            target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                            update_world_needed= True,
                            removing_primitives=["Smart_Conveyor", "world/obstacles"],
                            direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=1))
            Robot_2.render_exec(renderInstance= True,
                                    Show_Sphere= False)
    

            # Nail 2
            object=dc.get_rigid_body("/"+Robot_2._ROS_JS_robot_indicator+"/tool1")
            object_pose=dc.get_rigid_body_pose(object)
            Robot_2.plan(tcp_name= "tool1",
                            target_pose= [object_pose.p[0],
                                        object_pose.p[1] + -1 * (-(el_dims[1] / 2 / 0.866025403785) + push_to_nail + 0.1) * 0.866025403785,
                                        object_pose.p[2] - (-(el_dims[1] / 2 / 0.866025403785) + push_to_nail + 0.1) * 0.5],
                            target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                            update_world_needed= True,
                            removing_primitives=["Smart_Conveyor", "world/obstacles"],
                            direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
            Robot_2.render_exec(renderInstance= True,
                                    Show_Sphere= False)

            # Nail 2 Backward
            object=dc.get_rigid_body("/"+Robot_2._ROS_JS_robot_indicator+"/tool1")
            object_pose=dc.get_rigid_body_pose(object)
            Robot_2.plan(tcp_name= "tool1",
                            target_pose= [object_pose.p[0],
                                        object_pose.p[1] - -1 * (-(el_dims[1] / 2 / 0.866025403785) + push_to_nail + 0.1) * 0.866025403785,
                                        object_pose.p[2] + (-(el_dims[1] / 2 / 0.866025403785) + push_to_nail + 0.1) * 0.5],
                            target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                            update_world_needed= True,
                            removing_primitives=["Smart_Conveyor", "world/obstacles"],
                            direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
            Robot_2.render_exec(renderInstance= True,
                                    Show_Sphere= False)

            Robot_2.move_to_home(removing_primitives= ["Smart_Conveyor", "world/obstacles"])
        else:
            print("Right Nail is Not Possible ! Due To Robot NailGun Reachability")

def Place_Short_Vertical_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name: str = None,
        X: float = None,
        Y: float = None,
        L: float = None,
        H: float = None):
    # Moving Conveyor
    Smart_Conv.render_exec('Joint_1', -(Y - 1.8288) + 2.275)

    # # It Calculates the Conveyor Joint Value where the Stud is in 0 Position
    Conv_Curr_Loc: float = -(Y - 1.8288) + 2.275
    Solver_Flag: bool = False
    Side_Selector: float = 1

    # Check if Positive Value is Suitable for Robot !
    if((-(Y - (OVERALL_PANEL_LENGTH/2)) + (SMART_CONV_RANGE_OF_MOTION_J1/2)+NAILING_CONV_TARGET <= SMART_CONV_RANGE_OF_MOTION_J1)):
        Smart_Conv.render_exec('Joint_1', -(Y - 1.8288) + 2.775)
        # Robot 1 Pre Place
        Robot_1.plan(tcp_name= "tool0",
                        target_pose= [X - -(L / 2) + 0.632489,
                                        0.5,
                                        H + 1.08936],
                        target_orientation= [0, -ev, -ev, 0],
                        update_world_needed= True)
        Robot_1.render_exec(renderInstance= True,
                                Show_Sphere= False)

        # Robot 1 Drop
        Robot_1.plan(tcp_name= "tool0",
                        target_pose= [X - -(L / 2) + 0.632489,
                                        0.5,
                                        H + 0.98936],
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
                        target_pose= [X - -(L / 2) + 0.632489,
                                        0.5,
                                        H + 1.08936],
                        target_orientation= [0, -ev, -ev, 0],
                        update_world_needed= True,
                        removing_primitives=["Smart_Conveyor", "world/obstacles", "IRB6620_R2"],
                        direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
        Robot_1.render_exec(renderInstance= True,
                                Show_Sphere= False)

        Robot_1.move_to_home()

        # Robot_1_Do_Side_Nail(push_to_nail= PUSH_TO_NAIL_OFFSET, H= H, Side_Selector= 1)
        Side_Selector = 1

        Solver_Flag = True

    # Check if Negative Value is Suitable for Robot !
    if((-(Y - (OVERALL_PANEL_LENGTH/2)) + (SMART_CONV_RANGE_OF_MOTION_J1/2)-NAILING_CONV_TARGET*2 >= 0) and Solver_Flag == False):
        Smart_Conv.render_exec('Joint_1', -(Y - 1.8288) + 1.275)
        # Robot 1 Pre Place
        Robot_1.plan(tcp_name= "tool0",
                        target_pose= [X + -(L / 2) + 1.632691,
                                        -1.0,
                                        H + 1.08936],
                        target_orientation= [0, -ev, ev, 0],
                        update_world_needed= True)
        Robot_1.render_exec(renderInstance= True,
                                Show_Sphere= False)

        # Robot 1 Drop
        Robot_1.plan(tcp_name= "tool0",
                        target_pose= [X + -(L / 2) + 1.632691,
                                        -1.0,
                                        H + 0.98936],
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
                        target_pose= [X + -(L / 2) + 1.632691,
                                        -1.0,
                                        H + 1.08936],
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
        Side_Selector = -1

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
                        target_pose= [X + 1.13259,
                                        -(L / 2) + 0.500101,
                                        H + 1.08936],
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
        New_Loc = Gf.Vec3d(object_pose.p[0], object_pose.p[1], H / 2 + 0.89546)

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

        elif (OVERALL_PANEL_LENGTH/2)- Y +(SMART_CONV_RANGE_OF_MOTION_J1/2)-(NAILING_CONV_TARGET*2) < 0:
            Side_Selector = 1
    
    return Side_Selector 

def Place_Short_Vertical_Element_On_Smart_Conveyor_by_Rob2_Gripper(el_name: str = None,
        X: float = None,
        Y: float = None,
        L: float = None,
        H: float = None):
    # Moving Conveyor
    Smart_Conv.render_exec('Joint_1', -(Y - 1.8288) + 2.275)

    # Home
    Robot_2.move_to_home()

    # Pre Place1
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [X - L / 2 + 1.577611,
                                  0,
                                  H + 1.03936],
                    target_orientation= [0, -ev, -ev, 0],
                    update_world_needed= True)
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)

    # Pre Place2
    Robot_2.plan(tcp_name= "tool0",
                    target_pose= [X - L / 2 + 1.477611,
                                  0,
                                  H + 0.88936],
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
                    target_pose= [X - L / 2 + 1.577611,
                                  0,
                                  H + 1.03936],
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
        # 8ft For Now
        L: float = 2.4384,
        H: float = None):
    # Moving Conveyor To Start Point
    Smart_Conv.render_exec('Joint_1', 0)

    # We Consider this As a Constant Point Regardless of the Dimensions of the Passing L_U
    R2_LU_Pass_Loc: List[float] = [2.81634, 1.25053, 1.35738]

    Length_Diff_Term: float = L - 0.500021000104

    Robot_2.plan(tcp_name= "tool0",
                    target_pose= R2_LU_Pass_Loc,
                    target_orientation= [-0.6123661 ,  0.6123697 ,  0.35354862,  0.35357386],
                    update_world_needed= True,
                    removing_primitives=["world/obstacles"],)
    Robot_2.render_exec(renderInstance= True,
                            Show_Sphere= False)

    # Pre Passing
    Robot_1.plan(tcp_name= "tool1",
                    target_pose= [R2_LU_Pass_Loc[0] - Length_Diff_Term * 0.866025403785,
                                  R2_LU_Pass_Loc[1] + H / 2 + 0.173499901277,
                                  R2_LU_Pass_Loc[2] + Length_Diff_Term * 0.5 - 0.1],
                    target_orientation= [0.96593, 0, 0.25882, 0],
                    update_world_needed= True)
    Robot_1.render_exec(renderInstance= True,
                            Show_Sphere= False)

    # Passing
    Robot_1.plan(tcp_name= "tool1",
                    target_pose= [R2_LU_Pass_Loc[0] - Length_Diff_Term * 0.866025403785,
                                  R2_LU_Pass_Loc[1] + H / 2 + 0.173499901277,
                                  R2_LU_Pass_Loc[2] + Length_Diff_Term * 0.5 - 0.06],
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
    # Moving Conveyor
    Smart_Conv.render_exec('Joint_1', -(Y - 1.8288) + 2.275)

    Placement_Offset: float = 0.00565

    #Robot 1 Pre Place
    Robot_1.plan(tcp_name= "tool1",
                    target_pose= [-X - L / 2 - Placement_Offset + 3.720990000208,
                                  -0.179599901277,
                                  Z + W / 2 + 1.02546],
                    target_orientation= [0, 1, 0, 0],
                    update_world_needed= True)
    Robot_1.render_exec(renderInstance= True,
                            Show_Sphere= False)
    
    #Robot 1 Place
    Robot_1.plan(tcp_name= "tool1",
                    target_pose= [-X - L / 2 - Placement_Offset + 3.720990000208,
                                  -0.179599901277,
                                  Z + W / 2 + 0.92546],
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
                    target_pose= [-X - L / 2 - Placement_Offset + 3.720990000208,
                                  -0.179599901277,
                                  Z + W / 2 + 1.02546],
                    target_orientation= [0, 1, 0, 0],
                    update_world_needed= True,
                    removing_primitives=["world/obstacles", "Smart_Conveyor"],
                    direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
    Robot_1.render_exec(renderInstance= True,
                            Show_Sphere= False)

    Robot_1.move_to_home()

def Pick_2x10_Header(X: float = None,
        L: float = None,):
    global NUMBER_OF_HEADERS

    # Check if the Requested Length is less or equal to the Maximum Length of the Bear Loading elements pilled up
    if RAW_HEADER_DIMENSIONS[0] < L:
        print("The Requested Length is Less Than the Maximum Length of the Bear Loading Elements")
        return

    # Pick Orientation [0, ev, -ev, 0] => From The Pile
    # Pre Pick
    Robot_1.plan(tcp_name= "tool1",
                    target_pose= [-1.306599901277,
                                  1.60499956533,
                                  0.484],
                    target_orientation= [0, ev, -ev, 0],
                    update_world_needed= True)
    Robot_1.render_exec(renderInstance= True,
                            Show_Sphere= False)

    # Pick
    Robot_1.plan(tcp_name= "tool1",
                    target_pose= [-1.306599901277,
                                  1.60499956533,
                                  0.384],
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
                    target_pose= [-1.306599901277,
                                  1.60499956533,
                                  0.484],
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
    # Pre Cut
    Robot_1.plan(tcp_name= "tool1",
                    target_pose= [-L + 1.60499956533,
                                  1.179599901277,
                                  W + 1.03],
                    target_orientation= [0, 0, -1, 0],
                    update_world_needed= True)
    Robot_1.render_exec(renderInstance= True,
                            Show_Sphere= False)

    # Cut
    Robot_1.plan(tcp_name= "tool1",
                    target_pose= [-L + 1.60499956533,
                                  1.179599901277,
                                  W + 0.53],
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
                    target_pose= [-L + 1.60499956533,
                                  1.179599901277,
                                  W + 1.03],
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
    global NUMBER_OF_HEADERS

    # Moving Conveyor
    Smart_Conv.render_exec('Joint_1', -(Y - 1.8288) - L / 2 + 2.87999956533)

    Conv_Curr_Loc: float = -(Y - 1.8288) + 2.275

    # Pre Place
    Place_Doable: bool = Robot_1.plan(tcp_name= "tool1",
                    target_pose= [X + 1.312189901277,
                                  0,
                                  W + 1.22546],
                    target_orientation= [0, ev, ev, 0],
                    update_world_needed= True)

    if Place_Doable == True:
        # Execute Pre Drop
        Robot_1.render_exec(renderInstance= True,
                                Show_Sphere= False)
        # Drop
        Robot_1.plan(tcp_name= "tool1",
                        target_pose= [X + 1.312189901277,
                                    0,
                                    W + 1.07546],
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
                        target_pose= [X - W / 2 + 1.10259,
                                      0,
                                      H + 1.254659802554],
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
        New_Loc = Gf.Vec3d(object_pose.p[0], object_pose.p[1], Z + 0.89546)
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

def Nail_2x10_Header(push_to_nail: float = 0.025,
                        el_pose: List[float] = [],
                        el_dims: List[float] = [],
                        conv_current_location: float = None):
    quat = R.from_euler('xyz', [((np.pi/2)+np.radians(BR_NAILING_ANGLE))*(-1), 0, (np.pi/2)]).as_quat()
    quat[1] *= -1

    # Left Nail
    if(conv_current_location+(el_dims[0]/2)+NAILING_CONV_TARGET > SMART_CONV_RANGE_OF_MOTION_J1):
        print("Left Nail is Not Possible ! Due To Conveyor Reachability")
    else:
        Smart_Conv.render_exec('Joint_1', conv_current_location + el_dims[0] / 2 + 0.5)
        # Robot 1 Move For Nail
        # Pre Nail
        Nail_Doable: bool = Robot_1.plan(tcp_name= "tool2",
                        target_pose= [el_pose[0] + 0.2 * el_dims[2] + 1.13259,
                                    (el_dims[1] / 2 + 0.070710678119) * -1 + 0.5,
                                    el_pose[2] + 0.986490678119],
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
                                        object_pose.p[1] + (-(el_dims[1] / 2 / 0.707106781187) + push_to_nail + 0.1) * 0.707106781187,
                                        object_pose.p[2] - (-(el_dims[1] / 2 / 0.707106781187) + push_to_nail + 0.1) * 0.707106781186],
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
                                        object_pose.p[1] - (-(el_dims[1] / 2 / 0.707106781187) + push_to_nail + 0.1) * 0.707106781187,
                                        object_pose.p[2] + (-(el_dims[1] / 2 / 0.707106781187) + push_to_nail + 0.1) * 0.707106781186],
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
                            target_pose= [object_pose.p[0] - 0.4 * el_dims[2],
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
                                        object_pose.p[1] + (-(el_dims[1] / 2 / 0.707106781187) + push_to_nail + 0.1) * 0.707106781187,
                                        object_pose.p[2] - (-(el_dims[1] / 2 / 0.707106781187) + push_to_nail + 0.1) * 0.707106781186],
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
                                        object_pose.p[1] - (-(el_dims[1] / 2 / 0.866025403785) + push_to_nail + 0.1) * 0.866025403785,
                                        object_pose.p[2] + (-(el_dims[1] / 2 / 0.866025403785) + push_to_nail + 0.1) * 0.5],
                            target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                            update_world_needed= True,
                            removing_primitives=["Smart_Conveyor", "world/obstacles"],
                            direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
            Robot_1.render_exec(renderInstance= True,
                                    Show_Sphere= False)

            Robot_1.move_to_home(removing_primitives= ["Smart_Conveyor", "world/obstacles"])
        else:
            print("Left Nail is Not Possible ! Due To Robot NailGun Reachability")

    #NAILING OTHER SIDE
    quat = R.from_euler('xyz', [((np.pi/2)+np.radians(BR_NAILING_ANGLE)), 0, (np.pi/2)*(-1)]).as_quat()
    quat[1] *= -1

    # Right Nail
    if(conv_current_location-(el_dims[0]/2)-NAILING_CONV_TARGET < 0):
        print("Right Nail is Not Possible ! Due To Conveyor Reachability")
    else:
        Smart_Conv.render_exec('Joint_1', conv_current_location - el_dims[0] / 2 - 0.5)

        # Pre Nail
        Nail_Doable: bool = Robot_1.plan(tcp_name= "tool2",
                        target_pose= [el_pose[0] + 0.2 * el_dims[2] + 1.13259,
                                    el_dims[1] / 2 - 0.429289321881,
                                    el_pose[2] + 0.986490678119],
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
                                        object_pose.p[1] + -1 * (-(el_dims[1] / 2 / 0.707106781187) + push_to_nail + 0.1) * 0.707106781187,
                                        object_pose.p[2] - (-(el_dims[1] / 2 / 0.707106781187) + push_to_nail + 0.1) * 0.707106781186],
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
                                        object_pose.p[1] - -1 * (-(el_dims[1] / 2 / 0.707106781187) + push_to_nail + 0.1) * 0.707106781187,
                                        object_pose.p[2] + (-(el_dims[1] / 2 / 0.707106781187) + push_to_nail + 0.1) * 0.707106781186],
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
                            target_pose= [object_pose.p[0] - 0.4 * el_dims[2],
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
                                        object_pose.p[1] + -1 * (-(el_dims[1] / 2 / 0.707106781187) + push_to_nail + 0.1) * 0.707106781187,
                                        object_pose.p[2] - (-(el_dims[1] / 2 / 0.707106781187) + push_to_nail + 0.1) * 0.707106781186],
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
                                        object_pose.p[1] - -1 * (-(el_dims[1] / 2 / 0.707106781187) + push_to_nail + 0.1) * 0.707106781187,
                                        object_pose.p[2] + (-(el_dims[1] / 2 / 0.707106781187) + push_to_nail + 0.1) * 0.707106781186],
                            target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                            update_world_needed= True,
                            removing_primitives=["Smart_Conveyor", "world/obstacles"],
                            direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
            Robot_1.render_exec(renderInstance= True,
                                    Show_Sphere= False)

            Robot_1.move_to_home(removing_primitives= ["Smart_Conveyor", "world/obstacles"])
        else:
            print("Right Nail is Not Possible ! Due To Robot NailGun Reachability")

def Complementary_Nail_Operation(push_to_nail: float = 0.05,
                         H: float = None):
    for target in Smart_Conv._nail_poses:
        # Check if the target is reachable or not (for the conveyor !)
        if(target[0]+(NAILING_CONV_TARGET*target[1]) > SMART_CONV_RANGE_OF_MOTION_J1):
            Replacing_Target: Tuple[float, float] = (target[0] - 1.0, -target[1])
            target = Replacing_Target

        Smart_Conv.render_exec('Joint_1', target[0] + 0.5 * target[1])
        # Robot 1 Nail
        Robot_2.plan(tcp_name= "tool1",
                        target_pose= [3.69, 0.5 * target[1] * 2, H - 0.7 * H + 0.89546],
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
                        target_pose= [object_pose.p[0] - push_to_nail, object_pose.p[1], object_pose.p[2]],
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
                        target_pose= [object_pose.p[0] + push_to_nail, object_pose.p[1], object_pose.p[2]],
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
                        target_pose= [object_pose.p[0], object_pose.p[1], object_pose.p[2] + 0.4 * H],
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
                        target_pose= [object_pose.p[0] - push_to_nail, object_pose.p[1], object_pose.p[2]],
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
                        target_pose= [object_pose.p[0] + push_to_nail, object_pose.p[1], object_pose.p[2]],
                        target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                        update_world_needed= True,
                        removing_primitives=["Smart_Conveyor", "world/obstacles"],
                        direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
        Robot_2.render_exec(renderInstance= True,
                                Show_Sphere= False)

    Robot_2.move_to_home()

def Pick_OSB_Plate(el_name: str = None,
        L: float = None,
        W: float = None,
        H: float = None):
    # Pre Pick
    Robot_1.plan(tcp_name= "tool1",
                    target_pose= [0.179599901277,
                                  -2.08499956533,
                                  H + 0.7],
                    target_orientation= [0, ev, ev, 0],
                    update_world_needed= True)
    Robot_1.render_exec(renderInstance= True,
                            Show_Sphere= False)

    # Pick
    Robot_1.plan(tcp_name= "tool1",
                    target_pose= [0.179599901277,
                                  -2.08499956533,
                                  H + 0.63],
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
                    target_pose= [0.179599901277,
                                  -2.08499956533,
                                  H + 0.7],
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
    # Letting Space For the First Robot To Place The Sheathing Plate
    Robot_2.move_to_home(Customized_JS=[np.radians(90), -0.5, 0.5, 0, 0, 0])

    # Moving the Conveyor
    Smart_Conv.render_exec(
        'Joint_1',
        -(Y - 1.8288) + 2.095400098723
    )

    # Pre Place
    Robot_1.plan(
        tcp_name="tool1",
        target_pose=[
            -X + 3.17678956533,
            0,
            Z + 0.99546,
        ],
        target_orientation=[0, 0, -1, 0],
        update_world_needed=True,
    )
    Robot_1.render_exec(renderInstance=True, Show_Sphere=False)

    # Place
    Robot_1.plan(
        tcp_name="tool1",
        target_pose=[
            -X + 3.27678956533,
            0,
            Z + 0.92546,
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
            -X + 3.17678956533,
            0,
            Z + 0.99546,
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
        -(Y - 1.8288) + 2.275
    )
    return Sht_plate_on_conv_loc

def Nail_OSB_Plate(push_to_nail: float = 0.01,
                     nail_num: int = 6,
                     el_dims: list[float] = [],
                     Sht_plate_on_conv_loc: float = None):
    for nail in Smart_Conv._vertical_nail_poses:
        # It means that we should vertically nail the sheathing plate to the king at these locations
        if(nail <= Sht_plate_on_conv_loc + (el_dims[1]/2) and nail >= Sht_plate_on_conv_loc - (el_dims[1]/2)):
            # Move Nail Target To Y=0
            Smart_Conv.render_exec('Joint_1', nail)

            Starting_X: float = 1.15259
            Ending_X: float = 3.63099
            # Might Be Less Than 1, so 2 decimals
            L: float = np.round((Ending_X - Starting_X) / (nail_num - 1.0), 2)
            it: int = 0
            dc=_dynamic_control.acquire_dynamic_control_interface()

            # Robot 1 vertical nailing
            while it < (nail_num/2):
                # Pre Nail Rob1
                Robot_1.plan(tcp_name= "tool3",
                                target_pose= [it * L + 1.15259,
                                            0,
                                            el_dims[2] + 1.04706],
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
                                            object_pose.p[2] - push_to_nail - 0.05],
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
                                            object_pose.p[2] + push_to_nail + 0.05],
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
                                target_pose= [-(it * L) + 3.63099,
                                            0,
                                            el_dims[2] + 1.04706],
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
                                            object_pose.p[2] - push_to_nail - 0.05],
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
                                            object_pose.p[2] + push_to_nail + 0.05],
                                target_orientation= [object_pose.r[3], object_pose.r[0], object_pose.r[1], object_pose.r[2]],
                                update_world_needed= True,
                                removing_primitives=["Smart_Conveyor", "world/obstacles"],
                                direct_pose_cost= PoseCostMetric.create_grasp_approach_metric(offset_position=0.0, tstep_fraction=0.001,linear_axis=2))
                Robot_2.render_exec(renderInstance= True,
                                        Show_Sphere= False)
                it+=1

            Robot_2.move_to_home()
