# Pln-A case: 1_Door_1_Window
# Experimental condition: 1_Framing_Order_HLAFs_Human_Knowledge_Design
# Repetition: 013
# Execution order: 077

# Install Top Plate
Pick_Long_Element_From_Mat_Supply(el_name='Top_Stud', L=3.6576, W=0.04, H=0.1016)
Pass_Long_Element_G2G(el_name='Top_Stud', L=3.6576, H=0.1016)
Place_Long_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name='Top_Stud', X=0.02, Y=1.8288, L=3.6576, H=0.1016)
# Install first vertical stud F_Stud
Pick_8ft_Element_From_Sloped_Table(el_name='F_Stud', L=2.4384, W=0.04, H=0.1016)
side_selector_F = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.02, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name='F_Stud', X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_selector_F, Is_Held=False)
# Optional L/U Stud
Pick_8ft_Element_From_Sloped_Table(el_name='L_U_Element_1', L=2.4384, W=0.04, H=0.1016)
Pass_8ft_Element_G2S(el_name='L_U_Element_1', L=2.4384, H=0.1016)
Place_8ft_Vertical_Element_On_Smart_Conveyor_by_Rob1_Suction(el_name='L_U_Element_1', X=1.2592, Y=0.0908, Z=0.02, L=2.4384, W=0.04, H=0.1016)
# Install Left King Stud LK_1
Pick_8ft_Element_From_Sloped_Table(el_name='LK_1', L=2.4384, W=0.04, H=0.1016)
side_LK1 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.72, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name='LK_1', X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_LK1, Is_Held=False)
# Install Right King Stud RK_1
Pick_8ft_Element_From_Sloped_Table(el_name='RK_1', L=2.4384, W=0.04, H=0.1016)
side_RK1 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=1.68, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name='RK_1', X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_RK1, Is_Held=False)
# Install Left Jack Stud LJ_1
Pick_Short_Element_From_Mat_Supply(el_name='LJ_1', L=2.0, W=0.04, H=0.1016)
side_LJ1 = Drop_Short_Vertical_Element_With_Tangent_to_an_Element(el_name='LJ_1', X=1.4784, Y=0.76, L=2.0, H=0.1016, If_Tangent_From_Left=True)
Nail_Vertical_Element_With_Tangent_to_an_Element(push_to_nail=0.01, el_pose=[1.4784, 0.76, 0.0], el_dims=[2.0, 0.04, 0.1016], If_Tangent_From_Left=True, Side_Selector=side_LJ1)
# Install Right Jack Stud RJ_1
Pick_Short_Element_From_Mat_Supply(el_name='RJ_1', L=2.0, W=0.04, H=0.1016)
side_RJ1 = Drop_Short_Vertical_Element_With_Tangent_to_an_Element(el_name='RJ_1', X=1.4784, Y=1.64, L=2.0, H=0.1016, If_Tangent_From_Left=False)
Nail_Vertical_Element_With_Tangent_to_an_Element(push_to_nail=0.01, el_pose=[1.4784, 1.64, 0.0], el_dims=[2.0, 0.04, 0.1016], If_Tangent_From_Left=False, Side_Selector=side_RJ1)
# Install Bear Loading Header BL_1
Pick_Short_Element_From_Mat_Supply(el_name='BL_1', L=0.86, W=0.0508, H=0.254)
conv_BL1 = Place_Short_Horizontal_Element_On_Smart_Conveyor_by_Rob2_Gripper(el_name='BL_1', X=0.5514, Y=2.6876, L=0.86, H=0.254)
Nail_Short_Horizontal_Element_by_Rob2_NailGun(push_to_nail=0.01, el_pose=[0.5514, 2.6876, 0.0254], el_dims=[0.86, 0.0508, 0.254], conv_current_location=conv_BL1)
# Install Bear Loading Header BL_2 (stacked)
Pick_Short_Element_From_Mat_Supply(el_name='BL_2', L=0.86, W=0.0508, H=0.254)
conv_BL2 = Place_Short_Horizontal_Element_On_Smart_Conveyor_by_Rob2_Gripper(el_name='BL_2', X=0.5514, Y=2.6876, L=0.86, H=0.254)
Nail_Short_Horizontal_Element_by_Rob2_NailGun(push_to_nail=0.01, el_pose=[0.5514, 2.6876, 0.0762], el_dims=[0.86, 0.0508, 0.254], conv_current_location=conv_BL2)
# Install Lower Sill Plate BS_1
Pick_Short_Element_From_Mat_Supply(el_name='BS_1', L=0.78, W=0.04, H=0.1016)
conv_BS1 = Place_Short_Horizontal_Element_On_Smart_Conveyor_by_Rob2_Gripper(el_name='BS_1', X=1.8984, Y=2.6876, L=0.78, H=0.1016)
Nail_Short_Horizontal_Element_by_Rob2_NailGun(push_to_nail=0.01, el_pose=[1.8984, 2.6876, 0.0], el_dims=[0.78, 0.04, 0.1016], conv_current_location=conv_BS1)
# Install Lower Cripple Stud LC_1
Pick_Short_Element_From_Mat_Supply(el_name='LC_1', L=0.56, W=0.04, H=0.1016)
Place_Short_Vertical_Element_On_Smart_Conveyor_by_Rob2_Gripper(el_name='LC_1', X=2.1984, Y=2.5376, L=0.56, H=0.1016)
# Install Lower Cripple Stud LC_2
Pick_Short_Element_From_Mat_Supply(el_name='LC_2', L=0.56, W=0.04, H=0.1016)
Place_Short_Vertical_Element_On_Smart_Conveyor_by_Rob2_Gripper(el_name='LC_2', X=2.1984, Y=2.8376, L=0.56, H=0.1016)
# Install Left King Stud LK_2
Pick_8ft_Element_From_Sloped_Table(el_name='LK_2', L=2.4384, W=0.04, H=0.1016)
side_LK2 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=3.1376, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name='LK_2', X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_LK2, Is_Held=False)
# Install Right King Stud RK_2
Pick_8ft_Element_From_Sloped_Table(el_name='RK_2', L=2.4384, W=0.04, H=0.1016)
side_RK2 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=2.2376, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name='RK_2', X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_RK2, Is_Held=False)
# Install Left Jack Stud LJ_2
Pick_Short_Element_From_Mat_Supply(el_name='LJ_2', L=1.8, W=0.04, H=0.1016)
side_LJ2 = Drop_Short_Vertical_Element_With_Tangent_to_an_Element(el_name='LJ_2', X=1.5784, Y=3.0976, L=1.8, H=0.1016, If_Tangent_From_Left=True)
Nail_Vertical_Element_With_Tangent_to_an_Element(push_to_nail=0.01, el_pose=[1.5784, 3.0976, 0.0], el_dims=[1.8, 0.04, 0.1016], If_Tangent_From_Left=True, Side_Selector=side_LJ2)
# Install Right Jack Stud RJ_2
Pick_Short_Element_From_Mat_Supply(el_name='RJ_2', L=1.8, W=0.04, H=0.1016)
side_RJ2 = Drop_Short_Vertical_Element_With_Tangent_to_an_Element(el_name='RJ_2', X=1.5784, Y=2.2776, L=1.8, H=0.1016, If_Tangent_From_Left=False)
Nail_Vertical_Element_With_Tangent_to_an_Element(push_to_nail=0.01, el_pose=[1.5784, 2.2776, 0.0], el_dims=[1.8, 0.04, 0.1016], If_Tangent_From_Left=False, Side_Selector=side_RJ2)
# Install Bottom Plate
Pick_Long_Element_From_Mat_Supply(el_name='Bot_Stud', L=3.6576, W=0.04, H=0.1016)
Place_Long_Element_On_Smart_Conveyor_by_Rob2_Gripper(el_name='Bot_Stud', X=2.4984, Y=1.8288, L=3.6576, H=0.1016)
Nail_Short_Horizontal_Element_by_Rob2_NailGun(push_to_nail=0.01, el_pose=[2.4984, 1.8288, 0.06], el_dims=[3.6576, 0.04, 0.1016], conv_current_location=None)
