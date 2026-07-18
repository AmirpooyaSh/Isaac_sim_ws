# Pln-A case: 1_Door
# Experimental condition: 1_Framing_Order_HLAFs_Human_Knowledge_Design
# Repetition: 012
# Execution order: 069

# Install Top Plate
Pick_Long_Element_From_Mat_Supply(el_name="Top_Stud", L=3.6576, W=0.04, H=0.1016)
Pass_Long_Element_G2G(el_name="Top_Stud", L=3.6576, H=0.1016)
Place_Long_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name="Top_Stud", X=0.02, Y=1.8288, L=3.6576, H=0.1016)
# Install first Vertical Stud (F_Stud)
Pick_8ft_Element_From_Sloped_Table(el_name="F_Stud", L=2.4384, W=0.04, H=0.1016)
side_selector_F_Stud = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.02, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="F_Stud", X=1.2592, push_to_nail=0.05, L=2.4384, H=0.1016, Side_Selector=side_selector_F_Stud, Is_Held=False)
# Install optional L/U Stud
Pick_8ft_Element_From_Sloped_Table(el_name="L_U_Element_1", L=2.4384, W=0.04, H=0.1016)
Pass_Long_Element_G2G(el_name="L_U_Element_1", L=2.4384, H=0.1016)
Place_Long_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name="L_U_Element_1", X=1.2592, Y=0.0908, L=2.4384, H=0.1016)
# Install Vertical Stud R_Stud_2
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_2", L=2.4384, W=0.04, H=0.1016)
side_selector_R_Stud_2 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.3248, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_2", X=1.2592, push_to_nail=0.05, L=2.4384, H=0.1016, Side_Selector=side_selector_R_Stud_2, Is_Held=False)
# Install Vertical Stud R_Stud_4
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_4", L=2.4384, W=0.04, H=0.1016)
side_selector_R_Stud_4 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.6296, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_4", X=1.2592, push_to_nail=0.05, L=2.4384, H=0.1016, Side_Selector=side_selector_R_Stud_4, Is_Held=False)
# Install Vertical Stud R_Stud_6
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_6", L=2.4384, W=0.04, H=0.1016)
side_selector_R_Stud_6 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.9344, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_6", X=1.2592, push_to_nail=0.05, L=2.4384, H=0.1016, Side_Selector=side_selector_R_Stud_6, Is_Held=False)
# Install Vertical Stud R_Stud_8
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_8", L=2.4384, W=0.04, H=0.1016)
side_selector_R_Stud_8 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=1.2392, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_8", X=1.2592, push_to_nail=0.05, L=2.4384, H=0.1016, Side_Selector=side_selector_R_Stud_8, Is_Held=False)
# Install King Stud (left)
Pick_8ft_Element_From_Sloped_Table(el_name="Wooden_Element_6", L=2.4384, W=0.04, H=0.1016)
side_selector_King_L = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=1.52, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="Wooden_Element_6", X=1.2592, push_to_nail=0.05, L=2.4384, H=0.1016, Side_Selector=side_selector_King_L, Is_Held=False)
# Install King Stud (right)
Pick_8ft_Element_From_Sloped_Table(el_name="Wooden_Element_7", L=2.4384, W=0.04, H=0.1016)
side_selector_King_R = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=2.52, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="Wooden_Element_7", X=1.2592, push_to_nail=0.05, L=2.4384, H=0.1016, Side_Selector=side_selector_King_R, Is_Held=False)
# Install Jack Stud (left)
Pick_Short_Element_From_Mat_Supply(el_name="Wooden_Element_12", L=2.0, W=0.04, H=0.1016)
Place_Short_Vertical_Element_On_Smart_Conveyor_by_Rob2_Gripper(el_name="Wooden_Element_12", X=1.4784, Y=1.56, L=2.0, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="Wooden_Element_12", X=1.4784, push_to_nail=0.05, L=2.0, H=0.1016, Side_Selector=1, Is_Held=False)
# Install Jack Stud (right)
Pick_Short_Element_From_Mat_Supply(el_name="Wooden_Element_11", L=2.0, W=0.04, H=0.1016)
Place_Short_Vertical_Element_On_Smart_Conveyor_by_Rob2_Gripper(el_name="Wooden_Element_11", X=1.4784, Y=2.48, L=2.0, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="Wooden_Element_11", X=1.4784, push_to_nail=0.05, L=2.0, H=0.1016, Side_Selector=1, Is_Held=False)
# Install Bear Loading Header plank 1
Pick_Short_Element_From_Mat_Supply(el_name="Wooden_Element_13", L=0.96, W=0.0508, H=0.254)
conv_loc_Bear_1 = Place_Short_Horizontal_Element_On_Smart_Conveyor_by_Rob2_Gripper(el_name="Wooden_Element_13", X=0.3514, Y=2.02, L=0.96, H=0.254)
Nail_Short_Horizontal_Element_by_Rob2_NailGun(push_to_nail=0.01, el_pose=[0.3514, 2.02, 0.0254], el_dims=[0.96, 0.0508, 0.254], conv_current_location=conv_loc_Bear_1)
# Install Bear Loading Header plank 2
Pick_Short_Element_From_Mat_Supply(el_name="Wooden_Element_14", L=0.96, W=0.0508, H=0.254)
conv_loc_Bear_2 = Place_Short_Horizontal_Element_On_Smart_Conveyor_by_Rob2_Gripper(el_name="Wooden_Element_14", X=0.3514, Y=2.02, L=0.96, H=0.254)
Nail_Short_Horizontal_Element_by_Rob2_NailGun(push_to_nail=0.01, el_pose=[0.3514, 2.02, 0.0762], el_dims=[0.96, 0.0508, 0.254], conv_current_location=conv_loc_Bear_2)
# Install Vertical Stud R_Stud_18
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_18", L=2.4384, W=0.04, H=0.1016)
side_selector_R_Stud_18 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=2.7632, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_18", X=1.2592, push_to_nail=0.05, L=2.4384, H=0.1016, Side_Selector=side_selector_R_Stud_18, Is_Held=False)
# Install Vertical Stud R_Stud_20
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_20", L=2.4384, W=0.04, H=0.1016)
side_selector_R_Stud_20 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=3.068, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_20", X=1.2592, push_to_nail=0.05, L=2.4384, H=0.1016, Side_Selector=side_selector_R_Stud_20, Is_Held=False)
# Install Vertical Stud R_Stud_22
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_22", L=2.4384, W=0.04, H=0.1016)
side_selector_R_Stud_22 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=3.3728, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_22", X=1.2592, push_to_nail=0.05, L=2.4384, H=0.1016, Side_Selector=side_selector_R_Stud_22, Is_Held=False)
# Install Vertical Stud R_Stud_24
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_24", L=2.4384, W=0.04, H=0.1016)
side_selector_R_Stud_24 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=3.6376, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_24", X=1.2592, push_to_nail=0.05, L=2.4384, H=0.1016, Side_Selector=side_selector_R_Stud_24, Is_Held=False)
# Install Bottom Plate
Pick_Long_Element_From_Mat_Supply(el_name="Bot_Stud", L=3.6576, W=0.04, H=0.1016)
Place_Long_Element_On_Smart_Conveyor_by_Rob2_Gripper(el_name="Bot_Stud", X=2.4984, Y=1.8288, L=3.6576, H=0.1016)
Nail_Short_Horizontal_Element_by_Rob2_NailGun(push_to_nail=0.01, el_pose=[2.4984, 1.8288, 0.06], el_dims=[3.6576, 0.04, 0.1016], conv_current_location=2.275)
