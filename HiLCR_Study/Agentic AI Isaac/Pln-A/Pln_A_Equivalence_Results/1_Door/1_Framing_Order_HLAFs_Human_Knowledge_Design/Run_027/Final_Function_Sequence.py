# Pln-A case: 1_Door
# Experimental condition: 1_Framing_Order_HLAFs_Human_Knowledge_Design
# Repetition: 027
# Execution order: 159

# Pick and place Top Plate (Top_Stud)
Pick_Long_Element_From_Mat_Supply(el_name="Top_Stud", L=3.6576, W=0.04, H=0.1016)
Pass_Long_Element_G2G(el_name="Top_Stud", L=3.6576, H=0.1016)
Place_Long_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name="Top_Stud", X=0.02, Y=1.8288, L=3.6576, H=0.1016)
# Install first bay Vertical Stud (F_Stud)
Pick_8ft_Element_From_Sloped_Table(el_name="F_Stud", L=2.4384, W=0.04, H=0.1016)
side_selector_F = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.02, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="F_Stud", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_selector_F, Is_Held=False)
# Optional L/U Stud immediately after first stud
Pick_8ft_Element_From_Sloped_Table(el_name="L_U_Element_1", L=2.4384, W=0.04, H=0.1016)
Pass_Long_Element_G2G(el_name="L_U_Element_1", L=2.4384, H=0.1016)
Place_Long_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name="L_U_Element_1", X=1.2592, Y=0.0908, L=2.4384, H=0.1016)
# Standard stud R_Stud_2
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_2", L=2.4384, W=0.04, H=0.1016)
side_selector_R2 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.3248, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_2", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_selector_R2, Is_Held=False)
# Standard stud R_Stud_4
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_4", L=2.4384, W=0.04, H=0.1016)
side_selector_R4 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.6296, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_4", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_selector_R4, Is_Held=False)
# Standard stud R_Stud_6
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_6", L=2.4384, W=0.04, H=0.1016)
side_selector_R6 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.9344, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_6", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_selector_R6, Is_Held=False)
# Standard stud R_Stud_8
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_8", L=2.4384, W=0.04, H=0.1016)
side_selector_R8 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=1.2392, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_8", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_selector_R8, Is_Held=False)
# Left King Stud
Pick_8ft_Element_From_Sloped_Table(el_name="Wooden_Element_6", L=2.4384, W=0.04, H=0.1016)
side_selector_KL = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=1.52, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="Wooden_Element_6", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_selector_KL, Is_Held=False)
# Right King Stud
Pick_8ft_Element_From_Sloped_Table(el_name="Wooden_Element_7", L=2.4384, W=0.04, H=0.1016)
side_selector_KR = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=2.52, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="Wooden_Element_7", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_selector_KR, Is_Held=False)
# Left Jack Stud (tangent from left)
Pick_Short_Element_From_Mat_Supply(el_name="Wooden_Element_12", L=2.0, W=0.04, H=0.1016)
side_selector_JackL = Drop_Short_Vertical_Element_With_Tangent_to_an_Element(el_name="Wooden_Element_12", X=1.4784, Y=1.56, L=2.0, H=0.1016, If_Tangent_From_Left=True)
Nail_Vertical_Element_With_Tangent_to_an_Element(push_to_nail=0.01, el_pose=[1.4784, 1.56, 0.0], el_dims=[0.04, 0.1016, 2.0], If_Tangent_From_Left=True, Side_Selector=side_selector_JackL)
# Right Jack Stud (tangent from right)
Pick_Short_Element_From_Mat_Supply(el_name="Wooden_Element_11", L=2.0, W=0.04, H=0.1016)
side_selector_JackR = Drop_Short_Vertical_Element_With_Tangent_to_an_Element(el_name="Wooden_Element_11", X=1.4784, Y=2.48, L=2.0, H=0.1016, If_Tangent_From_Left=False)
Nail_Vertical_Element_With_Tangent_to_an_Element(push_to_nail=0.01, el_pose=[1.4784, 2.48, 0.0], el_dims=[0.04, 0.1016, 2.0], If_Tangent_From_Left=False, Side_Selector=side_selector_JackR)
# Bear Loading Header plank 1
Pick_2x10_Header(X=0.3514, L=0.96)
conv_header1 = Place_2x10_Header(X=0.3514, Y=2.02, Z=0.0254, L=0.96, W=0.0508, H=0.254)
Nail_2x10_Header(el_pose=[0.3514, 2.02, 0.0254], el_dims=[0.96, 0.0508, 0.254], conv_current_location=conv_header1)
# Bear Loading Header plank 2
Pick_2x10_Header(X=0.3514, L=0.96)
conv_header2 = Place_2x10_Header(X=0.3514, Y=2.02, Z=0.0762, L=0.96, W=0.0508, H=0.254)
Nail_2x10_Header(el_pose=[0.3514, 2.02, 0.0762], el_dims=[0.96, 0.0508, 0.254], conv_current_location=conv_header2)
# Standard stud R_Stud_18
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_18", L=2.4384, W=0.04, H=0.1016)
side_selector_R18 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=2.7632, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_18", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_selector_R18, Is_Held=False)
# Standard stud R_Stud_20
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_20", L=2.4384, W=0.04, H=0.1016)
side_selector_R20 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=3.068, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_20", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_selector_R20, Is_Held=False)
# Standard stud R_Stud_22
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_22", L=2.4384, W=0.04, H=0.1016)
side_selector_R22 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=3.3728, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_22", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_selector_R22, Is_Held=False)
# Standard stud R_Stud_24
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_24", L=2.4384, W=0.04, H=0.1016)
side_selector_R24 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=3.6376, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_24", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_selector_R24, Is_Held=False)
# Bottom Plate
Pick_Long_Element_From_Mat_Supply(el_name="Bot_Stud", L=3.6576, W=0.04, H=0.1016)
Place_Long_Element_On_Smart_Conveyor_by_Rob2_Gripper(el_name="Bot_Stud", X=2.4984, Y=1.8288, L=3.6576, H=0.1016)
# Complementary nailing to secure all vertical studs and bottom plate
Complementary_Nail_Operation(push_to_nail=0.05, H=0.1016)
