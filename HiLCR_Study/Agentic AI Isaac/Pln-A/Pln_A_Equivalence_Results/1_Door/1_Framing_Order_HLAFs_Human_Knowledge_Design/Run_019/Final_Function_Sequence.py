# Pln-A case: 1_Door
# Experimental condition: 1_Framing_Order_HLAFs_Human_Knowledge_Design
# Repetition: 019
# Execution order: 111

# Install Top Plate: Pick from material supply
Pick_Long_Element_From_Mat_Supply(el_name="Top_Stud", L=3.6576, W=0.04, H=0.1016)
# Pass Top Plate from Robot2 to Robot1
Pass_Long_Element_G2G(el_name="Top_Stud", L=3.6576, H=0.1016)
# Place Top Plate on smart conveyor by Robot1
Place_Long_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name="Top_Stud", X=0.02, Y=1.8288, L=3.6576, H=0.1016)
# Pick first standard vertical stud F_Stud
Pick_8ft_Element_From_Sloped_Table(el_name="F_Stud", L=2.4384, W=0.04, H=0.1016)
# Place and hold F_Stud on conveyor
side_F_Stud = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.02, L=2.4384, H=0.1016)
# Nail and release F_Stud
Nail_and_Release_Vertical_Element(el_name="F_Stud", X=1.2592, push_to_nail=0.05, L=2.4384, H=0.1016, Side_Selector=side_F_Stud, Is_Held=False)
# Pick optional L/U stud
Pick_8ft_Element_From_Sloped_Table(el_name="L_U_Element_1", L=2.4384, W=0.04, H=0.1016)
# Pass L/U stud from Robot2 to Robot1
Pass_8ft_Element_G2S(el_name="L_U_Element_1", L=2.4384, H=0.1016)
# Place L/U stud via Robot1 suction
Place_8ft_Vertical_Element_On_Smart_Conveyor_by_Rob1_Suction(el_name="L_U_Element_1", X=1.2592, Y=0.0908, Z=0.02, L=2.4384, W=0.04, H=0.1016)
# Pick second standard vertical stud R_Stud_2
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_2", L=2.4384, W=0.04, H=0.1016)
# Place and hold R_Stud_2
side_R_Stud_2 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.3248, L=2.4384, H=0.1016)
# Nail and release R_Stud_2
Nail_and_Release_Vertical_Element(el_name="R_Stud_2", X=1.2592, push_to_nail=0.05, L=2.4384, H=0.1016, Side_Selector=side_R_Stud_2, Is_Held=False)
# Pick left King Stud
Pick_8ft_Element_From_Sloped_Table(el_name="Wooden_Element_6", L=2.4384, W=0.04, H=0.1016)
# Place and hold left King Stud
side_King_L = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=1.52, L=2.4384, H=0.1016)
# Nail and release left King Stud
Nail_and_Release_Vertical_Element(el_name="Wooden_Element_6", X=1.2592, push_to_nail=0.05, L=2.4384, H=0.1016, Side_Selector=side_King_L, Is_Held=False)
# Pick right King Stud
Pick_8ft_Element_From_Sloped_Table(el_name="Wooden_Element_7", L=2.4384, W=0.04, H=0.1016)
# Place and hold right King Stud
side_King_R = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=2.52, L=2.4384, H=0.1016)
# Nail and release right King Stud
Nail_and_Release_Vertical_Element(el_name="Wooden_Element_7", X=1.2592, push_to_nail=0.05, L=2.4384, H=0.1016, Side_Selector=side_King_R, Is_Held=False)
# Pick left Jack Stud
Pick_Short_Element_From_Mat_Supply(el_name="Wooden_Element_12", L=2.0, W=0.04, H=0.1016)
# Pass left Jack to Robot1
Pass_Short_Element_G2G(el_name="Wooden_Element_12", H=0.1016)
# Place left Jack via Robot1 gripper
side_Jack_L = Place_Short_Vertical_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name="Wooden_Element_12", X=1.4784, Y=1.56, L=2.0, H=0.1016)
# Nail left Jack tangent to King Stud
Nail_Vertical_Element_With_Tangent_to_an_Element(push_to_nail=0.01, el_pose=[1.4784, 1.56, 0.0], el_dims=[2.0, 0.04, 0.1016], If_Tangent_From_Left=True, Side_Selector=side_Jack_L)
# Pick right Jack Stud
Pick_Short_Element_From_Mat_Supply(el_name="Wooden_Element_11", L=2.0, W=0.04, H=0.1016)
# Pass right Jack to Robot1
Pass_Short_Element_G2G(el_name="Wooden_Element_11", H=0.1016)
# Place right Jack via Robot1 gripper
side_Jack_R = Place_Short_Vertical_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name="Wooden_Element_11", X=1.4784, Y=2.48, L=2.0, H=0.1016)
# Nail right Jack tangent to King Stud
Nail_Vertical_Element_With_Tangent_to_an_Element(push_to_nail=0.01, el_pose=[1.4784, 2.48, 0.0], el_dims=[2.0, 0.04, 0.1016], If_Tangent_From_Left=False, Side_Selector=side_Jack_R)
# Pick first Bear Loading Header
Pick_2x10_Header(X=0.3514, L=0.96)
# Place first Bear Loading Header
conv_header1 = Place_2x10_Header(X=0.3514, Y=2.02, Z=0.0254, L=0.96, W=0.0508, H=0.254)
# Nail first Bear Loading Header
Nail_2x10_Header(push_to_nail=0.025, el_pose=[0.3514, 2.02, 0.0254], el_dims=[0.96, 0.0508, 0.254], conv_current_location=conv_header1)
# Pick second Bear Loading Header
Pick_2x10_Header(X=0.3514, L=0.96)
# Place second Bear Loading Header
conv_header2 = Place_2x10_Header(X=0.3514, Y=2.02, Z=0.0762, L=0.96, W=0.0508, H=0.254)
# Nail second Bear Loading Header
Nail_2x10_Header(push_to_nail=0.025, el_pose=[0.3514, 2.02, 0.0762], el_dims=[0.96, 0.0508, 0.254], conv_current_location=conv_header2)
# Install Bottom Plate: Pick from material supply
Pick_Long_Element_From_Mat_Supply(el_name="Bot_Stud", L=3.6576, W=0.04, H=0.1016)
# Place Bottom Plate on smart conveyor by Robot2
Place_Long_Element_On_Smart_Conveyor_by_Rob2_Gripper(el_name="Bot_Stud", X=2.4984, Y=1.8288, L=3.6576, H=0.1016)
# Nail Bottom Plate with Robot2 nail-gun
Nail_Short_Horizontal_Element_by_Rob2_NailGun(push_to_nail=0.01, el_pose=[2.4984, 1.8288, 0.06], el_dims=[3.6576, 0.04, 0.1016], conv_current_location=3.75878)
# Complementary nailing of recorded vertical elements
Complementary_Nail_Operation(push_to_nail=0.05, H=0.1016)
