# Pln-A case: 1_Door
# Experimental condition: 1_Framing_Order_HLAFs_Human_Knowledge_Design
# Repetition: 011
# Execution order: 063

# Pick Top Plate
Pick_Long_Element_From_Mat_Supply(el_name="Top_Stud", L=3.6576, W=0.04, H=0.1016)
# Pass Top Plate from Robot_2 to Robot_1
Pass_Long_Element_G2G(el_name="Top_Stud", L=3.6576, H=0.1016)
# Place Top Plate with Robot_1 gripper
Place_Long_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name="Top_Stud", X=0.02, Y=1.8288, L=3.6576, H=0.1016)
# Pick first vertical stud (F_Stud)
Pick_8ft_Element_From_Sloped_Table(el_name="F_Stud", L=2.4384, W=0.04, H=0.1016)
side_F_Stud = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.02, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="F_Stud", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_F_Stud, Is_Held=False)
# Pick L/U stud immediately after F_Stud
Pick_8ft_Element_From_Sloped_Table(el_name="L_U_Element_1", L=2.4384, W=0.04, H=0.1016)
side_LU = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.0908, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="L_U_Element_1", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_LU, Is_Held=False)
# Pick and install standard stud R_Stud_2
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_2", L=2.4384, W=0.04, H=0.1016)
side_R2 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.3248, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_2", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_R2, Is_Held=False)
# Pick and install standard stud R_Stud_4
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_4", L=2.4384, W=0.04, H=0.1016)
side_R4 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.6296, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_4", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_R4, Is_Held=False)
# Pick and install standard stud R_Stud_6
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_6", L=2.4384, W=0.04, H=0.1016)
side_R6 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.9344, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_6", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_R6, Is_Held=False)
# Pick and install standard stud R_Stud_8
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_8", L=2.4384, W=0.04, H=0.1016)
side_R8 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=1.2392, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_8", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_R8, Is_Held=False)
# Install left King Stud
Pick_8ft_Element_From_Sloped_Table(el_name="Wooden_Element_6", L=2.4384, W=0.04, H=0.1016)
side_King_L = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=1.52, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="Wooden_Element_6", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_King_L, Is_Held=False)
# Install right King Stud
Pick_8ft_Element_From_Sloped_Table(el_name="Wooden_Element_7", L=2.4384, W=0.04, H=0.1016)
side_King_R = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=2.52, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="Wooden_Element_7", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_King_R, Is_Held=False)
# Install left Jack Stud (short vertical)
Pick_Short_Element_From_Mat_Supply(el_name="Wooden_Element_12", L=2.0, W=0.04, H=0.1016)
Pass_Short_Element_G2G(el_name="Wooden_Element_12", H=0.1016)
side_Jack_L = Place_Short_Vertical_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name="Wooden_Element_12", X=1.4784, Y=1.56, L=2.0, H=0.1016)
Nail_Vertical_Element_With_Tangent_to_an_Element(el_pose=[1.4784, 1.56, 0.0], el_dims=[2.0, 0.04, 0.1016], If_Tangent_From_Left=True, Side_Selector=side_Jack_L)
# Install right Jack Stud (short vertical)
Pick_Short_Element_From_Mat_Supply(el_name="Wooden_Element_11", L=2.0, W=0.04, H=0.1016)
Pass_Short_Element_G2G(el_name="Wooden_Element_11", H=0.1016)
side_Jack_R = Place_Short_Vertical_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name="Wooden_Element_11", X=1.4784, Y=2.48, L=2.0, H=0.1016)
Nail_Vertical_Element_With_Tangent_to_an_Element(el_pose=[1.4784, 2.48, 0.0], el_dims=[2.0, 0.04, 0.1016], If_Tangent_From_Left=False, Side_Selector=side_Jack_R)
# Install first Bear Loading header plank
Pick_Short_Element_From_Mat_Supply(el_name="Wooden_Element_13", L=0.96, W=0.0508, H=0.254)
Pass_Short_Element_G2G(el_name="Wooden_Element_13", H=0.254)
header1_loc = Place_Short_Horizontal_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name="Wooden_Element_13", X=0.3514, Y=2.02, L=0.96, H=0.254)
Nail_Short_Horizontal_Element_by_Rob1_NailGun(el_pose=[0.3514, 2.02, 0.0254], el_dims=[0.96, 0.0508, 0.254], conv_current_location=header1_loc)
# Install second Bear Loading header plank
Pick_Short_Element_From_Mat_Supply(el_name="Wooden_Element_14", L=0.96, W=0.0508, H=0.254)
Pass_Short_Element_G2G(el_name="Wooden_Element_14", H=0.254)
header2_loc = Place_Short_Horizontal_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name="Wooden_Element_14", X=0.3514, Y=2.02, L=0.96, H=0.254)
Nail_Short_Horizontal_Element_by_Rob1_NailGun(el_pose=[0.3514, 2.02, 0.0762], el_dims=[0.96, 0.0508, 0.254], conv_current_location=header2_loc)
# Continue with right-side standard studs R_Stud_18
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_18", L=2.4384, W=0.04, H=0.1016)
side_R18 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=2.7632, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_18", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_R18, Is_Held=False)
# Install standard stud R_Stud_20
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_20", L=2.4384, W=0.04, H=0.1016)
side_R20 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=3.068, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_20", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_R20, Is_Held=False)
# Install standard stud R_Stud_22
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_22", L=2.4384, W=0.04, H=0.1016)
side_R22 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=3.3728, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_22", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_R22, Is_Held=False)
# Install standard stud R_Stud_24
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_24", L=2.4384, W=0.04, H=0.1016)
side_R24 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=3.6376, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_24", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_R24, Is_Held=False)
# Install Bottom Plate
Pick_Long_Element_From_Mat_Supply(el_name="Bot_Stud", L=3.6576, W=0.04, H=0.1016)
Pass_Long_Element_G2G(el_name="Bot_Stud", L=3.6576, H=0.1016)
Place_Long_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name="Bot_Stud", X=2.4984, Y=1.8288, L=3.6576, H=0.1016)
Nail_Short_Horizontal_Element_by_Rob1_NailGun(el_pose=[2.4984, 1.8288, 0.06], el_dims=[3.6576, 0.04, 0.1016], conv_current_location=None)
# Perform complementary nailing passes for any remaining vertical elements
Complementary_Nail_Operation(H=2.4384)
