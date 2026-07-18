# Pln-A case: No_Open
# Experimental condition: 1_Framing_Order_HLAFs_Human_Knowledge_Design
# Repetition: 028
# Execution order: 163

# === Install Top Plate ===
Pick_Long_Element_From_Mat_Supply(el_name="Top_Stud", L=3.6576, W=0.04, H=0.1016)
Pass_Long_Element_G2G(el_name="Top_Stud", L=3.6576, H=0.1016)
Place_Long_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name="Top_Stud", X=0.02, Y=1.8288, L=3.6576, H=0.1016)
# === Install Standard Vertical Stud: F_Stud ===
Pick_8ft_Element_From_Sloped_Table(el_name="F_Stud", L=2.4384, W=0.04, H=0.1016)
F_Stud_side = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.02, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="F_Stud", X=1.2592, L=2.4384, H=0.1016, Side_Selector=F_Stud_side)
# === Install Optional L/U Stud: L_U_Element_1 ===
Pick_8ft_Element_From_Sloped_Table(el_name="L_U_Element_1", L=2.4384, W=0.04, H=0.1016)
Pass_Long_Element_G2G(el_name="L_U_Element_1", L=2.4384, H=0.1016)
Place_Long_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name="L_U_Element_1", X=1.2592, Y=0.0908, L=2.4384, H=0.1016)
# === Install Standard Vertical Stud: R_Stud_2 ===
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_2", L=2.4384, W=0.04, H=0.1016)
R_Stud_2_side = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.3248, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_2", X=1.2592, L=2.4384, H=0.1016, Side_Selector=R_Stud_2_side)
# === Install Standard Vertical Stud: R_Stud_4 ===
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_4", L=2.4384, W=0.04, H=0.1016)
R_Stud_4_side = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.6296, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_4", X=1.2592, L=2.4384, H=0.1016, Side_Selector=R_Stud_4_side)
# === Install Standard Vertical Stud: R_Stud_6 ===
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_6", L=2.4384, W=0.04, H=0.1016)
R_Stud_6_side = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.9344, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_6", X=1.2592, L=2.4384, H=0.1016, Side_Selector=R_Stud_6_side)
# === Install Standard Vertical Stud: R_Stud_8 ===
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_8", L=2.4384, W=0.04, H=0.1016)
R_Stud_8_side = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=1.2392, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_8", X=1.2592, L=2.4384, H=0.1016, Side_Selector=R_Stud_8_side)
# === Install Standard Vertical Stud: R_Stud_10 ===
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_10", L=2.4384, W=0.04, H=0.1016)
R_Stud_10_side = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=1.544, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_10", X=1.2592, L=2.4384, H=0.1016, Side_Selector=R_Stud_10_side)
# === Install Standard Vertical Stud: R_Stud_12 ===
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_12", L=2.4384, W=0.04, H=0.1016)
R_Stud_12_side = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=1.8488, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_12", X=1.2592, L=2.4384, H=0.1016, Side_Selector=R_Stud_12_side)
# === Install Standard Vertical Stud: R_Stud_14 ===
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_14", L=2.4384, W=0.04, H=0.1016)
R_Stud_14_side = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=2.1536, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_14", X=1.2592, L=2.4384, H=0.1016, Side_Selector=R_Stud_14_side)
# === Install Standard Vertical Stud: R_Stud_16 ===
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_16", L=2.4384, W=0.04, H=0.1016)
R_Stud_16_side = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=2.4584, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_16", X=1.2592, L=2.4384, H=0.1016, Side_Selector=R_Stud_16_side)
# === Install Standard Vertical Stud: R_Stud_18 ===
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_18", L=2.4384, W=0.04, H=0.1016)
R_Stud_18_side = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=2.7632, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_18", X=1.2592, L=2.4384, H=0.1016, Side_Selector=R_Stud_18_side)
# === Install Standard Vertical Stud: R_Stud_20 ===
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_20", L=2.4384, W=0.04, H=0.1016)
R_Stud_20_side = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=3.068, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_20", X=1.2592, L=2.4384, H=0.1016, Side_Selector=R_Stud_20_side)
# === Install Standard Vertical Stud: R_Stud_22 ===
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_22", L=2.4384, W=0.04, H=0.1016)
R_Stud_22_side = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=3.3728, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_22", X=1.2592, L=2.4384, H=0.1016, Side_Selector=R_Stud_22_side)
# === Install Standard Vertical Stud: R_Stud_24 ===
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_24", L=2.4384, W=0.04, H=0.1016)
R_Stud_24_side = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=3.6376, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_24", X=1.2592, L=2.4384, H=0.1016, Side_Selector=R_Stud_24_side)
# === Install Bottom Plate ===
Pick_Long_Element_From_Mat_Supply(el_name="Bot_Stud", L=3.6576, W=0.04, H=0.1016)
bottom_conv_loc = Place_Short_Horizontal_Element_On_Smart_Conveyor_by_Rob2_Gripper(el_name="Bot_Stud", X=2.4984, Y=1.8288, L=3.6576, H=0.1016)
Nail_Short_Horizontal_Element_by_Rob2_NailGun(el_pose=[2.4984, 1.8288, 0.06], el_dims=[3.6576, 0.04, 0.1016], conv_current_location=bottom_conv_loc)
