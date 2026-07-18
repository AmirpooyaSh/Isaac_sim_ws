# Pln-A case: No_Open
# Experimental condition: 1_Framing_Order_HLAFs_Human_Knowledge_Design
# Repetition: 006
# Execution order: 031

# Install Top Plate
Pick_Long_Element_From_Mat_Supply(el_name="Top_Stud", L=3.6576, W=0.04, H=0.1016)
Pass_Long_Element_G2G(el_name="Top_Stud", L=3.6576, H=0.1016)
Place_Long_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name="Top_Stud", X=0.02, Y=1.8288, L=3.6576, H=0.1016)
# Install first Vertical Stud (F_Stud)
Pick_8ft_Element_From_Sloped_Table(el_name="F_Stud", L=2.4384, W=0.04, H=0.1016)
side_selector_F_Stud = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.02, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="F_Stud", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_selector_F_Stud, Is_Held=False)
# Install optional L/U Stud immediately after first stud
Pick_8ft_Element_From_Sloped_Table(el_name="L_U_Element_1", L=2.4384, W=0.04, H=0.1016)
Pass_8ft_Element_G2S(el_name="L_U_Element_1", L=2.4384, H=0.1016)
Place_8ft_Vertical_Element_On_Smart_Conveyor_by_Rob1_Suction(el_name="L_U_Element_1", X=1.2592, Y=0.0908, Z=0.02, L=2.4384, W=0.04, H=0.1016)
# Install Vertical Stud R_Stud_2
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_2", L=2.4384, W=0.04, H=0.1016)
side_selector_R_Stud_2 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.3248, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_2", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_selector_R_Stud_2, Is_Held=False)
# Install Vertical Stud R_Stud_4
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_4", L=2.4384, W=0.04, H=0.1016)
side_selector_R_Stud_4 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.6296, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_4", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_selector_R_Stud_4, Is_Held=False)
# Install Vertical Stud R_Stud_6
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_6", L=2.4384, W=0.04, H=0.1016)
side_selector_R_Stud_6 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.9344, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_6", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_selector_R_Stud_6, Is_Held=False)
# Install Vertical Stud R_Stud_8
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_8", L=2.4384, W=0.04, H=0.1016)
side_selector_R_Stud_8 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=1.2392, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_8", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_selector_R_Stud_8, Is_Held=False)
# Install Vertical Stud R_Stud_10
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_10", L=2.4384, W=0.04, H=0.1016)
side_selector_R_Stud_10 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=1.544, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_10", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_selector_R_Stud_10, Is_Held=False)
# Install Vertical Stud R_Stud_12
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_12", L=2.4384, W=0.04, H=0.1016)
side_selector_R_Stud_12 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=1.8488, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_12", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_selector_R_Stud_12, Is_Held=False)
# Install Vertical Stud R_Stud_14
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_14", L=2.4384, W=0.04, H=0.1016)
side_selector_R_Stud_14 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=2.1536, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_14", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_selector_R_Stud_14, Is_Held=False)
# Install Vertical Stud R_Stud_16
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_16", L=2.4384, W=0.04, H=0.1016)
side_selector_R_Stud_16 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=2.4584, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_16", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_selector_R_Stud_16, Is_Held=False)
# Install Vertical Stud R_Stud_18
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_18", L=2.4384, W=0.04, H=0.1016)
side_selector_R_Stud_18 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=2.7632, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_18", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_selector_R_Stud_18, Is_Held=False)
# Install Vertical Stud R_Stud_20
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_20", L=2.4384, W=0.04, H=0.1016)
side_selector_R_Stud_20 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=3.068, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_20", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_selector_R_Stud_20, Is_Held=False)
# Install Vertical Stud R_Stud_22
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_22", L=2.4384, W=0.04, H=0.1016)
side_selector_R_Stud_22 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=3.3728, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_22", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_selector_R_Stud_22, Is_Held=False)
# Install Vertical Stud R_Stud_24
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_24", L=2.4384, W=0.04, H=0.1016)
side_selector_R_Stud_24 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=3.6376, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_24", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_selector_R_Stud_24, Is_Held=False)
# Install Bottom Plate
Pick_Long_Element_From_Mat_Supply(el_name="Bot_Stud", L=3.6576, W=0.04, H=0.1016)
conv_loc_Bot_Stud = Place_Short_Horizontal_Element_On_Smart_Conveyor_by_Rob2_Gripper(el_name="Bot_Stud", X=2.4984, Y=1.8288, L=3.6576, H=0.1016)
Nail_Short_Horizontal_Element_by_Rob2_NailGun(el_pose=[2.4984, 1.8288, 0.06], el_dims=[3.6576, 0.04, 0.1016], conv_current_location=conv_loc_Bot_Stud)
