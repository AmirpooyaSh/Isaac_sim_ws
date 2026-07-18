# Pln-A case: No_Open
# Experimental condition: 1_Framing_Order_HLAFs_Human_Knowledge_Design
# Repetition: 001
# Execution order: 001

# Install Top Plate (Pick -> Pass -> Place)
Pick_Long_Element_From_Mat_Supply(el_name="Top_Stud", L=3.6576, W=0.04, H=0.1016)
Pass_Long_Element_G2G(el_name="Top_Stud", L=3.6576, H=0.1016)
Place_Long_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name="Top_Stud", X=0.02, Y=1.8288, L=3.6576, H=0.1016)
# Install First Vertical Stud (F_Stud)
Pick_8ft_Element_From_Sloped_Table(el_name="F_Stud", W=0.04, H=0.1016)
side_selector_F_Stud = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.02, H=2.4384)
Nail_and_Release_Vertical_Element(el_name="F_Stud", X=1.2592, H=2.4384, Side_Selector=side_selector_F_Stud, Is_Held=False)
# Install L/U Stud (L_U_Element_1)
Pick_8ft_Element_From_Sloped_Table(el_name="L_U_Element_1", W=0.04, H=0.1016)
side_selector_LU_1 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.0908, H=2.4384)
Nail_and_Release_Vertical_Element(el_name="L_U_Element_1", X=1.2592, H=2.4384, Side_Selector=side_selector_LU_1, Is_Held=False)
# Install Vertical Stud R_Stud_2
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_2", W=0.04, H=0.1016)
side_selector_R_Stud_2 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.3248, H=2.4384)
Nail_and_Release_Vertical_Element(el_name="R_Stud_2", X=1.2592, H=2.4384, Side_Selector=side_selector_R_Stud_2, Is_Held=False)
# Install Vertical Stud R_Stud_4
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_4", W=0.04, H=0.1016)
side_selector_R_Stud_4 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.6296, H=2.4384)
Nail_and_Release_Vertical_Element(el_name="R_Stud_4", X=1.2592, H=2.4384, Side_Selector=side_selector_R_Stud_4, Is_Held=False)
# Install Vertical Stud R_Stud_6
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_6", W=0.04, H=0.1016)
side_selector_R_Stud_6 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.9344, H=2.4384)
Nail_and_Release_Vertical_Element(el_name="R_Stud_6", X=1.2592, H=2.4384, Side_Selector=side_selector_R_Stud_6, Is_Held=False)
# Install Vertical Stud R_Stud_8
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_8", W=0.04, H=0.1016)
side_selector_R_Stud_8 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=1.2392, H=2.4384)
Nail_and_Release_Vertical_Element(el_name="R_Stud_8", X=1.2592, H=2.4384, Side_Selector=side_selector_R_Stud_8, Is_Held=False)
# Install Vertical Stud R_Stud_10
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_10", W=0.04, H=0.1016)
side_selector_R_Stud_10 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=1.544, H=2.4384)
Nail_and_Release_Vertical_Element(el_name="R_Stud_10", X=1.2592, H=2.4384, Side_Selector=side_selector_R_Stud_10, Is_Held=False)
# Install Vertical Stud R_Stud_12
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_12", W=0.04, H=0.1016)
side_selector_R_Stud_12 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=1.8488, H=2.4384)
Nail_and_Release_Vertical_Element(el_name="R_Stud_12", X=1.2592, H=2.4384, Side_Selector=side_selector_R_Stud_12, Is_Held=False)
# Install Vertical Stud R_Stud_14
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_14", W=0.04, H=0.1016)
side_selector_R_Stud_14 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=2.1536, H=2.4384)
Nail_and_Release_Vertical_Element(el_name="R_Stud_14", X=1.2592, H=2.4384, Side_Selector=side_selector_R_Stud_14, Is_Held=False)
# Install Vertical Stud R_Stud_16
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_16", W=0.04, H=0.1016)
side_selector_R_Stud_16 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=2.4584, H=2.4384)
Nail_and_Release_Vertical_Element(el_name="R_Stud_16", X=1.2592, H=2.4384, Side_Selector=side_selector_R_Stud_16, Is_Held=False)
# Install Vertical Stud R_Stud_18
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_18", W=0.04, H=0.1016)
side_selector_R_Stud_18 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=2.7632, H=2.4384)
Nail_and_Release_Vertical_Element(el_name="R_Stud_18", X=1.2592, H=2.4384, Side_Selector=side_selector_R_Stud_18, Is_Held=False)
# Install Vertical Stud R_Stud_20
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_20", W=0.04, H=0.1016)
side_selector_R_Stud_20 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=3.068, H=2.4384)
Nail_and_Release_Vertical_Element(el_name="R_Stud_20", X=1.2592, H=2.4384, Side_Selector=side_selector_R_Stud_20, Is_Held=False)
# Install Vertical Stud R_Stud_22
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_22", W=0.04, H=0.1016)
side_selector_R_Stud_22 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=3.3728, H=2.4384)
Nail_and_Release_Vertical_Element(el_name="R_Stud_22", X=1.2592, H=2.4384, Side_Selector=side_selector_R_Stud_22, Is_Held=False)
# Install Vertical Stud R_Stud_24
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_24", W=0.04, H=0.1016)
side_selector_R_Stud_24 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=3.6376, H=2.4384)
Nail_and_Release_Vertical_Element(el_name="R_Stud_24", X=1.2592, H=2.4384, Side_Selector=side_selector_R_Stud_24, Is_Held=False)
# Install Bottom Plate (Pick -> Place)
Pick_Long_Element_From_Mat_Supply(el_name="Bot_Stud", L=3.6576, W=0.04, H=0.1016)
Place_Long_Element_On_Smart_Conveyor_by_Rob2_Gripper(el_name="Bot_Stud", X=2.4984, Y=1.8288, L=3.6576, H=0.1016)
# Perform complementary nailing on all recorded vertical elements
Complementary_Nail_Operation(H=2.4384)
