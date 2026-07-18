# Pln-A case: No_Open
# Experimental condition: 1_Framing_Order_HLAFs_Human_Knowledge_Design
# Repetition: 027
# Execution order: 157

# Install Top Plate
Pick_Long_Element_From_Mat_Supply(el_name="Top_Stud", L=3.6576, W=0.04, H=0.1016)
Pass_Long_Element_G2G(el_name="Top_Stud", L=3.6576, H=0.1016)
Place_Long_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name="Top_Stud", X=0.02, Y=1.8288, L=3.6576, H=0.1016)
# Install first vertical stud F_Stud
Pick_8ft_Element_From_Sloped_Table(el_name="F_Stud", L=2.4384, W=0.04, H=0.1016)
side_F = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.02, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="F_Stud", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_F, Is_Held=False)
# Install optional L/U stud
Pick_8ft_Element_From_Sloped_Table(el_name="L_U_Element_1", L=2.4384, W=0.04, H=0.1016)
Pass_Long_Element_G2G(el_name="L_U_Element_1", L=2.4384, H=0.1016)
Place_Long_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name="L_U_Element_1", X=1.2592, Y=0.0908, L=2.4384, H=0.1016)
# Install vertical stud R_Stud_2
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_2", L=2.4384, W=0.04, H=0.1016)
side_R2 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.3248, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_2", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_R2, Is_Held=False)
# Install vertical stud R_Stud_4
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_4", L=2.4384, W=0.04, H=0.1016)
side_R4 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.6296, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_4", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_R4, Is_Held=False)
# Install vertical stud R_Stud_6
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_6", L=2.4384, W=0.04, H=0.1016)
side_R6 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.9344, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_6", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_R6, Is_Held=False)
# Install vertical stud R_Stud_8
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_8", L=2.4384, W=0.04, H=0.1016)
side_R8 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=1.2392, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_8", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_R8, Is_Held=False)
# Install vertical stud R_Stud_10
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_10", L=2.4384, W=0.04, H=0.1016)
side_R10 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=1.544, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_10", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_R10, Is_Held=False)
# Install vertical stud R_Stud_12
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_12", L=2.4384, W=0.04, H=0.1016)
side_R12 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=1.8488, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_12", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_R12, Is_Held=False)
# Install vertical stud R_Stud_14
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_14", L=2.4384, W=0.04, H=0.1016)
side_R14 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=2.1536, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_14", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_R14, Is_Held=False)
# Install vertical stud R_Stud_16
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_16", L=2.4384, W=0.04, H=0.1016)
side_R16 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=2.4584, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_16", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_R16, Is_Held=False)
# Install vertical stud R_Stud_18
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_18", L=2.4384, W=0.04, H=0.1016)
side_R18 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=2.7632, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_18", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_R18, Is_Held=False)
# Install vertical stud R_Stud_20
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_20", L=2.4384, W=0.04, H=0.1016)
side_R20 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=3.068, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_20", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_R20, Is_Held=False)
# Install vertical stud R_Stud_22
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_22", L=2.4384, W=0.04, H=0.1016)
side_R22 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=3.3728, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_22", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_R22, Is_Held=False)
# Install vertical stud R_Stud_24
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_24", L=2.4384, W=0.04, H=0.1016)
side_R24 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=3.6376, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_24", X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_R24, Is_Held=False)
# Install Bottom Plate
Pick_Long_Element_From_Mat_Supply(el_name="Bot_Stud", L=3.6576, W=0.04, H=0.1016)
Place_Long_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name="Bot_Stud", X=2.4984, Y=1.8288, L=3.6576, H=0.1016)
Complementary_Nail_Operation(H=0.1016)
