# Pln-A case: No_Open
# Experimental condition: 2_HLAFs_Design_Info_With_Sample
# Repetition: 015
# Execution order: 086

# Top plate: pick, handoff, and place
Pick_Long_Element_From_Mat_Supply(el_name="Top_Stud", L=3.6576, W=0.04, H=0.1016)
Pass_Long_Element_G2G(el_name="Top_Stud", L=3.6576, H=0.1016)
Place_Long_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name="Top_Stud", X=0.02, Y=1.8288, L=3.6576, H=0.1016)
# First full-height stud
Pick_8ft_Element_From_Sloped_Table(el_name="F_Stud", L=2.4384, W=0.04, H=0.1016)
F_Stud_SS = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.02, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="F_Stud", X=1.2592, L=2.4384, H=0.1016, Side_Selector=F_Stud_SS)
# L/U stud via suction transfer
Pick_8ft_Element_From_Sloped_Table(el_name="L_U_Element_1", L=2.4384, W=0.04, H=0.1016)
Pass_8ft_Element_G2S(el_name="L_U_Element_1", L=2.4384, H=0.1016)
Place_8ft_Vertical_Element_On_Smart_Conveyor_by_Rob1_Suction(el_name="L_U_Element_1", X=1.2592, Y=0.0908, Z=0.02, L=2.4384, W=0.04, H=0.1016)
# Repetitive studs: R_Stud_2
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_2", L=2.4384, W=0.04, H=0.1016)
R_Stud_2_SS = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.3248, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_2", X=1.2592, L=2.4384, H=0.1016, Side_Selector=R_Stud_2_SS)
# R_Stud_4
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_4", L=2.4384, W=0.04, H=0.1016)
R_Stud_4_SS = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.6296, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_4", X=1.2592, L=2.4384, H=0.1016, Side_Selector=R_Stud_4_SS)
# R_Stud_6
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_6", L=2.4384, W=0.04, H=0.1016)
R_Stud_6_SS = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.9344, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_6", X=1.2592, L=2.4384, H=0.1016, Side_Selector=R_Stud_6_SS)
# R_Stud_8
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_8", L=2.4384, W=0.04, H=0.1016)
R_Stud_8_SS = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=1.2392, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_8", X=1.2592, L=2.4384, H=0.1016, Side_Selector=R_Stud_8_SS)
# R_Stud_10
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_10", L=2.4384, W=0.04, H=0.1016)
R_Stud_10_SS = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=1.544, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_10", X=1.2592, L=2.4384, H=0.1016, Side_Selector=R_Stud_10_SS)
# R_Stud_12
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_12", L=2.4384, W=0.04, H=0.1016)
R_Stud_12_SS = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=1.8488, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_12", X=1.2592, L=2.4384, H=0.1016, Side_Selector=R_Stud_12_SS)
# R_Stud_14
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_14", L=2.4384, W=0.04, H=0.1016)
R_Stud_14_SS = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=2.1536, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_14", X=1.2592, L=2.4384, H=0.1016, Side_Selector=R_Stud_14_SS)
# R_Stud_16
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_16", L=2.4384, W=0.04, H=0.1016)
R_Stud_16_SS = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=2.4584, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_16", X=1.2592, L=2.4384, H=0.1016, Side_Selector=R_Stud_16_SS)
# R_Stud_18
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_18", L=2.4384, W=0.04, H=0.1016)
R_Stud_18_SS = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=2.7632, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_18", X=1.2592, L=2.4384, H=0.1016, Side_Selector=R_Stud_18_SS)
# R_Stud_20
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_20", L=2.4384, W=0.04, H=0.1016)
R_Stud_20_SS = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=3.068, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_20", X=1.2592, L=2.4384, H=0.1016, Side_Selector=R_Stud_20_SS)
# R_Stud_22
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_22", L=2.4384, W=0.04, H=0.1016)
R_Stud_22_SS = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=3.3728, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_22", X=1.2592, L=2.4384, H=0.1016, Side_Selector=R_Stud_22_SS)
# R_Stud_24
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_24", L=2.4384, W=0.04, H=0.1016)
R_Stud_24_SS = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=3.6376, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_24", X=1.2592, L=2.4384, H=0.1016, Side_Selector=R_Stud_24_SS)
# Bottom plate: pick, place, and complementary nail
Pick_Long_Element_From_Mat_Supply(el_name="Bot_Stud", L=3.6576, W=0.04, H=0.1016)
Place_Long_Element_On_Smart_Conveyor_by_Rob2_Gripper(el_name="Bot_Stud", X=2.4984, Y=1.8288, L=3.6576, H=0.1016)
Complementary_Nail_Operation(H=0.1016)
