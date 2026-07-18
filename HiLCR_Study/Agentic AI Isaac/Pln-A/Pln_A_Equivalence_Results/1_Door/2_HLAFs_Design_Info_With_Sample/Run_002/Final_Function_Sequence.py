# Pln-A case: 1_Door
# Experimental condition: 2_HLAFs_Design_Info_With_Sample
# Repetition: 002
# Execution order: 010

# Pick and place the top plate
Pick_Long_Element_From_Mat_Supply(el_name="Top_Stud", L=3.6576, W=0.04, H=0.1016)
Pass_Long_Element_G2G(el_name="Top_Stud", L=3.6576, H=0.1016)
Place_Long_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name="Top_Stud", X=0.02, Y=1.8288, L=3.6576, H=0.1016)
# Process first vertical stud F_Stud
Pick_8ft_Element_From_Sloped_Table(el_name="F_Stud", L=2.4384, W=0.04, H=0.1016)
F_Stud_SS = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.02, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="F_Stud", X=1.2592, push_to_nail=0.05, L=2.4384, H=0.1016, Side_Selector=F_Stud_SS)
# Process L/U Stud
Pick_8ft_Element_From_Sloped_Table(el_name="L_U_Element_1", L=2.4384, W=0.04, H=0.1016)
Pass_8ft_Element_G2S(el_name="L_U_Element_1", L=2.4384, H=0.1016)
Place_8ft_Vertical_Element_On_Smart_Conveyor_by_Rob1_Suction(el_name="L_U_Element_1", X=1.2592, Y=0.0908, Z=0.02, L=2.4384, W=0.04, H=0.1016)
# Process vertical stud R_Stud_2
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_2", L=2.4384, W=0.04, H=0.1016)
R_Stud_2_SS = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.3248, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_2", X=1.2592, push_to_nail=0.05, L=2.4384, H=0.1016, Side_Selector=R_Stud_2_SS)
# Process vertical stud R_Stud_4
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_4", L=2.4384, W=0.04, H=0.1016)
R_Stud_4_SS = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.6296, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_4", X=1.2592, push_to_nail=0.05, L=2.4384, H=0.1016, Side_Selector=R_Stud_4_SS)
# Process vertical stud R_Stud_6
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_6", L=2.4384, W=0.04, H=0.1016)
R_Stud_6_SS = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.9344, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_6", X=1.2592, push_to_nail=0.05, L=2.4384, H=0.1016, Side_Selector=R_Stud_6_SS)
# Process vertical stud R_Stud_8
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_8", L=2.4384, W=0.04, H=0.1016)
R_Stud_8_SS = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=1.2392, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_8", X=1.2592, push_to_nail=0.05, L=2.4384, H=0.1016, Side_Selector=R_Stud_8_SS)
# Process King Stud Wooden_Element_6
Pick_8ft_Element_From_Sloped_Table(el_name="Wooden_Element_6", L=2.4384, W=0.04, H=0.1016)
Wooden_Element_6_SS = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=1.52, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="Wooden_Element_6", X=1.2592, push_to_nail=0.05, L=2.4384, H=0.1016, Side_Selector=Wooden_Element_6_SS)
# Process King Stud Wooden_Element_7
Pick_8ft_Element_From_Sloped_Table(el_name="Wooden_Element_7", L=2.4384, W=0.04, H=0.1016)
Wooden_Element_7_SS = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=2.52, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="Wooden_Element_7", X=1.2592, push_to_nail=0.05, L=2.4384, H=0.1016, Side_Selector=Wooden_Element_7_SS)
# Process vertical stud R_Stud_18
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_18", L=2.4384, W=0.04, H=0.1016)
R_Stud_18_SS = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=2.7632, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_18", X=1.2592, push_to_nail=0.05, L=2.4384, H=0.1016, Side_Selector=R_Stud_18_SS)
# Process vertical stud R_Stud_20
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_20", L=2.4384, W=0.04, H=0.1016)
R_Stud_20_SS = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=3.068, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_20", X=1.2592, push_to_nail=0.05, L=2.4384, H=0.1016, Side_Selector=R_Stud_20_SS)
# Process vertical stud R_Stud_22
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_22", L=2.4384, W=0.04, H=0.1016)
R_Stud_22_SS = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=3.3728, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_22", X=1.2592, push_to_nail=0.05, L=2.4384, H=0.1016, Side_Selector=R_Stud_22_SS)
# Process vertical stud R_Stud_24
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_24", L=2.4384, W=0.04, H=0.1016)
R_Stud_24_SS = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=3.6376, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_24", X=1.2592, push_to_nail=0.05, L=2.4384, H=0.1016, Side_Selector=R_Stud_24_SS)
# Process Jack Stud Wooden_Element_12
Pick_Short_Element_From_Mat_Supply(el_name="Wooden_Element_12", L=2.0, W=0.04, H=0.1016)
Wooden_Element_12_SS = Drop_Short_Vertical_Element_With_Tangent_to_an_Element(el_name="Wooden_Element_12", X=1.4784, Y=1.56, L=2.0, H=0.1016, If_Tangent_From_Left=True)
Nail_Vertical_Element_With_Tangent_to_an_Element(push_to_nail=0.05, el_pose=[1.4784, 1.56, 0], el_dims=[2.0, 0.04, 0.1016], If_Tangent_From_Left=True, Side_Selector=Wooden_Element_12_SS)
# Process Jack Stud Wooden_Element_11
Pick_Short_Element_From_Mat_Supply(el_name="Wooden_Element_11", L=2.0, W=0.04, H=0.1016)
Wooden_Element_11_SS = Drop_Short_Vertical_Element_With_Tangent_to_an_Element(el_name="Wooden_Element_11", X=1.4784, Y=2.48, L=2.0, H=0.1016, If_Tangent_From_Left=False)
Nail_Vertical_Element_With_Tangent_to_an_Element(push_to_nail=0.05, el_pose=[1.4784, 2.48, 0], el_dims=[2.0, 0.04, 0.1016], If_Tangent_From_Left=False, Side_Selector=Wooden_Element_11_SS)
# Process Bear Loading Element Wooden_Element_13
Pick_2x10_Header(X=0.3514, L=0.96)
Cut_2x10_Header(L=0.96, W=0.0508, H=0.254)
BL_1_CurConv = Place_2x10_Header(X=0.3514, Y=2.02, Z=0.0254, L=0.96, W=0.0508, H=0.254)
Nail_2x10_Header(push_to_nail=0.125, el_pose=[0.3514, 2.02, 0.0254], el_dims=[0.96, 0.0508, 0.254], conv_current_location=BL_1_CurConv)
# Process Bear Loading Element Wooden_Element_14
Pick_2x10_Header(X=0.3514, L=0.96)
Cut_2x10_Header(L=0.96, W=0.0508, H=0.254)
BL_2_CurConv = Place_2x10_Header(X=0.3514, Y=2.02, Z=0.0762, L=0.96, W=0.0508, H=0.254)
Nail_2x10_Header(push_to_nail=0.125, el_pose=[0.3514, 2.02, 0.0762], el_dims=[0.96, 0.0508, 0.254], conv_current_location=BL_2_CurConv)
# Pick and place the bottom plate
Pick_Long_Element_From_Mat_Supply(el_name="Bot_Stud", L=3.6576, W=0.04, H=0.1016)
Place_Long_Element_On_Smart_Conveyor_by_Rob2_Gripper(el_name="Bot_Stud", X=2.4984, Y=1.8288, L=3.6576, H=0.1016)
# Complementary nailing operations along vertical elements
Complementary_Nail_Operation(push_to_nail=0.05, H=0.1016)
