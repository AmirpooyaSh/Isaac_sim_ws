# Pln-A case: 1_Door_1_Window
# Experimental condition: 2_HLAFs_Design_Info_With_Sample
# Repetition: 019
# Execution order: 114

# Pick, pass, and place the top plate
Pick_Long_Element_From_Mat_Supply(el_name="Top_Stud", L=3.6576, W=0.04, H=0.1016)
Pass_Long_Element_G2G(el_name="Top_Stud", L=3.6576, H=0.1016)
Place_Long_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name="Top_Stud", X=0.02, Y=1.8288, L=3.6576, H=0.1016)
# Pick, place, nail and release the front stud (F_Stud)
Pick_8ft_Element_From_Sloped_Table(el_name="F_Stud", L=2.4384, W=0.04, H=0.1016)
F_Stud_SS = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.02, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="F_Stud", X=1.2592, L=2.4384, H=0.1016, Side_Selector=F_Stud_SS)
# Pick, pass, and place the L/U element
Pick_8ft_Element_From_Sloped_Table(el_name="L_U_Element_1", L=2.4384, W=0.04, H=0.1016)
Pass_8ft_Element_G2S(el_name="L_U_Element_1", L=2.4384, H=0.1016)
Place_8ft_Vertical_Element_On_Smart_Conveyor_by_Rob1_Suction(el_name="L_U_Element_1", X=1.2592, Y=0.0908, Z=0.02, L=2.4384, W=0.04, H=0.1016)
# Pick, place, nail and release the first repetitive stud (R_Stud_1)
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_1", L=2.4384, W=0.04, H=0.1016)
R_Stud_1_SS = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.5, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_1", X=1.2592, L=2.4384, H=0.1016, Side_Selector=R_Stud_1_SS)
# Pick, place, nail and release the left king stud LK_1
Pick_8ft_Element_From_Sloped_Table(el_name="LK_1", L=2.4384, W=0.04, H=0.1016)
LK_1_SS = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.72, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="LK_1", X=1.2592, L=2.4384, H=0.1016, Side_Selector=LK_1_SS)
# Pick, place, nail and release the right king stud RK_1
Pick_8ft_Element_From_Sloped_Table(el_name="RK_1", L=2.4384, W=0.04, H=0.1016)
RK_1_SS = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=1.68, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="RK_1", X=1.2592, L=2.4384, H=0.1016, Side_Selector=RK_1_SS)
# Pick, drop tangent, and nail the left jack stud LJ_1
Pick_Short_Element_From_Mat_Supply(el_name="LJ_1", L=2.0, W=0.04, H=0.1016)
LJ_1_SS = Drop_Short_Vertical_Element_With_Tangent_to_an_Element(el_name="LJ_1", X=1.4784, Y=0.76, L=2.0, H=0.1016, If_Tangent_From_Left=True)
Nail_Vertical_Element_With_Tangent_to_an_Element(el_pose=[1.4784, 0.76, 0], el_dims=[2.0, 0.04, 0.1016], If_Tangent_From_Left=True, Side_Selector=LJ_1_SS)
# Pick, drop tangent, and nail the right jack stud RJ_1
Pick_Short_Element_From_Mat_Supply(el_name="RJ_1", L=2.0, W=0.04, H=0.1016)
RJ_1_SS = Drop_Short_Vertical_Element_With_Tangent_to_an_Element(el_name="RJ_1", X=1.4784, Y=1.64, L=2.0, H=0.1016, If_Tangent_From_Left=False)
Nail_Vertical_Element_With_Tangent_to_an_Element(el_pose=[1.4784, 1.64, 0], el_dims=[2.0, 0.04, 0.1016], If_Tangent_From_Left=False, Side_Selector=RJ_1_SS)
# Pick, pass, place, and nail the top sill plate (TSP_1)
Pick_Short_Element_From_Mat_Supply(el_name="TSP_1", L=0.92, W=0.04, H=0.1016)
Pass_Short_Element_G2G(el_name="TSP_1", H=0.1016)
TSP_1_CurConv = Place_Short_Horizontal_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name="TSP_1", X=0.4584, Y=1.2, L=0.92, H=0.1016)
Nail_Short_Horizontal_Element_by_Rob1_NailGun(el_pose=[0.4584, 1.2, 0], el_dims=[0.92, 0.04, 0.1016], conv_current_location=TSP_1_CurConv)
# Pick, pass, place, nail (hold) top cripple stud TCP_1
Pick_Short_Element_From_Mat_Supply(el_name="TCP_1", L=0.3984, W=0.04, H=0.1016)
Pass_Short_Element_G2G(el_name="TCP_1", H=0.1016)
TCP_1_SS = Place_Short_Vertical_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name="TCP_1", X=0.2392, Y=1.2, L=0.3984, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="TCP_1", X=0.2392, L=0.3984, H=0.1016, Side_Selector=TCP_1_SS, Is_Held=True)
# Pick, pass, place, nail (hold) top cripple stud TCP_2
Pick_Short_Element_From_Mat_Supply(el_name="TCP_2", L=0.3984, W=0.04, H=0.1016)
Pass_Short_Element_G2G(el_name="TCP_2", H=0.1016)
TCP_2_SS = Place_Short_Vertical_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name="TCP_2", X=0.2392, Y=0.98, L=0.3984, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="TCP_2", X=0.2392, L=0.3984, H=0.1016, Side_Selector=TCP_2_SS, Is_Held=True)
# Pick, pass, place, nail (hold) top cripple stud TCP_3
Pick_Short_Element_From_Mat_Supply(el_name="TCP_3", L=0.3984, W=0.04, H=0.1016)
Pass_Short_Element_G2G(el_name="TCP_3", H=0.1016)
TCP_3_SS = Place_Short_Vertical_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name="TCP_3", X=0.2392, Y=1.42, L=0.3984, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="TCP_3", X=0.2392, L=0.3984, H=0.1016, Side_Selector=TCP_3_SS, Is_Held=True)
# Pick, place, nail and release repetitive stud R_Stud_2
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_2", L=2.4384, W=0.04, H=0.1016)
R_Stud_2_SS = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=1.9588, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_2", X=1.2592, L=2.4384, H=0.1016, Side_Selector=R_Stud_2_SS)
# Pick, place, nail and release left king stud LK_2
Pick_8ft_Element_From_Sloped_Table(el_name="LK_2", L=2.4384, W=0.04, H=0.1016)
LK_2_SS = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=3.1376, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="LK_2", X=1.2592, L=2.4384, H=0.1016, Side_Selector=LK_2_SS)
# Pick, place, nail and release right king stud RK_2
Pick_8ft_Element_From_Sloped_Table(el_name="RK_2", L=2.4384, W=0.04, H=0.1016)
RK_2_SS = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=2.2376, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="RK_2", X=1.2592, L=2.4384, H=0.1016, Side_Selector=RK_2_SS)
# Pick, drop tangent, and nail left jack stud LJ_2
Pick_Short_Element_From_Mat_Supply(el_name="LJ_2", L=1.8, W=0.04, H=0.1016)
LJ_2_SS = Drop_Short_Vertical_Element_With_Tangent_to_an_Element(el_name="LJ_2", X=1.5784, Y=2.2776, L=1.8, H=0.1016, If_Tangent_From_Left=True)
Nail_Vertical_Element_With_Tangent_to_an_Element(el_pose=[1.5784, 2.2776, 0], el_dims=[1.8, 0.04, 0.1016], If_Tangent_From_Left=True, Side_Selector=LJ_2_SS)
# Pick, drop tangent, and nail right jack stud RJ_2
Pick_Short_Element_From_Mat_Supply(el_name="RJ_2", L=1.8, W=0.04, H=0.1016)
RJ_2_SS = Drop_Short_Vertical_Element_With_Tangent_to_an_Element(el_name="RJ_2", X=1.5784, Y=3.0976, L=1.8, H=0.1016, If_Tangent_From_Left=False)
Nail_Vertical_Element_With_Tangent_to_an_Element(el_pose=[1.5784, 3.0976, 0], el_dims=[1.8, 0.04, 0.1016], If_Tangent_From_Left=False, Side_Selector=RJ_2_SS)
# Pick, cut, place, and nail the first 2x10 header BL_1
Pick_2x10_Header(X=0.5514, L=0.86)
Cut_2x10_Header(L=0.86, W=0.0508, H=0.254)
BL_1_Placement_CurConv = Place_2x10_Header(X=0.5514, Y=2.6876, Z=0.0254, L=0.86, W=0.0508, H=0.254)
Nail_2x10_Header(el_pose=[0.5514, 2.6876, 0.0254], el_dims=[0.86, 0.0508, 0.254], conv_current_location=BL_1_Placement_CurConv)
# Pick, cut, place, and nail the second 2x10 header BL_2
Pick_2x10_Header(X=0.5514, L=0.96)
Cut_2x10_Header(L=0.86, W=0.0508, H=0.254)
BL_2_Placement_CurConv = Place_2x10_Header(X=0.5514, Y=2.6876, Z=0.0762, L=0.86, W=0.0508, H=0.254)
Nail_2x10_Header(el_pose=[0.5514, 2.6876, 0.0762], el_dims=[0.86, 0.0508, 0.254], conv_current_location=BL_2_Placement_CurConv)
# Pick, place, and nail lower sill plate BS_1
Pick_Short_Element_From_Mat_Supply(el_name="BS_1", L=0.78, W=0.04, H=0.1016)
BS_1_CurConv = Place_Short_Horizontal_Element_On_Smart_Conveyor_by_Rob2_Gripper(el_name="BS_1", X=1.8984, Y=2.6876, L=0.78, H=0.1016)
Nail_Short_Horizontal_Element_by_Rob2_NailGun(el_pose=[1.8984, 2.6876, 0], el_dims=[0.78, 0.04, 0.1016], conv_current_location=BS_1_CurConv)
# Pick and place lower cripple stud LC_1
Pick_Short_Element_From_Mat_Supply(el_name="LC_1", L=0.56, W=0.04, H=0.1016)
Place_Short_Vertical_Element_On_Smart_Conveyor_by_Rob2_Gripper(el_name="LC_1", X=2.1984, Y=2.5376, L=0.56, H=0.1016)
# Pick and place lower cripple stud LC_2
Pick_Short_Element_From_Mat_Supply(el_name="LC_2", L=0.56, W=0.04, H=0.1016)
Place_Short_Vertical_Element_On_Smart_Conveyor_by_Rob2_Gripper(el_name="LC_2", X=2.1984, Y=2.8376, L=0.56, H=0.1016)
# Pick, place, nail and release repetitive stud R_Stud_3
Pick_8ft_Element_From_Sloped_Table(el_name="R_Stud_3", L=2.4384, W=0.04, H=0.1016)
R_Stud_3_SS = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=3.6376, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name="R_Stud_3", X=1.2592, L=2.4384, H=0.1016, Side_Selector=R_Stud_3_SS)
# Pick and place bottom plate
Pick_Long_Element_From_Mat_Supply(el_name="Bot_Stud", L=3.6576, W=0.04, H=0.1016)
Place_Long_Element_On_Smart_Conveyor_by_Rob2_Gripper(el_name="Bot_Stud", X=2.4984, Y=1.8288, L=3.6576, H=0.1016)
# Complementary nail operation across all logged nail poses
Complementary_Nail_Operation(H=0.1016)
