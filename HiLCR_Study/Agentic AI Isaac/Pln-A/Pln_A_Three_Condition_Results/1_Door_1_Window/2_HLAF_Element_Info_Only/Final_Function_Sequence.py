# Pln-A case: 1_Door_1_Window
# Experimental condition: 2_HLAF_Element_Info_Only

# --- Place Top Plate ---
Pick_Long_Element_From_Mat_Supply(el_name='Top_Stud', L=3.6576, W=0.04, H=0.1016)
Pass_Long_Element_G2G(el_name='Top_Stud', L=3.6576, H=0.1016)
Place_Long_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name='Top_Stud', X=0.02, Y=1.8288, L=3.6576, H=0.1016)
# --- Place Bottom Plate ---
Pick_Long_Element_From_Mat_Supply(el_name='Bot_Stud', L=3.6576, W=0.04, H=0.1016)
Pass_Long_Element_G2G(el_name='Bot_Stud', L=3.6576, H=0.1016)
Place_Long_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name='Bot_Stud', X=2.4984, Y=1.8288, L=3.6576, H=0.1016)
# --- Install Front Stud (F_Stud) ---
Pick_8ft_Element_From_Sloped_Table(el_name='F_Stud', L=2.4384, W=0.04, H=0.1016)
side_F_Stud = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.02, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name='F_Stud', X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_F_Stud, Is_Held=False)
# --- Install Left King Stud (LK_1) ---
Pick_8ft_Element_From_Sloped_Table(el_name='LK_1', L=2.4384, W=0.04, H=0.1016)
side_LK_1 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=0.72, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name='LK_1', X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_LK_1, Is_Held=False)
# --- Install Right King Stud (RK_1) ---
Pick_8ft_Element_From_Sloped_Table(el_name='RK_1', L=2.4384, W=0.04, H=0.1016)
side_RK_1 = Place_and_Hold_8ft_Element_On_Smart_Conveyor(X=1.2592, Y=1.68, L=2.4384, H=0.1016)
Nail_and_Release_Vertical_Element(el_name='RK_1', X=1.2592, L=2.4384, H=0.1016, Side_Selector=side_RK_1, Is_Held=False)
# --- Install Left Jack Stud (LJ_1) ---
Pick_Short_Element_From_Mat_Supply(el_name='LJ_1', L=2.0, W=0.04, H=0.1016)
Pass_Short_Element_G2G(el_name='LJ_1', H=0.1016)
side_LJ_1 = Place_Short_Vertical_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name='LJ_1', X=1.4784, Y=0.76, L=2.0, H=0.1016)
Nail_Vertical_Element_With_Tangent_to_an_Element(el_pose=[1.4784, 0.76, 0.0], el_dims=[2.0, 0.04, 0.1016], If_Tangent_From_Left=True, Side_Selector=side_LJ_1)
# --- Install Right Jack Stud (RJ_1) ---
Pick_Short_Element_From_Mat_Supply(el_name='RJ_1', L=2.0, W=0.04, H=0.1016)
Pass_Short_Element_G2G(el_name='RJ_1', H=0.1016)
side_RJ_1 = Place_Short_Vertical_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name='RJ_1', X=1.4784, Y=1.64, L=2.0, H=0.1016)
Nail_Vertical_Element_With_Tangent_to_an_Element(el_pose=[1.4784, 1.64, 0.0], el_dims=[2.0, 0.04, 0.1016], If_Tangent_From_Left=False, Side_Selector=side_RJ_1)
# --- Install Top Sill Plate (TSP_1) ---
Pick_Short_Element_From_Mat_Supply(el_name='TSP_1', L=0.92, W=0.04, H=0.1016)
Pass_Short_Element_G2G(el_name='TSP_1', H=0.1016)
conv_TSP_1 = Place_Short_Horizontal_Element_On_Smart_Conveyor_by_Rob1_Gripper(el_name='TSP_1', X=0.4584, Y=1.2, L=0.92, H=0.1016)
Nail_Short_Horizontal_Element_by_Rob1_NailGun(el_pose=[0.4584, 1.2, 0.0], el_dims=[0.92, 0.04, 0.1016], conv_current_location=conv_TSP_1)
# --- Install Bear Loading Header (BL_1) ---
Pick_2x10_Header(X=0.5514, L=0.86)
Cut_2x10_Header(L=0.86, W=0.0508, H=0.254)
conv_BL_1 = Place_2x10_Header(X=0.5514, Y=2.6876, Z=0.0254, L=0.86, W=0.0508, H=0.254)
Nail_2x10_Header(el_pose=[0.5514, 2.6876, 0.0254], el_dims=[0.86, 0.0508, 0.254], conv_current_location=conv_BL_1)
# --- Complementary nailing on remaining positions ---
Complementary_Nail_Operation(H=2.4384)
