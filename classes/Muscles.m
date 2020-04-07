classdef Muscles < uint8
    enumeration %96 muscles; multif: 73-96
            digastric_post          (1)
            digastric_post_L        (2)
            digastric_ant           (3)
            digastric_ant_L         (4)                       
            mylohyoid_ant           (5)
            mylohyoid_ant_L         (6)
            stylohyoid_lat          (7)
            stylohyoid_lat_L        (8)
            stylohyoid_med          (9)
            stylohyoid_med_L        (10)
            sternohyoid             (11)
            sternohyoid_L           (12)        
            geniohyoid              (13)
            geniohyoid_L            (14)
            mylohyoid_post          (15)
            mylohyoid_post_L        (16)
            sternothyroid           (17)
            sternothyroid_L         (18)
            omohyoid                (19)            
            omohyoid_L              (20)
            stern_mast              (21)
            stern_mast_L            (22)
            cleid_mast              (23)
            cleid_mast_L            (24)
            cleid_occ               (25)
            cleid_occ_L             (26)
            scalenus_ant            (27)
            scalenus_ant_L          (28)
            scalenus_med            (29)
            scalenus_med_L          (30)
            scalenus_post           (31)
            scalenus_post_L         (32)
            long_cap_sklc4          (33)
            long_cap_sklc4_L        (34)
            long_col_c1thx          (35)
            long_col_c1thx_L        (36)
            long_col_c1c5           (37)
            long_col_c1c5_L         (38)
            long_col_c5thx          (39)
            long_col_c5thx_L        (40)
            trap_cl                 (41)
            trap_cl_L               (42)
            trap_acr                (43)
            trap_acr_L              (44)
            splen_cap_sklc6         (45)
            splen_cap_sklc6_L       (46)
            splen_cap_sklthx        (47)
            splen_cap_sklthx_L      (48)           
            semi_cap_sklc5          (49)
            semi_cap_sklc5_L        (50)
            semi_cap_sklthx         (51)
            semi_cap_sklthx_L       (52)
            splen_cerv_c3thx        (53)
            splen_cerv_c3thx_L      (54)
            semi_cerv_c3thx         (55)
            semi_cerv_c3thx_L       (56)
            levator_scap            (57)
            levator_scap_L          (58)
            longissi_cap_sklc6      (59)
            longissi_cap_sklc6_L    (60)
            longissi_cerv_c4thx     (61)
            longissi_cerv_c4thx_L   (62)
            iliocost_cerv_c5rib     (63)
            iliocost_cerv_c5rib_L   (64)
            rectcap_post_maj        (65)
            rectcap_post_maj_L      (66)
            rectcap_post_min        (67)
            rectcap_post_min_L      (68)
            obl_cap_sup             (69)
            obl_cap_sup_L           (70)
            obl_cap_inf             (71)
            obl_cap_inf_L           (72)
            mult_sup_C45C2          (73) 
            mult_sup_C45C2_L        (74)
            mult_sup_C56C2          (75)
            mult_sup_C56C2_L        (76)
            mult_sup_C67C2          (77)
            mult_sup_C67C2_L        (78)
            mult_sup_T1C4           (79)
            mult_sup_T1C4_L         (80)
            mult_sup_T1C5           (81)
            mult_sup_T1C5_L         (82)
            mult_sup_T2C6           (83)
            mult_sup_T2C6_L         (84)
            mult_deep_C45C2         (85)
            mult_deep_C45C2_L       (86)
            mult_deep_C56C3         (87)
            mult_deep_C56C3_L       (88)
            mult_deep_C67C4         (89)
            mult_deep_C67C4_L       (90)
            mult_deep_T1C5          (91)
            mult_deep_T1C5_L        (92)
            mult_deep_T1C6          (93)
            mult_deep_T1C6_L        (94)
            mult_deep_T2C7          (95)
            mult_deep_T2C7_L        (96)

    end
    methods (Static)
        function mm_string = getMuscleStringFromIndex(mm_index)
            mc = ?Muscles;
            mm_string = string(mc.EnumerationMemberList(mm_index).Name);
        end
        
        function mm_string = getMuscleStringFromEnum(mm_enum)
            mm_string = char(mm_enum);
        end
        
        function mm_index = getMuscleIndexFromString(mm_string)
            try
                mm_index = uint8(Muscles(mm_string));
                %can also use subsindex(Muscles(mm_string));
            catch
                
            end
        end
        
        function mm_index = getMuscleIndexFromEnum(mm_enum)
            mm_index = uint8(mm_enum);
        end
        
        function mmList = listMuscles()
            for i = 1 : 98
                mmList(i,:) = Muscles.getMuscleStringFromIndex(i); %#ok<*AGROW>
            end
        end
    end
    
end
