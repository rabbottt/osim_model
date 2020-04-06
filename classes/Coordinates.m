classdef Coordinates < uint8
    enumeration % coordinates 1-30
        
            gndpitch            (1)
            gndroll             (2)
            gndyaw              (3)
            spine_tx            (4)                       
            spine_ty            (5)
            spine_tz            (6)
            pitch8              (7)
            roll8               (8)
            yaw8                (9)
            pitch7              (10)
            roll7               (11)
            yaw7                (12)
            pitch6              (13)
            roll6               (14)
            yaw6                (15)
            pitch5              (16)
            roll5               (17)
            yaw5                (18)
            pitch4              (19)
            roll4               (20)
            yaw4                (21)
            pitch3              (22)
            roll3               (23)
            yaw3                (24)
            pitch2              (25)
            roll2               (26)
            yaw2                (27)
            pitch1              (28)
            roll1               (29)
            yaw1                (30)            

    end
    
    methods (Static)
        function coord_str = getCoordinateStringFromIndex(cc_index)
            mc = ?Coordinates;
            coord_str = string(mc.EnumerationMemberList(cc_index).Name);
        end
        
        function coord_str = getCoordinateStringFromEnum(cc_enum)
            coord_str = char(cc_enum);
        end
        
        function cc_index = getCoordinateIndexFromString(cc_string)
            try
                cc_index = uint8(Coordinates(cc_string));
            catch
                error('String did not match a coordinate name')
            end
        end
        
        function cc_index = getCoordinateIndexFromEnum(cc_enum)
            cc_index = uint8(cc_enum);
        end
        
        function ccList = listCoordinates()
            for cc = 1 : 30
                ccList(cc,:) = Coordinates.getCoordinateStringFromIndex(cc); %#ok<*AGROW>
            end
        end
    end
    
end