offset_x = 0.1805;
offset_y = 0.047;
offset_z = 0.0;

frontBackSign = 1;
legSign = 1;

R_B0 = [0 0 1;%for coordinate flipping, before q1 applied
            -1 0 0;
             0 -1 0];
d_BodyToHip = [offset_x * frontBackSign; offset_y * legSign; offset_z];
T_B0_together = [R_B0,d_BodyToHip;zeros(1,3),1]

T_B0rot = [R_B0,zeros(3,1);zeros(1,3),1];
T_B0trans = [eye(3),d_BodyToHip;zeros(1,3),1];
T_B0 = T_B0trans*T_B0rot