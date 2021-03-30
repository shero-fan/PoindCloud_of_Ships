
ptCloud_x = pcread('.\Output_Pc\sections_x.pcd');
ptCloud_y = pcread('.\Output_Pc\sections_y.pcd');
ptCloud_z = pcread('.\Output_Pc\sections_z.pcd');

figure
pcshow(ptCloud_x);
hold on
pcshow(ptCloud_y);
hold on
pcshow(ptCloud_z);
title('Point Cloud Data');