%% ---------------------------------------------------------------------
%%
%% Copyright (c) 2016 - 2019 by the IBAMR developers
%% All rights reserved.
%%
%% This file is part of IBAMR.
%%
%% IBAMR is free software and is distributed under the 3-clause BSD
%% license. The full text of the license can be found in the file
%% COPYRIGHT at the top level directory of IBAMR.
%%
%% ---------------------------------------------------------------------

%% Filename : Cylinder3d.m
% Adapted from Cylinder2d.m by Claude
clear all;
clc;
%% Mesh parameters.
Lx = 16.0; Ly = 16.0; Lz = 16.0;
Nx = 32*4*4; Ny = 32*4*4; Nz = 32*4*4;
dx = Lx/Nx; dy = Ly/Ny; dz = Lz/Nz;
L = 3.0;

%% cylinder parameters.
X_com = 0.0; Y_com = 0.0; Z_com = 0.0;
Radius = 0.5;
num_pts_x = ceil(2*Radius/dx);
num_pts_y = ceil(2*Radius/dy);
num_pts_z = ceil(L/dz);

%% write out the points.
idx = 0;
for i = 1:num_pts_x
    x = X_com + ( (i-1)*dx - Radius );
    
    for j = 1:num_pts_y
        y = Y_com + ( (j-1)*dy - Radius );
        
        for k = 1:num_pts_z
            z = Z_com + ( (k-1)*dz - Lz/2 );
            
            if( ((x- X_com)^2 + (y-Y_com)^2) <= Radius^2 )
                idx = idx+1;
                X_array(idx) = x; Y_array(idx) = y; Z_array(idx) = z;
            end
        end
    end
end

XCOM = sum(X_array)/length(X_array);
YCOM = sum(Y_array)/length(Y_array);
ZCOM = sum(Z_array)/length(Z_array);

X_array = X_array - XCOM;
Y_array = Y_array - YCOM;
Z_array = Z_array - ZCOM;

%% plot the cylinder
scatter3(X_array, Y_array, Z_array, '.');
axis equal

%% write the coordinates in the file
fid = fopen('./cylinder3d.vertex','wt');
fprintf(fid,'%d\n', length(X_array));

for i = 1:length(X_array)
    fprintf(fid,'%f\t%f\t%f\n',X_array(i),Y_array(i),Z_array(i));
end

fclose(fid);
