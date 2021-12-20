% Used for Motion Planning for Mobile Robots
% Thanks to HKUST ELEC 5660 
close all; clear all; clc;
addpath('A_star')

% Environment map in 2D space 
xStart = 1.0;
yStart = 1.0;
xTarget = 9.0;
yTarget = 9.0;
MAX_X = 10;
MAX_Y = 10;
map = obstacle_map(xStart, yStart, xTarget, yTarget, MAX_X, MAX_Y);
% map=[
%      1     1;
%      1     2;
%      1     5;
%      2     2;
%      2     5;
%      2     7;
%      3     3;
%      3     7;
%      4     3;
%      4     7;
%      4     9;
%      5     1;
%      5     2;
%      5     6;
%      5     7;
%      5     9;
%      6     5;
%      6     6;
%      7     2;
%      7     4;
%      7     5;
%      8     4;
%      8    10;
%      9     1;
%      9     3;
%      9     4;
%      9    10;
%     10     3;
%     10     9;
%      9     9];
% Waypoint Generator Using the A* 
tic
path = A_star_search(map, MAX_X,MAX_Y);
toc
% visualize the 2D grid map
visualize_map(map, path, []);

% save map
% save('Data/map.mat', 'map', 'MAX_X', 'MAX_Y');
