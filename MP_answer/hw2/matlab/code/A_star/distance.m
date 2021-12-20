function dist = distance(x1,y1,x2,y2)
%This function calculates the distance between any two cartesian 
%coordinates.
%   Copyright 2009-2010 The MathWorks, Inc.
% dist=sqrt((x1-x2)^2 + (y1-y2)^2);% Euclidean

% dist=abs(x1-x2) + abs(y1-y2);% Manhattan

% Diagonal
straight=abs(x1-x2) + abs(y1-y2);
diagonal=min(abs(x1-x2),abs(y1-y2));
dist = sqrt(2) *diagonal + (straight-2*diagonal);% or max(abs(x1-x2),abs(y1-y2));


