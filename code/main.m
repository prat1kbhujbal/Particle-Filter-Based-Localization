% Robotics: Estimation and Learning 
% WEEK 4
% 
% This script is to help run your algorithm and visualize the result from it.

%% Load data
clear;
close all;
clc;

load practice.mat
% This will load four variables: ranges, scanAngles, t, pose
% [1] t is K-by-1 array containing time in second. (K=3701)
%     You may not need to use time info for implementation.
% [2] ranges is 1081-by-K lidar sensor readings. 
%     e.g. ranges(:,k) is the lidar measurement at time index k.
% [3] scanAngles is 1081-by-1 array containing at what angles the 1081-by-1 lidar
%     values ranges(:,k) were measured. This holds for any time index k. The
%     angles are with respect to the body coordinate frame.
% [4] M is a 2D array containing the occupancy grid map of the location
%     e.g. map(x,y) is a log odds ratio of occupancy probability at (x,y)
load practice-answer.mat;
%% Set parameters
param = {};
% 1. Decide map resolution, i.e., the number of grids for 1 meter.
param.resol = 25;

% 3. Indicate where you will put the origin in pixels
param.origin = [685,572]';

param.init_pose = -init_pose;

start = 1;
N = length(t)-1;
param.init_pose = pose(:,start);

% Adding actual data in param object to plot
param.pose = pose;
param.t = t;


%% Run algorithm
% Call your mapping function here.
% Running time could take long depending on the efficiency of your code.
% For a quicker test, you may take some hundreds frames as input arguments as
% shown.

pose_calc = particleLocalization(ranges(:,start:start+N), scanAngles, M, param);
pose = pose(:,start:start+N);
ranges = ranges(:,start:start+N);
