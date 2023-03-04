% Pure Pursuit Model Initialization
% Copyright 2020 The MathWorks, Inc.

%% add Image to the path
addpath(genpath('Images'));

%% load the road data from Driving Scenario Designer
load('CurvedRoad.mat');
%% add actor and Generate Waypoints from road center
roadX=data.RoadSpecifications.Centers(:,1);
roadY=data.RoadSpecifications.Centers(:,2);
roadPsi=data.RoadSpecifications.Centers(:,3);

data.ActorSpecifications.Waypoints=[roadX,roadY,roadPsi];
data.ActorSpecifications.Position=[roadX(1),roadY(1),roadPsi(1)];
%data.ActorSpecifications.Speed=20;
%% define reference points
refPose = data.ActorSpecifications.Waypoints;
xRef = refPose(:,1);
yRef = -refPose(:,2);
 

%% define reference time for plotting 
Ts = 500; % simulation time
s = size(xRef);
tRef = (linspace(0,Ts,s(1)))'; % this time variable is used in the "2D Visualization" block for plotting the reference points.

%% define parameters used in the models
L = data.ActorSpecifications.Length; % bicycle length
ld = 15; % lookahead distance
X_o = refPose(1,1); % initial vehicle position
Y_o = -refPose(1,2); % initial vehicle position 
psi_o = refPose(1,3); % initial yaw angle

%% define data for velocity lookup table
%aracın hızının konuma göre değişikliğini istemediğimiz icin sabit tuttuk
% lookUpt = readmatrix('velocityDistribution.xlsx');
% xlt = lookUpt(2:27,1);
% ylt = lookUpt(1,2:32);
% vel = lookUpt(2:27,2:32);