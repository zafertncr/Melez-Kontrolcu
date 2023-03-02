scenario = drivingScenario;
%% yol merkez noktaları
roadCenters =[  0   40  49  50 100  50  49  40  -40  -49  -50  -100  -50  -49  -40    0
               -50 -50 -50 -50   0  50  50  50   50   50   50     0  -50  -50  -50  -50
                0   0   0    0   0   0   0   0    0    0    0     0    0    0    0    0]';

road(scenario, roadCenters, 'lanes', lanespec(2));
plot(scenario);


%% arac olculeri tanimlama yolun ilk pose una yerleştirme
baslangicPose=roadCenters(1,:);
araba= actor(scenario,'ClassID',1,'Length',3,'Position',baslangicPose) ;
figure

yol_sinirlari = roadBoundaries(araba);
yol_dis = yol_sinirlari{1};
yol_ic = yol_sinirlari{2};

set(gcf,"Name","Yol sinirlari")
plot3(yol_ic(:,1),yol_ic(:,2),yol_ic(:,3),'r', ...
      yol_dis(:,1),yol_dis(:,2),yol_dis(:,3),'g')
axis equal

%% 
aracpose = actorPoses(scenario);
scenario.SampleTime = 0.02;

%% % Pure Pursuit Model Initialization
addpath(genpath('Images'));
refPose= (yol_dis+yol_ic)/2;
xRef=refPose(:,1);
yRef=refPose(:,2);

%% define reference time for plotting 
Ts = 16; % simulation time
s = size(xRef);
tRef = (linspace(0,Ts,s(1)))'; % this time variable is used in the "2D Visualization" block for plotting the reference points. 
%% define parameters used in the models
L = araba.Length; % bicycle length
ld= 5; %lookahead distance
X_o = refPose(1,1); % initial vehicle position
Y_o = -refPose(1,2); % initial vehicle position 
psi_o = refPose(1,3); % initial yaw angle
