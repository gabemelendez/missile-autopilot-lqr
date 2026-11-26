clc; close all; clear all;
format short

% introduce the state matrices, these are found in flight dynamics for
% standard missile, reference rytheon research paper
A = [-1.064 1.000; 290.26 0.00]; %natural dynamics, untouched, how air frame moves on its own
B = [-0.25; -331.40]; % what fin does to states (the input)
C = [-123.34 0.00; 0.00 1.00]; % how the sensors see the states
D = [-13.51; 0.00]; %the instant fin to sensor path, 

states = {'AoA', 'q'}; %angle of attack and pitch rate full describe motion at any instant
inputs = {'\delta_c'}; %the pitch fin deflection
outputs = {'Az', 'q'}; % what the sensors measure, and rate gyros

sys = ss(A, B, C, D, 'StateName', states, ...
    'InputName', inputs, 'OutputName', outputs);

%define transfer function so can view stability of system

TFs = tf(sys);
TF = TFs(2,1);
fprintf('pole values for ss system transfer function: ');
disp(pole(TF));

%adding max values so i can use brysons rule 
alphaMax = deg2rad(10);
qMax = deg2rad(100);
deltaMax = deg2rad(20);

%LQR weight matrices so can penalize the state error and the control effort
%(using brysons rule so i can tune Q and R based on stabilities)
Q = diag([1/alphaMax^2, 1/qMax^2]);
R = 1/deltaMax^2 ;

%LQR gain, feedback system to resolve the unstable system
[K, S, e] = lqr(A, B, Q, R);
fprintf('eigenvalues of A-BK: ')
disp(eig(A - B*K)); %now closed loop system, with system evolving on its own
fprintf('Feedback gain K: ')
disp(K);

%closed loop system so can simulate the stabalized missile
Acl = A - B*K ;
Bcl = B;
syscl = ss(Acl, Bcl, C, D, 'statename', states,...
    'inputname', inputs,...
    'outputname', outputs);

%calculate the closed loop transfer funciton
TFsc = tf(syscl);
TFc = TFsc(2,1);
fprintf('pole values for closed system transfer function: ');
disp(pole(TFc));

%LQG, kalman + lqr design
%produce the best estimate of x^ of x from both the process noise and the
%measurement noise

G = eye(2); %the identity matrix
H = 0*eye(2); %these are the zeros of the identity matrix

% noise covariences defined (values are found from kalmanfilter video
% (referencd)
Qbar = diag(0.00015*ones(1,2)); %how much to distrust model
Rbar = diag(0.55*ones(1,2)); %how much distrust the actual sensors

sys_n = ss(A, [B G], C, [D H]); %process noise now added as extra inputs

[kest, L, P] = kalman(sys_n, Qbar, Rbar, 0); %blend the new model with the sensors
%kest will be able to output the best guess of AoA and q based on this
%L will help blend the amount and how much sensors disagree, P is
%confidence

%observer poles can be found from the closed loop of kalman gain
Aob = A-L*C;
fprintf('observer eignevalues: ');
display(eig(Aob));

%noise time constants, used in simulink 
dT1 = 0.75;
dT2 = 0.25;

%missile parameters designed from control results (the mission design)
%using metric units

R_earth = 6371e3; %earth radius , not perfect sphere but wtvr
vel = 1021.08; %speed of missile taken from paper
m2f = 3.2811; %meters to feet conversion for future calcs

%target location for mission plan (lat and lon in degrees)
lat_tar = 34.6588;
lon_tar = -118.769745;
elev_tar = 795; %this is the m - mean sea level

%set initial location
lat_init = 34.2329;
lon_init = -119.4573;
elev_init = 10000;

%obstacle location need to avoid
lat_obs = 34.61916;
lon_obs = -118.8429;

d2r = pi/180; %need this for lon and lan conversions

%convert to radians, need these for haversine
l1 = lat_init*d2r;
u1 = lon_init*d2r;
l2 = lat_tar*d2r;
u2 = lon_tar*d2r;

dl = l2-l1;
du = u2-u1;

%haversine formula (great-circle distance on a spherical Earth)
a = sin(dl/2)^2 + cos(l1)*cos(l2)*sin(du/2)^2;  %square of half the chord length between the points
c = 2*atan2(sqrt(a), sqrt(1-a));                %central angle between the points (rad)
d = R_earth*c;                                        %ground distance along Earth's surface (m)

%slant range from Pythagoras (adds the height difference)
dZ = elev_tar - elev_init;                      %height change (m), negative = descending
r  = sqrt(d^2 + dZ^2);                          %straight-line range (m)

%initial azimuth: direction to target, clockwise from north
az_rad = atan2(sin(du)*cos(l2), cos(l1)*sin(l2) - sin(l1)*cos(l2)*cos(du));
az_rad = mod(az_rad, 2*pi);                     %keep in 0 to 2*pi (rad)
yaw    = az_rad;                                %initial heading (rad), missile assumed pointed at target

%initial flight path angle (positive magnitude, as in the reference video)
dh = abs(dZ);                                   %height difference magnitude (m)
FPA_INIT = atan(dh/d);                          %rad

%display
fprintf('ground range:      %.2f km\n',  d/1000);
fprintf('slant range:       %.2f km\n',  r/1000);
fprintf('azimuth (yaw):     %.1f deg\n', yaw/d2r);
fprintf('flight path angle: %.1f deg\n', FPA_INIT/d2r);


