clc; close all; clear all;
format short

% introduce the state matrices, these are found in flight dynamics for
% standard missile, reference rytheon research paper
A = [-1.064 1.000; 290.26 0.00]; %natural dynamics, untouched, how airframe moves on its own
B = [-0.25; -331.40]; % what fin does to states (the input)
C = [-123.34 0.00; 0.00 1.00]; % how the sensors see the states
D = [-13.51; 0.00]; %the instant fin to sensor path, 

states = {'AoA', 'q'}; %angle of attack and pitch rate full describe motion at any instant
inputs = {'\delta_c'}; %the pitch fin deflection
outputs = {'Az', 'q'}; % what the sensors measure, and rate gyros

missilePlant = ss(A, B, C, D, 'StateName', states, ...
    'InputName', inputs, 'OutputName', outputs);

%define transfer funciton so ca

