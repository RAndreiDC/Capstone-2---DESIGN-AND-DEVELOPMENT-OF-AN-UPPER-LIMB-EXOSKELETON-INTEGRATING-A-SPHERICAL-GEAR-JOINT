% Clear
clear
clc
close all

disp('EXOSKELETON')
syms a1 a2 a3 a4

%% Link lengths
a1 = 5;
a2 = 5;
a3 = 4;
a4 = 4;

%% D-H Parameters [theta, d, r, alpha, offset]
% Link([theta d a alpha offset])
H0_1 = Link([0, 0, 0, pi/2, 0, pi/2]);
H0_1.qlim = [-pi/2 pi/2];

H1_2 = Link([0, 0, a1, pi/2, 0, pi/2]);
H1_2.qlim = [-pi/2 pi/2];

H2_3 = Link([0, a3, 0, pi/2, 0, pi/2]);
H2_3.qlim = [-pi/2 pi/2];

H3_4 = Link([0, 0, 0, pi/2, 0, pi/2]);
H3_4.qlim = [-pi/2 pi/2];

exo = SerialLink([H0_1 H1_2 H2_3 H3_4], 'name', 'EXOSKELETON');

%% --- Raise the base above the ground (e.g. shoulder height)
exo.base = transl(0, 0, 9);  % 5 units above the ground


% Plot robot in initial position
exo.plot([0 0 0 0], 'workspace', [-10 15 -15 15 0 15]);

% Open teach window
figure(1)
exo.teach
