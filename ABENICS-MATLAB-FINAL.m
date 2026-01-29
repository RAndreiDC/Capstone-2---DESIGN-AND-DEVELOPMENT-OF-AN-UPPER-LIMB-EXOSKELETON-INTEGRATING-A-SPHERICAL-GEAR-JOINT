%% ============================================================
%  ABENICS 3-DoF Spherical Joint Visualization (Peter Corke RTB)
%  With automatic DH parameter table in the Command Window
%  Author: Rod Andrei De Castro
%  Reference: Abe et al., IEEE Trans. Robotics (2021)
%% ============================================================

clear; clc; close all;

%% ============ USER SETTINGS ============
use_rx_ry_rx = false;     % true -> Rx–Ry–Rx (paper), false -> Rx–Ry–Rz (RPY)
deg_gui      = true;      % sliders in degrees
ws           = 0.7;       % workspace half-size (tight for zoomed-in view)
zoom_factor  = 2.0;       % >1 zooms in
cam_angle    = 8;         % smaller = closer camera
view_dir     = [130 25];  % az, el

%% ============ TOOLBOX CHECK ============
hasETS = exist('ETS3','class')==8;
hasSL  = exist('SerialLink','class')==8;

if ~(hasETS || hasSL)
    error('Peter Corke Robotics Toolbox not found. Install RTB10+ or classic RTB.');
end

%% ============ BUILD MODEL ============
if hasETS
    % --- Modern ETS3 path (RTB v10 +) ---
    if use_rx_ry_rx
        ets = ETS3.Rx('q1') * ETS3.Ry('q2') * ETS3.Rx('q3');
        seq = 'Rx–Ry–Rx';
    else
        ets = ETS3.Rx('q1') * ETS3.Ry('q2') * ETS3.Rz('q3');
        seq = 'Rx–Ry–Rz';
    end

    q0 = [0 0 0];
    fig = figure('Color','w','Name',['ABENICS ETS3 (' seq ')']);
    ets.plot(q0,'workspace',[-ws ws -ws ws -ws ws],'view',view_dir);
    ets.teach(q0,'deg',deg_gui,'limits',[-90 90; -60 60; -90 90]);

    disp(' ');
    disp('=== ABENICS 3-DoF Spherical Joint (ETS3 Model) ===');
    fprintf('Rotation Sequence: %s\n\n', seq);
    disp('No classical DH table for ETS3 model (uses exponential coordinates).');

else
    % --- Classic SerialLink fallback ---
    if use_rx_ry_rx
        seq = 'Rx–Ry–Rx';
        L1 = Revolute('d',0,'a',0,'alpha', pi/2);
        L2 = Revolute('d',0,'a',0,'alpha',-pi/2);
        L3 = Revolute('d',0,'a',0,'alpha', pi/2);
    else
        seq = 'Rx–Ry–Rz';
        L1 = Revolute('d',0,'a',0,'alpha', pi/2);
        L2 = Revolute('d',0,'a',0,'alpha', 0);
        L3 = Revolute('d',0,'a',0,'alpha', 0);
    end

    robot = SerialLink([L1 L2 L3],'name',['ABENICS (' seq ')']);
    robot.base = troty(-pi/2);  % rotate base so joint 1 ≈ world X
    robot.tool = transl(0,0,1e-9);
    fig = figure('Color','w','Name',robot.name);
    robot.plot([0 0 0],'workspace',[-ws ws -ws ws -ws ws],...
               'view',view_dir,'scale',0.8,'jointdiam',0.07);
    robot.teach([0 0 0],'deg',deg_gui);

    %% --------- PRINT DH PARAMETER TABLE ----------
    disp(' ');
    disp('=== ABENICS 3-DoF Spherical Joint (Classic DH Parameters) ===');
    fprintf('Rotation Sequence: %s\n\n', seq);
    DH = [robot.links.d; robot.links.a; robot.links.alpha; zeros(1,3)]';
    T = array2table(DH, 'VariableNames',{'d (m)','a (m)','alpha (rad)','theta (rad)'});
    disp(T);
    fprintf('\nEach row corresponds to [Joint%d]\n', 1:3);
end

%% ============ ZOOM-IN CAMERA SETTINGS ============
ax = gca;
axis(ax,'equal');
xlim(ax,[-ws ws]);
ylim(ax,[-ws ws]);
zlim(ax,[-0.2 0.8]);
view(ax,view_dir);
camzoom(ax,zoom_factor);   % 2× zoom-in
camva(ax,cam_angle);       % narrow field of view
title(ax,['ABENICS 3-DoF Spherical Joint (' seq ', Zoomed View)']);
