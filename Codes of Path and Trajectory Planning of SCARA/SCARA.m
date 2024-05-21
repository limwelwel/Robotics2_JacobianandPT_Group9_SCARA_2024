% CLear
clear
clc
close

disp('SCARA')

syms a1 a2 a3 a4 a5

%% Link Lengths
a1 = 50;
a2 = 60;
a3 = 50;
a4 = 60;
a5 = 50;

%% D-H Parameters [theta, d, r, alpha, offset]

H1 = Link([0,a1,a2,0,0,0]);
H1.qlim = pi/180*[-90 90];

H2 = Link([0,a3,a4,(180*pi)/180,0,0]);
H2.qlim = pi/180*[-90 90];

H3 = Link([0,0,0,0,1,a5]);
H3.qlim = [0 10];

Scara = SerialLink([H1 H2 H3], 'name', 'SCARA')
Scara.plot([0 0 0], 'workspace', [-75 75 -75 75 -50 75])
figure(1)
Scara.teach

Af = ([5,pi/2,pi/2]); 
FK = Scara_V3.fkine(Af)

%% Path and Trajectory
t = 0:0.5:2

% q paths
q0 = [0 0 0]; 
q1 = [0 2 2];
q2 = [pi/2 4 2.831];
q3 = [ 1.5 3 2.831];
q4 = [3*pi/2 2 2.831];
q5 = [2*pi 1.5 2.831];
q6 = [2*pi 0 0];

% Trajectory
Traj1 = jtraj(q0,q1,t)
Traj2 = jtraj(q1,q2,t)
Traj3 = jtraj(q2,q3,t)
Traj4 = jtraj(q3,q4,t)
Traj5 = jtraj(q4,q5,t)
Traj6 = jtraj(q5,q6,t)

figure(2)
plot(Scara,Traj1)
plot(Scara,Traj2)
plot(Scara,Traj3)
plot(Scara,Traj4)
plot(Scara,Traj5)
plot(Scara,Traj6)
    
