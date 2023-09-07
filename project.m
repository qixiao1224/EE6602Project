%% Load Parameters
%Inertia: kg*m^2
clear;
Ix = 0.014484;
Iy = Ix;
Iz = 0.004026;
Iw = 0.000189;
%Length: m
L = 0.108;
%mass: kg
m = 1.146;
%Moment constant of motor: N*m/A
Kt = 0.0251;
%Resistor: olm
Rm = 0.0464;
%Gravity constant: m/s^2
g = 9.81;


%% System Description
% x = [dα dβ dγ α β dθA dθB dθC]
modelSelect = 0;
%0 means simplified model
%1 means original model.

if modelSelect == 0
    A = [0 0 0 m*g*L/(Iy-Iw) 0             sqrt(6).*Kt.^2/(3*Rm.*(Iy-Iw))  -sqrt(6).*Kt.^2/(3*Rm.*(Iy-Iw)) -sqrt(6).*Kt.^2/(3*Rm.*(Iy-Iw));
         0 0 0 0             m*g*L/(Ix-Iw) 0                               sqrt(2).*Kt.^2/(2*Rm.*(Ix-Iw))  -sqrt(2).*Kt.^2/(2*Rm.*(Ix-Iw));
         0 0 0 0             0             -sqrt(3).*Kt.^2/(3*Rm.*(Iz-Iw)) -sqrt(3).*Kt.^2/(3*Rm.*(Iz-Iw)) -sqrt(3).*Kt.^2/(3*Rm.*(Iz-Iw));
         1 0 0 0             0             0                               0                               0                              ;
         0 1 0 0             0             0                               0                               0                              ;
         0 0 0 0             0             -Kt.^2/(Iw.*Rm)                 0                               0                              ;
         0 0 0 0             0             0                               -Kt.^2/(Iw.*Rm)                 0                              ;
         0 0 0 0             0             0                               0                               -Kt.^2/(Iw.*Rm)                ];

    B = [-sqrt(6).*Kt/(3.*Rm.*(Iy-Iw)) sqrt(6).*Kt/(6.*Rm.*(Iy-Iw))    sqrt(6).*Kt/(6.*Rm.*(Iy-Iw)) ;
         0                             -sqrt(2).*Kt/(2.*Rm.*(Ix-Iw))   sqrt(2).*Kt/(2.*Rm.*(Ix-Iw)) ;
         -sqrt(3).*Kt/(3.*Rm.*(Iz-Iw)) -sqrt(3).*Kt/(3.*Rm.*(Iz-Iw))   -sqrt(3).*Kt/(3.*Rm.*(Iz-Iw));
         0                             0                               0                            ;
         0                             0                               0                            ;
         Kt/(Rm.*Iw)                   0                               0                            ;
         0                             Kt/(Rm.*Iw)                     0                            ;
         0                             0                               Kt/(Rm.*Iw)                  ];
else
    %dtheta A
    K1 = Kt/Rm*(1/Iw+ 2/(3*(Iy-Iw)) + 1/(3*(Iz-Iw)));
    K2 = Kt/Rm*( -1/(3*(Iy-Iw)) + 1/(3*(Iz-Iw)));
    K3 = Kt/Rm*( -1/(3*(Iy-Iw)) + 1/(3*(Iz-Iw)));

    %dtheta B
    K4 = Kt/Rm*(-1/(3*(Iy-Iw)) + 1/(3*(Iz-Iw)));
    K5 = Kt/Rm*(1/Iw + 1/(6*(Iy-Iw)) + 1/(2*(Ix-Iw)) + 1/(3*(Iz-Iw)));
    K6 = Kt/Rm*(1/(6*(Iy-Iw)) - 1/(2*(Ix-Iw)) + 1/(3*(Iz-Iw)));

    %dtheta C
    K7 = Kt/Rm*(-1/(3*(Iy-Iw)) + 1/(3*(Iz-Iw)));
    K8 = Kt/Rm*(1/(6*(Iy-Iw)) - 1/(2*(Ix-Iw)) + 1/(3*(Iz-Iw)));
    K9 = Kt/Rm*(1/Iw + 1/(6*(Iy-Iw)) + 1/(2*(Ix-Iw)) + 1/(3*(Iz-Iw)));


     A = [0 0 0 m*g*L/(Iy-Iw)            0                          sqrt(6).*Kt.^2/(3*Rm.*(Iy-Iw))     -sqrt(6).*Kt.^2/(3*Rm.*(Iy-Iw))   -sqrt(6).*Kt.^2/(3*Rm.*(Iy-Iw));
         0 0 0 0                         m*g*L/(Ix-Iw)              0                                  sqrt(2).*Kt.^2/(2*Rm.*(Ix-Iw))    -sqrt(2).*Kt.^2/(2*Rm.*(Ix-Iw));
         0 0 0 0                         0                          -sqrt(3).*Kt.^2/(3*Rm.*(Iz-Iw))    -sqrt(3).*Kt.^2/(3*Rm.*(Iz-Iw))   -sqrt(3).*Kt.^2/(3*Rm.*(Iz-Iw));
         1 0 0 0                         0                          0                                  0                                 0                              ;
         0 1 0 0                         0                          0                                  0                                 0                              ;
         0 0 0 m*g*L*sqrt(6)/(3*(Iw-Iy)) 0                          -Kt*K1                             -Kt*K2                            -Kt*K3                         ;
         0 0 0 m*g*L*sqrt(6)/(6*(Iy-Iw)) -m*g*L*sqrt(2)/(2*(Ix-Iw))  -Kt*K4                             -Kt*K5                            -Kt*K6                         ;%
         0 0 0 m*g*L*sqrt(6)/(6*(Iy-Iw)) m*g*L*sqrt(2)/(2*(Ix-Iw)) -Kt*K7                             -Kt*K8                            -Kt*K9                         ];%

    B = [-sqrt(6).*Kt/(3.*Rm.*(Iy-Iw)) sqrt(6).*Kt/(6.*Rm.*(Iy-Iw))    sqrt(6).*Kt/(6.*Rm.*(Iy-Iw)) ;
         0                             -sqrt(2).*Kt/(2.*Rm.*(Ix-Iw))   sqrt(2).*Kt/(2.*Rm.*(Ix-Iw)) ;
         -sqrt(3).*Kt/(3.*Rm.*(Iz-Iw)) -sqrt(3).*Kt/(3.*Rm.*(Iz-Iw))   -sqrt(3).*Kt/(3.*Rm.*(Iz-Iw));
         0                             0                               0                            ;
         0                             0                               0                            ;
         K1                            K2                              K3                           ;
         K4                            K5                              K6                           ;%
         K7                            K8                              K9                           ];%
 
end
%C = [zeros(5,3) diag(ones(5,1))];
C =[0 0 1 0 0 0 0 0;
    0 0 0 1 0 0 0 0;
    0 0 0 0 1 0 0 0;
    0 0 0 0 0 1 0 0;
    0 0 0 0 0 0 1 0;
    0 0 0 0 0 0 0 1];

D=0;

%% Check Controllability
rank(ctrb(A,B))

%% Check Observability
rank(obsv(A,C))
%% Zero Input Simulation
states = {'d\alpha','d\beta','d\gamma','\alpha','\beta','\theta_A','\theta_B','\theta_C'};
inputs = {'v_a','v_b','v_c'};
outputs = {'d\gamma','\alpha','\beta','\theta_A','\theta_B','\theta_C'};
T = 0:0.01:0.5; %Simulate for 0.5 second

%Zero Input Simulation
U = zeros(3,length(T));
x0 = [0 0 0 20/180*pi 0 0 0 0]; %20 degree
sys = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
lsim(sys,U,T,x0)


%% Zero State Simulation
U = [zeros(1,length(T));0.5*ones(1,length(T));-0.5*ones(1,length(T))];
%U = [0.5*ones(1,length(T));-0.25*ones(1,length(T));-0.25*ones(1,length(T))];
%U = [0.25*ones(1,length(T));0.25*ones(1,length(T));0.25*ones(1,length(T))];


%initial condition
x0 = [0 0 0 0 0 0 0 0];

sys = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
lsim(sys,U,T,x0)

%impulse(sys)
%% LQR control
Q = diag(100*ones(1,8));
R = diag(0.01*ones(1,3));
[K,S,P] = lqr(sys,Q,R);
T = 0:0.01:1; %Simulate for 1 second
U = [zeros(3,length(T))];
sys2 = ss(A-B*K,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
x0 = [0 0 0 20/180*pi -30/180*pi 0 0 0];
lsim(sys2,U,T,x0)


%% Luenberger Observer

cvx_begin sdp
    variable W(8,8) symmetric 
    variable V(6,8)
    LMI = A'*W+W*A+C'*V+V'*C;
    W >= 0.01*eye(8);
    LMI <= -10000*eye(8);
cvx_end

L = W\V';

%Compute eigenvalue of A+LC to determine the stabiblity
eig(A+L*C)

%Observer Simulation
A_ob = [A -B*K; -L*C A+L*C-B*K];
B_ob = [zeros(8,3);zeros(8,3)];
C_ob = eye(16); %Set output to be the states
D_ob = 0;

sys_ob = ss(A_ob,B_ob,C_ob,D_ob);

x0 = [0 0 0 20/180*pi -30/180*pi 0 0 0 zeros(1,8)];
[y_cl,t,x_cl] = initial(sys_ob,x0);

%Plot
subplot(2,1,1)
plot(t,x_cl(:,1),t,x_cl(:,9),t,zeros(size(t)),'r--')
set(legend('$$d\alpha$$','$$d\hat{\alpha}$$'),'Interpreter','Latex','FontSize', 10);
xlim([0,1.6])
xlabel('Time(sec)')
ylabel('d\alpha')

subplot(2,1,2)
plot(t,x_cl(:,2),t,x_cl(:,10),t,zeros(size(t)),'r--')
set(legend('$$d\beta$$','$$d\hat{\beta}$$'),'Interpreter','Latex','FontSize', 10);
xlim([0,1.6])
xlabel('Time(sec)')
ylabel('d\beta')

sgtitle('Simulation of states and their observer')

%% H-2 Optimal Control
B1 = 0.1.*B;
cvx_begin sdp
    variable X(8,8) symmetric
    variable Z(3,8)
    variable W(8,8) symmetric
    variable gsq
    minimize ( gsq )
    [A B]*[X; Z]+[X Z']*[A'; B']+ B1*B1' <= -100*eye(8);
    [X   X';
     X   W] >= 1*eye(16);
    trace (W) <= gsq ;
    X >= 0;
cvx_end

F = Z/X;

sysh2 = ss(A+B*F,B1,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
x0 = [0 0 0 60/180*pi -60/180*pi 0 0 0];

%T = 0:0.01:1; %Simulate for 1 second
%U = [3*randn(3,length(T))];
%lsim(sysh2,U,T,x0)
[y_h2,t,x_h2] = initial(sysh2,x0);

%Plot
subplot(5,1,1)
plot(t,x_h2(:,4),t,zeros(size(t)),'r--')
set(legend('$$\alpha$$'),'Interpreter','Latex','FontSize', 10);
xlim([0,1])

ylabel('\alpha')

subplot(5,1,2)
plot(t,x_h2(:,5),t,zeros(size(t)),'r--')
set(legend('$$\beta$$'),'Interpreter','Latex','FontSize', 10);
xlim([0,1])

ylabel('\beta')

subplot(5,1,3)
plot(t,x_h2(:,6),t,zeros(size(t)),'r--')
set(legend('$$d\theta_a$$'),'Interpreter','Latex','FontSize', 10);
xlim([0,1])

ylabel('d\theta_a')

subplot(5,1,4)
plot(t,x_h2(:,7),t,zeros(size(t)),'r--')
set(legend('$$d\theta_b$$'),'Interpreter','Latex','FontSize', 10);
xlim([0,1])

ylabel('d\theta_b')

subplot(5,1,5)
plot(t,x_h2(:,8),t,zeros(size(t)),'r--')
set(legend('$$d\theta_c$$'),'Interpreter','Latex','FontSize', 10);
xlim([0,1])
xlabel('Time(sec)')
ylabel('d\theta_c')


sgtitle('Simulation of the system with H-2 controller')

%% H2 with disturbance
%Add disturbance to the input
T = 0:0.01:1; %Simulate for 1 second
U = [3*randn(3,length(T))];
lsim(sysh2,U,T,x0)

max(abs(F*x_h2'),[],2)

%% H-infty
cvx_begin sdp
    variable Y(8,8) symmetric
    variable gam
    variable Z(3,8)
    min gam
    [Y*A'+A*Y+Z'*B'+B*Z,   B1,               Y,              zeros(8,8);
     B1',                  -gam*eye(3),    zeros(3,8),       zeros(3,8);
     Y,                    zeros(8,3),     -gam*eye(8),      zeros(8,8);
     zeros(8,8),           zeros(8,3),    zeros(8,8),         -Y]<=0;
cvx_end 

F = Z/Y;

syshinf = ss(A+B*F,B1,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
x0 = [0 0 0 60/180*pi -60/180*pi 0 0 0];

%T = 0:0.01:1; %Simulate for 1 second
%U = [3*randn(3,length(T))];
%lsim(sysh2,U,T,x0)
[y_hinf,t,x_hinf] = initial(syshinf,x0);

%Plot
subplot(5,1,1)
plot(t,x_hinf(:,4),t,zeros(size(t)),'r--')
set(legend('$$\alpha$$'),'Interpreter','Latex','FontSize', 10);
xlim([0,2.5])

ylabel('\alpha')

subplot(5,1,2)
plot(t,x_hinf(:,5),t,zeros(size(t)),'r--')
set(legend('$$\beta$$'),'Interpreter','Latex','FontSize', 10);
xlim([0,2.5])

ylabel('\beta')

subplot(5,1,3)
plot(t,x_hinf(:,6),t,zeros(size(t)),'r--')
set(legend('$$d\theta_a$$'),'Interpreter','Latex','FontSize', 10);
xlim([0,2.5])

ylabel('d\theta_a')

subplot(5,1,4)
plot(t,x_hinf(:,7),t,zeros(size(t)),'r--')
set(legend('$$d\theta_b$$'),'Interpreter','Latex','FontSize', 10);
xlim([0,2.5])

ylabel('d\theta_b')

subplot(5,1,5)
plot(t,x_hinf(:,8),t,zeros(size(t)),'r--')
set(legend('$$d\theta_c$$'),'Interpreter','Latex','FontSize', 10);
xlim([0,2.5])
xlabel('Time(sec)')
ylabel('d\theta_c')


sgtitle('Simulation of the system with H-infty controller')
%% H infty disturbance
%Add disturbance to the input
T = 0:0.01:1; %Simulate for 1 second
U = [3*randn(3,length(T))];
lsim(syshinf,U,T,x0)

max(abs(F*x_hinf'),[],2)