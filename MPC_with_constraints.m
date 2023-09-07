% ==========================================================================
% Numerical Example - Model Predictive Control (MPC)
% Leonardo F. Toso
% ==========================================================================

clear all; clc;close all

%Problem data

R=0.1*eye(6);
Q=eye(6);
Np=3; %prediction horizon
a=1e-3; %step-size
b=1e-3;
rp=[[10;5];[10;5];[10;5]]; %unit step samples


% Continuous-time model

A=[-0.003 0.039  0  -0.322;1 0 7.74  0; 0.020 -0.101 -0.429  0;0  0  1  0];
B=[0.010  1;-0.18 -0.04;-1.16 0.598;0.5 0.1]; C=[1 0 0 0;0 -1 0 7.74]; D=zeros(2,2);


% Generate the discrete state-space model

h=0.1; %sampling time
[Ad,Bd,Cd,Dd]=c2dm(A,B,C,D,h,'zoh');
n=size(Ad,1);%number of states
p=size(Bd,2);%number of inputs

Aa=[Ad zeros(n,p);Cd*Ad eye(p)];
Ba=[Bd;Cd*Bd];
Ca=[zeros(n,p)' eye(p)];


%Define the matrices W and Z
W=[Ca*Aa;Ca*(Aa^2);Ca*(Aa^3)];
Z=[Ca*Ba zeros(2,2) zeros(2,2);Ca*Aa*Ba Ca*Ba zeros(2,2);Ca*(Aa^2)*Ba Ca*Aa*Ba Ca*Ba];
%Initial conditions: 
u=[0;0];
xa=[[0;0;0;0];[0;0]];
y=[xa(5:6,1)];
Del_U=[[0;0];[0;0];[0;0]];
J=[];
G=[-eye(2) zeros(2,2) zeros(2,2);-eye(2) -eye(2) zeros(2,2);-eye(2) -eye(2) -eye(2);eye(2) zeros(2,2) zeros(2,2);eye(2) eye(2) zeros(2,2);eye(2) eye(2) eye(2)];
mu=[[0;0];[0;0];[0;0];[0;0];[0;0];[0;0]];

bound=[5;5];
N=50;%time horizion
L=1e4;%number of iterations of the gradient descent
for k=2:N
     
    %Gradient Descent
    for r=1:L
        H=[bound+u(k-1); bound+u(k-1); bound+u(k-1); bound-u(k-1); bound-u(k-1); bound-u(k-1)];
        g=G*Del_U-H;
        grad_J=-(rp-W*xa-Z*Del_U)'*Q*Z + Del_U'*R;
        D_g=G;
        Del_U = Del_U - a*(grad_J' + D_g'*mu);
        mu=mu+b*(g);
        mu=max(zeros(12,1),mu);
    end
    J=[J 0.5*(rp-W*xa-Z*Del_U)'*Q*(rp-W*xa-Z*Del_U) + (0.5)*Del_U'*R*Del_U];
    u=[u Del_U(1:2)+u(k-1)];
    xa=Aa*xa+Ba*Del_U(1:2); 
    y=[y Ca*xa];
end

%% Plotting the results

subplot(2,1,1);
time=0:N-1;
stem(time,u(1,:));
hold on
stem(time,u(2,:));
xlab=xlabel('Time','Interpreter','latex');
ylab=ylabel('$u$','Interpreter','latex');
title('Control Input')
subplot(2,1,2); 
stem(0:N-1,y(1,:));
hold on 
stem(0:N-1,y(2,:));
title('System output')
xlab=xlabel('Time','Interpreter','latex');
ylab=ylabel('$y$','Interpreter','latex');

