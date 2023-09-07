% ==========================================================================
% Numerical Example - Model Predictive Control (MPC)
% Leonardo F. Toso
% ==========================================================================

clear all; clc;close all

%Problem data

R=0.01*eye(3);
Q=eye(3);
Np=3; %prediction horizon
alpha=0.005; beta=alpha; %step-size
rp=[1;1;1]; %unit step samples


% Continuous-time model

A=[-0.1 -3;1 0]; B=[1;0]; C=[0 10]; D=0;

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
Z=[Ca*Ba 0 0;Ca*Aa*Ba Ca*Ba 0;Ca*(Aa^2)*Ba Ca*Aa*Ba Ca*Ba];
%Initial conditions: 
u=[0];
xa=zeros(n+p,p);
y=[xa(3)];
Del_U=zeros(Np,1);
G=[-1 0 0;-1 -1 0;-1 -1 -1;1 0 0;1 1 0;1 1 1];
nc=6;%number of constraints
mu=2*ones(nc,1);
J=[];
N=30;%time horizion
L=5e3;%number of iterations of the gradient descent
for k=2:N
     
    %Gradient Descent
    for r=1:L
        H=[0.28+u(k-1) 0.28+u(k-1) 0.28+u(k-1) 0.48-u(k-1) 0.48-u(k-1) 0.48-u(k-1)]';
        g=G*Del_U-H;
        grad_J=-(rp-W*xa-Z*Del_U)'*Q*Z + Del_U'*R;
        D_g=G;
        Del_U = Del_U - alpha*(grad_J' + D_g'*mu);
        mu=mu+beta*(g);
        mu=max(zeros(nc,1),mu);
    end
    J=[J 0.5*(rp-W*xa-Z*Del_U)'*Q*(rp-W*xa-Z*Del_U) + (0.5)*Del_U'*R*Del_U];
    u=[u Del_U(1)+u(k-1)];
    xa=Aa*xa+Ba*Del_U(1); 
    y=[y xa(3)];
end

%% Plotting the results

subplot(2,1,1);
time=0:N-1;
stem(time,u);
xlab=xlabel('Time','Interpreter','latex');
ylab=ylabel('$u$','Interpreter','latex');
title('Control Input')
subplot(2,1,2); 
stem(0:N-1,y);
title('System output')
xlab=xlabel('Time','Interpreter','latex');
ylab=ylabel('$y$','Interpreter','latex');

