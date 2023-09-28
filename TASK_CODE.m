% Student Name: Soumya Ganguly
% PID : A53274333
% Final Project
% Linear Systems Theory (MAE 280A), Fall 2018
% Final Assignment

clc
% TASK 0: reference and wind generators are set to zero and system response
% has been recorded by defining the following: 

sys=ss(ALDmIp,BLDmIp,CLDmIp,DLDmIp,0.005)


% TASK 1:
% part 1:
o1=obsv(sys)   % observability matrix
r_o=rank(o1)   
if(r_o==5)
    disp('The system is observable')
else
    disp('The system is not observable')
end

%part2:
r1=ctrb(sys) %reachability matrix
r_r=rank(r1)
if(r_r==5)
    disp('The system is reachable')
else
    disp('The system is not reachable')
end

% part3 :
e_given=eig(ALDmIp);
eigIlike_K=0.95*e_given;
eigIlike_L=0.95*e_given;

K_new=place(ALDmIp,BLDmIp,eigIlike_K) % K Matrix for the feedback
L_new=(place(ALDmIp',CLDmIp',eigIlike_L))' % L matrix for our controller-observer


%TASK2
sys2=ss(ALDmIp-BLDmIp*K_new-L_new*CLDmIp, L_new, K_new, 0,0.005);

%in the last command we are defining the sys2 which acts as the
%feedback-gain
sys_feedback=feedback(sys,sys2);%command to get a feedback system

% simulation of feedback system
ti=0:0.005:20;
x0=[0;0.1;0;0;0;0;0.1;0;0;0];
u=zeros(1,length(ti));
plot1=lsim(sys_feedback,u,ti,x0,'zoh'); %simulating with zero order hold

figure(1)
subplot(2,2,1);
plot(ti,plot1(:,1))
title('$\theta$ $state (discrete)$','Interpreter','latex');
ylabel('$\theta$','Interpreter','latex');
xlabel('$time(seconds)$','Interpreter','latex');

subplot(2,2,2);
plot(ti,plot1(:,2))
title('$\phi$ $state (discrete)$','Interpreter','latex');
ylabel('$\phi$','Interpreter','latex');
xlabel('$time(seconds)$','Interpreter','latex');

%TASK 3
syscont=d2c(sys_feedback,'zoh');
plot2=lsim(syscont,u,ti,x0,'zoh');

subplot(2,2,3);
plot(ti,plot2(:,1))
title('$\theta$ $state (CONT)$','Interpreter','latex');
ylabel('$\theta$','Interpreter','latex');
xlabel('$time(seconds)$','Interpreter','latex');

subplot(2,2,4);
plot(ti,plot2(:,2))
title('$\phi$ $state (CONT)$','Interpreter','latex');
ylabel('$\phi$','Interpreter','latex');
xlabel('$time(seconds)$','Interpreter','latex');

%TASK 4
% eigIlike_K=0.955*e_given
% eigIlike_L=0.9545*e_given

K_new=place(ALDmIp,BLDmIp,eigIlike_K); % K Matrix for the feedback
L_new=(place(ALDmIp',CLDmIp',eigIlike_L))'; % new L matrix for our controller-observer
KLDmIp=K_new;
LLDmIp=L_new;
ALCBKLDmIp =ALDmIp-BLDmIp*KLDmIp-LLDmIp*CLDmIp;
 

%TASK 5
sys3=ss(ALDmIp-BLDmIp*K_new-L_new*CLDmIp, L_new, K_new, 0,0.005);
sys_feedback2=feedback(sys,sys3);
plot3=lsim(sys_feedback2,u,ti,x0,'zoh');

figure(2)
plot4=plot(ti,plot1(:,1))
title('$\theta$ $state$ $(discrete)$ $comparison$','Interpreter','latex');
ylabel('$\theta$','Interpreter','latex');
xlabel('$time(seconds)$','Interpreter','latex');
hold on;
plot(ti,plot3(:,1),'.')

figure(3)
plot5=plot(ti,plot1(:,2))
title('$\phi$ $state$ $(discrete)$ $comparison$','Interpreter','latex');
ylabel('$\phi$','Interpreter','latex');
xlabel('$time(seconds)$','Interpreter','latex');
hold on;
plot(ti,plot3(:,2),'.')

% Task 7 : done in simulink
% TASK 8 : 
ALDmIp1=[ALDmIp [0;0;0;0;0];[0 0 0 0.005 0 1]];
BLDmIp1=[BLDmIp;0];
CLDmIp1=[CLDmIp [0;0]];
syswind=ss(ALDmIp1,BLDmIp1,CLDmIp1,DLDmIp,0.005);
% owind=obsv(syswind);   % observability matrix
% r_owind=rank(owind)
% 
% rwind=ctrb(syswind); %reachability matrix
% r_cwind=rank(rwind)
eigIlike_kwind=[eigIlike_K; 0.951];
eigIlike_lwind=[eigIlike_L;1];
K_wind=place(ALDmIp1, BLDmIp1, eigIlike_kwind); % K Matrix for the feedback
L_wind=(place(ALDmIp1',CLDmIp1',eigIlike_lwind))'; % new L matrix for our controller-observer
KLDmIp1=K_wind;
LLDmIp1=L_wind;
ALCBKLDmIp1=ALDmIp1-BLDmIp1*KLDmIp1-LLDmIp1*CLDmIp1;



