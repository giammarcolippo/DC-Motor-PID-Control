%% PROJECT 2: DC Motor Position Control using PID
% Case A: No Torque Disturbance
clc
close all
clear all

%Plant Parameters
Vmax=24;                             % Max Voltage [V]
J=0.1;                               % Rotational Inertia [kg*m^2]
b=0.008;                             % Viscous friction coefficient [Nms]
Kb=1.25;                             % Back EMf constant [V/rad/sec]
Kt=0.5;                              % Torque constant [Nm/Amp]
R=0.8;                               % Armature Resistance [ohm]
L=0.02 ;                             % Armature Inductance H]
T_dist=[0,0.3*Vmax/R*Kt];            % Torque array [Nm]
Torque=T_dist(1);                    % Select torque value for this case [N*m][Nm]

%PID Controller
Kp_PID=[30,50,20];           % proportional coefficient
Kd_PID=[5,8,2];              % derivative coefficient
Ki_PID=[2,1,4];              % integral coefficient

%Simulation Loop  & post-processing
% Output matrix pre-allocation
sim_time=6;                                   % simulation time [s]
dt=0.01;                                      % delta sim_time [s]
N=sim_time/dt+1;                              % tout array length [s]
theta_out_noload=zeros(N,length(Kp_PID));     % angular position matrix preallocation 
Volt_PID_noload=zeros(N,length(Kp_PID));      % saturated control signal matrix preallocation


for i = 1:length(Kp_PID)
    Kp = Kp_PID(i);                                
    Kd = Kd_PID(i);                                
    Ki = Ki_PID(i);
    out = sim('DC_motor');                                 % Simulink Simulation
    theta_out_noload(:,i)=out.output;                      % Angular postion [rad]
    theta_PID_noload(:,i)=rad2deg(theta_out_noload(:,i));  % Angular postion [deg]
    Volt_PID_noload(:,i)=out.sig_limited;                  % Saturated Control signal
                   
end

time=out.tout;                   % time-array [s]
input=out.input;                 % ref. angular position [rad]
theta_ref=rad2deg(input);        % ref. angular position [deg]

theta_PID1_noload = theta_PID_noload(:,1);      % angular position PID1 [deg]  
theta_PID2_noload = theta_PID_noload(:,2);      % angular position PID2 [deg]
theta_PID3_noload = theta_PID_noload(:,3);      % angular position PID3 [deg]
Volt_PID1_noload = Volt_PID_noload(:,1);        % Saturated Control signal PID1 [V]
Volt_PID2_noload = Volt_PID_noload(:,2);        % Saturated Control signal PID2 [V]
Volt_PID3_noload = Volt_PID_noload(:,3);        % Saturated Control signal PID3 [V]

%plotting section
figure
set(gcf,'Units','normalized','Position',[0.025 0.025 0.95 0.9]) 

subplot(2,2,1);
hold on
plot(time,theta_ref,'--','LineWidth',1.5)
plot(time,theta_PID1_noload,'r','LineWidth',1)
plot(time,theta_PID2_noload,'b','LineWidth',1)
plot(time,theta_PID3_noload,'k','LineWidth',1)
x_rect = [0.8 0.8 4.5 4.5 0.8];
y_rect = [170 40 40 170 170];
plot(x_rect,y_rect,'r','LineWidth',1.5)
xlim([0,sim_time])
xlabel('Time [s]')
ylabel({'\theta','[°]'})
title({'DC Motor Response','Tuned response vs Step'});
legend('Reference','PID1','PID2','PID3','Location','northeast')
grid on

subplot(2,2,2);
box on
hold on
plot(time,theta_ref,'--','LineWidth',1.5)
plot(time,theta_PID1_noload,'r','LineWidth',1)
plot(time,theta_PID2_noload,'b','LineWidth',1)
plot(time,theta_PID3_noload,'k','LineWidth',1)
xlim([0,sim_time])
xlabel('Time [s]')
ylabel({'\theta','[°]'})
xlim([0.8 4.5])
ylim([40 170])
title({'Zoom: Tuned response vs Step','Initial Transient'})
legend('Reference','PID1','PID2','PID3','Location','northeast')
grid on

subplot(2,2,3);
hold on
plot(time,Volt_PID1_noload,'r','LineWidth',1)
plot(time,Volt_PID2_noload,'b','LineWidth',1)
plot(time,Volt_PID3_noload,'k','LineWidth',1)
yline(Vmax,'k--','LineWidth', 1.5,'HandleVisibility','off')  
yline(-Vmax,'k--','LineWidth', 1.5,'HandleVisibility','off')
a_rect = [0.8 0.8 4.5 4.5 0.8];
b_rect = [30 -20 -20 30 30];
plot(a_rect,b_rect,'r','LineWidth',1.5)
xlim([0,sim_time])
ylim([-1.3*Vmax,1.3*Vmax])
xlabel('Time [s]')
ylabel({'Voltage','[V]'})
title({'Saturated PID Control Signal'})
legend('PID1','PID2','PID3','Location','southeast')
text(sim_time/1.25, Vmax+2.8,'V_m_a_x = 24 V','Color','k','FontSize',10,'FontWeight','bold') 
text(sim_time/4.25,-Vmax-2.8,'V_m_i_n = -24 V','Color','k','FontSize',10,'FontWeight','bold') 
grid on

subplot(2,2,4);
box on
hold on
plot(time,Volt_PID1_noload,'r','LineWidth',1)
plot(time,Volt_PID2_noload,'b','LineWidth',1)
plot(time,Volt_PID3_noload,'k','LineWidth',1)
yline(Vmax,'k--','LineWidth', 1.5,'HandleVisibility','off')  
xlim([0.8 4.5])
ylim([-20,30])
xlabel('Time [s]')
ylabel({'Voltage','[V]'})
title({'Zoom: Saturated PID Control Signal','Initial Transient'})
legend('PID1','PID2','PID3','Location','southeast')
text(sim_time/2.5, Vmax+2.8,'V_m_a_x = 24 V','Color','k','FontSize',10,'FontWeight','bold') 
grid on
sgtitle(    {'Project #2: DC Motor Position Control using PID',...
             'Case A: No Torque Disturbance'},...
             'FontSize', 12,'FontWeight', 'bold');

%salvataggio file csv
T1 = table(time,theta_PID1_noload,theta_PID2_noload,theta_PID3_noload,theta_ref, ...
    Volt_PID1_noload,Volt_PID2_noload,Volt_PID3_noload,...
           'VariableNames', {'time','theta_PID1_noload','theta_PID2_noload',...
           'theta_PID3_noload','theta_ref','Volt_PID1_noload',...
           'Volt_PID2_noload', 'Volt_PID3_noload'});
writetable(T1,'noload_DC_motor.csv')

%------------------------------------------------------------------------%
% CASE B: TORQUE DISTURBANCE

Torque=T_dist(2);                          % Torque value [Nm]

dt=0.01;                                   % delta time_sim [s]
N=sim_time/dt+1;                           % tout array length [s]
theta_out=zeros(N,length(Kp_PID));         % angular position matrix preallocation 
Volt_PID3=zeros(N,length(Kp_PID));         % saturated control signal matrix preallocation

%Post-processing
Kp = Kp_PID(3);                            % proportional coefficient PID3
Kd = Kd_PID(3);                            % Derivative coefficient PID3
Ki = Ki_PID(3);                            % Integral coefficient PID3
out = sim('DC_motor');                     % Simulink Simulation
theta_out=out.output;                      % Angular position PID3 [rad]
theta_PID3=rad2deg(theta_out);             % Angular position PID3 [deg]
Volt_PID3=out.sig_limited;                 % Saturated Control signal PID3 [V]
torque=out.torque;                         % Torque disturbance [Nm]
                   
%plotting section
figure 
set(gcf,'Units','normalized','Position',[0.025 0.025 0.95 0.9]) 

subplot(2,3,1);
hold on
plot(time,theta_ref,'--','LineWidth',1)
plot(time,theta_PID3_noload,'k','LineWidth',1)
plot(time,theta_PID3,'r','LineWidth',1)
x_rect = [0.8 0.8 4.5 4.5 0.8];
y_rect = [135 60 60 135 135];
plot(x_rect,y_rect,'b','LineWidth',1.5)
xlim([0,sim_time])
ylim([0,140])
xlabel('Time [s]')
ylabel({'\theta','[°]'})
title({'DC Motor Response','Tuned response vs Step'});
legend('Reference','PID3','PID3 Torque disturbance','Location','southeast')
grid on

subplot(2,3,[2 3]);
box on
hold on
plot(time,theta_ref,'--','LineWidth',1)
plot(time,theta_PID3_noload,'k','LineWidth',1)
plot(time,theta_PID3,'r','LineWidth',1)
xlim([0,sim_time])
xlabel('Time [s]')
ylabel({'\theta','[°]'})
xlim([0.8 4.5])
ylim([60 135])
title({'Zoom: Tuned response vs Step','Initial Transient'})
legend('Reference','PID3','PID3 Torque disturbance','Location','northeast')
grid on

subplot(2,3,4);
hold on
plot(time,Volt_PID3_noload,'k','LineWidth',1)
plot(time,Volt_PID3,'r','LineWidth',1)
yline(Vmax,'k--','LineWidth', 1.5,'HandleVisibility','off')  
yline(-Vmax,'k--','LineWidth', 1.5,'HandleVisibility','off')
a_rect = [0.8 0.8 4.3 4.3 0.8];
b_rect = [30 -15 -15 30 30];
plot(a_rect,b_rect,'b','LineWidth',1.5)
xlim([0,sim_time])
ylim([-1.3*Vmax,1.3*Vmax])
xlabel('Time [s]')
ylabel({'Voltage','[V]'})
title({'Saturated PID Control Signal'})
legend('PID3','PID3 Torque disturbance','Location','southeast')
text(sim_time/1.36, Vmax+2.5,'V_m_a_x = 24 V','Color','k','FontSize',10,'FontWeight','bold') 
text(sim_time/18,-Vmax+2.5,'V_m_i_n = - 24 V','Color','k','FontSize',10,'FontWeight','bold') 
grid on

subplot(2,3,5);
hold on
box on
plot(time,Volt_PID3_noload,'k','LineWidth',1)
plot(time,Volt_PID3,'r','LineWidth',1)
yline(Vmax,'k--','LineWidth', 1.5,'HandleVisibility','off')  
xlim([0.8 4.5])
ylim([-15,30])
xlabel('Time [s]')
ylabel({'Voltage','[V]'})
title({'Zoom: Saturated PID Control Signal','Initial Transient'})
legend('PID3','PID3 Torque disturbance','Location','southeast')
text(sim_time/2.5, Vmax+2.5,'V_m_a_x = 24 V','Color','k','FontSize',10,'FontWeight','bold') 
grid on

subplot(2,3,6);
hold on
grid on
plot(time,torque,'k','LineWidth',1)
xlim([0 sim_time])
ylim([-1,6])
xlabel('Time [s]')
ylabel({'Torque Disturbance','[Nm]'})
title('Step-Torque Disturbance')

sgtitle(    {'Project #2: DC Motor Position Control using PID',...
             'Case B:Torque Disturbance Applied'},...
             'FontSize', 12,'FontWeight', 'bold');

%salvataggio file csv
T2 = table(time,theta_PID3_noload,theta_PID3,theta_ref,Volt_PID3_noload,Volt_PID3,torque,...
           'VariableNames', {'time','theta_PID3_noload','theta_PID3','theta_ref','Volt_PID3_noload','Volt_PID3','torque'});
writetable(T2,'load_DC_motor.csv')