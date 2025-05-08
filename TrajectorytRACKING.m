% UAV-VT Tracking Simulation Using Tangential Guidance Logic
clear; clc;
dt = 0.01;             
T = 50;                 
N = T/dt;               
R_star=100;
%Uav Parameters
Va = 10;               
xa = zeros(1, N);       
ya = zeros(1, N);       
alpha_a = zeros(1, N);    
%Target Parameters
Vt = 20;              
xt = zeros(1, N);      
yt = zeros(1, N);      
alpha_t = deg2rad(50);     
xa(1) = 20; ya(1) = 60; alpha_a(1) = deg2rad(-45);  
xt(1) = 50; yt(1) = 140;
R= zeros(1, N);
theta=zeros(1,N);
sigma=zeros(1,N);  
d=zeros(1,N);
Kp = -2;

for k = 2:N
    xt(k) = xt(k-1) + Vt* cos(alpha_t) * dt;
    yt(k) = yt(k-1) + Vt * sin(alpha_t) * dt;
    delta_x=xt(k-1)-xa(k-1);
    delta_y=yt(k-1)-ya(k-1);
    R(k)=sqrt(delta_x^2 + delta_y^2);
    theta(k)=atan2(delta_y,delta_x);
    theta_dot=(theta(k)-theta(k-1))/dt
    alpha_d=2*theta(k)-alpha_t;
    d_dot=2*Va*sin(alpha_d);
    d(k)=d(k-1)+d_dot*dt;
    a_a=Va*theta_dot -Kp*(alpha_d-alpha_a(k-1)); %lateral accn
    alpha_a_dot= a_a/Va;
    alpha_a(k) = alpha_a(k-1) + alpha_a_dot * dt;
    xa(k) = xa(k-1) + Va * cos(alpha_a(k)) * dt;
    ya(k) = ya(k-1) + Va * sin(alpha_a(k)) * dt;

end


figure;
plot(xt, yt, 'k--', 'LineWidth', 2); hold on;
plot(xa, ya, 'b', 'LineWidth', 2);
legend('Virtual Target Path', 'UAV Path');
xlabel('X [m]'); ylabel('Y [m]');
title('UAV Tangential Guidance Path Tracking');
grid on;
axis equal;

figure;
plot((0:N-1)*dt, d);
xlabel('Time [s]'); ylabel('Lateral Deviation d [m]');
title('Lateral Deviation over Time');
grid on;
