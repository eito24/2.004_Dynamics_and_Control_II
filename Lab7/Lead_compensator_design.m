% Lead controller design based on frequency response
% close all

%% Plant Model

% Update tau and Kdc values below to match your plant's values 
tau = 0.15;     % time constant
Kdc = 8.78;      % DC gain

vc2pwm = 51;
K = Kdc/vc2pwm;    % composite gain

% define Laplace transform operator
s = tf('s');

% open-loop plant TF
Gp = K/( tau * s^2 + s );   % flywheel angular position plant
set(Gp,'InputName','Input PWM')
set(Gp,'OutputName','Output Position')
disp('Plant transfer function:')
zpk(Gp)

%% Lead Controller Design
% 
% Controller: Gc = K (Td*s + 1)/(alpha*Td*s + 1)
%          K - DC Gain, Alpha- Pole/Zero Separation Ratio, Td - Time Constant
%
% Specifications: Percent overshoot <= 7%
%                 2% settling time <= 0.4 seconds

% === TO DO: Input your values for the following parameters === %
phi_m = deg2rad(40);    % compensator max phase in radians
omega_c = 15.48;           % desired crossover-frequency in rad/s

% Find compensator's pole-zero separation ratio alpha
% Max Phase phi_m = sin^-1((1-alpha)/(1+alpha))
%     - Solving for alpha we get:
% formula for alpha
alpha = (1-sin(phi_m))/(1+sin(phi_m));

% Crossover Frequency -> Controller Time Constant
% formula for Td (function of alpha and omega_c)
Td = 1/(sqrt(alpha)*omega_c);  % seconds

G = (Td*s+1)/(alpha*Td*s+1);

% we need to figure out the controller gain to get us the desired crossover 
% frequnecy with unity gain.
Kc = 1/abs(freqresp(G*Gp,omega_c)); % determine the overall gain to ensure the correct crossover point

% Final lead controller
Gc = Kc*(Td*s + 1)/(alpha*Td*s + 1);

disp('Lead controller transfer function:');
zpk(Gc)

% Verify the desired phase margin at the crossover frequency 
figure
bode(Gp)
hold
bode(Gc)
margin(Gc*Gp)
legend('Gp','Gc','Gc*Gp');

%% Verify the Lead controller with sisotool and examine various time and frequency responses

sisotool(Gp,Gc)