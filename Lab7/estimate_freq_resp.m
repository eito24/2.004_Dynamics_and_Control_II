function estimate_freq_resp(data, Gp, Gc, RESPONSE_TYPE)
% 2.004 flywheel plant frequency analysis
% Frequency response of time response data and transfer function
%
% Syntax:
% estimate_freq_resp(data, Gp, Gc, RESPONSE_TYPE);
% data : time domain response data (acquired by SerialRead.m)
% Gp : plant model transfer function (tf data type)
% Gc : controller transfer function (tf data type)
% RESPONSE_TYPE : a string variable for 'open_loop" or 'closed_loop'
%
% Example: to plot the Bode diagram of the open-loop plant
%          >> estimate_freq_resp(data, Gp, 1, 'open_loop')

%           Version 2.0 (11/01/2018) H.C.
%           Version 3.0 (11/05/2021) H.C.

% === Make necessary changes to the lines below to reflect your data === %
tmin = 0;           % start time (sec)
tmax = 100;          % end time (sec)

min_freq = 0.05 * (2*pi);       % Hz to rad/s
max_freq = 10.0 * (2*pi);       % Hz to rad/s
% ====================================================================== %

% set response_type to 'open_loop' or 'closed_loop'
% RESPONSE_TYPE = 'open_loop';

% create necessary variables from saved data
index = find(tmin <= data(:,1) & data(:,1) <= tmax);
n = length(index);
time = data(index,1);
dt = mean(diff(time));

% % apply detrend function to remove any DC component or linear trend
setpoint = detrend(data(index, 2));
sensor = detrend(data(index, 3));
pwm = detrend(data(index, 4));    % PWM/100

% use tfestimate function to estimate open loop transfer function
% "setpoint" as input and "sensor" as output
[Txy,F] = tfestimate( setpoint, sensor, [], 2048, 2048, 1/dt );

switch RESPONSE_TYPE
    case 'open_loop'
        systfest_ol = frd(Txy,2*pi*F);
    case 'closed_loop'
        systfest_cl = frd(Txy,2*pi*F);
    otherwise
        disp('Please set the RESPONSE_TYPE to ''open_loop'' or ''closed_loop''.');
end

% === Plant TF model here ============================================== %
% Update tau and Kdc values below to match your plant's values 
% tau = 0.143;   % sec
% Kdc = 9.96;     %rad/(sec * Volt)
% vc2pwm = 51;
% K = Kdc/vc2pwm;    % composite gain
% 
% % define Laplace transform operator
% s = tf('s');
% 
% % open-loop plant TF
% Gp = K/( tau * s^2 + s );   % flywheel angular position plant
set(Gp,'InputName','Input PWM')
set(Gp,'OutputName','Output Position')

% Modify the controller TF below with actual controller
% Gc = 1;

% closed-loop TF with feedback
Gp_cl = feedback( Gc * Gp, 1 );

% ====================================================================== %

% Plot either open or closed loop Bode diagram
switch RESPONSE_TYPE
    case 'open_loop'
        figure(10);
        h_ol = bodeplot(Gc*Gp, systfest_ol, {min_freq, max_freq}); grid;
        % setoptions(h_ol,'FreqUnits','Hz');
        title('Open Loop Transfer Functions');
        legend('Model','Actual')
    case 'closed_loop'
        figure(11)
        h_cl = bodeplot(Gp_cl, systfest_cl, {min_freq, max_freq}); grid;
        % setoptions(h_cl,'FreqUnits','Hz');
        title('Closed Loop Transfer Functions');
        legend('Model','Actual')
    otherwise
        disp('Please set the switch to ''open_loop'' or ''closed_loop''.');
end

