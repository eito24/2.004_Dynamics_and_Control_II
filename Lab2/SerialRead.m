% Matlab Script to Read and Plot Seral Data
%
%           Version 4.0 (11/1/2018) H.C.
%           Version 5.0 (9/24/2020) H.C.
%

% make sure no residual serial object in Matlab workspace
if (exist('s1','var'))
    delete(s1);
    clear s1;
end

% clear all figures and variables in workspace
close all
clear data data_char status
delete(instrfind);
  
disp(' ');
disp('*** Serial Data Capture ***');
disp('*** Press any key to stop ***');
disp(' ');

% define serial object with matching com port and baud rate
% change com port number and/or baud rate if needed
% s1 = serialport("COM3",115200);       % Windows
s1 = serialport("/dev/cu.usbmodem14201",115200);        % MacOS

status = getpinstatus(s1);
configureTerminator(s1,"LF");

% initialize variables
i = 1;
value = [];
figure(1);

% here we define 3 data lines, add or substract lines if needed
h1 = animatedline ('Color','g');
h2 = animatedline ('Color','b');
h3 = animatedline ('Color','r');

title('Streaming Serial Data <Press any key to stop>')
xlabel('Time (sec)'),ylabel('Values'), grid
legend('Data1','Data2','Data3');

% get data from the serial object if no key is pressed
while ( isempty(value) )
    data_char{i} = readline(s1);
    
    disp(['Collecting data sample #',num2str(i), '...',...
        'Press any key to stop.']);
    if(~strcmp(data_char{i},''))
        data(i,:) = str2num(data_char{i});
    else
        break;
    end
    
    addpoints(h1, data(i,1), data(i,2));
    addpoints(h2, data(i,1), data(i,3));
    addpoints(h3, data(i,1), data(i,4));
    
    drawnow limitrate
    value = double(get(gcf,'CurrentCharacter'));
    i = i+1;
end

drawnow;
disp('*** Done');
disp(' ');

% re-plot the data once the data collection is done to get all figure features
figure(1)
G = tf(8.7755102, [0.15,1]); % create transfer function for the plant
[Y, T] = step(2.94*G, 2); % generate a step response with 2.94V input (PWM = 150)
plot(data(:,1)-data(1,1), data(:,2),'g', data(:,1), data(:,3),'b', data(:,1), data(:,4),'r')
hold on
plot(T, Y, 'm')
plot(out.ang_vel);
title('Serial Data')
xlabel('Time (sec)'),ylabel('Values'), grid
legend('Data1','Data2','Vin','Simulated','Simscape');

dt = mean(diff(data(:,1)));
disp(['Sampling period (sec) = ', num2str(dt)])
disp(['Sampling frequency (Hz) = ', num2str(1/dt)])

% To disconnect the serial port object from the serial port
delete(s1);
clear s1;

% Use the two commands below to force clear any open com port if needed
% delete(instrfind);
