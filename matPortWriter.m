%% Setup
clc; clear; close all

%% Bruger definerede variabler (?ndre p? disse)
% Definer samplings frekvens
fs = 1024;
% Indstil antal sekunder der skal plottes
secToPlot = 50;
% Fil der skal loades
%mappeMedFil = 'DataUdenArtefakter/Jacob';
%fileToLoad = {'Jacob', 'walk_knee_omgang1'};
fileToLoad = 'bike150_knee_omgang1';
% Opdateringsrate
pointsPerUpdate = 100;
% Simuleret smaple rate i Hz
simulatedfs = 100; 
% Antal points to skip for at få simuleredt sample rate
pointsToSkip = floor(fs / simulatedfs);

%% Load data
% Load fil
data = load(fileToLoad + ".mat");
% Udpak fil
standardizedData = data.cutData; %standardizeDataStructure(data, 'struct');
% Omdan data til data med simuleret fs
simulatedData = standardizedData(1:pointsToSkip:end,:);
% Find l?ngde af standardizedData og st?rrelse
dataLen = length(simulatedData);
dataSize = size(simulatedData);
% Find antal datapunkter der skal plottes
pointsToPlot = simulatedfs * secToPlot;
% Lav start/stop index fra hvor der skal plottes
plotIndexes = [(floor(dataLen/2) - floor(pointsToPlot/2)) (floor(dataLen/2) + floor(pointsToPlot/2))];
% Find udsnit der skal plottes
dataToPlot = int16(simulatedData(plotIndexes(1):plotIndexes(2),2:7));

%% Start COM port og skriv data til port
% Port til ESP32
comport = '/dev/ttyUSB7';
% Start seriel port instance
serial_port = serial(comport, 'TimeOut', 10, 'BaudRate', 115200, 'InputBufferSize', 4096);

try 
    % Åben port
    fopen(serial_port)
    port_open = 1;
    pause(1)
catch err
    % Giv fejl hvis port ikke kan åbnes
    disp(err.message)  
    disp(sprintf('\n Tjek jeres device manager for at finde jeres egen com-port\n'))
    port_open = 0;
end

allPlot = figure();
plot(dataToPlot)

cutPlot = figure();
bufferSize = 512;


dataIndex = 1;
if port_open
   while dataIndex < dataLen - 2
       fwrite(serial_port, dataToPlot(dataIndex,1), 'int16');
       pause(0.005)
       fwrite(serial_port, dataToPlot(dataIndex,2), 'int16'); 
       pause(0.005)
       fwrite(serial_port, dataToPlot(dataIndex,3), 'int16');  
       pause(0.005)
       fwrite(serial_port, dataToPlot(dataIndex,4), 'int16'); 
       pause(0.005)
       fwrite(serial_port, dataToPlot(dataIndex,5), 'int16'); 
       pause(0.005)
       fwrite(serial_port, dataToPlot(dataIndex,6), 'int16');         
       %pause(1)
       dataIndex = dataIndex + 1;
       pause(0.104)
       disp("Data: " + int2str(dataToPlot(dataIndex,1)) + " " + int2str(dataToPlot(dataIndex,2)) ...
           + " " + int2str(dataToPlot(dataIndex,3)) + " " + int2str(dataToPlot(dataIndex,4)) ...
           + " " + int2str(dataToPlot(dataIndex,5)) + " " + int2str(dataToPlot(dataIndex,6)))
       if mod(dataIndex, 50) == 0
           xline(dataIndex);
       end
   end
end
fclose(serial_port);