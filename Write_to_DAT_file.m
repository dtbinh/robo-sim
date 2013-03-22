clc, clear all;
%%Instantiating constants %%
c = 3e8; %speed of light
f = 2.4e9; %frequency to calculate lambda

%USER INPUTS
GAP = 10; %Used to calculate # of steps which determines array size
Ant_Tx_Power = 9; %gain in dB
Ant_Rx_Sens = 1; %receiving sensitivity in dB

STEPS = 360/GAP; %creates the number of steps from degrees gaps
lambda = c/f; %wavelength
Data_array_size= single((STEPS)^2+3); % "+3" is for lambda, Ant_Tx_Power, Ant_Rx_Sens
Data_array_bytesize = Data_array_size*4; %Gives size of single precision(float) array

%% Create results array to pull data from %%

results = single(zeros([1,Data_array_size]));
results(1)= lambda;
results(2)= Ant_Tx_Power;
results(3)= Ant_Rx_Sens;

%filling with values equal to current index as a check
for i=4:Data_array_size
    results(i)=i;
end
%% Write to .dat file
fid = fopen('Yagi9dBi.dat','wb');
fwrite(fid, results, 'single');
fclose(fid);
