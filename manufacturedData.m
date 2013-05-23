%% DEFINE NON-GAIN ATTRIBUTES
    c = 3e8; %speed of light
    f = 2.4e9; %frequency to calculate lambda
    lambda = single(c/f); %wavelength
    
    Ant_Tx_Power = single(0.001); %output power in watts
    Ant_Rx_Sens = single(0.000001); %receiving sensitivity in watts
    ANT_R_Coef = single(0); %antenna reflection coefficient
    pol_vec = [0, 0, 0];
    axial_ratio = 1;
 
    %% CREATE SAMPLE DATA
    sigma = 20;
    center = [18 18];
    gsize = size(zeros(36));
    [R,C] = ndgrid(1:gsize(1), 1:gsize(2));
    mat = gaussC(R,C, sigma, center);
    flatmat = reshape(mat, numel(mat), 1);
    
    %% WRITE TO DATA FILE
    results = zeros(8 + 36^2, 1);
    results(1)= lambda;
    results(2)= Ant_Tx_Power;
    results(3)= Ant_Rx_Sens;
    results(4)= ANT_R_Coef;
    results(5:7)= pol_vec;
    results(8)= axial_ratio;
    results(9:end)= 1;
    
    fid = fopen('SampleAnt.dat','wb')
    fwrite(fid, results, 'single');
    fclose(fid);