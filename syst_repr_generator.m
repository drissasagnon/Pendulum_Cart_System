function [plant_ss, plant_tf, plant_zpk] = syst_repr_generator(params,output)


A = [0 0                                                  1                               0;
     0 0                                                  0                               1;
     0 params.m*params.g/params.M                        -params.k1/params.M             -params.k2/(params.M*params.l);
     0 (params.M+params.m)*params.g/(params.M*params.l)  -params.k1/(params.M*params.l)  -(params.M+params.m)*params.k2/(params.M*params.m*(params.l^2))];

B = [0;
     0;
     1/(params.M*params.l);
     (params.M+params.m)/(params.M*params.m*(params.l^2))];

% B = [0;
%      0;
%      1/(params.M);
%      1/(params.M*(params.l))];

if output == "q1"
    C = [1, 0, 0, 0];
elseif output == "q2"
    C = [0, 1, 0, 0];
end
D = 0;

plant_ss = ss(A,B,C,D);
plant_tf = ss2tf(A,B,C,D);
plant_zpk = zpk(plant_ss);


end

