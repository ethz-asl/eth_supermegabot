%% set up filter (used to generate coefficient test data)
format long g
gain = 1.0;
tau = 0.1;
damp = 0.9;
nom = [gain, 0];
denom = [tau, 1];
sys = tf(nom, denom)

T = 0.0025;
sys_dt = c2d(sys, T, 'tustin')

[Num,Den,Ts] = tfdata(sys_dt);
Num{1}
Den{1}

%% Filter data for test data generation
t = 0:T:10;
%u = sin(2*pi/10*t) + rand(1,length(t))/10; % generate sine with noise
u = t>=5; % generate step
y = lsim(sys_dt, u, t);
%%
% create c source code file
fileId = fopen('test_data.h', 'w');
fprintf(fileId, '// Test data generated with matlab_filter_coeff.m\n');
fprintf(fileId, 'const unsigned int len = %d;\n', length(t));
fprintf(fileId, 'const double dt = %f;\n', T);
fprintf(fileId, 'const double tau = %f;\n', tau);
fprintf(fileId, 'const double gain = %f;\n', gain);
%fprintf(fileId, strcat('const double t[%d] = {', join(string(t), ', '), '};\n'), length(t));
fprintf(fileId, strcat('const double u[%d] = {', join(string(u), ', '), '};\n'), length(u));
fprintf(fileId, strcat('const double y[%d] = {', join(string(y), ', '), '};\n'), length(y));
fclose(fileId);
%% setup variables for DT coeff calculation
nom = fliplr(nom);
denom = fliplr(denom);

len = max(length(nom), length(denom));
if length(nom) > length(denom)
    denom(length(nom)) = 0;
elseif length(denom) > length(nom)
    nom(length(denom)) = 0;
end

h = 2/T;

%% calculate coeffs with matlab operators
% based on "AN ALGORITHM FOR THE COMPUTATION OF THE TUSTIN  BILINEAR
% TRANSFORMATION" by D. Westreich
R = 1;
P = nom(len);
Q = denom(len);

i = len - 1;
while i >= 1
    R = R*(z+1);
    P = h*P*(z-1) + nom(i)*R;
    Q = h*Q*(z-1) + denom(i)*R;
    i = i - 1;
end

sys_dt2 = simplify(P/Q)
%% 'c++-style' coeff calculation
% vectors containing coefficients to [z^0, z^1, ..., z^k]
nom_z = zeros(len,1);
denom_z = zeros(len,1);
R = zeros(len,1);

R(1) = 1;
nom_z(1) = nom(len);
denom_z(1) = denom(len);
for i=len-1:-1:1
    for j=len:-1:2
        R(j) = R(j-1) + R(j); % R = R*z + R;
        nom_z(j) = h*(nom_z(j-1) - nom_z(j)) + nom(i)*R(j);
        denom_z(j) = h*(denom_z(j-1) - denom_z(j)) + denom(i)*R(j);
    end
    nom_z(1) = h*(-nom_z(1)) + nom(i)*R(1);
    denom_z(1) = h*(-denom_z(1)) + denom(i)*R(1);

    i
    nom_z
    denom_z
end

nom_z/denom_z(end)
denom_z/denom_z(end)