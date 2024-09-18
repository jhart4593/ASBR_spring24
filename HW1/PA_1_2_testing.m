%% PA1a

% Test Code
% Test Error Condition 1 (matrix size check) 
RotMat = zeros(4,3); 
[axis, angle] = RotMat2AxisAngle(RotMat)

% Test Error Condition 2 (orthogonal)
RotMat = zeros(3,3);
[axis, angle] = RotMat2AxisAngle(RotMat)

% Test Error Condition 2 (preserves orientation)
RotMat = -eye(3);
[axis, angle] = RotMat2AxisAngle(RotMat)

% First Case
RotMat = eye(3);
[axis, angle] = RotMat2AxisAngle(RotMat)

% 2nd Case (x rotation)
phi = pi;
Rx = [1 0 0; 
    0 cos(phi) -sin(phi)
    0 sin(phi) cos(phi)]; 
[axis, angle] = RotMat2AxisAngle(Rx)

% 2nd Case (y rotation)
theta = pi; 
Ry = [cos(theta) 0 sin(theta); 
    0 1 0; 
    -sin(theta) 0 cos(theta)];
[axis, angle] = RotMat2AxisAngle(Ry)

% 2nd Case (z rotation)
psi = pi;
Rz = [cos(psi) -sin(psi) 0
sin(psi) cos(psi) 0 
0 0 1];
[axis, angle] = RotMat2AxisAngle(Rz)

% 3rd Case (arbitrary none pi rotation)
phi = .132;
theta = -.643; 
psi = -1.432;

Rx = [1 0 0; 
    0 cos(phi) -sin(phi)
    0 sin(phi) cos(phi)]; 

Ry = [cos(theta) 0 sin(theta); 
    0 1 0; 
    -sin(theta) 0 cos(theta)];

Rz = [cos(psi) -sin(psi) 0
sin(psi) cos(psi) 0 
0 0 1];

[axis, angle] = RotMat2AxisAngle(Rx*Rz*Ry)

%% PA 1b

% Test Error Condition 1 (matrix size check) 
RotMat = zeros(4,3); 
[Quat] = RotMat2Quat(RotMat);

% Test Error Condition 2 (orthogonal)
RotMat = zeros(3,3);
[Quat] = RotMat2Quat(RotMat);

% Test Error Condition 2 (preserves orientation)
RotMat = -eye(3);
[Quat] = RotMat2Quat(RotMat);

% Test Computation
theta = rand(1); % angle
w = rand(3,1); % axis

w = w/norm(w);
w1 = w(1); 
w2 = w(2); 
w3 = w(3);

vt = 1-cos(theta); 
st = sin(theta); 
ct = cos(theta); 

ewht = [w1^2*vt+ct w1*w2*vt-w3*st w1*w3*vt+w2*st 
    w1*w2*vt+w3*st w2^2*vt+ct w2*w3*vt-w1*st
    w1*w3*vt-w2*st w2*w3*vt+w1*st w3^2*vt+ct]; % Rotation matrix angle/axis

Q = [cos(theta/2); w*sin(theta/2)]; % Quaterion from axis angle

[Quat] = RotMat2Quat(ewht);
isequal(round(Q,8),round(Quat,8))

%% PA 1c

% Test Code
% Test Error Condition 1 (matrix size check) 
RotMat = zeros(4,3); 
[ZYZEuler, RollPitchYawEuler]  = RotMat2_ZYZ_RPY_EulerAngles(RotMat)

% Test Error Condition 2 (orthogonal)
RotMat = zeros(3,3);
[ZYZEuler, RollPitchYawEuler]  = RotMat2_ZYZ_RPY_EulerAngles(RotMat)

% Test Error Condition 2 (preserves orientation)
RotMat = -eye(3);
[ZYZEuler, RollPitchYawEuler]  = RotMat2_ZYZ_RPY_EulerAngles(RotMat)

% Test Non Singularity
ang1 = -.21442; 
ang2 = .38713; 
ang3 = 1.4217; 

RotMat = Rz(ang1)*Ry(ang2)*Rz(ang3);
[ZYZEuler,~]  = RotMat2_ZYZ_RPY_EulerAngles(RotMat);

isequal(round(ZYZEuler,10),round([ang1 ang2 ang3],10))

RotMat = Rz(ang3)*Ry(ang2)*Rx(ang1);
[~,RollPitchYawEuler]  = RotMat2_ZYZ_RPY_EulerAngles(RotMat);

isequal(round(RollPitchYawEuler,10),round([ang1 ang2 ang3],10))

% Test Singularity (ZYZ)
RotMat = eye(3);
[ZYZEuler,~]  = RotMat2_ZYZ_RPY_EulerAngles(RotMat);

% Test Singularity (ZYX)
ang1 = -1.71384; 
ang2 = pi/2; 
ang3 = -0.4217; 

RotMat = Rz(ang3)*Ry(ang2)*Rx(ang1);
[~,RollPitchYawEuler]  = RotMat2_ZYZ_RPY_EulerAngles(RotMat);

isequal(round(RollPitchYawEuler,10),round([ang1 ang2 ang3],10))

%% PA 2a

% Test Code

% Fails axis size check
axis = zeros(3,2);
angle = 0;
[RotMat] = AxisAngle2RotMat(axis,angle);

% Passes axis size check variation 1
axis = zeros(1,3);
angle = 0;
[RotMat] = AxisAngle2RotMat(axis,angle)

% Passes axis size check variation 2
axis = zeros(3,1);
angle = 0;
[RotMat] = AxisAngle2RotMat(axis,angle)

% Fails angle size check
axis = zeros(3,1);
angle = [2 3];
[RotMat] = AxisAngle2RotMat(axis,angle);

% Test Computation
phi = .132;
theta = -.643; 
psi = -1.432;

Rx = [1 0 0; 
    0 cos(phi) -sin(phi)
    0 sin(phi) cos(phi)]; 

Ry = [cos(theta) 0 sin(theta); 
    0 1 0; 
    -sin(theta) 0 cos(theta)];

Rz = [cos(psi) -sin(psi) 0
sin(psi) cos(psi) 0 
0 0 1];
RotMatIn = Rx*Rz*Ry
[axis, angle] = RotMat2AxisAngle(RotMatIn); 
[RotMat] = AxisAngle2RotMat(axis,angle)
isequal(round(RotMatIn,6), round(RotMat,6))
%% PA 2b

% Test Code
% Fails quat size check
Q = zeros(3,2);
[RotMat] = Quat2RotMat(Q);

% Passes quat size check variation 1
Q = ones(1,4);
[RotMat] = Quat2RotMat(Q)

% Passes quat size check variation 2
Q = ones(4,1);
[RotMat] = Quat2RotMat(Q)

% Test Computation
theta = rand(1); % angle

w = rand(3,1); % axis
w = w/norm(w);
w1 = w(1); 
w2 = w(2); 
w3 = w(3);

vt = 1-cos(theta); 
st = sin(theta); 
ct = cos(theta); 

ewht = [w1^2*vt+ct w1*w2*vt-w3*st w1*w3*vt+w2*st 
    w1*w2*vt+w3*st w2^2*vt+ct w2*w3*vt-w1*st
    w1*w3*vt-w2*st w2*w3*vt+w1*st w3^2*vt+ct]; % Rotation matrix angle/axis

Q = [cos(theta/2); w*sin(theta/2)]; % Quaterion from axis angle

[RotMat] = Quat2RotMat(Q);
isequal(round(ewht,8),round(RotMat,8))
