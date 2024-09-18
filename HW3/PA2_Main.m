addpath('./HW3-PA2/')
addpath('./CommonFunctions/')
%%
[q_Robot_config, q_camera_config,t_Robot_config,t_camera_config ]=data_quaternion();
[q_Robot_configN, q_camera_configN,t_Robot_configN,t_camera_configN ] = data_quaternion_noisy();
% A = inv(E1)*E2 relative motion of the robot
% B = S1*inv(S2) relative motion of the sensor

numPts = size(q_camera_config,1); 
qA = q_Robot_config; 
qB = q_camera_config; 
pA = t_Robot_config;
pB = t_camera_config; 

% Individual Points Kron Product (Check)
for i = 1:numPts
    [X,errorNorm] = HandEyeCalRotKronProd(qA(i,:),qB(i,:)); 
    XStore(i,1) = {X}; 
    errorStore(i,1) = errorNorm; 
end

% Half Set Kron Prod
[XHalfKron,rotErrorHalfKron] = HandEyeCalRotKronProd(qA(1:5,:),qB(1:5,:)); 
[pxHalfKron, transErrorHalfKron] = LeastSquaresTranslation(qA(1:5,:),pA,pB,XHalfKron);

% Full point set Kron Product
[XFullKron,rotErrorFullKron] = HandEyeCalRotKronProd(qA,qB); 
[pxFullKron, transErrorFullKron] = LeastSquaresTranslation(qA,pA,pB,XFullKron);

% Half Set quat
[XHalfQuat,rotErrorHalfQuat] = HandEyeCalRotQuat(qA(1:5,:),qB(1:5,:)); 
[pxHalfQuat, transErrorHalfQuat] = LeastSquaresTranslation(qA(1:5,:),pA,pB,XHalfQuat);

% Full point set quat
[XFullQuat,rotErrorFullQuat] = HandEyeCalRotQuat(qA,qB);
[pxFullQuat, transErrorFullQuat] = LeastSquaresTranslation(qA,pA,pB,XFullQuat);

% Repeat with noisy data 
numPts = size(q_camera_configN,1); 
qA = q_Robot_configN; 
qB = q_camera_configN; 
pA = t_Robot_configN;
pB = t_camera_configN; 

% Half Set Kron Prod
[XHalfKronN,rotErrorHalfKronN] = HandEyeCalRotKronProd(qA(1:5,:),qB(1:5,:)); 
[pxHalfKronN, transErrorHalfKronN] = LeastSquaresTranslation(qA(1:5,:),pA,pB,XHalfKronN);

% Full point set Kron Product
[XFullKronN,rotErrorFullKronN] = HandEyeCalRotKronProd(qA,qB); 
[pxFullKronN, transErrorFullKronN] = LeastSquaresTranslation(qA,pA,pB,XFullKronN);

% Half Set quat
[XHalfQuatN,rotErrorHalfQuatN] = HandEyeCalRotQuat(qA(1:5,:),qB(1:5,:)); 
[pxHalfQuatN, transErrorHalfQuatN] = LeastSquaresTranslation(qA(1:5,:),pA,pB,XHalfQuatN);

% Full point set quat
[XFullQuatN,rotErrorFullQuatN] = HandEyeCalRotQuat(qA,qB);
[pxFullQuatN, transErrorFullQuatN] = LeastSquaresTranslation(qA,pA,pB,XFullQuatN);

figure(1); clf
subplot(2,1,1)
plot(rotErrorFullKron,'-*')
hold on 
plot(rotErrorFullQuat,'-o')
plot(rotErrorFullKronN,'-*')
plot(rotErrorFullQuatN,'-o')
ylabel('Norm(AX-XB) Full Set')
legend('Kron', 'Quat', 'Noisy Kron', 'Noisy Quat')
title('Rot Error')
grid on; grid minor

subplot(2,1,2)
plot(rotErrorHalfKron,'-*')
hold on 
plot(rotErrorHalfQuat,'-o')
plot(rotErrorHalfKronN,'-*')
plot(rotErrorHalfQuatN,'-o')
ylabel('Norm(AX-XB) Half Set')
xlabel('Data Point')
grid on; grid minor


figure(2); clf
subplot(2,1,1)
plot(transErrorFullKron,'-*')
hold on 
plot(transErrorFullQuat,'-o')
plot(transErrorFullKronN,'-*')
plot(transErrorFullQuatN,'-o')
ylabel('norm((R_a-I)*p_x - R_x*p_B+p_A) Full Set')
legend('Kron', 'Quat', 'Noisy Kron', 'Noisy Quat')
title('Trans Error')
grid on; grid minor

subplot(2,1,2)
plot(transErrorHalfKron,'-*')
hold on 
plot(transErrorHalfQuat,'-o')
plot(transErrorHalfKronN,'-*')
plot(transErrorHalfQuatN,'-o')
ylabel('norm((R_a-I)*p_x - R_x*p_B+p_A) Half Set')
xlabel('Data Point')
grid on; grid minor