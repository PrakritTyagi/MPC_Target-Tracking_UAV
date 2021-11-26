    % 4%
%-------------------------------------------------%
% Written by - Prakrit Tyagi                      %
% Dated - 23-01-2019                              %
% Project Under PB.Sujit(Coral Lab-IIIT-Delhi)    %
%-------------------------------------------------%
import casadi.*
% phi theta psi roll yaw pitch
clear 
% MPC Parameters
N = 15;
T = 0.2;
% Camera Parameters //to be changed in Graph_MPC_casadi//
VFOV_deg = 44.4; % vertical field of view in degree 94.4
HFOV_deg = 62.6; % horizontal field of view in degree 122.6

VFOV_rad = VFOV_deg*pi/180;
HFOV_rad = HFOV_deg*pi/180;
% Gimbal servo paramters
pitch_maxG = pi/30; roll_maxG = pi/30; yaw_maxG = pi/30;
% UAV Parameters (controls limits)
v_max = 30; v_min = 14;
yaw_max = pi/21; yaw_min = -yaw_max;
pitch_max = pi/30; pitch_min = -pitch_max;
%obstacle dimensions
n_obs = 14;
r = 30;
% obs(1) = struct('x',180,'y',200,'h',95);%
% obs(2) = struct('x',170,'y',960,'h',120);%
% obs(3) = struct('x',180,'y',750,'h',100);
% obs(4) = struct('x',160,'y',900,'h',90);%
% obs(5) = struct('x',300,'y',970,'h',80);
% obs(6) = struct('x',400,'y',800,'h',95);%
% obs(7) = struct('x',750,'y',800,'h',75);%
% obs(8) = struct('x',950,'y',900,'h',120);
% obs(9) = struct('x',850,'y',920,'h',80);
% obs(10) = struct('x',900,'y',850,'h',90);%
% obs(11) = struct('x',950,'y',200,'h',75);%
% obs(12) = struct('x',750,'y',50,'h',110);
% obs(13) = struct('x',500,'y',200,'h',140);
% obs(14) = struct('x',180,'y',50,'h',110);
obs(1) = struct('x',180,'y',200,'h',80);%
obs(2) = struct('x',40,'y',350,'h',100);%
obs(3) = struct('x',180,'y',550,'h',100);
obs(4) = struct('x',40,'y',750,'h',75);%
obs(5) = struct('x',300,'y',970,'h',65);
obs(6) = struct('x',500,'y',800,'h',85);%
obs(7) = struct('x',750,'y',800,'h',75);%
obs(8) = struct('x',950,'y',900,'h',100);
obs(9) = struct('x',950,'y',600,'h',100);
obs(10) = struct('x',750,'y',350,'h',80);%
obs(11) = struct('x',950,'y',200,'h',75);%
obs(12) = struct('x',750,'y',50,'h',100);
obs(13) = struct('x',500,'y',200,'h',100);
obs(14) = struct('x',180,'y',50,'h',100);

%% Defining model for UAV
x = SX.sym('x'); y = SX.sym('y'); z = SX.sym('z'); theta = SX.sym('theta'); psi = SX.sym('psi');
states = [x;y;z;theta;psi]; n_states = length(states);

v = SX.sym('v'); yaw = SX.sym('yaw'); pitch = SX.sym('pitch');
controls = [v;yaw;pitch]; n_controls = length(controls);

model = [v*cos(theta)*cos(psi);v*sin(theta)*cos(psi);v*sin(psi);yaw;pitch];
f = Function('f',{states,controls},{model});

P = SX.sym('P',n_states);
X = SX.sym('X',n_states,(N+1));
U = SX.sym('U',n_controls,N);

% calculate next state 
X(:,1) = P;
for k = 1:N
    c_X = X(:,k); c_U = U(:,k);
    f_value = f(c_X,c_U);
    next_X = c_X + T*f_value;
    X(:,k+1) = next_X;
end
ff = Function('ff',{U,P},{X});

%% Defining model for gimbal
psiG = SX.sym('psiG'); phiG = SX.sym('phiG'); thetaG = SX.sym('thetaG');
statesG = [psiG;phiG;thetaG]; n_statesG = length(statesG);

pitchG = SX.sym('pitchG'); rollG = SX.sym('rollG'); yawG = SX.sym('yawG');
controlsG = [pitchG;rollG;yawG]; n_controlsG = length(controlsG);

modelG = [pitchG;rollG;yawG];
fG = Function('fG',{statesG,controlsG},{modelG});

PG = SX.sym('PG',n_statesG);
XG = SX.sym('XG',n_statesG,(N+1));
UG = SX.sym('UG',n_controlsG,N); 
% calculate next state
XG(:,1) = PG;
for k = 1:N
    c_XG = XG(:,k); c_UG = UG(:,k);
    fG_value = fG(c_XG,c_UG);
    next_XG = c_XG + T*fG_value;
    XG(:,k+1) = next_XG;
end
ffG = Function('ffG',{UG,PG},{XG});


%% Defining model for Target
xT = SX.sym('xT'); yT = SX.sym('yT'); ang = SX.sym('ang');
statesT = [xT;yT;ang]; n_statesT = length(statesT);

vT = SX.sym('vT'); angRate = SX.sym('angRate');
controlsT = [vT;angRate]; n_controlsT = length(controlsT);

modelT = [vT*cos(ang);vT*sin(ang);angRate];
fT = Function('fT',{statesT,controlsT},{modelT});

PT = SX.sym('PT',n_statesT);
XT = SX.sym('XT',n_statesT,2);
UT = SX.sym('UT',n_controlsT);

% Calcuate next state
XT(:,1) = PT;
c_XT = XT(:,1); c_UT = UT(:,1);

fT_value = fT(c_XT,c_UT);
new_XT = c_XT + T*fT_value;
XT(:,2) = new_XT;

ffT = Function('ffT',{UT,PT},{XT});


%% Defining the objective cost function %%
obj = 0;
w_1 = [0.1,0.1]; % x y
% w_2 = 1.2; % z
w_3 = 1; % ellipse
w_4 = 5; % FOV
w_5 = 100; % avoid obstacle increase height
Q = diag(w_1);

for k = 1:N
   % relative distance cost function
   obj = obj + sqrt((X(1:2,k)-PT(1:2))'*Q*(X(1:2,k)-PT(1:2)));
   
%    % altitude maintain cost function
%    z_st = X(3,k);
%    obj = obj + (z_st - 100)*w_2*(z_st - 100); 
   
   % Field of View cost function
%    a = X(3,k)*tan(VFOV_rad/2);
%    b = X(3,k)*tan(HFOV_rad/2);
%    a =  0.5*( z_uav*tan(Vgimbalrad + VFOV_rad/2 ) - z_uav*tan( Vgimbalrad - VFOV_rad/2 ));  
%    b =  0.5*( z_uav*tan(Hgimbalrad + HFOV_rad/2 ) - z_uav*tan( Hgimbalrad - HFOV_rad/2 ));
   a = 0.5*( X(3,k)*tan(XG(1,k) + X(5,k) + VFOV_rad/2) - X(3,k)*tan(XG(1,k) + X(5,k) - VFOV_rad/2) );
   b = 0.5*( X(3,k)*tan(XG(2,k) + HFOV_rad/2) - X(3,k)*tan(XG(2,k) - HFOV_rad/2) );
   shifta = a + X(3,k)*tan(XG(1,k) + X(5,k) - VFOV_rad/2);
   shiftb = b + X(3,k)*tan(XG(2,k) - HFOV_rad/2);
   A = ((cos(X(4,k) + XG(3,k)))^2)/(a^2) + ((sin(X(4,k) + XG(3,k)))^2)/(b^2) ;
   B = 2*cos(X(4,k) + XG(3,k))*sin(X(4,k) + XG(3,k))*( (1/a^2) - (1/b^2) );
   C = (sin(X(4,k) + XG(3,k)))^2/(a^2) + (cos(X(4,k) + XG(3,k)))^2/(b^2);
   ellipse = A*(PT(1) - X(1,k) - shifta)^2 + B*(PT(1) - X(1,k) - shifta)*(PT(2) - X(2,k) - shiftb) + C*(PT(2) - X(2,k) - shiftb)^2 - 1; 
   obj = obj + w_3*ellipse;
   
   logic_TargetInVision = floor(ellipse/10000000)*-1; % 1 if inside ellipse else 0 for outside
   
   % Visibility cost function obstacle 1
   slope = (X(2,k)-PT(2))/(X(1,k)-PT(1)); % slope of ray(x-yplane) from target to UAV 
   center1_to_LOS = (abs(slope*obs(1).x - 1*obs(1).y + PT(2) - PT(1)*slope)/sqrt(slope^2 + (-1)^2))-r; % distance between line LOS 2D and center of obstacle1
   slopeRef1 = (obs(1).h/(sqrt( (PT(1)-obs(1).x)^2 + (PT(2)-obs(1).y)^2 )-r)) - (X(3,k)/(sqrt( (PT(1)-X(1,k))^2 + (PT(2)-X(2,k))^2 ))); % Slope of LOS between obstacle_1 Top and target in 3D
   logic_LosIntersectObstacle1 = floor(center1_to_LOS/100000000)*-1;
   costlos1 = logic_TargetInVision*logic_LosIntersectObstacle1*max(0,slopeRef1);
   obj = obj + w_4*costlos1;
   
   % Visibility cost function obstacle 2
%    slope = (X(2,k)-PT(2))/(X(1,k)-PT(1)); % slope of ray(x-yplane) from target to UAV 
   center2_to_LOS = (abs(slope*obs(2).x - 1*obs(2).y + PT(2) - PT(1)*slope)/sqrt(slope^2 + (-1)^2))-r; % distance between line LOS 2D and center of obstacle2
   slopeRef2 = (obs(2).h/(sqrt( (PT(1)-obs(2).x)^2 + (PT(2)-obs(2).y)^2 )-r)) - (X(3,k)/(sqrt( (PT(1)-X(1,k))^2 + (PT(2)-X(2,k))^2 ))); % Slope of LOS between obstacle_2 Top and target in 3D
   logic_LosIntersectObstacle2 = floor(center2_to_LOS/100000000)*-1;
   costlos2 = logic_TargetInVision*logic_LosIntersectObstacle2*max(0,slopeRef2);
   obj = obj + w_4*costlos2;
   
   % Visibility cost function obstacle 3
%    slope = (X(2,k)-PT(2))/(X(1,k)-PT(1)); % slope of ray(x-yplane) from target to UAV 
   center3_to_LOS = (abs(slope*obs(3).x - 1*obs(3).y + PT(2) - PT(1)*slope)/sqrt(slope^2 + (-1)^2))-r; % distance between line LOS 2D and center of obstacle3
   slopeRef3 = (obs(3).h/(sqrt( (PT(1)-obs(3).x)^2 + (PT(2)-obs(3).y)^2 )-r)) - (X(3,k)/(sqrt( (PT(1)-X(1,k))^2 + (PT(2)-X(2,k))^2 ))); % Slope of LOS between obstacle_3 Top and target in 3D
   logic_LosIntersectObstacle3 = floor(center3_to_LOS/100000000)*-1;
   costlos3 = logic_TargetInVision*logic_LosIntersectObstacle3*max(0,slopeRef3);
   obj = obj + w_4*costlos3;
   
   % Visibility cost function obstacle 4
%    slope = (X(2,k)-PT(2))/(X(1,k)-PT(1)); % slope of ray(x-yplane) from target to UAV 
   center4_to_LOS = (abs(slope*obs(4).x - 1*obs(4).y + PT(2) - PT(1)*slope)/sqrt(slope^2 + (-1)^2))-r; % distance between line LOS 2D and center of obstacle4
   slopeRef4 = (obs(4).h/(sqrt( (PT(1)-obs(4).x)^2 + (PT(2)-obs(4).y)^2 )-r)) - (X(3,k)/(sqrt( (PT(1)-X(1,k))^2 + (PT(2)-X(2,k))^2 ))); % Slope of LOS between obstacle_4 Top and target in 3D
   logic_LosIntersectObstacle4 = floor(center4_to_LOS/100000000)*-1;
   costlos4 = logic_TargetInVision*logic_LosIntersectObstacle4*max(0,slopeRef4);
   obj = obj + w_4*costlos4;
   
   % Visibility cost function obstacle 5
%    slope = (X(2,k)-PT(2))/(X(1,k)-PT(1)); % slope of ray(x-yplane) from target to UAV 
   center5_to_LOS = (abs(slope*obs(5).x - 1*obs(5).y + PT(2) - PT(1)*slope)/sqrt(slope^2 + (-1)^2))-r; % distance between line LOS 2D and center of obstacle5
   slopeRef5 = (obs(5).h/(sqrt( (PT(1)-obs(5).x)^2 + (PT(2)-obs(5).y)^2 )-r)) - (X(3,k)/(sqrt( (PT(1)-X(1,k))^2 + (PT(2)-X(2,k))^2 ))); % Slope of LOS between obstacle_5 Top and target in 3D
   logic_LosIntersectObstacle5 = floor(center5_to_LOS/100000000)*-1;
   costlos5 = logic_TargetInVision*logic_LosIntersectObstacle5*max(0,slopeRef5);
   obj = obj + w_4*costlos5;
   
   % Visibility cost function obstacle 6
%    slope = (X(2,k)-PT(2))/(X(1,k)-PT(1)); % slope of ray(x-yplane) from target to UAV 
   center6_to_LOS = (abs(slope*obs(6).x - 1*obs(6).y + PT(2) - PT(1)*slope)/sqrt(slope^2 + (-1)^2))-r; % distance between line LOS 2D and center of obstacle6
   slopeRef6 = (obs(6).h/(sqrt( (PT(1)-obs(6).x)^2 + (PT(2)-obs(6).y)^2 )-r)) - (X(3,k)/(sqrt( (PT(1)-X(1,k))^2 + (PT(2)-X(2,k))^2 ))); % Slope of LOS between obstacle_6 Top and target in 3D
   logic_LosIntersectObstacle6 = floor(center6_to_LOS/100000000)*-1;
   costlos6 = logic_TargetInVision*logic_LosIntersectObstacle6*max(0,slopeRef6);
   obj = obj + w_4*costlos6;
   
   % Visibility cost function obstacle 7
%    slope = (X(2,k)-PT(2))/(X(1,k)-PT(1)); % slope of ray(x-yplane) from target to UAV 
   center7_to_LOS = (abs(slope*obs(7).x - 1*obs(7).y + PT(2) - PT(1)*slope)/sqrt(slope^2 + (-1)^2))-r; % distance between line LOS 2D and center of obstacle7
   slopeRef7 = (obs(7).h/(sqrt( (PT(1)-obs(7).x)^2 + (PT(2)-obs(7).y)^2 )-r)) - (X(3,k)/(sqrt( (PT(1)-X(1,k))^2 + (PT(2)-X(2,k))^2 ))); % Slope of LOS between obstacle_7 Top and target in 3D
   logic_LosIntersectObstacle7 = floor(center7_to_LOS/100000000)*-1;
   costlos7 = logic_TargetInVision*logic_LosIntersectObstacle7*max(0,slopeRef7);
   obj = obj + w_4*costlos7;
   
   % Visibility cost function obstacle 8
%    slope = (X(2,k)-PT(2))/(X(1,k)-PT(1)); % slope of ray(x-yplane) from target to UAV 
   center8_to_LOS = (abs(slope*obs(8).x - 1*obs(8).y + PT(2) - PT(1)*slope)/sqrt(slope^2 + (-1)^2))-r; % distance between line LOS 2D and center of obstacle8
   slopeRef8 = (obs(8).h/(sqrt( (PT(1)-obs(8).x)^2 + (PT(2)-obs(8).y)^2 )-r)) - (X(3,k)/(sqrt( (PT(1)-X(1,k))^2 + (PT(2)-X(2,k))^2 ))); % Slope of LOS between obstacle_8 Top and target in 3D
   logic_LosIntersectObstacle8 = floor(center8_to_LOS/100000000)*-1;
   costlos8 = logic_TargetInVision*logic_LosIntersectObstacle8*max(0,slopeRef8);
   obj = obj + w_4*costlos8;
   
   % Visibility cost function obstacle 9
%    slope = (X(2,k)-PT(2))/(X(1,k)-PT(1)); % slope of ray(x-yplane) from target to UAV 
   center9_to_LOS = (abs(slope*obs(9).x - 1*obs(9).y + PT(2) - PT(1)*slope)/sqrt(slope^2 + (-1)^2))-r; % distance between line LOS 2D and center of obstacle9
   slopeRef9 = (obs(9).h/(sqrt( (PT(1)-obs(9).x)^2 + (PT(2)-obs(9).y)^2 )-r)) - (X(3,k)/(sqrt( (PT(1)-X(1,k))^2 + (PT(2)-X(2,k))^2 ))); % Slope of LOS between obstacle_9 Top and target in 3D
   logic_LosIntersectObstacle9 = floor(center9_to_LOS/100000000)*-1;
   costlos9 = logic_TargetInVision*logic_LosIntersectObstacle9*max(0,slopeRef9);
   obj = obj + w_4*costlos9;
   
   % Visibility cost function obstacle 10
%    slope = (X(2,k)-PT(2))/(X(1,k)-PT(1)); % slope of ray(x-yplane) from target to UAV 
   center10_to_LOS = (abs(slope*obs(10).x - 1*obs(10).y + PT(2) - PT(1)*slope)/sqrt(slope^2 + (-1)^2))-r; % distance between line LOS 2D and center of obstacle10
   slopeRef10 = (obs(10).h/(sqrt( (PT(1)-obs(10).x)^2 + (PT(2)-obs(10).y)^2 )-r)) - (X(3,k)/(sqrt( (PT(1)-X(1,k))^2 + (PT(2)-X(2,k))^2 ))); % Slope of LOS between obstacle_10 Top and target in 3D
   logic_LosIntersectObstacle10 = floor(center10_to_LOS/100000000)*-1;
   costlos10 = logic_TargetInVision*logic_LosIntersectObstacle10*max(0,slopeRef10);
   obj = obj + w_4*costlos10;

   % Visibility cost function obstacle 11
%    slope = (X(2,k)-PT(2))/(X(1,k)-PT(1)); % slope of ray(x-yplane) from target to UAV 
   center11_to_LOS = (abs(slope*obs(11).x - 1*obs(11).y + PT(2) - PT(1)*slope)/sqrt(slope^2 + (-1)^2))-r; % distance between line LOS 2D and center of obstacle11
   slopeRef11 = (obs(11).h/(sqrt( (PT(1)-obs(11).x)^2 + (PT(2)-obs(11).y)^2 )-r)) - (X(3,k)/(sqrt( (PT(1)-X(1,k))^2 + (PT(2)-X(2,k))^2 ))); % Slope of LOS between obstacle_11 Top and target in 3D
   logic_LosIntersectObstacle11 = floor(center11_to_LOS/100000000)*-1;
   costlos11 = logic_TargetInVision*logic_LosIntersectObstacle11*max(0,slopeRef11);
   obj = obj + w_4*costlos11;
   
   % Visibility cost function obstacle 12
%    slope = (X(2,k)-PT(2))/(X(1,k)-PT(1)); % slope of ray(x-yplane) from target to UAV 
   center12_to_LOS = (abs(slope*obs(12).x - 1*obs(12).y + PT(2) - PT(1)*slope)/sqrt(slope^2 + (-1)^2))-r; % distance between line LOS 2D and center of obstacle12
   slopeRef12 = (obs(12).h/(sqrt( (PT(1)-obs(12).x)^2 + (PT(2)-obs(12).y)^2 )-r)) - (X(3,k)/(sqrt( (PT(1)-X(1,k))^2 + (PT(2)-X(2,k))^2 ))); % Slope of LOS between obstacle_12 Top and target in 3D
   logic_LosIntersectObstacle12 = floor(center12_to_LOS/100000000)*-1;
   costlos12 = logic_TargetInVision*logic_LosIntersectObstacle12*max(0,slopeRef12);
   obj = obj + w_4*costlos12;
   
   % Visibility cost function obstacle 13
%    slope = (X(2,k)-PT(2))/(X(1,k)-PT(1)); % slope of ray(x-yplane) from target to UAV 
   center13_to_LOS = (abs(slope*obs(13).x - 1*obs(13).y + PT(2) - PT(1)*slope)/sqrt(slope^2 + (-1)^2))-r; % distance between line LOS 2D and center of obstacle13
   slopeRef13 = (obs(13).h/(sqrt( (PT(1)-obs(13).x)^2 + (PT(2)-obs(13).y)^2 )-r)) - (X(3,k)/(sqrt( (PT(1)-X(1,k))^2 + (PT(2)-X(2,k))^2 ))); % Slope of LOS between obstacle_13 Top and target in 3D
   logic_LosIntersectObstacle13 = floor(center13_to_LOS/100000000)*-1;
   costlos13 = logic_TargetInVision*logic_LosIntersectObstacle13*max(0,slopeRef13);
   obj = obj + w_4*costlos13;
   
   % Visibility cost function obstacle 14
%    slope = (X(2,k)-PT(2))/(X(1,k)-PT(1)); % slope of ray(x-yplane) from target to UAV 
   center14_to_LOS = (abs(slope*obs(14).x - 1*obs(14).y + PT(2) - PT(1)*slope)/sqrt(slope^2 + (-1)^2))-r; % distance between line LOS 2D and center of obstacle14
   slopeRef14 = (obs(14).h/(sqrt( (PT(1)-obs(14).x)^2 + (PT(2)-obs(14).y)^2 )-r)) - (X(3,k)/(sqrt( (PT(1)-X(1,k))^2 + (PT(2)-X(2,k))^2 ))); % Slope of LOS between obstacle_14 Top and target in 3D
   logic_LosIntersectObstacle14 = floor(center14_to_LOS/100000000)*-1;
   costlos14 = logic_TargetInVision*logic_LosIntersectObstacle14*max(0,slopeRef14);
   obj = obj + w_4*costlos14;
   
   %obstacle collision avoidance
   obs1 = ((X(1,k)-obs(1).x)^2 + (X(2,k)-obs(1).y)^2 - r^2);
   checkobs1 = -1*floor(obs1/100000000);
   costobs1 = checkobs1*(obs(1).h-X(3,k))*(obs(1).h-X(3,k))^2;
   obj = obj + w_5*costobs1;
   
   obs2 = ((X(1,k)-obs(2).x)^2 + (X(2,k)-obs(2).y)^2 - r^2);
   checkobs2 = -1*floor(obs2/100000000);
   costobs2 = checkobs2*(obs(2).h-X(3,k))*(obs(2).h-X(3,k))^2;
   obj = obj + w_5*costobs2;
   
   obs3 = ((X(1,k)-obs(3).x)^2 + (X(2,k)-obs(3).y)^2 - r^2);
   checkobs3 = -1*floor(obs3/100000000);
   costobs3 = checkobs3*(obs(3).h-X(3,k))*(obs(3).h-X(3,k))^2;
   obj = obj + w_5*costobs3;
   
   obs4 = ((X(1,k)-obs(4).x)^2 + (X(2,k)-obs(4).y)^2 - r^2);
   checkobs4 = -1*floor(obs4/100000000);
   costobs4 = checkobs4*(obs(4).h-X(3,k))*(obs(4).h-X(3,k))^2;
   obj = obj + w_5*costobs4;
   
   obs5 = ((X(1,k)-obs(5).x)^2 + (X(2,k)-obs(5).y)^2 - r^2);
   checkobs5 = -1*floor(obs5/100000000);
   costobs5 = checkobs5*(obs(5).h-X(3,k))*(obs(5).h-X(3,k))^2;
   obj = obj + w_5*costobs5;

   obs6 = ((X(1,k)-obs(6).x)^2 + (X(2,k)-obs(6).y)^2 - r^2);
   checkobs6 = -1*floor(obs6/100000000);
   costobs6 = checkobs6*(obs(6).h-X(3,k))*(obs(6).h-X(3,k))^2;
   obj = obj + w_5*costobs6;
   
   obs7 = ((X(1,k)-obs(7).x)^2 + (X(2,k)-obs(7).y)^2 - r^2);
   checkobs7 = -1*floor(obs7/100000000);
   costobs7 = checkobs7*(obs(7).h-X(3,k))*(obs(7).h-X(3,k))^2;
   obj = obj + w_5*costobs7;

   obs8 = ((X(1,k)-obs(8).x)^2 + (X(2,k)-obs(8).y)^2 - r^2);
   checkobs8 = -1*floor(obs8/100000000);
   costobs8 = checkobs8*(obs(8).h-X(3,k))*(obs(8).h-X(3,k))^2;
   obj = obj + w_5*costobs8;
   
   obs9 = ((X(1,k)-obs(9).x)^2 + (X(2,k)-obs(9).y)^2 - r^2);
   checkobs9 = -1*floor(obs9/100000000);
   costobs9 = checkobs9*(obs(9).h-X(3,k))*(obs(9).h-X(3,k))^2;
   obj = obj + w_5*costobs9;
   
   obs10 = ((X(1,k)-obs(10).x)^2 + (X(2,k)-obs(10).y)^2 - r^2);
   checkobs10 = -1*floor(obs10/100000000);
   costobs10 = checkobs10*(obs(10).h-X(3,k))*(obs(10).h-X(3,k))^2;
   obj = obj + w_5*costobs10;
   
   obs11 = ((X(1,k)-obs(11).x)^2 + (X(2,k)-obs(11).y)^2 - r^2);
   checkobs11 = -1*floor(obs11/100000000);
   costobs11 = checkobs11*(obs(11).h-X(3,k))*(obs(11).h-X(3,k))^2;
   obj = obj + w_5*costobs11;
   
   obs12 = ((X(1,k)-obs(12).x)^2 + (X(2,k)-obs(12).y)^2 - r^2);
   checkobs12 = -1*floor(obs12/100000000);
   costobs12 = checkobs12*(obs(12).h-X(3,k))*(obs(12).h-X(3,k))^2;
   obj = obj + w_5*costobs12;
   
   obs13 = ((X(1,k)-obs(13).x)^2 + (X(2,k)-obs(13).y)^2 - r^2);
   checkobs13 = -1*floor(obs13/100000000);
   costobs13 = checkobs13*(obs(13).h-X(3,k))*(obs(13).h-X(3,k))^2;
   obj = obj + w_5*costobs13;
   
   obs14 = ((X(1,k)-obs(14).x)^2 + (X(2,k)-obs(14).y)^2 - r^2);
   checkobs14 = -1*floor(obs14/100000000);
   costobs14 = checkobs14*(obs(14).h-X(3,k))*(obs(14).h-X(3,k))^2;
   obj = obj + w_5*costobs14;
end

% constraints

g = [];%constraint vector

for i = 1:N+1
    g = [g;X(1,k)]; %x 
    g = [g;X(2,k)]; %y 
    g = [g;X(3,k)]; %z 
    g = [g;X(4,k)]; % theta
    g = [g;X(5,k)]; % psi
    g = [g;XG(1,k)]; % gimbal pitch
    g = [g;XG(2,k)]; % gimbal roll
    g = [g;XG(3,k)]; % gimbal yaw
    % obstacle avoidance constraint
    g = [g;(-sqrt((X(1,k)-obs(1).x)^2 + (X(2,k)-obs(1).y)^2) + r)]; % obstacle 1
    g = [g;(-sqrt((X(1,k)-obs(2).x)^2 + (X(2,k)-obs(2).y)^2) + r)]; % obstacle 2
    g = [g;(-sqrt((X(1,k)-obs(3).x)^2 + (X(2,k)-obs(3).y)^2) + r)]; % obstacle 3
    g = [g;(-sqrt((X(1,k)-obs(4).x)^2 + (X(2,k)-obs(4).y)^2) + r)]; % obstacle 4
    g = [g;(-sqrt((X(1,k)-obs(5).x)^2 + (X(2,k)-obs(5).y)^2) + r)]; % obstacle 5
    g = [g;(-sqrt((X(1,k)-obs(6).x)^2 + (X(2,k)-obs(6).y)^2) + r)]; % obstacle 6
    g = [g;(-sqrt((X(1,k)-obs(7).x)^2 + (X(2,k)-obs(7).y)^2) + r)]; % obstacle 7
    g = [g;(-sqrt((X(1,k)-obs(8).x)^2 + (X(2,k)-obs(8).y)^2) + r)]; % obstacle 8
    g = [g;(-sqrt((X(1,k)-obs(9).x)^2 + (X(2,k)-obs(9).y)^2) + r)]; % obstacle 9
    g = [g;(-sqrt((X(1,k)-obs(10).x)^2 + (X(2,k)-obs(10).y)^2) + r)]; % obstacle 10
    g = [g;(-sqrt((X(1,k)-obs(11).x)^2 + (X(2,k)-obs(11).y)^2) + r)]; % obstacle 11
    g = [g;(-sqrt((X(1,k)-obs(12).x)^2 + (X(2,k)-obs(12).y)^2) + r)]; % obstacle 12
    g = [g;(-sqrt((X(1,k)-obs(13).x)^2 + (X(2,k)-obs(13).y)^2) + r)]; % obstacle 13
    g = [g;(-sqrt((X(1,k)-obs(14).x)^2 + (X(2,k)-obs(14).y)^2) + r)]; % obstacle 14
end

U0 = [U;UG];
OPT_variables = reshape(U0,(n_controls+n_controlsG)*N,1); % control variables
p = [P;PT;PG]; % initial parameters of uaav and target
nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', p);

opts = struct;
opts.ipopt.max_iter = 100;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob, opts);

args = struct;

args.ubg(1:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) =  inf;            args.lbg(1:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) = -inf; % constraint x
args.ubg(2:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) =  inf;            args.lbg(2:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) = -inf; % constraint y
args.ubg(3:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) =  150;            args.lbg(3:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) = 75;   % constraint z
args.ubg(4:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) =  inf;            args.lbg(4:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) = -inf;  % constraint theta (yaw)
args.ubg(5:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) =  0.1178;           args.lbg(5:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) = -0.1178; % constraint psi (pitch)
args.ubg(6:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) =  pi/6;           args.lbg(6:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) = -pi/6; % 
args.ubg(7:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) =  pi/6;           args.lbg(7:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) = -pi/6;
args.ubg(8:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) =  pi/2;           args.lbg(8:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) = -pi/2;
args.ubg(9:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) =  0;              args.lbg(9:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) = -inf;   %obstacle constraint 1
args.ubg(10:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) =  0;             args.lbg(10:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) = -inf;   %obstacle constraint 2
args.ubg(11:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) =  0;             args.lbg(11:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) = -inf;   %obstacle constraint 3
args.ubg(12:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) =  0;             args.lbg(12:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) = -inf;   %obstacle constraint 4
args.ubg(13:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) =  0;             args.lbg(13:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) = -inf;   %obstacle constraint 5
args.ubg(14:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) =  0;             args.lbg(14:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) = -inf;   %obstacle constraint 6
args.ubg(15:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) =  0;             args.lbg(15:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) = -inf;   %obstacle constraint 7
args.ubg(16:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) =  0;             args.lbg(16:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) = -inf;   %obstacle constraint 7
args.ubg(17:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) =  0;             args.lbg(17:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) = -inf;   %obstacle constraint 7
args.ubg(18:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) =  0;             args.lbg(18:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) = -inf;   %obstacle constraint 7
args.ubg(19:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) =  0;             args.lbg(19:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) = -inf;   %obstacle constraint 7
args.ubg(20:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) =  0;             args.lbg(20:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) = -inf;   %obstacle constraint 7
args.ubg(21:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) =  0;             args.lbg(21:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) = -inf;   %obstacle constraint 7
args.ubg(22:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) =  0;             args.lbg(22:(n_states+n_statesG+n_obs):(n_states+n_statesG+n_obs)*(N+1),1) = -inf;   %obstacle constraint 7

args.ubx(1:(n_controls+n_controlsG):(n_controls+n_controlsG)*N,1) = v_max;      args.lbx(1:(n_controls+n_controlsG):(n_controls+n_controlsG)*N,1) = v_min; % constraint velocity
args.ubx(2:(n_controls+n_controlsG):(n_controls+n_controlsG)*N,1) = yaw_max ; args.lbx(2:(n_controls+n_controlsG):(n_controls+n_controlsG)*N,1) = yaw_min; % constraint yaw (psi_dot)
args.ubx(3:(n_controls+n_controlsG):(n_controls+n_controlsG)*N,1) = pitch_max;    args.lbx(3:(n_controls+n_controlsG):(n_controls+n_controlsG)*N,1) = pitch_min; % constraint pitch (theta_dot)
args.ubx(4:(n_controls+n_controlsG):(n_controls+n_controlsG)*N,1) = pitch_maxG;  args.lbx(4:(n_controls+n_controlsG):(n_controls+n_controlsG)*N,1) = -pitch_maxG;
args.ubx(5:(n_controls+n_controlsG):(n_controls+n_controlsG)*N,1) = roll_maxG;   args.lbx(5:(n_controls+n_controlsG):(n_controls+n_controlsG)*N,1) = -roll_maxG;
args.ubx(6:(n_controls+n_controlsG):(n_controls+n_controlsG)*N,1) = yaw_maxG;    args.lbx(6:(n_controls+n_controlsG):(n_controls+n_controlsG)*N,1) = -yaw_maxG;
%%
%%THE SIMULATION LOOP STARTS HERE%%

x0 = [90;150;80;pi/2;0]; % also change below
x0T = [100;150;pi/2];
x0G = [0;0;0];  % also change below

u0 = zeros((n_controls + n_controlsG),N);
velT = 12;
angRateT = 0;
u0T = [velT;angRateT];

sim_tim = 250;
x_prediction = zeros((n_states+n_statesG),N+1);
u_prediction = zeros((n_controls+n_controlsG),N);
x_historyT = x0T;

% Starting MPC
count =0;
start_timer = tic;
while(norm((x0(1:2,1)-x0T(1:2,1)),2) < 1e-2 || count < sim_tim/T)
    args.p = [x0;x0T;x0G];
    args.x0 = reshape(u0,(n_controls+n_controlsG)*N,1);
    
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
        'lbg', args.lbg, 'ubg', args.ubg, 'p', args.p);
    
    u = reshape(full(sol.x),(n_controls+n_controlsG),N); % predicted controls
    ff_value = ff(u(1:n_controls,:),args.p(1:n_states)); % try with args.p(1:n_states)  % predicted states
    u0 = [u(:,2:N),u(:,N)];
    x0 = full(ff_value(:,2));
    
    ffG_value = ffG(u(n_controls+1:(n_controls+n_controlsG),:),args.p(n_states+n_statesT+1:n_states+n_statesT+n_statesG));
    x0G = full(ffG_value(:,2));
    
    ffT_value = ffT(u0T,x0T);
    x0T = full(ffT_value(:,2));
    velT = 12;
    angRateT = 0;
    u0T = [velT;angRateT];
    
    if (count >= 250 && count <= 349)
        velT = 12;
        angRateT = -pi/40;
        u0T = [velT;angRateT];
    elseif(count > 349)
        angRateT = 0;
        u0T = [velT;angRateT];
    end
    if (count >= 550 && count <= 649)
        velT = 12;
        angRateT = -pi/40;
        u0T = [velT;angRateT];
    elseif(count > 649)
        angRateT = 0;
        u0T = [velT;angRateT];
    end
    if (count >= 850 && count <= 949)
        velT = 12;
        angRateT = -pi/40;
        u0T = [velT;angRateT];
    elseif(count > 949)
        angRateT = 0;
        u0T = [velT;angRateT];
    end
    % storing values
    x_prediction(:,:,(count+1)) = [full(ff_value);full(ffG_value)];
    u_prediction(:,:,(count+1)) = u;
    x_historyT = [x_historyT,x0T];
    
    count
    count = count + 1;
end

end_timer = toc(start_timer);
average_time = end_timer/(count+1)
total_time = average_time*count

% Storing data
x_history = zeros((n_states+n_statesG),(count+1));
u_history = zeros((n_controls+n_controlsG),(count+1));
x_history(:,1) = [[90;150;80;pi/2;0];[0;0;0]]; % uav + gimbal initial states
u_history(:,1) = [[0;0;0];[0;0;0]];      % uav + gimbal initial controls
for k = 1:count
   x_history(:,k+1) = x_prediction(:,2,k);
   u_history(:,k+1) = u_prediction(:,1,k);
end

% calling plotting function
Graph_MPC_casadi (count,x_history,x_historyT,u_history,obs,r,T)