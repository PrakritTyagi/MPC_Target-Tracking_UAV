%-------------------------------------------------%
% Written by - Prakrit Tyagi                      %
% Dated - 23-01-2019                              %
% Project Under PB.Sujit(Coral Lab-IIIT-Delhi)    %
%-------------------------------------------------%
import casadi.*
% phi theta psi roll yaw pitch
clear
% MPC Parameters
N =15;
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
n_obs = 7;
r = 30;
xobs1 = 180; yobs1 = 550; h1 = 80;
xobs2 = -40;  yobs2 = 300; h2 = 100;
xobs3 = -40;  yobs3 = 750; h3 = 75;
xobs4 = 350; yobs4 = 1300; h4 = 85;
xobs5 = 470; yobs5 = 1600; h5 = 75;
xobs6 = 480; yobs6 = 2150; h6 = 80;
xobs7 = 900; yobs7 = 1800; h7 = 75;
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

%% Defining model for Target1
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

%% Defining model for Target2
xT2 = SX.sym('xT2'); yT2 = SX.sym('yT2'); ang2 = SX.sym('ang2');
statesT2 = [xT2;yT2;ang2]; n_statesT2 = length(statesT2);

vT2 = SX.sym('vT2'); angRate2 = SX.sym('angRate2');
controlsT2 = [vT2;angRate2]; n_controlsT2 = length(controlsT2);

modelT2 = [vT2*cos(ang2);vT2*sin(ang2);angRate2];
fT2 = Function('fT2',{statesT2,controlsT2},{modelT2});

PT2 = SX.sym('PT2',n_statesT2);
XT2 = SX.sym('XT2',n_statesT2,2);
UT2 = SX.sym('UT2',n_controlsT2);

% Calcuate next state
XT2(:,1) = PT2;
c_XT2 = XT2(:,1); c_UT2 = UT2(:,1);

fT_value2 = fT2(c_XT2,c_UT2);
new_XT2 = c_XT2 + T*fT_value2;
XT2(:,2) = new_XT2;

ffT2 = Function('ffT2',{UT2,PT2},{XT2});

%% Defining the objective cost function
obj = 0;
w_1 = [0,0]; % x y
% w_2 = 1.2; % z
w_3 = 80; % ellipse
w_4 = 5; % FOV
w_5 = 100; % avoid obstacle increase height
Q = diag(w_1);

for k = 1:N
   % relative distance cost function
   obj = obj + sqrt((X(1:2,k)-PT(1:2))'*Q*(X(1:2,k)-PT(1:2))) + sqrt((X(1:2,k)-PT2(1:2))'*Q*(X(1:2,k)-PT2(1:2))) ;
   
%    Field of View cost function
   a = 0.5*( X(3,k)*tan(XG(1,k) + X(5,k) + VFOV_rad/2) - X(3,k)*tan(XG(1,k) + X(5,k) - VFOV_rad/2) );
   b = 0.5*( X(3,k)*tan(XG(2,k) + HFOV_rad/2) - X(3,k)*tan(XG(2,k) - HFOV_rad/2) );
   shifta = a + X(3,k)*tan(XG(1,k) + X(5,k) - VFOV_rad/2);
   shiftb = b + X(3,k)*tan(XG(2,k) - HFOV_rad/2);
   A = ((cos(X(4,k) + XG(3,k)))^2)/(a^2) + ((sin(X(4,k) + XG(3,k)))^2)/(b^2) ;
   B = 2*cos(X(4,k) + XG(3,k))*sin(X(4,k) + XG(3,k))*( (1/a^2) - (1/b^2) );
   C = (sin(X(4,k) + XG(3,k)))^2/(a^2) + (cos(X(4,k) + XG(3,k)))^2/(b^2);
   ellipse = A*(PT(1) - X(1,k) - shifta)^2 + B*(PT(1) - X(1,k) - shifta)*(PT(2) - X(2,k) - shiftb) + C*(PT(2) - X(2,k) - shiftb)^2 - 1;
   ellipse2 = A*(PT2(1) - X(1,k) - shifta)^2 + B*(PT2(1) - X(1,k) - shifta)*(PT2(2) - X(2,k) - shiftb) + C*(PT2(2) - X(2,k) - shiftb)^2 - 1;
   obj = obj + w_3*ellipse + w_3*ellipse2;
   
   logic_TargetInVision = floor(ellipse/10000000)*-1; % 1 if inside ellipse else 0 for outside
   
   % Visibility cost function obstacle 1
   slope = (X(2,k)-PT(2))/(X(1,k)-PT(1)); % slope of ray(x-yplane) from target to UAV 
   center1_to_LOS = (abs(slope*xobs1 - 1*yobs1 + PT(2) - PT(1)*slope)/sqrt(slope^2 + (-1)^2))-r; % distance between line LOS 2D and center of obstacle1
   slopeRef1 = (h1/(sqrt( (PT(1)-xobs1)^2 + (PT(2)-yobs1)^2 )-r)) - (X(3,k)/(sqrt( (PT(1)-X(1,k))^2 + (PT(2)-X(2,k))^2 ))); % Slope of LOS between obstacle_1 Top and target in 3D
   logic_LosIntersectObstacle1 = floor(center1_to_LOS/100000000)*-1;
   costlos1 = logic_TargetInVision*logic_LosIntersectObstacle1*max(0,slopeRef1);
   obj = obj + w_4*costlos1;
   
   % Visibility cost function obstacle 2
%    slope = (X(2,k)-PT(2))/(X(1,k)-PT(1)); % slope of ray(x-yplane) from target to UAV 
   center2_to_LOS = (abs(slope*xobs2 - 1*yobs2 + PT(2) - PT(1)*slope)/sqrt(slope^2 + (-1)^2))-r; % distance between line LOS 2D and center of obstacle2
   slopeRef2 = (h2/(sqrt( (PT(1)-xobs2)^2 + (PT(2)-yobs2)^2 )-r)) - (X(3,k)/(sqrt( (PT(1)-X(1,k))^2 + (PT(2)-X(2,k))^2 ))); % Slope of LOS between obstacle_2 Top and target in 3D
   logic_LosIntersectObstacle2 = floor(center2_to_LOS/100000000)*-1;
   costlos2 = logic_TargetInVision*logic_LosIntersectObstacle2*max(0,slopeRef2);
   obj = obj + w_4*costlos2;
   
   % Visibility cost function obstacle 3
%    slope = (X(2,k)-PT(2))/(X(1,k)-PT(1)); % slope of ray(x-yplane) from target to UAV 
   center3_to_LOS = (abs(slope*xobs3 - 1*yobs3 + PT(2) - PT(1)*slope)/sqrt(slope^2 + (-1)^2))-r; % distance between line LOS 2D and center of obstacle3
   slopeRef3 = (h3/(sqrt( (PT(1)-xobs3)^2 + (PT(2)-yobs3)^2 )-r)) - (X(3,k)/(sqrt( (PT(1)-X(1,k))^2 + (PT(2)-X(2,k))^2 ))); % Slope of LOS between obstacle_3 Top and target in 3D
   logic_LosIntersectObstacle3 = floor(center3_to_LOS/100000000)*-1;
   costlos3 = logic_TargetInVision*logic_LosIntersectObstacle3*max(0,slopeRef3);
   obj = obj + w_4*costlos3;
   
   % Visibility cost function obstacle 4
%    slope = (X(2,k)-PT(2))/(X(1,k)-PT(1)); % slope of ray(x-yplane) from target to UAV 
   center4_to_LOS = (abs(slope*xobs4 - 1*yobs4 + PT(2) - PT(1)*slope)/sqrt(slope^2 + (-1)^2))-r; % distance between line LOS 2D and center of obstacle4
   slopeRef4 = (h4/(sqrt( (PT(1)-xobs4)^2 + (PT(2)-yobs4)^2 )-r)) - (X(3,k)/(sqrt( (PT(1)-X(1,k))^2 + (PT(2)-X(2,k))^2 ))); % Slope of LOS between obstacle_4 Top and target in 3D
   logic_LosIntersectObstacle4 = floor(center4_to_LOS/100000000)*-1;
   costlos4 = logic_TargetInVision*logic_LosIntersectObstacle4*max(0,slopeRef4);
   obj = obj + w_4*costlos4;
   
   % Visibility cost function obstacle 5
%    slope = (X(2,k)-PT(2))/(X(1,k)-PT(1)); % slope of ray(x-yplane) from target to UAV 
   center5_to_LOS = (abs(slope*xobs5 - 1*yobs5 + PT(2) - PT(1)*slope)/sqrt(slope^2 + (-1)^2))-r; % distance between line LOS 2D and center of obstacle5
   slopeRef5 = (h5/(sqrt( (PT(1)-xobs5)^2 + (PT(2)-yobs5)^2 )-r)) - (X(3,k)/(sqrt( (PT(1)-X(1,k))^2 + (PT(2)-X(2,k))^2 ))); % Slope of LOS between obstacle_5 Top and target in 3D
   logic_LosIntersectObstacle5 = floor(center5_to_LOS/100000000)*-1;
   costlos5 = logic_TargetInVision*logic_LosIntersectObstacle5*max(0,slopeRef5);
   obj = obj + w_4*costlos5;
   
   % Visibility cost function obstacle 5
%    slope = (X(2,k)-PT(2))/(X(1,k)-PT(1)); % slope of ray(x-yplane) from target to UAV 
   center6_to_LOS = (abs(slope*xobs6 - 1*yobs6 + PT(2) - PT(1)*slope)/sqrt(slope^2 + (-1)^2))-r; % distance between line LOS 2D and center of obstacle5
   slopeRef6 = (h6/(sqrt( (PT(1)-xobs6)^2 + (PT(2)-yobs6)^2 )-r)) - (X(3,k)/(sqrt( (PT(1)-X(1,k))^2 + (PT(2)-X(2,k))^2 ))); % Slope of LOS between obstacle_5 Top and target in 3D
   logic_LosIntersectObstacle6 = floor(center6_to_LOS/100000000)*-1;
   costlos6 = logic_TargetInVision*logic_LosIntersectObstacle6*max(0,slopeRef6);
   obj = obj + w_4*costlos6;
   
   % Visibility cost function obstacle 5
%    slope = (X(2,k)-PT(2))/(X(1,k)-PT(1)); % slope of ray(x-yplane) from target to UAV 
   center7_to_LOS = (abs(slope*xobs7 - 1*yobs7 + PT(2) - PT(1)*slope)/sqrt(slope^2 + (-1)^2))-r; % distance between line LOS 2D and center of obstacle5
   slopeRef7 = (h7/(sqrt( (PT(1)-xobs7)^2 + (PT(2)-yobs7)^2 )-r)) - (X(3,k)/(sqrt( (PT(1)-X(1,k))^2 + (PT(2)-X(2,k))^2 ))); % Slope of LOS between obstacle_5 Top and target in 3D
   logic_LosIntersectObstacle7 = floor(center7_to_LOS/100000000)*-1;
   costlos7 = logic_TargetInVision*logic_LosIntersectObstacle7*max(0,slopeRef7);
   obj = obj + w_4*costlos7;
   
   %obstacle collision avoidance
   obs1 = ((X(1,k)-xobs1)^2 + (X(2,k)-yobs1)^2 - r^2);
   checkobs1 = -1*floor(obs1/100000000);
   costobs1 = checkobs1*(h1-X(3,k))*(h1-X(3,k))^2;
   obj = obj + w_5*costobs1;
   
   obs2 = ((X(1,k)-xobs2)^2 + (X(2,k)-yobs2)^2 - r^2);
   checkobs2 = -1*floor(obs2/100000000);
   costobs2 = checkobs2*(h2-X(3,k))*(h2-X(3,k))^2;
   obj = obj + w_5*costobs2;
   
   obs3 = ((X(1,k)-xobs3)^2 + (X(2,k)-yobs3)^2 - r^2);
   checkobs3 = -1*floor(obs3/100000000);
   costobs3 = checkobs3*(h3-X(3,k))*(h3-X(3,k))^2;
   obj = obj + w_5*costobs3;
   
   obs4 = ((X(1,k)-xobs4)^2 + (X(2,k)-yobs4)^2 - r^2);
   checkobs4 = -1*floor(obs4/100000000);
   costobs4 = checkobs4*(h4-X(3,k))*(h4-X(3,k))^2;
   obj = obj + w_5*costobs4;
   
   obs5 = ((X(1,k)-xobs5)^2 + (X(2,k)-yobs5)^2 - r^2);
   checkobs5 = -1*floor(obs5/100000000);
   costobs5 = checkobs5*(h5-X(3,k))*(h5-X(3,k))^2;
   obj = obj + w_5*costobs5;

   obs6 = ((X(1,k)-xobs6)^2 + (X(2,k)-yobs6)^2 - r^2);
   checkobs6 = -1*floor(obs6/100000000);
   costobs6 = checkobs6*(h6-X(3,k))*(h6-X(3,k))^2;
   obj = obj + w_5*costobs6;
   
   obs7 = ((X(1,k)-xobs7)^2 + (X(2,k)-yobs7)^2 - r^2);
   checkobs7 = -1*floor(obs7/100000000);
   costobs7 = checkobs7*(h7-X(3,k))*(h7-X(3,k))^2;
   obj = obj + w_5*costobs7;
end

%% constraints

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
    g = [g;(-sqrt((X(1,k)-xobs1)^2 + (X(2,k)-yobs1)^2) + r)]; % obstacle 1
    g = [g;(-sqrt((X(1,k)-xobs2)^2 + (X(2,k)-yobs2)^2) + r)]; % obstacle 2
    g = [g;(-sqrt((X(1,k)-xobs3)^2 + (X(2,k)-yobs3)^2) + r)]; % obstacle 3
    g = [g;(-sqrt((X(1,k)-xobs4)^2 + (X(2,k)-yobs4)^2) + r)]; % obstacle 4
    g = [g;(-sqrt((X(1,k)-xobs5)^2 + (X(2,k)-yobs5)^2) + r)]; % obstacle 5
    g = [g;(-sqrt((X(1,k)-xobs6)^2 + (X(2,k)-yobs6)^2) + r)]; % obstacle 6
    g = [g;(-sqrt((X(1,k)-xobs7)^2 + (X(2,k)-yobs7)^2) + r)]; % obstacle 7
end

U0 = [U;UG];
OPT_variables = reshape(U0,(n_controls+n_controlsG)*N,1); % control variables
p = [P;PT;PG;PT2]; % initial parameters of uaav and target
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

args.ubx(1:(n_controls+n_controlsG):(n_controls+n_controlsG)*N,1) = v_max;      args.lbx(1:(n_controls+n_controlsG):(n_controls+n_controlsG)*N,1) = v_min; % constraint velocity
args.ubx(2:(n_controls+n_controlsG):(n_controls+n_controlsG)*N,1) = yaw_max ; args.lbx(2:(n_controls+n_controlsG):(n_controls+n_controlsG)*N,1) = yaw_min; % constraint yaw (psi_dot)
args.ubx(3:(n_controls+n_controlsG):(n_controls+n_controlsG)*N,1) = pitch_max;    args.lbx(3:(n_controls+n_controlsG):(n_controls+n_controlsG)*N,1) = pitch_min; % constraint pitch (theta_dot)
args.ubx(4:(n_controls+n_controlsG):(n_controls+n_controlsG)*N,1) = pitch_maxG;  args.lbx(4:(n_controls+n_controlsG):(n_controls+n_controlsG)*N,1) = -pitch_maxG;
args.ubx(5:(n_controls+n_controlsG):(n_controls+n_controlsG)*N,1) = roll_maxG;   args.lbx(5:(n_controls+n_controlsG):(n_controls+n_controlsG)*N,1) = -roll_maxG;
args.ubx(6:(n_controls+n_controlsG):(n_controls+n_controlsG)*N,1) = yaw_maxG;    args.lbx(6:(n_controls+n_controlsG):(n_controls+n_controlsG)*N,1) = -yaw_maxG;
%% THE SIMULATION LOOP STARTS HERE%%

x0 = [40;0;140;pi/2;0]; % also change below
x0T = [20;0;pi/2];
x0T2 = [70;0;pi/2];
x0G = [0;0;0];  % also change below

u0 = zeros((n_controls + n_controlsG),N);
velT = 12;
angRateT = 0;
angRateT2 = 0;
% angRateT2 = -pi/80; 
u0T = [velT;angRateT];
u0T2 = [velT;angRateT2];

sim_tim = 220;
x_prediction = zeros((n_states+n_statesG),N+1);
u_prediction = zeros((n_controls+n_controlsG),N);
x_historyT = x0T;
x_historyT2 = x0T2;

% Starting MPC
count =0;
start_timer = tic;
while(norm((x0(1:2,1)-x0T(1:2,1)),2) < 1e-2 || count < sim_tim/T || norm((x0(1:2,1)-x0T2(1:2,1)),2) < 1e-2)
    args.p = [x0;x0T;x0G;x0T2];
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
    ffT_value2 = ffT(u0T2,x0T2);
    x0T = full(ffT_value(:,2));
    x0T2 = full(ffT_value2(:,2));
    
% long
    if (count >= 660 && count <= 664)
        velT = 12;
        angRateT = -pi/4;
        u0T = [velT;angRateT]; 
    elseif(count > 664)
        angRateT = 0;
        u0T = [velT;angRateT]; 
    end
% 
    if (count >= 250 && count <= 254)
        angRateT2 = -pi/4;
        u0T2 = [velT;angRateT2]; 
    elseif(count > 254)
        angRateT2 = 0;
        u0T2 = [velT;angRateT2]; 
    end
    if (count >= 490 && count <= 499)
        velT = 12;
        angRateT2 = pi/4;
        u0T2 = [velT;angRateT2]; 
    elseif(count > 499)
        angRateT2 = 0;
        u0T2 = [velT;angRateT2]; 
    end
    if (count >= 690 && count <= 699)
        velT = 12;
        angRateT2 = -pi/4;
        u0T2 = [velT;angRateT2]; 
    elseif(count > 699)
        angRateT2 = 0;
        u0T2 = [velT;angRateT2]; 
    end
    
    % storing values
    x_prediction(:,:,(count+1)) = [full(ff_value);full(ffG_value)];
    u_prediction(:,:,(count+1)) = u;
    x_historyT = [x_historyT,x0T];
    x_historyT2 = [x_historyT2,x0T2];
    
    count
    count = count + 1;
end

end_timer = toc(start_timer);
average_time = end_timer/(count+1)
total_time = average_time*count

% Storing data
x_history = zeros((n_states+n_statesG),(count+1));
u_history = zeros((n_controls+n_controlsG),(count+1));
x_history(:,1) = [[40;0;150;pi/2;0];[0;0;0]]; % uav + gimbal initial states
u_history(:,1) = [[0;0;0];[0;0;0]];      % uav + gimbal initial controls
for k = 1:count
   x_history(:,k+1) = x_prediction(:,2,k);
   u_history(:,k+1) = u_prediction(:,1,k);
end

% calling plotting function
Graph_MPC_casadi (count,x_history,x_historyT,x_historyT2,u_history,xobs1,yobs1,xobs2,yobs2,xobs3,yobs3,xobs4,yobs4,xobs5,yobs5,xobs6,yobs6,xobs7,yobs7,r,h1,h2,h3,h4,h5,h6,h7,T)

%     if (count >= 100 )
%        velT = 12;
%        angRateT = -pi/120; angRateT2 = -pi/120;
%        u0T = [velT;angRateT];  u0T2 = [velT;angRateT2];
%     end
% 

% long
%     if (count >= 660 && count <= 664)
%         velT = 12;
%         angRateT = -pi/4;
%         u0T = [velT;angRateT]; 
%     elseif(count > 664)
%         angRateT = 0;
%         u0T = [velT;angRateT]; 
%     end
% % 
%     if (count >= 250 && count <= 254)
%         angRateT2 = -pi/4;
%         u0T2 = [velT;angRateT2]; 
%     elseif(count > 254)
%         angRateT2 = 0;
%         u0T2 = [velT;angRateT2]; 
%     end
%     if (count >= 490 && count <= 499)
%         velT = 12;
%         angRateT2 = pi/4;
%         u0T2 = [velT;angRateT2]; 
%     elseif(count > 499)
%         angRateT2 = 0;
%         u0T2 = [velT;angRateT2]; 
%     end
%     if (count >= 690 && count <= 699)
%         velT = 12;
%         angRateT2 = -pi/4;
%         u0T2 = [velT;angRateT2]; 
%     elseif(count > 699)
%         angRateT2 = 0;
%         u0T2 = [velT;angRateT2]; 
%     end

% square circle
%     if (count >= 150 && count <= 159)
%         angRateT = -pi/4;
%         u0T = [velT;angRateT]; 
%     elseif(count > 159)
%         angRateT = 0;
%         u0T = [velT;angRateT]; 
%     end
%     if (count >= 490 && count <= 499)
%         velT = 12;
%         angRateT = -pi/4;
%         u0T = [velT;angRateT]; 
%     elseif(count > 499)
%         angRateT = 0;
%         u0T = [velT;angRateT]; 
%     end
%     if (count >= 820 && count <= 829)
%         velT = 12;
%         angRateT = -pi/4;
%         u0T = [velT;angRateT]; 
%     elseif(count > 829)
%         angRateT = 0;
%         u0T = [velT;angRateT]; 
%     end
%     if (count >= 1160 && count <= 1169)
%         velT = 12;
%         angRateT = -pi/4;
%         u0T = [velT;angRateT]; 
%     elseif(count > 1169)
%         angRateT = 0;
%         u0T = [velT;angRateT]; 
%     end
