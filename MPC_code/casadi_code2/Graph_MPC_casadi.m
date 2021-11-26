function Graph_MPC_casadi (count,x_history,x_historyT,x_historyT2,u_history,xobs1,yobs1,xobs2,yobs2,xobs3,yobs3,xobs4,yobs4,xobs5,yobs5,xobs6,yobs6,xobs7,yobs7,r,h1,h2,h3,h4,h5,h6,h7,T)
fig = figure;
for k = 1:(count+1)
    
    clf 
    
    x_uav = x_history(1,k);
    y_uav = x_history(2,k);
    z_uav = x_history(3,k);
    yaw_uav = x_history(4,k);
    pitch_uav = x_history(5,k);
    pitchG = x_history(6,k);
    rollG = x_history(7,k);
    yawG = x_history(8,k);
    
    hold on
%     axis('equal')
%     %plot the current location of UAV
%     plot3(x_uav,y_uav,z_uav,'go','LineWidth',3,'MarkerSize', 8)
    yaw_uavD = yaw_uav*180/pi;
    pitch_uavD = pitch_uav*180/pi;
    c130(x_uav,y_uav-9,z_uav,'yaw',(yaw_uavD-90),'pitch',pitch_uavD);
    
    % plot trajectory of UAV
    p1 = plot3(x_history(1,1:k),x_history(2,1:k),x_history(3,1:k),'color','red','LineWidth',3);
    
    % plot trajectory of Target
    p2 = plot3(x_historyT(1,1:k),x_historyT(2,1:k),0*x_historyT(2,1:k),'--','color','blue','linewidth',2);
%     plot3(x_historyT(1,k),x_historyT(2,k),x_historyT(3,k),'go','LineWidth',1,'MarkerSize', 3)
    
    % plot trajectory of Target2
    p2 = plot3(x_historyT2(1,1:k),x_historyT2(2,1:k),0*x_historyT2(2,1:k),'--','color','blue','linewidth',2);
%     plot3(x_historyT2(1,k),x_historyT2(2,k),x_historyT2(3,k),'go','LineWidth',1,'MarkerSize', 3)
    
    
    %Camera Parameters and plotting FOV 
    %https://photo.stackexchange.com/questions/56596/how-do-i-calculate-the-ground-footprint-of-an-aerial-camera
    VFOV_deg = 54.4;
    HFOV_deg = 62.6;
    VFOV_rad = VFOV_deg*pi/180;
    HFOV_rad = HFOV_deg*pi/180;
    th = 0:pi/50:2*pi;
    
    a =  0.5*( z_uav*tan( pitchG + pitch_uav + VFOV_rad/2 ) - z_uav*tan( pitchG + pitch_uav - VFOV_rad/2 ));  
    b =  0.5*( z_uav*tan( rollG + HFOV_rad/2 ) - z_uav*tan( rollG - HFOV_rad/2 ));

    x = a*sin(th);
    y = b*cos(th);
    xprime = x_uav + a + z_uav*tan( pitchG + pitch_uav - VFOV_rad/2 ) + x*cos(yaw_uav + yawG) - y*sin(yaw_uav + yawG);
    yprime = y_uav + b + z_uav*tan( rollG - HFOV_rad/2 ) + x*sin(yaw_uav + yawG) + y*cos(yaw_uav + yawG);
    p3 = plot3(xprime,yprime,0*cos(th),'color','black');
    
    rayx = [xprime(1:25:100);(x_uav + 0*cos(th(1:25:100)))];
    rayy = [yprime(1:25:100);(y_uav + 0*cos(th(1:25:100)))];
    rayz = [0*cos(th(1:25:100));(z_uav + 0*cos(th(1:25:100)))];
%     rayx = [(x_uav + a + z_uav*tan( pitchG + pitch_uav - VFOV_rad/2 ));x_uav];
%     rayy = [(y_uav + b + z_uav*tan( rollG - HFOV_rad/2 ));y_uav];
%     rayz = [0;z_uav];
    p3 = plot3(rayx,rayy,rayz,'color','black');
    
    %plotting obstacles
    x_unit1 = xobs1 + r*cos(th);
    y_unit1 = yobs1 + r*sin(th);
    z_unit1 = 0*r*cos(th);
%     plot3(x_unit1,y_unit1,z_unit1,'color','blue') % obstacle 1
    [X1,Y1,Z1] = cylinder(r);
    X1 = X1 + xobs1; Y1 = Y1 + yobs1; Z1 = Z1*h1;
    p4 = surf(X1,Y1,Z1);
    
    x_unit2 = xobs2 + r*cos(th);
    y_unit2 = yobs2 + r*sin(th);
    z_unit2 = 0*r*cos(th);
%     plot3(x_unit2,y_unit2,z_unit2,'color','blue') % obstacle 2
    [X2,Y2,Z2] = cylinder(r);
    X2 = X2 + xobs2; Y2 = Y2 + yobs2; Z2 = Z2*h2;
    p4 = surf(X2,Y2,Z2);
    
    x_unit3 = xobs3 + r*cos(th);
    y_unit3 = yobs3 + r*sin(th);
    z_unit3 = 0*r*cos(th);
%     plot3(x_unit3,y_unit3,z_unit3,'color','blue') % obstacle 3
    [X3,Y3,Z3] = cylinder(r);
    X3 = X3 + xobs3; Y3 = Y3 + yobs3; Z3 = Z3*h3;
    p4 = surf(X3,Y3,Z3);
    
    x_unit4 = xobs4 + r*cos(th);
    y_unit4 = yobs4 + r*sin(th);
    z_unit4 = 0*r*cos(th);
%     plot3(x_unit4,y_unit4,z_unit4,'color','blue') % obstacle 4
    [X4,Y4,Z4] = cylinder(r);
    X4 = X4 + xobs4; Y4 = Y4 + yobs4; Z4 = Z4*h4;
    p4 = surf(X4,Y4,Z4);
    
    x_unit5 = xobs5 + r*cos(th);
    y_unit5 = yobs5 + r*sin(th);
    z_unit5 = 0*r*cos(th);
%     plot3(x_unit5,y_unit5,z_unit5,'color','blue') % obstacle 5
    [X5,Y5,Z5] = cylinder(r);
    X5 = X5 + xobs5; Y5 = Y5 + yobs5; Z5 = Z5*h5;
    p4 = surf(X5,Y5,Z5);
    
    x_unit6 = xobs6 + r*cos(th);
    y_unit6 = yobs6 + r*sin(th);
    z_unit6 = 0*r*cos(th);
%     plot3(x_unit6,y_unit6,z_unit6,'color','blue') % obstacle 6
    [X6,Y6,Z6] = cylinder(r);
    X6 = X6 + xobs6; Y6 = Y6 + yobs6; Z6 = Z6*h6;
    p4 = surf(X6,Y6,Z6);
    
    x_unit7 = xobs7 + r*cos(th);
    y_unit7 = yobs7 + r*sin(th);
    z_unit7 = 0*r*cos(th);
%     plot3(x_unit7,y_unit7,z_unit7,'color','blue') % obstacle 7
    [X7,Y7,Z7] = cylinder(r);
    X7 = X7 + xobs7; Y7 = Y7 + yobs7; Z7 = Z7*h7;
    p4 = surf(X7,Y7,Z7);
    
    grid on 
    h = [p1(1);p2(1);p3(1);p4(1)];
    legend(h,'UAV Trajectory','Target Trajectory 1 & 2','FOV','Obstacles');
    pbaspect([1 1 0.25])
%     axis([-100 1150 -300 950 0 200])
    xlabel('x(m)','fontweight','bold') 
    ylabel('y(m)','fontweight','bold')
    zlabel('z(m)','fontweight','bold')
%     view([10+0.1 20+0.2])
    
    drawnow
%     pause(0.08)
    movieVector(k) = getframe(fig, [40 40 470 400]); 
end

%save the movie
myWriter = VideoWriter('UAV_MPC1', 'MPEG-4');
myWriter.FrameRate = 13;
open(myWriter);
writeVideo(myWriter, movieVector);
close(myWriter);

%velocity plot
figure;
y = linspace(0,T*(count-1),count);
plot(y,u_history(1,2:(count+1)),'lineWidth',2);
grid on
xlim([0 T*(count-1)])
ylim([0 30])
xlabel('Time stamp','fontweight','bold') 
ylabel('Velocity(m/s)','fontweight','bold')

%yaw rate
figure;
y = linspace(0,T*(count-1),count);
plot(y,u_history(2,2:(count+1)),'lineWidth',2);
grid on
xlim([0 T*(count-1)])
% ylim([0 30])
xlabel('Time(sec)','fontweight','bold') 
ylabel('\Omega_{3_{U,k}}','fontweight','bold')

%pitch rate
figure;
y = linspace(0,T*(count-1),count);
plot(y,u_history(3,2:(count+1)),'lineWidth',2);
grid on
xlim([0 T*(count-1)])
% ylim([0 30])
xlabel('Time','fontweight','bold') 
ylabel('\Omega_{2_{U,k}}','fontweight','bold')

%altitude
figure;
y = linspace(0,T*(count-1),count);
plot(y,x_history(3,2:(count+1)),'lineWidth',2);
grid on
xlim([0 T*(count-1)])
% ylim([0 30])
xlabel('Time','fontweight','bold') 
ylabel('Altitude(m)','fontweight','bold')

%yaw
figure;
y = linspace(0,T*(count-1),count);
plot(y,x_history(4,2:(count+1)),'lineWidth',2);
grid on
xlim([0 T*(count-1)])
% ylim([0 30])
xlabel('Time','fontweight','bold') 
ylabel('\psi_{U,k}','fontweight','bold')

%pitch
figure;
y = linspace(0,T*(count-1),count);
plot(y,x_history(5,2:(count+1)),'lineWidth',2);
grid on
xlim([0 T*(count-1)])
% ylim([0 30])
xlabel('Time','fontweight','bold') 
ylabel('\theta_{U,k}','fontweight','bold')

%E(n) plot
figure
ellipse_history = zeros(1,(count+1));
ellipse_history2 = zeros(1,(count+1));
    VFOV_deg = 44.4;
    HFOV_deg = 62.6;
    VFOV_rad = VFOV_deg*pi/180;
    HFOV_rad = HFOV_deg*pi/180;
notinVision = 0;
notinVision2 = 0;
for k = 1:(count+1)
a = 0.5*( x_history(3,k)*tan(x_history(6,k) + x_history(5,k) + VFOV_rad/2) - x_history(3,k)*tan(x_history(6,k) + x_history(5,k) - VFOV_rad/2) );
b = 0.5*( x_history(3,k)*tan(x_history(7,k) + HFOV_rad/2) - x_history(3,k)*tan(x_history(7,k) - HFOV_rad/2) );
shifta = a + x_history(3,k)*tan(x_history(6,k) + x_history(5,k) - VFOV_rad/2);
shiftb = b + x_history(3,k)*tan(x_history(7,k) - HFOV_rad/2);
A = ((cos(x_history(4,k) + x_history(8,k)))^2)/(a^2) + ((sin(x_history(4,k) + x_history(8,k)))^2)/(b^2) ;
B = 2*cos(x_history(4,k) + x_history(8,k))*sin(x_history(4,k) + x_history(8,k))*( (1/a^2) - (1/b^2) );
C = (sin(x_history(4,k) + x_history(8,k)))^2/(a^2) + (cos(x_history(4,k) + x_history(8,k)))^2/(b^2);
ellipse = A*(x_historyT(1,k) - x_history(1,k) - shifta)^2 + B*(x_historyT(1,k) - x_history(1,k) - shifta)*(x_historyT(2,k) - x_history(2,k) - shiftb) + C*(x_historyT(2,k) - x_history(2,k) - shiftb)^2 - 1; 
ellipse2 = A*(x_historyT2(1,k) - x_history(1,k) - shifta)^2 + B*(x_historyT2(1,k) - x_history(1,k) - shifta)*(x_historyT2(2,k) - x_history(2,k) - shiftb) + C*(x_historyT2(2,k) - x_history(2,k) - shiftb)^2 - 1;
logic_TargetInVision = floor(ellipse/10000000)*-1; % 1 if inside ellipse else 0 for outside
logic_TargetInVision2 = floor(ellipse2/10000000)*-1;
ellipse_history(1,k) = logic_TargetInVision;
ellipse_history2(1,k) = logic_TargetInVision2;
if logic_TargetInVision ==0
    notinVision = notinVision +1
end
if logic_TargetInVision2 ==0
    notinVision2 = notinVision2 +1
end
end
z = linspace(0,T*count,(count+1));
plot(z,ellipse_history(1,:),'lineWidth',2);
grid on
xlim([0 T*(count-1)])
xlabel('Time','fontweight','bold') 
ylabel('E(n)','fontweight','bold')
figure
plot(z,ellipse_history2(1,:),'lineWidth',2);
grid on
xlim([0 T*(count-1)])
xlabel('Time','fontweight','bold') 
ylabel('E(n)2','fontweight','bold')

% obs1 collision avoidance
figure;
w_5 = 55;
obs1_history = zeros(1,(count+1));
for k = 1:(count+1)
    obs1 = ((x_history(1,k)-xobs1)^2 + (x_history(2,k)-yobs1)^2 - r^2);
    checkobs1 = -1*floor(obs1/1000000);
    costobs1 = w_5*checkobs1*max(0,(h1-x_history(3,k)));
    obs1_history(1,k) = costobs1;
end
z = linspace(0,T*count,(count+1));
plot(z,obs1_history(1,:),'lineWidth',2);
xlabel('Time stamp') 
ylabel('obs1 collision avoidance')
