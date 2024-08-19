%% controller parameters
sampleTime =0.1;               % Sample time [s]
r = rateControl(1/sampleTime);
resolution=50/10;
%% Initialize connection with V-Rep
connection = simulation_setup();
connection = simulation_openConnection(connection, 0);
simulation_start(connection);
[bodyDiameter ,wheelDiameter ,interWheelDist] = omni_init(connection);
coupling_matrix=(2/wheelDiameter)*[-0.7071  0.7071 -bodyDiameter/2; ...
                0.7071  0.7071 -bodyDiameter/2; ...
                0.7071  -0.7071 -bodyDiameter/2; ...
                -0.7071  -0.7071 -bodyDiameter/2];
%% manually generated trejectory
% robot_vel=[0;0.5;0];
% times = [0;3;6;9;12];
% waypoints = [1,1,0; 3,2,pi/4; 6,1,pi/2; 7,4,2*pi/3; 
%           8,8,pi];
% % waypoints = [1,1,0; 2,6,0;4,9,0;6,4,0; 8,2,0];
% tVec = 0:sampleTime:times(end);      % Time array
% ref = interp1(times,waypoints,tVec,'spline');
% %% Define a small map
bodyR=1.5/2;
map = false(50,50);
map(floor((3- bodyR)*resolution):floor((3+bodyR)*resolution),1:floor((4+bodyR)*resolution))=true;
map(floor((3-bodyR)*resolution):floor((3+bodyR)*resolution),floor((6-bodyR)*resolution):50)=true;
start_coords = [resolution,resolution];
dest_coords  = [30,40];

%% find the path_way from A* algorithm
[robot_pose_xy,p,q]= AStarGrid (map, start_coords, dest_coords);
robot_pose=zeros(size(robot_pose_xy,1),3);

robot_pose(:,1:2)=robot_pose_xy/resolution;
for i=2:size(robot_pose_xy,1)
    robot_pose(i,3)=normalizeAngle(atan2(robot_pose_xy(i,2)-robot_pose_xy(i-1,2),robot_pose_xy(i,1)-robot_pose_xy(i-1,1)));
end

time_allocated=20; %in s
rng(1000);
noise = (5*rand(301,3)*sin(i/5))/time_allocated;
times =((cumsum(ones(size(robot_pose_xy,1),1))-1)/(size(robot_pose_xy,1)-1))*time_allocated;
tVec = 0.0:sampleTime:times(end);      % Time array
% ref = interp1(times,robot_pose,tVec,'spline'); % robot_pose=Function(times)
[ref, d_ref] = bsplinepolytraj(robot_pose',[tVec(1) tVec(end)],tVec);
ref=ref';

%% find neceassary position,velocity,acceleration,theta,curvature
pose=zeros(size(tVec,2),3);
%% initialize object for each motor
% for i=1:4
% motor_obj(i)=fuzzy_pid(10,10,0);
% input_wSpeed(i)=0;
% end
angle_pid=fuzzy_pid(4.0,3.0,0.0);
%angle_pid=user_pid_continuous(8,3,0);
%angle_pid=user_pid_discrete(5,0,0);

r_pid=fuzzy_pid(4.0,3.0,0.0);
%r_pid=user_pid_continuous(5.0,3.0,0.0);
%r_pid=user_pid_discrete(4,1,0);

% x_pid=fuzzy_pid(3,3,0);

% y_pid=fuzzy_pid(3,3,0);

vu_prev=0;
%% start simulation
tic;
[x, y ,theta] = omni_getPose(connection);
% x = x + noise(1,1);
% y = y + noise(1,2);
% theta = theta + noise(1,3);
pose(1,:)=[x, y, theta];
for idx = 2:numel(tVec)
 fprintf('%f  %f  %f\n',x, y, theta);
 vx=ref(idx,1)-x;
 vy=ref(idx,2)-y;
 lambda = normalizeAngle(atan2(ref(idx,2)-y,ref(idx,1)-x)); 
 theta=normalizeAngle(theta);
 alpha=lambda-theta;
 dir=normalizeAngle(alpha);
 vu=sqrt(vx*vx+vy*vy);
 vu=r_pid.compute_speed(vu,0,0.5,1.0,10.0);   
 %vu=r_pid.compute_speed(vu,0);  
 ref(idx,3)=normalizeAngle(ref(idx,3));
%omega=angle_pid.compute_speed(normalizeError(ref(idx,3),3.0),normalizeError(theta,3.0));
omega=angle_pid.compute_speed(ref(idx,3),theta,0.5,1.0,10.0);
%omega=angle_pid.compute_speed(ref(idx,3),theta);
%apply kinematics
 robot_vel=[vu*cos(dir) ;vu*sin(dir);omega];
 w=coupling_matrix*robot_vel;
 % w(1) = w(1) + noise(idx);
 % w(2) = w(2) + noise(idx);
 % w(3) = w(3) + noise(idx);
 % w(4) = w(4) + noise(idx);
 omni_setWheelSpeeds(connection,w(1),w(2),w(3),w(4));
 
%% section
 %[x_lid,y_lid]=omni_getLaserData(connection);
 %fprintf('%f %f\n',x_lid,y_lid);
 %[scannerPose] = omni_getScannerPose(connection);
 %fprintf('%f\n',scannerPose);
 waitfor(r);
 [x, y ,theta] = omni_getPose(connection);  % get current pose of robot
 % x = x + noise(idx,1);
 % y = y + noise(idx,2);
 % theta = theta + noise(idx,3);
 pose(idx,:)=[x, y, theta];
end
fprintf('%f\n',toc);
%% Bring omni to standstill
simulation_stop(connection);
simulation_closeConnection(connection);
% figure;
% plot(tVec ,ref(:,1),tVec ,pose(:,1));
% ylim([-1 11]);
% xlim([-1 times(end)+1]);
% title('trajectory Plot of robot x position')
% xlabel('t')
% ylabel('pose x')
% legend('x ref','x pose')
% % 
% figure
% plot(tVec ,ref(:,2),tVec ,pose(:,2));
% ylim([-1 11]);
% xlim([-1 times(end)+1]);
% title('trajectory Plot of robot y position')
% xlabel('t')
% ylabel('pose y')
% legend('y ref','y pose')
% 
figure;
plot(ref(:,1) ,ref(:,2),pose(:,1) ,pose(:,2));
ylim([-1 11]);
xlim([-1 11]);
title('trajectory Plot of robot xy position')
xlabel('x')
ylabel('y')
legend('xy ref','xy pose')

figure;
plot(tVec ,ref(:,3),tVec ,pose(:,3));
ylim([-3.14 3.14]);
xlim([-1 times(end)+1]);
title('trajectory Plot of robot theta')
xlabel('t')
ylabel('pose theta')
legend('theta ref','theta pose')
msgbox('Simulation ended');
error_eucl_dist_x_y=mean(sqrt(sum((pose(:,1:2) - ref(:,1:2)) .^ 2,2)));
error_theta=mean(abs(pose(:,3) - ref(:,3)));
fprintf('%f  %f\n',error_eucl_dist_x_y, error_theta);
