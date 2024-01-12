% Trajectory Tracking + Multiple shooting
%% controller parameters
sampleTime =0.1;               % Sample time [s]
r = rateControl(1/sampleTime);
resolution=50/10;
% CasADi v3.4.5
%% import casadi dependencies
addpath('C:\Users\Love Panta\OneDrive\Documents\casadi-windows-matlabR2016a-v3.5.5')
import casadi.*

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
time_allocated = 30; %in s
times =((cumsum(ones(size(robot_pose_xy,1),1))-1)/(size(robot_pose_xy,1)-1))*time_allocated;
tVec = 0:sampleTime:times(end);      % Time array
%ref = interp1(times,robot_pose,tVec,'spline'); % robot_pose=Function(times)
[ref, d_ref] = bsplinepolytraj(robot_pose',[times(1) times(end)],tVec);
ref=ref';
robot_speed=sqrt(ref(1,1)-ref(2,1)+(ref(1,2)-ref(2,2))^2)/sampleTime;
%% initialize the MPC parameters
N = 15; % prediction horizon
T = sampleTime;
rob_diam = 0.3;
v_base = 1.5;
v_max = v_base*10/time_allocated; v_min = -v_max;
omega_max = pi/2; omega_min = -omega_max;

x = SX.sym('x'); y = SX.sym('y'); theta = SX.sym('theta');
states = [x;y;theta]; n_states = length(states);

v = SX.sym('v'); omega = SX.sym('omega');
controls = [v;omega]; n_controls = length(controls);
rhs = [v*cos(theta);v*sin(theta);omega]; % system r.h.s

f = Function('f',{states,controls},{rhs}); % nonlinear mapping function f(x,u)
U = SX.sym('U',n_controls,N); % Decision variables (controls)
P = SX.sym('P',n_states + N*(n_states+n_controls));

X = SX.sym('X',n_states,(N+1));
% A vector that represents the states over the optimization problem.

obj = 0; % Objective function
g = [];  % constraints vector
Q_base = 15.0;
Q = zeros(3,3); Q(1,1)=Q_base;Q(2,2) =Q_base;Q(3,3) = Q_base/3; % weighing matrices (states)
R = zeros(2,2); R(1,1) = 1.0; R(2,2) = 1.0; % weighing matrices (controls)

st  = X(:,1); % initial state
g = [g;st-P(1:3)]; % initial condition constraints
tic;
for k = 1:N
    st = X(:,k);  con = U(:,k);
    obj = obj+(st-P(5*k-1:5*k+1))'*Q*(st-P(5*k-1:5*k+1)) + ...
              (con-P(5*k+2:5*k+3))'*R*(con-P(5*k+2:5*k+3)) ; % calculate obj
    % the number 5 is (n_states+n_controls)
    st_next = X(:,k+1);
    f_value = f(st,con);
    st_next_euler = st+ (T*f_value);
    g = [g;st_next-st_next_euler]; % compute constraints
end
% make the decision variable one column  vector
OPT_variables = [reshape(X,3*(N+1),1);reshape(U,2*N,1)];

nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);

args = struct;

args.lbg(1:3*(N+1)) = 0;  % -1e-20  % Equality constraints
args.ubg(1:3*(N+1)) = 0;  % 1e-20   % Equality constraints

args.lbx(1:3:3*(N+1),1) = -inf; %state x lower bound % new - adapt the bound
args.ubx(1:3:3*(N+1),1) = inf; %state x upper bound  % new - adapt the bound
args.lbx(2:3:3*(N+1),1) = -inf; %state y lower bound
args.ubx(2:3:3*(N+1),1) = inf; %state y upper bound
args.lbx(3:3:3*(N+1),1) = -inf; %state theta lower bound
args.ubx(3:3:3*(N+1),1) = inf; %state theta upper bound

args.lbx(3*(N+1)+1:2:3*(N+1)+2*N,1) = v_min; %v lower bound
args.ubx(3*(N+1)+1:2:3*(N+1)+2*N,1) = v_max; %v upper bound
args.lbx(3*(N+1)+2:2:3*(N+1)+2*N,1) = omega_min; %omega lower bound
args.ubx(3*(N+1)+2:2:3*(N+1)+2*N,1) = omega_max; %omega upper bound
%----------------------------------------------
% ALL OF THE ABOVE IS JUST A PROBLEM SET UP


% THE SIMULATION LOOP SHOULD START FROM HERE
%-------------------------------------------
t0 = 0;
[x_s, y_s ,~] = omni_getPose(connection);
x0 =[double(x_s);double(y_s) ;pi/4];    % initial condition.

xx(:,1) = x0; % xx contains the history of states
t=[];
t(1) = t0;

u0 = zeros(N,2);        % two control inputs for each robot
X0 = repmat(x0,1,N+1)'; % initialization of the states decision variables

% Start MPC
mpciter = 0;
xx1 = [];
u_cl=[];
trejectory=[];
% the main simulaton loop... it works as long as the error is greater
% than 10^-6 and the number of mpc steps is less than its maximum
% value.
set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', 12)

line_width = 1.5; 
fontsize_labels = 14;
x_r_1 = [];
y_r_1 = [];
pose=zeros(size(tVec,2),3);    
for idx = 1:numel(tVec)
          trejectory=[trejectory; [ref(idx,1),  ref(idx,2), ref(idx,3)]];
end   
args.p(4:6) = [ref(1,1), ref(1,2), ref(1,3)];
args.p(7:8) = [sqrt((ref(1,1)^2+ref(1,2)^2)), ref(1,3)/sampleTime];
for k = 2:N % new - set the reference to track
        args.p(5*k-1:5*k+1) = [ref(k,1), ref(k,2), ref(k,3) ];
        args.p(5*k+2:5*k+3) = [sqrt((ref(k-1,1)-ref(k,1))^2+(ref(k-1,2)-ref(k,2))^2)/sampleTime, (ref(k,3)-ref(k-1,3))/sampleTime];
end
[x_s, y_s ,theta_s] = omni_getPose(connection);
pose(1,:) = [x_s, y_s, theta_s];
for idx = 2:(numel(tVec))
    theta_s = normalizeAngle(double(theta_s));
    x0 = [double(x_s); double(y_s) ; theta_s];
    args.p(1:3) = x0; % initial condition of the robot posture
        
    if idx<=(numel(tVec))-N
        k=idx-1+N;
    else
        k=numel(tVec);
    end
    %----------------------------------------------------------------------    
    % initial value of the optimization variables
    args.x0  = [reshape(X0',3*(N+1),1);reshape(u0',2*N,1)];
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
        'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
    u = reshape(full(sol.x(3*(N+1)+1:end))',2,N)'; % get controls only from the solution
    xx1(:, 1:3, mpciter+1)= reshape(full(sol.x(1:3*(N+1)))', 3, N+1)'; % get solution TRAJECTORY
    u_cl= [u_cl ; u(1,:)];
    t(mpciter+1) = t0;
    subplot(11,1,1:3);
    yyaxis left
    stairs(t,u_cl(:,1),'k','linewidth',1.5); 
    ylabel('v (rad/s)')
    yyaxis right
    stairs(t,u_cl(:,2),'r','linewidth',1.5);
    xlabel('time (seconds)')
    ylabel('\omega (rad/s)')
    grid on
    % Apply the control and shift the solution
    [t0, x0, u0] = shift(T, t0, x0, u,f);
    [~,~,theta_s] = omni_getPose(connection);
    theta_s=normalizeAngle(double(theta_s));
    x1 = xx(1,mpciter+1,1); y1 = xx(2,mpciter+1,1); th1 = xx(3,mpciter+1,1);
    x_r_1 = [x_r_1 x1];
    y_r_1 = [y_r_1 y1];
    subplot(11,1,4:11);
    plot(trejectory(:,1),trejectory(:,2),'-g','linewidth',1.2);hold on
    plot(x_r_1,y_r_1,'-r','linewidth',line_width);hold on % plot exhibited trajectory
    plot(xx1(1:N,1,mpciter+1),xx1(1:N,2,mpciter+1),'b--*')
    fill(x1,y1,'-sk','MarkerSize',25)% plot robot position
    hold off
    ylabel('$y$-position (m)','interpreter','latex','FontSize',fontsize_labels)
    xlabel('$x$-position (m)','interpreter','latex','FontSize',fontsize_labels)
    axis([min(trejectory(:,1))-1 max(trejectory(:,1))+1 min(trejectory(:,2))-1 max(trejectory(:,2))+1])
    box on;
    grid on;
    drawnow
    xx(:,mpciter+2) = x0;
    X0 = reshape(full(sol.x(1:3*(N+1)))',3,N+1)'; % get solution TRAJECTORY
    % Shift trajectory to initialize the next step
    X0 = [X0(2:end,:);X0(end,:)];
    mpciter = mpciter + 1;   
    robot_vel=[u(1,1)*cos(normalizeAngle(x0(3)-theta_s)) ;u(1,1)*sin(normalizeAngle(x0(3)-theta_s));u(1,2)];  
    w=coupling_matrix*robot_vel;
    omni_setWheelSpeeds(connection,w(1),w(2),w(3),w(4));
    args.p(4:5*(N-1)+3) = args.p(9:5*N+3);
    args.p(5*N-1:5*N+1) = [ref(k,1), ref(k,2), ref(k,3)];
    args.p(5*N+2:5*N+3) = [sqrt((ref(N-1,1)-ref(N,1))^2+(ref(N-1,2)-ref(N,2))^2)/sampleTime, (ref(N,3)-ref(N-1,3))/sampleTime];
    waitfor(r);
    [x_s, y_s ,theta_s] = omni_getPose(connection);
    pose(idx,:) = [x_s, y_s, theta_s];
end
fprintf('%f\n',toc);
%% Bring omni to standstill
simulation_stop(connection);
simulation_closeConnection(connection);
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


