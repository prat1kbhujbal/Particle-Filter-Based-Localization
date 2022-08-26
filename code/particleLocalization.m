% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)

% Number of poses to calculate
N = size(ranges, 2);
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters 
% 
% the number of grids for 1 meter.
myResol = param.resol;
% the origin of the map in pixels
myOrigin = param.origin; 

% The initial pose is given
myPose(:,1) = param.init_pose;
% You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
% The pose(:,1) should be the pose when ranges(:,j) were measured.

% Used for making video
t = param.t; % Time vector
pose = param.pose; % Actual pose

% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = 500;                        % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.                                                 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
P = repmat(myPose(:,1), [1, M]);

hFig = figure(1);
set( hFig,'Name','Particles','NumberTitle','off');
imagesc(map);
hold on;
axis equal;
particles = scatter(P(1,:)*myResol+myOrigin(1),P(2,:)*myResol+myOrigin(2), 20, 'MarkerEdgeColor',[0 0 0],...
              'MarkerFaceColor',[0 1 1],...
              'LineWidth',1.5);
          
figure('Name','Path and Lidar data','NumberTitle','off');       
imagesc(map);
hold on;
axis equal;
lidar_global(:,1) =  (ranges(:,1).*cos(scanAngles + myPose(3,1)) + myPose(1,1))*myResol + myOrigin(1);
lidar_global(:,2) = (-ranges(:,1).*sin(scanAngles + myPose(3,1)) + myPose(2,1))*myResol + myOrigin(2);
lidarPlot = plot(lidar_global(:,1), lidar_global(:,2), 'g.'); 
current_pos = plot(myPose(1,1)*param.resol+param.origin(1), ...
    myPose(2,1)*param.resol+param.origin(2), 'k-');

figure('Name','Result','NumberTitle','off');
grid;
hold on;
xlabel('X'); 
ylabel('Y'); 
actual_plt = plot(pose(1,1),pose(2,1),'k-.', 'LineWidth', 2.5);
estimate_plt= plot(myPose(1,1),myPose(2,1),'b-', 'LineWidth', 1.5);
legend('Grount Truth', 'Estimate' );
pause;

corln = zeros(1,M);
smt = diag([0.0015,0.0015,0.0005]);    

for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).
    
    % 1) Propagate the particles 
    P = diag(myPose(:,j-1))*ones(3,M) +  mvnrnd([0;0;0],smt,M)';
    
    % 2) Measurement Update 
    for k = 1:M
    %   2-1) Find grid cells hit by the rays (in the grid map coordinate
    %   frame)
        range = ceil(1.0*size(ranges,1));
        index = 1:range;
        xocp = ranges(index,j).*cos(P(3,k)+scanAngles(index)) + P(1,k);
        yocp = -ranges(index,j).*sin(P(3,k)+scanAngles(index)) + P(2,k);
        ocp = ceil([xocp';yocp']*myResol) + myOrigin*ones(1,range);
        remocp =  ocp(1,:)<1 | ocp(2,:)<1 |  ocp(1,:) > size(map,2) |  ocp(2,:) > size(map,1);
        ocp(:,remocp) = [];
        
    %   2-2) For each particle, calculate the correlation scores of the particles
        ocpindx = sub2ind(size(map),ocp(2,:),ocp(1,:));
        ocpv = map(ocpindx);
        corln(1,k) = sum(ocpv(ocpv>=0.5)*10) + sum(ocpv(ocpv<=-0.2)*2);% + sum(free_values(free_values<0)*-3) + sum(free_values(free_values>0)*-5);% - sum(sumnum)*0.05;
    end
    %   2-3) Update the particle weights  
    corln(corln<0)= 0;
    weights = corln;
    weights = weights./sum(weights);
    if sum(weights<0)>0
        pause
    end
 
    %   2-4) Choose the best particle to update the pose
  
    
    % 3) Resample if the effective number of particles is smaller than a threshold

    
    % 4) Visualize the pose on the map as needed


end

end

