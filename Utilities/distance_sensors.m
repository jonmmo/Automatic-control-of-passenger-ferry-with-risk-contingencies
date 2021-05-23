function [int1,int2,d1_pos,d2_pos,d1,d2]...
         = distance_sensors(d1_x, d2_x, d1_y, d2_y, eta_dock, eta)
% simulates the distance sensors. Adapted from Gasulaa(2020)

%% Calculating dock top and bottom point, from center point and angle
L = 5;                                            % Length of dock
R_dock = [cos(eta_dock(3)) -sin(eta_dock(3));
     sin(eta_dock(3)) cos(eta_dock(3))];  
 
dock_top = eta_dock(1:2) + R_dock*[L/2; 0];
dock_bot = eta_dock(1:2) + R_dock*[-L/2; 0];

%% Calculating position of sensors in global frame
R = [cos(eta(3)) -sin(eta(3));
     sin(eta(3)) cos(eta(3))];               % Rotation matrix with heading
d1_pos = [eta(1); eta(2)] + R*[d1_x; d1_y];  % Global position of sensor 1
d2_pos = [eta(1); eta(2)] + R*[d2_x; d2_y];  % Global position of sensor 2

%% Calculating normal vector to the vector from d1 to d2
dx = d2_pos(1)-d1_pos(1);
dy = d2_pos(2)-d1_pos(2);
J_sens = [-dy; dx];         % Normal vector out from ship side

%% Constructing lines out from sens 1 and 2, and line representing dock
N1 = polyfit([d1_pos(1), d1_pos(1)+J_sens(1)],...
             [d1_pos(2), d1_pos(2)+J_sens(2)],1); % Line from sensor 1
N2 = polyfit([d2_pos(1), d2_pos(1)+J_sens(1)],...
             [d2_pos(2), d2_pos(2)+J_sens(2)],1); % Line from sensor 2
dock_line = polyfit([dock_top(1),dock_bot(1)],...
                    [dock_top(2),dock_bot(2)],1); % Line repesenting dock
                
%% Calc. intersection points of lines from sensors and line represent dock             
x_intersect1 = fzero(@(x) polyval(N1-dock_line,x),3); % x-intersect of 1
y_intersect1 = polyval(N1,x_intersect1);              % y-intersect of 1
x_intersect2 = fzero(@(x) polyval(N2-dock_line,x),3); % x-intersect of 2
y_intersect2 = polyval(N2,x_intersect2);              % y-intersect of 2
inter_1=[x_intersect1; y_intersect1];                 % Inter.point. of 1
inter_2=[x_intersect2; y_intersect2];                 % Inter.point. of 2

%% Calculating distance from sensors to intersection points
d1 = norm(inter_1-d1_pos);                            % Distance from sens1
d2 = norm(inter_2-d2_pos);                            % Distance from sens2
int1 = inter_1;                                       % Output int.point 1
int2 = inter_2;                                       % Output int.point 2
