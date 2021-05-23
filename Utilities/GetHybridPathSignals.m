function [pd, pd_der] = GetHybridPathSignals(NumSubpaths, Order, WPx, WPy, LinSysA, LinSysbx, LinSysby, coeffa, coeffb,...
        coeffa_der, coeffb_der,s)
%
% PathSignals = GetHybridPathSignals(Path,s)
%
% Function that generates the coefficients for subpaths between given waypoints.
% Each subpath is parametrized by t in [0,1), and smoothly connected at the waypoints. This 
% then correspond to a hybrid parametrized path, where Index i identifies the subpath, and 
% the parameter t in [0,1) identifies the location along the subpath.
%
% Input data:
%
%  Path         - A Matlab data structure with the following fields:
%                .NumSubpaths: Number of subpaths.
%                .Order: Order of polynomials.
%                .WP: x- and y-coordinates of waypoints.
%                .LinSys: Liner set of equations Ax=b to solve the subpaths:
%                   .A:     Common A matrix for both x- and y-coefficients.
%                   .bx:    Cell structure that contains the b-vector for each subpath for the x-coordinates.
%                   .by:    Cell structure that contains the b-vector for each subpath for the y-coordinates.
%                .coeff: Coefficients for the subpaths:
%                   .a:     Cell structure that contains the a-coefficients for the x-coordinates.
%                   .b:     Cell structure that contains the b-coefficients for the y-coordinates.
%                   .a_der: Cell structure that contains the a-coefficients for the derivatives 
%                           for the x-coordinates of the path.
%                   .b_der: Cell structure that contains the b-coefficients for the derivatives 
%                           for the y-coordinates of the path.
%  s            - Scalar path variable in interval [0,N).
%
% Output data:
%
%  PathSignals  - A Matlab data structure with the following fields:
%                .pd: Location (xd,yd) along the path corresponding to the value s.
%                .pd_der: Value of path derivatives at (xd,yd) corresponding to s. 
%                         This contains a cell for each derivative vector up to the 
%                         polynomial order of the path basis functions, i.e., 
%                         PathSignals.pd_der{1} contains 1st derivatives, 
%                         PathSignals.pd_der{2} the 2nd derivatives, and so on.
%                 Note that the path derivatives are guaranteed to be continuous 
%                 at the waypoints only for derivatives up to r = (Order-1)/2.
%
%
%    Copyright: 	Roger Skjetne, NTNU
%    Author:        Roger Skjetne
%    Date created:  2014.03.03  Roger Skjetne.
%    Revised:      	
%
%


%% Initialization
ord = Order;
if s < 0
    s = 0.0;
elseif s >= NumSubpaths
    s = NumSubpaths-1e3*eps;
end
idx = floor(s)+1;
t   = s-(idx-1);


%% Getting values
xd = 0;
yd = 0;
for k=0:ord
    xd = xd + coeffa(k+1,idx)*t^(k);
    yd = yd + coeffb(k+1,idx)*t^(k);
end
pd = [xd yd];

pd_der=zeros(1,2,ord);
for k=1:ord
    xd = 0; yd = 0;
    a_vec = coeffa_der(k,:,idx);
    b_vec = coeffb_der(k,:,idx);
    
    for j=1:length(a_vec)
        xd = xd + a_vec(j)*t^(j-1);
    end
    for j=1:length(b_vec)
        yd = yd + b_vec(j)*t^(j-1);
    end
    pd_der(:,:,k) = [xd yd];
end
    

