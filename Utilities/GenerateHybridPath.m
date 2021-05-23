    function [NumSubpaths, Order, WPx, WPy, LinSysA, LinSysbx, LinSysby, coeffa, coeffb,...
        coeffa_der, coeffb_der] = GenerateHybridPath(WP,r,lambda)
%
% Path = GenerateHybridPath(WP,r,lambda,PlotFlag)
%
% Function that generates the coefficients for subpaths between given waypoints.
% Each subpath is parametrized by t in [0,1), and smoothly connected at the waypoints. This 
% then correspond to a hybrid parametrized path, where Index i identifies the subpath, and 
% the parameter t in [0,1) identifies the location along the subpath. The path and its 
% derivatives up to r = (Order-1)/2 are guaranteed to be continuous at the waypoints.
%
% Input data:
%
%    WP         - A matrix of waypoints, with the x-coordinates in the 1st column 
%                 and y-coordinates in the 2nd column.
%    r          - Order of derivatives with continuity at the waypoints. 
%    lambda     - Constant that tunes the curvature at the intermediate waypoints. 
%    PlotHandle - Figure number for plotting the path; Empty: No plot.
%
% Output data:
%
%    Path       - A Matlab data structure with the following fields:
%                .NumSubpaths: Number of subpaths.
%                .Order: Order of polynomials (=2*ord+1).
%                .WP: x- and y-coordinates of waypoints.
%                .LinSys: Liner set of equations Ax=b to solve the subpaths:
%                   .A:     Common A matrix for both x- and y-coefficients.
%                   .bx:    Cell structure that contains the b-vector for each subpath for the x-coordinates.
%                   .by:    Cell structure that contains the b-vector for each subpath for the y-coordinates.
%                .coeff: Coefficients for the subpaths:
%                   .a:     Cell structure that contains the a-coefficients for the x-coordinates.
%                   .b:     Cell structure that contains the b-coefficients for the y-coordinates.
%                   .a_der: Cell structure that contains the a-coefficients for the derivatives 
%                           for the x-coordinates of the path up to the polynomial order.
%                   .b_der: Cell structure that contains the b-coefficients for the derivatives 
%                           for the y-coordinates of the path up to the polynomial order.
%
%
%    Copyright: 	Roger Skjetne, NTNU
%    Author:        Roger Skjetne
%    Date created:  2014.03.06  Roger Skjetne.
%    Revised:      	
%


%% Initialization
ord  = 2*r+1;
% if mod(ord,2)==0
%     disp('Order of polynomial must be odd. Stopping execution...');
%     return;
% end
Path = [];
N    = length(WP)-1;

NumSubpaths = N;
Order = ord;
WPx  = WP(:,1);
WPy  = WP(:,2);

%% Calculating subpaths 
A       = zeros(ord+1,ord+1);
coefs   = ones(1,ord+1);
A(1,1)  = coefs(1);
A(2,:)  = coefs;
c_der   = coefs;
    LinSysbx = zeros(ord+1,1,N);
    LinSysby = zeros(ord+1,1,N);
    coeffa   = zeros(ord+1,N);
    coeffb   = zeros(ord+1,N);
    coeffa_der = zeros(7,7,N);
    coeffb_der = zeros(7,7,N);

    

    
for k=2:(ord+1)/2
    c_der       = fliplr(polyder(fliplr(c_der)));
    coefs       = [zeros(1,k-1) c_der];
    A(2*k-1,k)  = c_der(1);
    A(2*k,:)    = coefs;
end
LinSysA = A;

ax = zeros(ord+1,1);
bx = zeros(ord+1,1);
for j=1:N
    ax     = zeros(ord+1,1);
    bx     = zeros(ord+1,1);
    
    ax(1)  = WP(j,1);
    ax(2)  = WP(j+1,1);
    bx(1)  = WP(j,2);
    bx(2)  = WP(j+1,2);
    if ord>2
        if j==1
            ax(3) = WP(j+1,1)-WP(j,1);
            bx(3) = WP(j+1,2)-WP(j,2);
            ax(4) = lambda*(WP(j+2,1)-WP(j,1));
            bx(4) = lambda*(WP(j+2,2)-WP(j,2));
        elseif j==N
            ax(3) = lambda*(WP(j+1,1)-WP(j-1,1));
            bx(3) = lambda*(WP(j+1,2)-WP(j-1,2));
            ax(4) = WP(j+1,1)-WP(j,1);
            bx(4) = WP(j+1,2)-WP(j,2);
        else
            ax(3) = lambda*(WP(j+1,1)-WP(j-1,1));
            bx(3) = lambda*(WP(j+1,2)-WP(j-1,2));
            ax(4) = lambda*(WP(j+2,1)-WP(j,1));
            bx(4) = lambda*(WP(j+2,2)-WP(j,2));
        end
    end
    a_vec  = A\ax;
    b_vec  = A\bx;
    LinSysbx(:,:,j) = ax;
    LinSysby(:,:,j) = bx;
    coeffa(:,j)   = a_vec;
    coeffb(:,j)   = b_vec;
    a_vec = a_vec';
    b_vec = b_vec';
    for k=1:ord
        a_vec = fliplr(polyder(fliplr(a_vec)));
        %Path.coeff.a_der{j}{k} = a_vec;
        b_vec = fliplr(polyder(fliplr(b_vec)));
        %Path.coeff.b_der{j}{k} = b_vec;
            for z = 1:length(a_vec)
            coeffa_der(k,z,j) = a_vec(z);
            end
            for z = 1:length(b_vec)
            coeffb_der(k,z,j) = b_vec(z);
            end

    end
end
