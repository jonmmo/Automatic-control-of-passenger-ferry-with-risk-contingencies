function [A, T] = find_turning_circle_params(x, ex, WP)
    
    p_ex = x(4:5, ex);
    p_end = x(4:5, end);
    
    [x_p, y_p, y_e] = crosstrack(WP(1,3), WP(2,3), WP(1,2), WP(2,2), p_end(1), p_end(2),1);
    T = y_e;
    A = norm([x_p y_p]' - p_ex);
end