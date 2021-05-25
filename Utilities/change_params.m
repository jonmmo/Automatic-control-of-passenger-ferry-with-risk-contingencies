function [mode, p_o] = change_params(sim, mode, p_o, i)
% function to change parameters during simulation, based on which
% simulation is run. Can change mode and/or make obstacles "appear"
% suddenly.

switch sim
    case 2
       if i == 540
           p_o = [31; 38]; % obstacvle straight ahead
       end
    case 3
        if i == 530
            p_o = [33 ; 35]; % trajectory to SB edge of obstacle
        end
    case 30
        if i == 490
            p_o = [33 ; 35];% trajectory to P edge of obstacle
        end        
    case 4 
        if i == 2 % visible early
            p_o = [31 ; 38];
        end
    case 41 
        if i == 480 % visible with enough time
           p_o = [31 ; 38];
        end
    case 42 
        if i == 520
            p_o = [31 ; 38]; % visible too late 
        end
    case 43 
        if i == 540
            p_o = [31 ; 38]; %already in risk zone when visible
        end
    case 5
        if i == 530
            p_o = [33; 35];
        end
    case 51 
        if i == 510  
           p_o = [33; 35];
        end
    case 52 
        if i == 500
            p_o = [33; 35];
        end
    case 53 
        if i == 480
            p_o = [33; 35];
        end
    case 6
        if i == 870
            p_o = [8 43 58;
                18 57 48];
        end
end

end