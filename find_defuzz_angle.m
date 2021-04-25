function [IS1, IS2]=find_defuzz_angle(err, d_err)
    scale_err = [-1.0 -0.9 -0.8 -0.7 -0.6 -0.5 -0.4 -0.3 -0.2 -0.1 0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0];
    scale_d_err = [-1.0 -0.9 -0.8 -0.7 -0.6 -0.5 -0.4 -0.3 -0.2 -0.1 0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0];
    err_step = 0.1; d_err_step = 0.1;
    % err membership에 속하는 구간 정하기
    if err <= scale_err(1)
        IS1 = 1;
    elseif err >= scale_err(21)
        IS1 = 21;
    else
        for i= 2:21
            if err > scale_err(i-1) & err <=scale_err(i)
                if (err-scale_err(i-1)) < err_step/2
                    IS1 = i-1;
                else
                    IS1 = i;
                end
            end
        end
    end
    % d_err membership에 속하는 구간 정하기
    if d_err <= scale_d_err(1)
        IS2 = 1;
    elseif d_err >= scale_d_err(21)
        IS2 = 21;
    else
        for j= 2:21
            if d_err > scale_d_err(j-1) & d_err <= scale_d_err(j)
                if (d_err-scale_d_err(j-1)) < d_err_step/2
                    IS2 = j-1;
                else
                    IS2 = j;
                end
            end
        end
    end    
%     disp(IS1);
%     disp(IS2);
    
end