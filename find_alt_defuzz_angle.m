function [IS1, IS2]=find_alt_defuzz_angle(err, d_err)
    scale_err = [-1.0:0.1:1.0];
    scale_d_err = [-1.0:0.1:1.0];
    [first_err,last_err] = size(scale_err);
    [first_d_err,last_d_err] = size(scale_d_err);
    err_step = 0.1; d_err_step = 0.1;
    % err membership에 속하는 구간 정하기
    if err <= scale_err(first_err)
        IS1 = first_err;
    elseif err >= scale_err(last_err)
        IS1 = last_err;
    else
        for i= first_err+1:last_err
            if err > scale_err(i-1) && err <=scale_err(i)
                if (err-scale_err(i-1)) < err_step/2
                    IS1 = i-1;
                else
                    IS1 = i;
                end
            end
        end
    end
    % d_err membership에 속하는 구간 정하기
    if d_err <= scale_d_err(first_d_err)
        IS2 = first_d_err;
    elseif d_err >= scale_d_err(last_d_err)
        IS2 = last_d_err;
    else
        for j= first_err+1:last_err
            if d_err > scale_d_err(j-1) && d_err <= scale_d_err(j)
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