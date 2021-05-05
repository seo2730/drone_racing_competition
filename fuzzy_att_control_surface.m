function [lookup_table_p, lookup_table_d] = fuzzy_control_surface()
% 제어평면 구성 함수 마지막 lookup_table이 제어평면이 된다.
    membership_interval = [-1:0.1:1];
    [first_size,last_size] = size(membership_interval);
    err_memb_func = membership_main(5,membership_interval);
    d_err_memb_func = membership_main(5,membership_interval);
    
    % NB=1, NS =2, ZE=3, PS=4, PB=5
    Kp_fam = [
    5 4 4 4 3;
    4 4 4 3 2;
    4 4 3 2 2;
    4 3 2 2 2;
    3 2 2 2 1;
    ];
    Kd_fam = [
    1 2 2 2 3;
    1 2 2 3 4;
    2 2 3 4 5;
    2 3 4 4 5;
    3 4 4 4 5;
    ];
    
    % 각 membership Function의 중심 값 
%     pattern = [-4.0, -2.0, 0, 2.0, 4.0];
%     pattern = [-1.6, -1.2, -0.8, -0.4,  0, 0.4,  0.8, 1.2, 1.6];
    pattern = [-0.8, -0.4, 0, 0.4, 0.8];
    [first_ij,last_ij] = size(pattern);
    for I=first_size:last_size
        for J=first_size:last_size
            for i = first_ij:last_ij
                for j = first_ij:last_ij
                    member(i,j) = min(err_memb_func(i,I), d_err_memb_func(j,J));
                    sum_p(i,j) = member(i,j)*pattern(Kp_fam(i,j));
                    sum_d(i,j) = member(i,j)*pattern(Kd_fam(i,j));
                end
            end
            num_p = 0.0; den_p = 0.0;
            num_d = 0.0; den_d = 0.0;
            for i = first_ij:last_ij
                for j = first_ij:last_ij
                    num_p = num_p+sum_p(i,j); % 분자
                    den_p = den_p+member(i,j); % 분모
                    num_d = num_d+sum_d(i,j); % 분자
                    den_d = den_d+member(i,j); % 분모
                end
            end
            if den_p == 0
                lookup_table_p(I,J) = 0; % p제어기 제어평면 생성
            else
                lookup_table_p(I,J) = num_p/den_p; % p제어기 제어평면 생성
            end
            if den_d == 0
                lookup_table_d(I,J) = 0; % i제어기 제어평면 생성
            else
                lookup_table_d(I,J) = num_d/den_d; % i제어기 제어평면 생성
            end
        end
    end
end













