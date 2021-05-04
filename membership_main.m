function output = membership_main(membership_number,interval)
n = membership_number;
interval_size = size(interval,2);
membership_center = zeros(1, n);
center_interval = (interval(interval_size) - interval(1))/ n;
membership_center_num = (n+1)/2;
membership_center(membership_center_num) = 0;
membership_slope = 1/center_interval;
k=1;
l=1;
%% right center of membership
for i = membership_center_num+1 : n
    membership_center(i) = membership_center(membership_center_num)+ k * center_interval;
    k = k+1;
end
%% left center of membership
for j = 1 : membership_center_num - 1
    membership_center(membership_center_num-j) = membership_center(membership_center_num) - l * center_interval;
    l = l+1;
end
output_arr = membership_arr(interval,membership_center,membership_slope);
output = output_arr;
end
function output_arr = membership_arr(input,membership_center,membership_slope)
input_size = size(input,2);
membership_size = size(membership_center,2);
output_arr = zeros(membership_size, input_size);
for j = 1:membership_size
    for i = 1:input_size
        if j == 1
            output_arr(j,i) = membershipfnc_type2(input(i),membership_center(j),membership_slope);
        elseif j>1 && j< membership_size
            output_arr(j,i) = membershipfnc_type1(input(i),membership_center(j),membership_slope);
        else
            output_arr(j,i) = membershipfnc_type3(input(i),membership_center(j),membership_slope);
        end
    end
end
end
%% middle membership function
function output = membershipfnc_type1(x,m,c)
if x <= m-(1/c)
    output = 0;
elseif x > m-(1/c) && x <= m
    output = c*(x - (m-(1/c)));
elseif x > m && x <= m+(1/c)
    output = -1*c*(x - (m+(1/c)));
else
    output = 0;
end
end
%% left membership function
function output = membershipfnc_type2(x,m,c)
if  x <= m
    output = 1;
elseif x > m && x <= m+(1/c)
    output = -1*c*(x - (m+(1/c)));
else
    output = 0;
end
end
%% right membership function
function output = membershipfnc_type3(x,m,c)

if x <= m-(1/c)
    output = 0;
elseif x > m-(1/c) && x <= m
    output = c*(x - (m-1/c));
else
    output = 1;
end

end