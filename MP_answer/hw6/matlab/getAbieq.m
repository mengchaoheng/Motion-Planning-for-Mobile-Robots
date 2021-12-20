function [Aieq, bieq] = getAbieq(n_seg, n_order, corridor_range, ts, v_max, a_max)
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % STEP 3.2.1 p constraint
    Aieq_p = [];
    bieq_p = [];
    coeff_p = [];
%     c = [c_1;c_2;...;c_m]
%     where c_j=[c_j1;c_j2;...;c_jn], j=1,2,...,n_seg-1. 
%     c_ji=c( (j-1)*(n_order+1) + i), i=1,2,...,n
    % min<=Ac<=max  
    % [A;-A]c<=[max;-min] 
    %-----------------------------------------------
    % max
    for k = 1:n_seg 
        coeff_p = [coeff_p; ones(n_order+1,1) * ts(k)^(1)];    % coeff_p(1 + (k-1)*(n_order+1):k*(n_order+1))
        bieq_p  = [bieq_p;  ones(n_order+1,1) * corridor_range(k,2)];
    end
    % -min
    for k = 1:n_seg 
        bieq_p  = [bieq_p;  ones(n_order+1,1) * corridor_range(k,1)*(-1)];
    end
    Aieq_p = diag(coeff_p,0);
    Aieq_p = [Aieq_p; -Aieq_p];
    %#####################################################
    % STEP 3.2.2 v constraint 
%     The derivative curve B(1)(t) of a Bezier curve B(t) with the controls points
%     (c(i))- c(i-1))
    Aieq_v = [];
%     bieq_v = [];
    n_ctr = n_order;      % the number of control posints after first deferention: n_order 
    n_eq = n_ctr*n_seg*2; % number of equations (max and min constraints)
    for k = 1:n_seg
        for n = 1:n_ctr
            index_col = (k-1)*(n_order+1)+n;
            Aieq_v(n+(k-1)*n_ctr,index_col:index_col+1) = n_order * [-1, 1] * ts(k)^(0);
        end
    end
    Aieq_v = [Aieq_v;-Aieq_v];
    bieq_v = ones(n_eq,1)* v_max;

    %#####################################################
    % STEP 3.2.3 a constraint  
%     The derivative curve B(2)(t) of a Bezier curve B(t) with the controls points
%     (c(i))- 2*c(i-1) + c(i-2))
    Aieq_a = [];
%     bieq_a = [];
    n_ctr = n_order-1;    % the number of control posints after second deferention: n_order - 1 
    n_eq = n_ctr*n_seg*2; % number of equations (max and min constraints)
    for k = 1:n_seg
        for n = 1:n_ctr
            index_col = (k-1)*(n_order+1)+n;
            Aieq_a(n+(k-1)*n_ctr,index_col:index_col+2) = n_order * (n_order-1) * [1, -2, 1] * ts(k)^(-1);
        end
    end
    Aieq_a = [Aieq_a;-Aieq_a];
    bieq_a = ones(n_eq,1)*a_max;
    %#####################################################
%     jerk:
%     The derivative curve B(1)(t) of a Bezier curve B(t) with the controls points
%     (c(i))- 3*c(i-1) + 3*c(i-2) - c(i-3)), i>=3
    %#####################################################
    % combine all components to form Aieq and bieq   
    Aieq = [Aieq_p; Aieq_v; Aieq_a];
    bieq = [bieq_p; bieq_v; bieq_a];

end