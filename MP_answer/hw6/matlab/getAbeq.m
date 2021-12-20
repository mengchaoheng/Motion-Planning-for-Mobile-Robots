function [Aeq, beq] = getAbeq(n_seg, n_order, ts, start_cond, end_cond)
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % STEP 2.1 p,v,a constraint in start 
%     Aeq_start = [];
%     beq_start = [];
    Aeq_start = zeros(3, n_all_poly); 
    Aeq_start(1,1) = 1 * ts(1)^(1);
    Aeq_start(2,1:2) = n_order * [-1,1] * ts(1)^(0);
    Aeq_start(3,1:3) = n_order * (n_order-1) * [1, -2, 1] * ts(1)^(-1);
    beq_start =  start_cond';
    %#####################################################
    % STEP 2.2 p,v,a constraint in end
%     Aeq_end = [];
%     beq_end = [];
    Aeq_end = zeros(3, n_all_poly); 
    Aeq_end(1,end) = 1 * ts(end)^(1);
    Aeq_end(2,end-1:end) = n_order * [-1, 1] * ts(end)^(0);
    Aeq_end(3,end-2:end) = n_order * (n_order-1) * [1, -2, 1]  * ts(end)^(-1);    
    beq_end = end_cond';
    %#####################################################
    % STEP 2.3 position continuity constrain between 2 segments
%     Aeq_con_p = [];
%     beq_con_p = [];
    Aeq_con_p = zeros(n_seg-1, n_all_poly);
    for k = 1:n_seg-1
        Aeq_con_p(k,k*(n_order+1)) = 1 * ts(k)^(1);
        Aeq_con_p(k,k*(n_order+1)+1) = -1 * ts(k+1)^(1);
    end
    beq_con_p = zeros(n_seg-1,1);

    %#####################################################
    % STEP 2.4 velocity continuity constrain between 2 segments
%     Aeq_con_v = [];
%     beq_con_v = [];
    Aeq_con_v =  zeros(n_seg-1, n_all_poly);
    for k = 1:n_seg-1
        Aeq_con_v(k,k*(n_order+1)-1:k*(n_order+1)) = [-1, 1] * ts(k)^(0);
        Aeq_con_v(k,k*(n_order+1)+1:k*(n_order+1)+2) = -1 * [-1, 1] * ts(k+1)^(0);
    end    
    beq_con_v = zeros(n_seg-1,1);
    %#####################################################
    % STEP 2.5 acceleration continuity constrain between 2 segments
%     Aeq_con_a = [];
%     beq_con_a = [];
    Aeq_con_a = zeros(n_seg-1, n_all_poly);
    for k = 1:n_seg-1
        Aeq_con_a(k,k*(n_order+1)-2:k*(n_order+1)) = [1, -2, 1] * ts(k)^(-1);
        Aeq_con_a(k,k*(n_order+1)+1:k*(n_order+1)+3) = -1 * [1, -2, 1] * ts(k+1)^(-1);
    end  
    beq_con_a = zeros(n_seg-1,1);
    %#####################################################
    
    % combine all components to form Aeq and beq   
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a];
    beq_con = [beq_con_p; beq_con_v; beq_con_a];
    Aeq = [Aeq_start; Aeq_end; Aeq_con];
    beq = [beq_start; beq_end; beq_con];
end