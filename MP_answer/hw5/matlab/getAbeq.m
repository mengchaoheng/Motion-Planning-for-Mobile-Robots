function [Aeq, beq]= getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond)
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % p,v,a,j constraint in start, 
    Aeq_start = zeros(4, n_all_poly);
    beq_start = zeros(4, 1);
    % STEP 2.1: write expression of Aeq_start and beq_start
    %
    %
    %
    %
    for k=1:4
        Aeq_start(k,k)=factorial(k-1);
    end
    beq_start(1)=start_cond(1);
    %#####################################################
    % p,v,a,j constraint in end
    Aeq_end = zeros(4, n_all_poly);
    beq_end = zeros(4, 1);
    % STEP 2.2: write expression of Aeq_end and beq_end
    %
    %
    %
    %
    for k=1:4
        for i=k:n_order+1
            Aeq_end(k,i+(n_seg-1)*(n_order+1))=factorial(i-1)/factorial(i-1-(k-1)) * ts(end)^(i-1-(k-1));  
        end
    end
    beq_end(1)=end_cond(1);
    %#####################################################
    % position constrain in all middle waypoints
    Aeq_wp = zeros(n_seg-1, n_all_poly);
    beq_wp = zeros(n_seg-1, 1);
    % STEP 2.3: write expression of Aeq_wp and beq_wp
    %
    %
    %
    %
    for j=1:n_seg-1
        for i=1:n_order+1
            Aeq_wp(j,i+(j-1)*(n_order+1))=ts(j)^(i-1);  
        end
        beq_wp(j)=waypoints(j+1);
    end
    %#####################################################
    % position continuity constrain between each 2 segments
    Aeq_con_p = zeros(n_seg-1, n_all_poly);
    beq_con_p = zeros(n_seg-1, 1);
    % STEP 2.4: write expression of Aeq_con_p and beq_con_p
    %
    %
    %
    %
    for j=1:n_seg-1
        for i=1:n_order+1
            Aeq_con_p(j,i+(j-1)*(n_order+1))=ts(j)^(i-1);  
        end
        Aeq_con_p(j,1+j*(n_order+1))=-1;
    end
    %#####################################################
    % velocity continuity constrain between each 2 segments
    Aeq_con_v = zeros(n_seg-1, n_all_poly);
    beq_con_v = zeros(n_seg-1, 1);
    % STEP 2.5: write expression of Aeq_con_v and beq_con_v
    %
    %
    %
    %
    for j=1:n_seg-1
        for i=2:n_order+1
            Aeq_con_v(j,i+(j-1)*(n_order+1))=(i-1)*ts(j)^(i-1-1);  
        end
        Aeq_con_v(j,2+j*(n_order+1))=-1;
    end
    %#####################################################
    % acceleration continuity constrain between each 2 segments
    Aeq_con_a = zeros(n_seg-1, n_all_poly);
    beq_con_a = zeros(n_seg-1, 1);
    % STEP 2.6: write expression of Aeq_con_a and beq_con_a
    %
    %
    %
    %
    for j=1:n_seg-1
        for i=3:n_order+1
            Aeq_con_a(j,i+(j-1)*(n_order+1))=(i-1)*(i-2)*ts(j)^(i-1-2);  
        end
        Aeq_con_a(j,3+j*(n_order+1))=-2;
    end
    %#####################################################
    % jerk continuity constrain between each 2 segments
    Aeq_con_j = zeros(n_seg-1, n_all_poly);
    beq_con_j = zeros(n_seg-1, 1);
    % STEP 2.7: write expression of Aeq_con_j and beq_con_j
    %
    %
    %
    %
    for j=1:n_seg-1
        for i=4:n_order+1
            Aeq_con_j(j,i+(j-1)*(n_order+1))=(i-1)*(i-2)*(i-3)*ts(j)^(i-1-3);  
        end
        Aeq_con_j(j,4+j*(n_order+1))=-3*2;
    end
    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a; Aeq_con_j];
    beq_con = [beq_con_p; beq_con_v; beq_con_a; beq_con_j];
  
    Aeq = [Aeq_start; Aeq_end; Aeq_wp; Aeq_con];
    beq = [beq_start; beq_end; beq_wp; beq_con];
end