function M = getM(n_seg, n_order, ts)
    M = [];
    for k = 1:n_seg
%         M_k = [];
        %#####################################################
        % STEP 1.1: calculate M_k of the k-th segment 
        %
        %
        %
        %
        M_k_0 = zeros(4,n_order+1);
        M_k_T = zeros(4,n_order+1);
        for n=1:4 % The n-th row corresponds to the n-1 order derivative
            M_k_0(n,n)=factorial(n-1); 
            for i=n:n_order+1 % The i-th column corresponds to p_(i-1)
                M_k_T(n,i)=factorial(i-1)/factorial(i-1-(n-1)) * ts(k)^(i-1-(n-1));  
            end
        end
        M_k=[M_k_0;M_k_T];
        M = blkdiag(M, M_k);
    end
end