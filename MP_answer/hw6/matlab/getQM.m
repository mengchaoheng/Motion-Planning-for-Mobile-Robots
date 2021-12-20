function [Q, M] = getQM(n_seg, n_order, d_order, ts)
    Q = [];
    M = [];
%     d_order = 3;
    M_j = getM(n_order);
    for k = 1:n_seg
        Q_k=zeros(n_order+1,n_order+1);
        for i=d_order:n_order
            for j=d_order:n_order
                Q_k(i+1,j+1)=(factorial(i)/factorial(i-d_order) * factorial(j)/factorial(j-d_order) / (i-d_order+j-d_order+1)) /ts(k)^(2*d_order-3); % Integrate between 0 and 1, 
%                 ts(k)^(2*d_order-3) is scale factor in position of each piece of the curve achieves better numerical stability for the optimization program.
            end
        end
        Q = blkdiag(Q, Q_k);
        M = blkdiag(M, M_j);
    end
end
