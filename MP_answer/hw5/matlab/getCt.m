function Ct = getCt(n_seg, n_order)
    %#####################################################
    % STEP 2.1: finish the expression of Ct
    %
    %
    %
    %
    %
    d_order=(n_order+1)/2;
    % d -------------- []2*d_order*n_seg x 1
    % [d_F d_p] ------ []d_order*(n_seg+1) x 1
    Ct=zeros(2*d_order*n_seg,d_order*(n_seg+1));
    % from 1 to n_seg-th segment, each segment has a start and end point
    for j=1:n_seg 
        % first segment
        if(j==1) 
            for n=1:d_order
                Ct(n,n)=1; % start point
            end
        end
        % end points from 1 to (n_seg-1)-th segment
        if(j<=n_seg-1)
            Ct((j-1)*2*d_order+1+d_order,j+d_order)=1; % end point's p  = the next piont's p
            for n=1:d_order-1
                Ct((j-1)*2*d_order+1+d_order+n,n_seg-1+2*d_order+(j-1)*3+n)=1; % end point's v a j is free
            end
        end
        % start points from 2 to n_seg-th segment
        if(j>=2)
            Ct((j-1)*2*d_order+1,j+d_order-1)=1; % start point's p  = the previous point's p
            for n=1:d_order-1
                Ct((j-1)*2*d_order+1+n,n_seg-1+2*d_order+(j-2)*3+n)=1; % start point's v a j  = the pre piont's v a j
            end
        end
        % last segment
        if(j==n_seg) 
            for n=1:d_order
                Ct((j-1)*2*d_order+d_order+n,n_seg+d_order-1+n)=1; % end point
            end
        end
        
    end   
end