function C_ij = match_with_cvx(A,B, opts)


nA = size(A,1);
nB = size(B,1);

alpha = 500;
r = nB/nA;

W_ij = pdist2(A,B).^2;

cvx_begin

variable C_ij(nA,nB)

cost = sum(vec(C_ij.*W_ij)) + opts.deficient_row_penalty*sum(r-sum(C_ij,2)) +opts.deficient_col_penalty*sum(1-sum(C_ij,1));

minimize(cost)
subject to 
    sum(C_ij,1) <= 1;
    sum(C_ij,2) <= r;
    C_ij >= 0;
cvx_end


end